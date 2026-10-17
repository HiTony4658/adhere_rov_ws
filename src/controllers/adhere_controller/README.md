# `adhere_controller`

## 职责

`AdhereController` 是唯一的 `controller_interface::ControllerInterface`
`pluginlib` 插件（`adhere_controller/AdhereController`，Phase 3.3 合并自此前的
`adhere_controller_common` + `adhere_kamr_controller` 两个包）。当前（Phase
3.3）承载 FSM 管理、生命周期管理、接口装配、均匀整数抽取多速率调度、4 话题
订阅/发布、Xbox 手柄 `0.25s` 失联看门狗与状态切换，以及 `PASSIVE`/
`DEPTH_HOLD`/`OPEN_LOOP`/`PID_HOLD` 四个 FSM 状态的完整实现。未来阶段将在本包
内继续承载 `KAMR-MPPI`（`pr_wbc_mppi_v4`）算法的完整实现：250Hz 内环（Planar
Impedance PD + Pad-Force DOB + 12D Elastic Rhombic-Kern OSQP）、25Hz 异步 CUDA
MPPI 外环。

## 目录结构

```text
include/adhere_controller/
├── adhere_controller.hpp        # AdhereController 声明
├── CtrlInterfaces.h             # 零拷贝状态/命令包装 + ControlInputs
├── FSM/
│   ├── FSMState.h               # 抽象基类（checkFsmCommandChange()/targetPose() 共享实现）
│   ├── StatePassive.h
│   ├── StateDepthHold.h
│   ├── StateOpenLoop.h
│   └── StatePidHold.h
└── common/
    ├── enumClass.h              # FSMStateName（PASSIVE/DEPTH_HOLD/OPEN_LOOP/PID_HOLD）/ FSMMode
    ├── mathTypes.h              # Eigen 类型别名（Vec3/Vec6/Vec8/Mat68/Mat86/RotMat）
    ├── mathTools.h              # quatToRotMat / rotMatToRPY / saturation
    ├── MultiRateDecimator.h     # 均匀整数抽取多速率调度（CLAUDE.md Phase 3.1.1）
    ├── ModelParams.h            # 运行时加载的模型解耦物理/增益参数（CLAUDE.md Phase 3.3）
    └── ThrusterAllocator.h      # 由 ModelParams 几何动态构建的 6x8 推力分配矩阵 B 及其阻尼伪逆

src/
├── adhere_controller.cpp        # 生命周期方法 + 均匀多速率调度 + 4 话题订阅/发布 + FSM 切换逻辑 + pluginlib 导出
└── FSM/
    ├── StatePassive.cpp
    ├── StateDepthHold.cpp
    ├── StateOpenLoop.cpp
    └── StatePidHold.cpp

config/controllers.yaml          # controller_manager 参数服务器文档（历史参考，见下）
launch/mujoco.launch.py          # 闭环启动文件：robot_state_publisher + adhere_mujoco_node
                                  # (autostart_controllers=['adhere_controller']) + RViz2
                                  # + launch_teleop:=true 时自动包含 adhere_keyboard_input 的
                                  # teleop_gcs.launch.py（joy_node + xbox_teleop_node + fsm_console_node）

test/
├── test_multirate_decimator.cpp  # 纯逻辑 gtest，不依赖 rclcpp/hardware_interface
├── test_thruster_allocator.cpp   # 纯 Eigen gtest：B*B^+ ~= I_6，零寄生力矩
└── test_fsm_states.cpp           # 闭环 gtest：假 CtrlInterfaces + 质点平面"虚拟物理"验证 3 态收敛性
```

## 模型解耦：`ModelParams` + 动态 `ThrusterAllocator`（CLAUDE.md 第 3 节）

`adhere_controller` 的 C++ 代码里不包含任何机器人专属几何或物理常数。
`ModelParams::loadFromNode(get_node())` 在 `AdhereController::on_configure()`
中被调用一次，从 `simplified_v1_description/config/robot_control.yaml`（经由
`launch/mujoco.launch.py` 作为 `--params-file` 传给 `adhere_mujoco_node`）读取：

- `thruster_names/thruster_pos_xyz/thruster_dir_xyz/thruster_min/thruster_max`
  （8 个推力器的站点坐标、推力方向单位向量与限幅，body 坐标系，相对质心）；
- `pad_names/pad_pos_xyz`（4 个贴垫坐标，供未来 WBC/Rhombic Kern 使用）；
- `rho_water/gravity/vehicle_mass/augmented_mass/displaced_volume/
  cob_offset_body/z_surface/z_transition/D_lin/D_quad/D_ang_lin/D_ang_quad`
  （Fossen 水动力前馈参数）；
- `workspace_z_min/workspace_z_max/workspace_x_min`（世界坐标安全边界）；
- `depth_hold.*`/`open_loop.*`/`pid_hold.*`（各 FSM 状态的增益/限幅）。

`ThrusterAllocator.h::buildThrusterAllocationMatrix(pos, dir)` 对每个推力器
通用地构造 `b_i = [dir_i; pos_i x dir_i] ∈ R^6`（叉乘公式，不再硬编码角点/
`zaxis` 几何），拼成 `B ∈ R^{6x8}`；`buildThrusterAllocationPseudoInverse()`
计算阻尼 Moore-Penrose 伪逆 `B^+ = B^T(BB^T + 1e-6*I_6)^-1`。切换到几何/质量
不同的新机型只需改 `robot_control.yaml`，本包零 C++ 改动、零重新编译。

> **对 `CLAUDE.md` 第 4.3 节占位几何的一处偏离（Phase 3.2 起延续）**：
> `robot_control.yaml` 的 `thruster_pos_xyz`/`thruster_dir_xyz` 编码的是真实
> `rov_wall.xml` 几何（全部 8 个站点在 body `X=0.0`，与质心同一 `X` 坐标，
> 零杠杆臂），而非 `CLAUDE.md` 第 4.3 节给出的示意性 `X=-0.10m` 杠杆臂占位
> 常数（二者描述的推力器安装偏移不同，已与用户确认按真实几何实现）。

## `CtrlInterfaces`：零拷贝状态/命令包装

```cpp
struct CtrlInterfaces {
  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
    thruster_command_interface_;   // 8: thr_N1..thr_N4, thr_T5..thr_T8
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
    thruster_state_interface_,     // 8
    imu_state_interface_,          // 10: orientation.w/x/y/z, angular_velocity.*, linear_acceleration.*
    odom_state_interface_,         // 10: pos.*, quat.w/x/y/z, vel_body.*
    pad_contact_state_interface_;  // 8: force TL/TR/BL/BR, v_sep TL/TR/BL/BR
  ControlInputs control_inputs_;
  int frequency_ = 500;
};
```

`ControlInputs` 对应 CLAUDE.md Phase 3.2 的 4 话题解耦契约，持有 3 组正交字段
（均为 `std::atomic<T>`，由 `AdhereController` 的订阅回调——ROS executor
线程——写入，由当前活跃 `FSMState::run()`——500Hz 物理线程——无锁读取）：
`command`（`/rov/fsm_command`）、`setpoint_*` + `setpoint_seq`
（`/rov/setpoint_cmd`，沿边检测）、`teleop_*` + `teleop_last_rx_time_sec`
（`/rov/teleop_cmd`，`0.25s` 失联看门狗判据）。

## 当前状态（4 态，CLAUDE.md Phase 3.2/3.3）

| 状态 | ID | 职责 |
|---|---|---|
| `PASSIVE` (`state_passive`) | 1 | `enter()`/`run()` 均将 8 个推力器指令置零，安全默认态 |
| `DEPTH_HOLD` (`state_depth_hold`) | 2 | 世界坐标系 Z 向 PID 定深叠加姿态阻尼；`target_z_` 接受 `/rov/setpoint_cmd` 离散设定（沿边应用）与 Xbox 左摇杆连续微调，钳制在 `robot_control.yaml` 的 `workspace_z_min/max` 内 |
| `OPEN_LOOP` (`state_open_loop`) | 3 | 无记忆地把 `/rov/teleop_cmd` 的 4 维遥操作输入线性映射为机体力矩（增益来自 `open_loop.max_wrench`），用于验证 `B`/`B^+` 分配零寄生耦合 |
| `PID_HOLD` (`state_pid_hold`) | 4 | `enter()` 锁存当前 `(x,y,z,yaw)` 为保持点；摇杆推动期间按机体对齐速度平移/旋转该设定点（增益来自 `pid_hold.*`），松手瞬间冻结；6-DOF PID + 浮力/科氏力/阻力前馈 |

## `MultiRateDecimator`：均匀整数抽取多速率调度

```cpp
class MultiRateDecimator {
public:
  MultiRateDecimator(int cm_update_rate, int inner_decimation, int outer_decimation);
  struct TickResult { bool inner_due; bool outer_due; };
  TickResult tick();                     // 对 500Hz 主时钟做纯整数取模抽取
  double innerPeriodSeconds() const;     // inner_decimation / cm_update_rate（恒定）
  double outerPeriodSeconds() const;     // outer_decimation / cm_update_rate（恒定）
  void reset();                          // 下次 tick() 令 inner_due/outer_due 同时为真
};
```

默认 `inner_decimation=2`、`outer_decimation=20` 使内环精确 `250Hz`
（`dt=4.0ms` 恒定）、外环精确 `25Hz`（`dt=40.0ms` 恒定），`outer_decimation`
也可配置为 `25` 得到 `20Hz` 外环。

### `robot_control.yaml` 参数文件与节点重映射陷阱

闭环参数由 `simplified_v1_description/config/robot_control.yaml` 提供，通过
`launch/mujoco.launch.py` 作为 `--params-file` 传给 `adhere_mujoco_node`
可执行文件。**关键陷阱**：`ros2 launch` 的 `Node` 动作总会附加一条无作用域的
`-r __node:=<name>` 重映射规则（此处 `<name>` 即该可执行文件被赋予的名字
`adhere_mujoco_node`），对进程内所有使用默认 `NodeOptions` 构造的节点都生效
——不仅是顶层 `rclcpp::Node`，连内部 `ControllerManager` 节点、以及
`AdhereController` 自己的 `controller_interface` 节点，三者在 ROS 图里都显示
为同一个 `/adhere_mujoco_node`。因此 `robot_control.yaml` 里
`controller_manager` 与 `AdhereController` 各自的 `ros__parameters`
**必须合并在同一个顶层键 `adhere_mujoco_node:` 下**。

## 线程安全

本包所有类均**不**自带线程同步原语，仅 `ControlInputs` 使用
`std::atomic<T>`；`CtrlInterfaces` 的读写发生在 `controller_manager` 的单一
物理线程（500Hz）内，天然串行。

## 运行

```bash
colcon build --packages-up-to adhere_controller --symlink-install
source install/setup.bash

ros2 launch adhere_controller mujoco.launch.py
ros2 launch adhere_controller mujoco.launch.py headless:=true use_rviz:=false \
    launch_teleop:=false
```

与 `adhere_mujoco_hardware/launch/mujoco_sim.launch.py`（纯开环，0 个控制器）
不同，本启动文件会把 `autostart_controllers:=['adhere_controller']` 与
`simplified_v1_description/config/robot_control.yaml` 一并传给同一个通用
`adhere_mujoco_node` 可执行文件。`launch_teleop:=false` 时的上位机/下位机分布式
部署方式见 [`adhere_keyboard_input/README.md`](../../commands/adhere_keyboard_input/README.md)。
