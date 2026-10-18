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
├── CtrlInterfaces.h             # 零拷贝命令包装 + ActiveState（唯一的 FSM 可见状态） + ControlInputs
├── estimator/
│   └── RovStateEstimator.h     # 500Hz 多速率传感器融合（CLAUDE.md Phase 3.5）
├── FSM/
│   ├── FSMState.h               # 抽象基类（checkFsmCommandChange()/targetPose() 共享实现）
│   ├── StatePassive.h
│   ├── StateDepthHold.h
│   ├── StateOpenLoop.h
│   └── StatePidHold.h
└── common/
    ├── enumClass.h              # FSMStateName（PASSIVE/DEPTH_HOLD/OPEN_LOOP/PID_HOLD）/ FSMMode
    ├── mathTypes.h              # Eigen 类型别名（Vec3/Vec6/Vec8/Mat68/Mat86/Mat34/Mat43/Mat48/RotMat）
    ├── mathTools.h              # quatToRotMat / rpyToRotMat / rotMatToRPY / quatMul / quatInverse /
    │                            # quatFromRPY / quatError / smallAngleQuat / saturation（Phase 3.6）
    ├── MultiRateDecimator.h     # 均匀整数抽取多速率调度（CLAUDE.md Phase 3.1.1）
    ├── LoopProfiler.h           # 频率(Hz)/执行延迟(ms)/超时计数实时观测（CLAUDE.md Phase 3.6）
    ├── ModelParams.h            # 运行时加载的模型解耦物理/增益/传感器安装参数（CLAUDE.md Phase 3.3/3.5）
    └── ThrusterAllocator.h      # 6x8 推力分配矩阵 B/B^+ + Pre-MPPI 的 G/G^+/A_contact/C_kern（Phase 3.6）

src/
├── adhere_controller.cpp        # 生命周期方法 + 均匀多速率调度 + 500Hz 状态估计/融合 + 4 话题订阅/发布 + FSM 切换逻辑 + pluginlib 导出
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
├── test_multirate_decimator.cpp      # 纯逻辑 gtest，不依赖 rclcpp/hardware_interface
├── test_thruster_allocator.cpp       # 纯 Eigen gtest：B*B^+ ~= I_6，零寄生力矩，A_contact/C_kern（Phase 3.6）
├── test_fsm_states.cpp               # 闭环 gtest：假 CtrlInterfaces + 质点平面"虚拟物理"验证 3 态收敛性
├── test_sensor_fusion_and_teleop.cpp # DVL 杠杆臂补偿收敛性 + sensor_fusion 闭环跟踪 + LT/RT 映射（CLAUDE.md Phase 3.5）
└── test_loop_profiler.cpp            # 纯逻辑 gtest：Hz EMA 收敛性 + 执行延迟/超时计数（CLAUDE.md Phase 3.6）
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

## `CtrlInterfaces`：零拷贝命令包装 + 严格地面真值隔离（CLAUDE.md Phase 3.5）

```cpp
struct ActiveState {
  double pos_w[3];        // 世界坐标系质心位置 [x, y, z]
  double quat_w[4];       // 世界坐标系姿态四元数 [w, x, y, z]（Phase 3.6）
  double rpy[3];          // 由 quat_w 派生，仅供 StateDepthHold 的小角度阻尼使用
  double vel_body[3];     // 机体坐标系线速度
  double omega_body[3];   // 机体坐标系角速度
  double pad_force[4];    // TL/TR/BL/BR 法向接触力
  double v_sep[4];         // TL/TR/BL/BR 分离速度
};

struct CtrlInterfaces {
  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
    thruster_command_interface_;   // 8: thr_N1..thr_N4, thr_T5..thr_T8
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
    thruster_state_interface_;     // 8（推力器力反馈，非位姿/速度，不受隔离约束）
  ActiveState active_state;        // 唯一允许 FSMState 读取的位姿/速度/接触力来源
  ControlInputs control_inputs_;
  int frequency_ = 500;
};
```

**严格地面真值隔离**：`imu_state_interface_`/`odom_state_interface_`/
`pad_contact_state_interface_`（地面真值）与 4 组噪声多速率传感器状态接口
**均不是** `CtrlInterfaces` 的成员——它们是 `AdhereController` 自己的私有成员
（`gt_*_state_interface_`/`noisy_*_state_interface_`），任何 `FSMState` 都
无法访问。`AdhereController::update()` 在每个 500Hz tick 的内环门控**之前**，
依据 `estimation_mode`（见下）把地面真值或 `RovStateEstimator` 的融合输出
写入 `ctrl_interfaces_.active_state`——这是 4 个 `FSMState` 读取位姿/速度/
接触力的唯一入口。

`ControlInputs` 对应 CLAUDE.md Phase 3.2 的 4 话题解耦契约，持有 3 组正交字段
（均为 `std::atomic<T>`，由 `AdhereController` 的订阅回调——ROS executor
线程——写入，由当前活跃 `FSMState::run()`——500Hz 物理线程——无锁读取）：
`command`（`/rov/fsm_command`）、`setpoint_*` + `setpoint_seq`
（`/rov/setpoint_cmd`，沿边检测，Phase 3.5 新增 `setpoint_target_roll/pitch`）、
`teleop_*` + `teleop_last_rx_time_sec`（`/rov/teleop_cmd`，`0.25s` 失联看门狗
判据，Phase 3.5 新增 `teleop_pitch_rate`/`teleop_roll_rate`）。

## `RovStateEstimator`：500Hz 多速率传感器融合（CLAUDE.md Phase 3.5）

`model_params_.estimation_mode`（`robot_control.yaml`/`mujoco.launch.py` 的
`estimation_mode` 启动参数；注意该参数**不**写在 `robot_control.yaml` 里，
见该文件内的说明注释）控制 `ActiveState` 的来源：

- `"ground_truth"`（默认）：直接拷贝 `AdhereMujocoHardware` 的无噪声
  `imu_sensor`/`rov_odom`/`pad_contact` 状态接口。
- `"sensor_fusion"`：每个 500Hz tick 调用 `RovStateEstimator::predict()`
  （融合噪声 IMU 陀螺/加计）与 `correctAttitude()`（融合噪声 IMU 姿态四元数），
  并在噪声 `depth_sensor_noisy`/`dvl_sensor_noisy` 采样保持值发生变化时调用
  `correctDepth()`/`correctDvl()`（含精确杠杆臂补偿
  `y = v_dvl_meas - omega_hat x r_dvl`）。`RovStateEstimator` 是纯数学头文件
  （无 ROS/MuJoCo 依赖），实现的是**按通道解耦的固定增益（稳态）
  Kalman/互补滤波**，而非单一联合协方差 EKF——增益依据 CLAUDE.md Phase 3.5
  给定的噪声标准差，用指数滑动平均稳态方差公式
  `Var(y_hat) = K/(2-K) * Var(meas)` 解析选取，并在
  `test_sensor_fusion_and_teleop.cpp` 中验证满足 `<0.015 m/s`/`<15mm`/
  `<1.0deg` 的 RMSE 契约。

无论哪种模式，地面真值都会被读入 `AdhereController::update()` 的局部变量，
仅用于首次 `reset()` 估计器与填充 `ground_truth` 模式的 `active_state`
——Phase 3.6 起**不再**通过本包发布地面真值（`/rov/fsm_state` 已删除）：
地面真值改由 `adhere_mujoco_node` 单独发布在 `/rov/ground_truth_odom`
（`nav_msgs/Odometry`），仅供离线评估参考。

## 当前状态（4 态，CLAUDE.md Phase 3.2/3.3/3.5）

| 状态 | ID | 职责 |
|---|---|---|
| `PASSIVE` (`state_passive`) | 1 | `enter()`/`run()` 均将 8 个推力器指令置零，安全默认态 |
| `DEPTH_HOLD` (`state_depth_hold`) | 2 | 世界坐标系 Z 向 PID 定深叠加姿态阻尼；`target_z_` 接受 `/rov/setpoint_cmd` 离散设定（沿边应用）与 Xbox 左摇杆连续微调，钳制在 `robot_control.yaml` 的 `workspace_z_min/max` 内 |
| `OPEN_LOOP` (`state_open_loop`) | 3 | 无记忆地把 `/rov/teleop_cmd` 的 6 维遥操作输入（含 Phase 3.5 新增的 `pitch_rate`/`roll_rate`）线性映射为机体力矩（增益来自 `open_loop.max_wrench`），用于验证 `B`/`B^+` 分配零寄生耦合 |
| `PID_HOLD` (`state_pid_hold`) | 4 | `enter()` 锁存当前 `(x,y,z,roll,pitch,yaw)` 为保持点；摇杆/扳机推动期间按机体对齐速度平移/旋转该设定点（增益来自 `pid_hold.*`，roll/pitch 目标钳制在 `±max_tilt_deg`），松手瞬间冻结；6-DOF PID + 浮力/科氏力/阻力前馈 |

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

## Pre-MPPI 基础设施：`A_contact`/`C_kern` 预计算与异步外环工作线程骨架（CLAUDE.md Phase 3.6）

`ThrusterAllocator.h` 在 `B`/`B^+` 之外新增 4 个函数，供 `AdhereController::
on_activate()`（与各 `FSMState` 构造 `B`/`B^+` 同一时机）预计算：

- `buildPadNormalMap(pad_pos)` → `G ∈ R^{3x4}`：第 `i` 列为
  `[+1, -z_i, y_i]^T`，把 4 个贴垫法向力 `F_N,i >= 0`（沿 body `+X`）映射到
  `[Fx, Ty, Tz]^T`（**注意**：代码按数值算例取 `+1`，与 `CLAUDE.md` §4.4
  文字公式字面写的 `-1` 不一致，详见 `ThrusterAllocator.h` 内联注释）；
- `buildPadNormalMapPseudoInverse(G)` → `G^+ ∈ R^{4x3}`（阻尼伪逆）；
- `buildContactMap(G^+, B)` → `A_contact = -G^+ B_{[Fx,Ty,Tz]} ∈ R^{4x8}`，
  估计 `F_N_hat ≈ A_contact * u`；
- `buildKernConstraintMatrix(B, Ly, Lz, eps_margin)` → `C_kern ∈ R^{4x8}`
  （Rhombic Kern 4 条边约束，尚未接入任何实时约束，留给 Phase 4 的 12D
  OSQP）。

`a_contact_`/`c_kern_` 用于把 `/rov/controller_state` 的 `contact_forces`/
`kern_margin` 由**已下达的推力指令**估计出来（而非直接读测量贴垫力
传感器），与未来 KAMR-MPPI 约束用的是同一套准静态预测模型。

`AdhereController` 新增一个真实的异步外环工作**线程**
（`outerWorkerLoop()`，`on_activate()` 启动、`on_deactivate()`/析构函数
`join()`），通过 `mutex`+`condition_variable` 保护的 `OuterLoopInput`/
`OuterLoopOutput` 双缓冲结构体与 500Hz 物理线程非阻塞握手：每个
`tick.outer_due` 把当前 `active_state`+`targetPose()` 快照写入输入缓冲并
`notify_one()`。目前 worker 只做"回传序号，不做任何真实求解"的占位
处理——Phase 4 只需替换 worker 循环体，线程生命周期/握手架构本身不用再改。

## 实时诊断：`LoopProfiler` + `RuntimeDiagnostics`（CLAUDE.md Phase 3.6）

`LoopProfiler.h`（纯头文件，无 ROS/Eigen 依赖，风格与 `MultiRateDecimator.h`
一致）提供 `tickMarker()`（基于 `steady_clock` 调用间隔的 EMA 估计触发
频率）与 `beginExec()`/`endExec()`（测最新/EMA均值/最大执行耗时，并对
`budget_ms` 计超时次数）。`AdhereController` 为估计器、内环（预算
`4.0ms`）、外环（预算 `40.0ms`）、IMU/称重传感器（逐 tick）、深度/DVL
（采样保持值变化时边沿触发）各建一个 `LoopProfiler` 实例，每 50 个 500Hz
tick（固定 10Hz，独立于 `inner_decimation_`/`outer_decimation_`）汇总发布
`/rov/runtime_diagnostics`，并发布一个绿/红
`visualization_msgs::Marker`（`TEXT_VIEW_FACING`）到
`/rov/diagnostics_hud_marker`，浮在 ROV 正上方。`hw_loop_hz`/
`hw_step_exec_ms` 两路直接读取 `AdhereMujocoHardware` 新增的
`diagnostics/hw_loop_hz`/`hw_step_exec_ms` 状态接口（镜像其
`MujocoVisBridge` 既有的物理线程 EMA 遥测，本包不反向依赖
`adhere_mujoco_hardware` 内部类型）。`estimator_hz` 在
`estimation_mode=="ground_truth"` 时合法地报 `0Hz`（估计器本就没跑），
因此被排除在 `timing_healthy` 的频率检查之外。

### `robot_control.yaml` 参数文件与节点重映射陷阱

闭环参数由 `simplified_v1_description/config/robot_control.yaml` 提供，通过
`simplified_v1_description/launch/mujoco.launch.py`（CLAUDE.md Phase 3.4：本包
不再持有任何启动文件）作为 `--params-file` 传给 `adhere_mujoco_node` 可执行文件。
**关键陷阱**：`ros2 launch` 的 `Node` 动作总会附加一条无作用域的
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

本包（CLAUDE.md Phase 3.4）不再持有任何启动文件——是一个纯粹的、通用的
`controller_interface` 插件工具箱。实际启动入口在
[`simplified_v1_description/launch/mujoco.launch.py`](../../descriptions/simplified_v1_description/README.md)：

```bash
colcon build --packages-up-to adhere_controller --symlink-install
source install/setup.bash

ros2 launch simplified_v1_description mujoco.launch.py
ros2 launch simplified_v1_description mujoco.launch.py headless:=true use_rviz:=false \
    launch_teleop:=false
```

默认行为即把 `autostart_controllers:=['adhere_controller']` 与
`simplified_v1_description/config/robot_control.yaml` 一并传给同一个通用
`adhere_mujoco_node` 可执行文件；传入 `autostart_controllers:="[]"` 可还原为纯
开环、0 个控制器加载的行为。`launch_teleop:=false` 时的上位机/下位机分布式
部署方式见 [`adhere_keyboard_input/README.md`](../../commands/adhere_keyboard_input/README.md)。
