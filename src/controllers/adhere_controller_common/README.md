# `adhere_controller_common`

## 职责

自研控制基础设施库（纯 C++ 共享库，非 `ros2_control` 插件），被
`adhere_kamr_controller` 依赖。提供零拷贝的硬件接口包装（`CtrlInterfaces`）、
有限状态机（FSM）抽象基类与 4 个具体状态实现、共享的推力分配矩阵工具
（`ThrusterAllocator`），以及通用数学工具。未来的 `FossenHydrodynamics`
（当前位于 `adhere_mujoco_hardware`）迁移将在 Phase 4+ 加入本包。

## 目录结构

```text
include/adhere_controller_common/
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
    └── ThrusterAllocator.h      # 共享 6x8 推力分配矩阵 B 及其阻尼伪逆（CLAUDE.md Phase 3.2）

src/FSM/
├── StatePassive.cpp
├── StateDepthHold.cpp
├── StateOpenLoop.cpp
└── StatePidHold.cpp

test/
├── test_multirate_decimator.cpp  # 纯逻辑 gtest，不依赖 rclcpp/hardware_interface
├── test_thruster_allocator.cpp   # 纯 Eigen gtest：B*B^+ ~= I_6，零寄生力矩
└── test_fsm_states.cpp           # 闭环 gtest：假 CtrlInterfaces + 质点平面"虚拟物理"验证 3 态收敛性
```

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

`frequency_` 默认为 `500`（物理主时钟频率），`AdhereKamrController::on_activate()`
会将其改写为实际内环频率 `cm_update_rate_ / inner_decimation_`（默认
`500 / 2 = 250`），供任何想读取"自己当前被调用频率"而非假设硬编码常量的
`FSMState` 使用。

`AdhereMujocoHardware::export_state_interfaces()`/`export_command_interfaces()`
导出的 `double*` 裸指针，经 `controller_manager` 装配后，以
`LoanedStateInterface`/`LoanedCommandInterface` 的形式注入
`AdhereKamrController::on_activate()`，并按固定顺序存入本结构体。读写即直接解引用
（`< 0.1 us`，零 DDS 序列化）。

`ControlInputs` 对应 CLAUDE.md Phase 3.2 的 4 话题解耦契约，持有 3 组正交字段
（均为 `std::atomic<T>`，由 `AdhereKamrController` 的订阅回调——ROS executor
线程——写入，由当前活跃 `FSMState::run()`——500Hz 物理线程——无锁读取）：

- **`command`**：`/rov/fsm_command` 的最新状态切换请求（`0`=`NO_CHANGE`
  .. `4`=`PID_HOLD`）。
- **`setpoint_*` + `setpoint_seq`**：`/rov/setpoint_cmd` 的最新离散设定点
  （`mode`/`lock_current_pose`/`target_x/y/z/yaw`）。`setpoint_seq` 每收到一条
  新消息自增一次，供 `StateDepthHold`/`StatePidHold` 做"沿边检测"（只在
  `setpoint_seq` 变化时应用一次，而非每个 tick 都重复应用，否则会与 Xbox
  连续摇杆微调互相打架）。
- **`teleop_*`**：`/rov/teleop_cmd` 的最新连续遥操作指令（`surge_x/sway_y/
  heave_z/yaw_rate/active`），以及 `teleop_last_rx_time_sec`（供
  `AdhereKamrController` 的 `0.25s` 失联看门狗判断陈旧性，陈旧时集中把这几个
  字段强制清零，各 `FSMState` 自身无需关心断联处理）。

## `FSMState` 抽象基类

```cpp
class FSMState {
public:
  virtual void enter() = 0;
  virtual void run(const rclcpp::Time & time, const rclcpp::Duration & period) = 0;
  virtual void exit() = 0;

  // 默认实现：把 control_inputs_.command 映射到对应 FSMStateName（无变化请求时返回
  // 自身），4 个具体状态的 checkChange() 均只需 `return checkFsmCommandChange();`
  FSMStateName checkFsmCommandChange() const;
  virtual FSMStateName checkChange() { return checkFsmCommandChange(); }

  // /rov/fsm_state 的 target_pose 字段来源：默认全零，StateDepthHold 覆写第 3 项
  // (Z)，StatePidHold 覆写全部 4 项。
  virtual std::array<double, 4> targetPose() const { return {0.0, 0.0, 0.0, 0.0}; }
protected:
  CtrlInterfaces & ctrl_interfaces_;
};
```

> `run(time, period)` 的参数在 Foxy 下由 `AdhereKamrController::update()` 内部
> 传入——`time` 取自 `get_node()->now()`，`period` 则是 `MultiRateDecimator`
> 根据 `inner_decimation_ / cm_update_rate_` 算出的**恒定**时长（默认
> `2/500 = 0.004 s`），而非两次调用时刻的实测差值。这样一来 `period` 对任何
> 离散积分器/微分器/低通滤波器（例如 `StateDepthHold` 的 PID 积分项）而言永远
> 是精确值，不受调度抖动影响（CLAUDE.md Phase 3.1.1，见下方"均匀整数抽取
> 多速率调度"小节）。`FSMState` 本身不直接依赖
> `controller_interface::ControllerInterface::update()` 的 Foxy 无参签名，
> 保持可独立测试性。

## 当前状态（4 态，CLAUDE.md Phase 3.2）

`checkChange()` 在全部 4 个状态中均为 `checkFsmCommandChange()` 的直接转发，
因此状态间可任意两两切换（由 `/rov/fsm_command` 决定），下表只列各状态自身的
职责：

| 状态 | ID | 职责 |
|---|---|---|
| `PASSIVE` (`state_passive`) | 1 | `enter()`/`run()` 均将 8 个推力器指令置零，安全默认态 |
| `DEPTH_HOLD` (`state_depth_hold`) | 2 | 世界坐标系 Z 向 PID 定深（测量值微分 + 抗饱和积分）叠加滚转/俯仰/偏航姿态阻尼；`target_z_` 同时接受 `/rov/setpoint_cmd` 离散设定（沿边应用）与 Xbox 左摇杆上下连续微调（`target_z_ += 0.30*heave_z*dt`，摇杆回中后原地锁定） |
| `OPEN_LOOP` (`state_open_loop`) | 3 | 无记忆地把 `/rov/teleop_cmd` 的 4 维遥操作输入线性映射为机体力矩（`Fx=-60*surge_x, Fy=-50*sway_y, Fz=50*heave_z, Tz=8*yaw_rate`），用于验证 `B`/`B^+` 分配零寄生耦合 |
| `PID_HOLD` (`state_pid_hold`) | 4 | `enter()` 锁存当前 `(x,y,z,yaw)` 为保持点；摇杆推动期间按机体对齐速度平移/旋转该设定点，松手瞬间冻结；6-DOF PID + 浮力/科氏力/阻力前馈 |

所有状态的推力分配均经由 `ThrusterAllocator.h` 的同一个 `B`/`B^+`（见下），
`enter()` 中各自清零自身积分器；通过 `/rov/setpoint_cmd`/`/rov/teleop_cmd`
更新设定点（不经过状态切换）不会触发 `enter()`，因此不会重置积分器。

### 共享推力分配矩阵 `B`（`common/ThrusterAllocator.h`）

依据 `rov_wall.xml` 的实际几何（推力器/贴垫角点 `(y,z) = (+-0.25, +-0.15) m`，
法向推力器 `gear="-1 0 0 0 0 0"`，切向推力器沿各自 `zaxis` 方向、与推力器
站点坐标 `X=0.0` 恰好等于质心 `X` 坐标，无额外杠杆臂）逐项推导的精确
`6x8` 刚体力/力矩映射（行序 `[Fx,Fy,Fz,Tx,Ty,Tz]`，列序
`thr_N1..N4, thr_T5..T8`），而非套用 `CLAUDE.md` §4.3 中为完整
KAMR-MPPI 分配矩阵给出的示意性占位几何常数（二者描述的推力器安装偏移不同）。
`buildThrusterAllocationPseudoInverse()` 计算阻尼 Moore-Penrose 伪逆
`B^+ = B^T(BB^T + 1e-6*I_6)^-1`；因 `rank(B)=6`，`1e-6` 阻尼项对结果的影响
在数值精度范围内可忽略，仅起条件数安全边际作用（`test_thruster_allocator.cpp`
验证 `B*B^+` 在 `1e-4` 内等于 `I_6`）。`StateDepthHold`/`StateOpenLoop`/
`StatePidHold` 均调用这同一对函数，不再各自维护独立的几何推导副本。

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

`AdhereKamrController::update()` 每次被 `controller_manager`（500Hz）调用时都会先
`tick()`，取代了此前为凑出精确 `200Hz` 而在 `3` 拍（`6ms`）/`2` 拍（`4ms`）之间
交替的 `[3,3,3,3,3,2,2,2,2,2]` 子步调度表——那种交替会给离散积分器/微分器/低通
滤波器（如 Pad-Force DOB）引入 `±20%` 采样周期抖动。改用纯取模（`tick % N == 0`）
后，默认 `inner_decimation=2`、`outer_decimation=20` 使内环精确 `250Hz`
（`dt=4.0ms` 恒定）、外环精确 `25Hz`（`dt=40.0ms` 恒定，每 10 个内环步一次），
`outer_decimation` 也可配置为 `25` 得到 `20Hz` 外环。纯逻辑实现（零 `rclcpp`/
`hardware_interface` 依赖），由 `test/test_multirate_decimator.cpp` 独立 gtest 验证
`500` 次 tick 内精确触发 `250` 次内环 / `25` 次外环。

## 线程安全

本包内所有类均**不**自带线程同步原语，仅 `ControlInputs` 使用
`std::atomic<T>`；`CtrlInterfaces` 的读写发生在 `controller_manager` 的单一
物理线程（500Hz）内，天然串行。
