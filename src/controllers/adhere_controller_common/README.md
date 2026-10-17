# `adhere_controller_common`

## 职责

自研控制基础设施库（纯 C++ 共享库，非 `ros2_control` 插件），被
`adhere_kamr_controller` 依赖。提供零拷贝的硬件接口包装（`CtrlInterfaces`）、
有限状态机（FSM）抽象基类与具体状态实现、以及通用数学工具。未来的
`FossenHydrodynamics`（当前位于 `adhere_mujoco_hardware`）迁移、
`PreloadAttach`/`EmergencyDetach` 等 KAMR-MPPI 专用状态将在后续阶段加入本包。

## 目录结构

```text
include/adhere_controller_common/
├── CtrlInterfaces.h             # 零拷贝状态/命令包装 + ControlInputs
├── FSM/
│   ├── FSMState.h               # 抽象基类
│   ├── StatePassive.h
│   └── StateDepthHold.h
└── common/
    ├── enumClass.h              # FSMStateName / FSMMode
    ├── mathTypes.h              # Eigen 类型别名（Vec3/Vec6/Vec8/Mat68/Mat86/RotMat）
    └── mathTools.h              # quatToRotMat / rotMatToRPY / saturation

src/FSM/
├── StatePassive.cpp
└── StateDepthHold.cpp
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

`AdhereMujocoHardware::export_state_interfaces()`/`export_command_interfaces()`
导出的 `double*` 裸指针，经 `controller_manager` 装配后，以
`LoanedStateInterface`/`LoanedCommandInterface` 的形式注入
`AdhereKamrController::on_activate()`，并按固定顺序存入本结构体。读写即直接解引用
（`< 0.1 us`，零 DDS 序列化）。

`ControlInputs` 持有 `/rov/fsm_command` 的最新目标（`command`/`target_z`/
`target_y`/`target_yaw`），均为 `std::atomic<T>`：由
`AdhereKamrController` 的订阅回调（ROS executor 线程）写入，由当前活跃
`FSMState::run()`（500Hz 物理线程）无锁读取。

## `FSMState` 抽象基类

```cpp
class FSMState {
public:
  virtual void enter() = 0;
  virtual void run(const rclcpp::Time & time, const rclcpp::Duration & period) = 0;
  virtual void exit() = 0;
  virtual FSMStateName checkChange() { return FSMStateName::INVALID; }
protected:
  CtrlInterfaces & ctrl_interfaces_;
};
```

> `run(time, period)` 的参数在 Foxy 下由 `AdhereKamrController::update()`
> 内部通过 `get_node()->now()` 与上次调用时刻的差值计算得出并传入——`FSMState`
> 本身不直接依赖 `controller_interface::ControllerInterface::update()` 的
> Foxy 无参签名，保持可独立测试性。

## 当前状态

| 状态 | 职责 |
|---|---|
| `PASSIVE` (`state_passive`) | `enter()`/`run()` 均将 8 个推力器指令置零，安全默认态；`checkChange()` 在 `control_inputs_.command == 2` 时迁移至 `DEPTH_HOLD` |
| `DEPTH_HOLD` (`state_depth_hold`) | 世界坐标系 Z 向 PID 定深（测量值微分 + 抗饱和积分）叠加滚转/俯仰/偏航姿态阻尼，经 `rov_wall.xml` 精确几何推导的 6x8 推力分配矩阵 `B` 的 Moore-Penrose 伪逆映射到 8 个推力器；`checkChange()` 在 `control_inputs_.command == 1` 时迁移至 `PASSIVE` |

`integral_z_`（定深积分项）仅在 `enter()`（即 `PASSIVE -> DEPTH_HOLD` 迁移发生时）
清零；用户在 `DEPTH_HOLD` 状态下通过终端控制台更新 `target_z` 不会触发
`enter()`，因此不会重置积分器。

### 推力分配矩阵 `B`（`StateDepthHold.cpp`）

依据 `rov_wall.xml` 的实际几何（推力器/贴垫角点 `(y,z) = (+-0.25, +-0.15) m`，
法向推力器 `gear="-1 0 0 0 0 0"`，切向推力器沿各自 `zaxis` 方向、与推力器
站点坐标 `X=0.0` 恰好等于质心 `X` 坐标，无额外杠杆臂）逐项推导的精确
`6x8` 刚体力/力矩映射（行序 `[Fx,Fy,Fz,Tx,Ty,Tz]`，列序
`thr_N1..N4, thr_T5..T8`），而非套用 `CLAUDE.md` §4.3 中为完整
KAMR-MPPI 分配矩阵给出的示意性占位几何常数（二者描述的推力器安装偏移不同）。

## 线程安全

本包内所有类均**不**自带线程同步原语，仅 `ControlInputs` 使用
`std::atomic<T>`；`CtrlInterfaces` 的读写发生在 `controller_manager` 的单一
物理线程（500Hz）内，天然串行。
