# `adhere_controller_common`

## 职责

自研控制基础设施库（纯 C++ 共享库，非 `ros2_control` 插件），被
`adhere_mujoco_hardware`（复用 `FossenHydrodynamics`）与
`adhere_kamr_controller`（复用全部内容）共同依赖。参见
[`src/controllers/README.md`](../README.md) 中与 `adhere_kamr_controller` 的
详细职责划分表。

## 类与文件规划

```text
include/adhere_controller_common/
├── CtrlInterfaces.hpp
├── FSM/
│   ├── FSMState.hpp             # 抽象基类
│   ├── StatePassive.hpp
│   ├── StatePreloadAttach.hpp
│   └── StateEmergencyDetach.hpp
├── common/
│   ├── mathTypes.hpp            # Eigen 类型别名（Vec6, Mat8x6 等）
│   └── mathTools.hpp            # 通用数学工具（四元数转换、饱和函数、softplus 等）
└── hydrodynamics/
    └── FossenHydrodynamics.hpp

src/
├── CtrlInterfaces.cpp
├── FSM/
│   ├── FSMState.cpp
│   ├── StatePassive.cpp
│   ├── StatePreloadAttach.cpp
│   └── StateEmergencyDetach.cpp
└── hydrodynamics/
    └── FossenHydrodynamics.cpp
```

## `CtrlInterfaces`：零拷贝状态/命令包装

```cpp
struct CtrlInterfaces {
  std::vector<hardware_interface::LoanedStateInterface>   state_interfaces_;
  std::vector<hardware_interface::LoanedCommandInterface>  command_interfaces_;
  // 按名称索引的便捷访问器，例如 get_pad_force(PadId::TL) -> double&
};
```

`AdhereMujocoHardware::export_state_interfaces()`/`export_command_interfaces()`
导出的 `double*` 裸指针，经 `controller_manager` 装配后，以
`LoanedStateInterface`/`LoanedCommandInterface` 的形式注入
`AdhereKamrController::on_configure()`，并存入本结构体。读写即直接解引用，
`< 0.1 us`，零 DDS 序列化。

## `FSMState` 抽象基类

```cpp
class FSMState {
public:
  virtual void enter() = 0;
  virtual void run(const rclcpp::Time & time, const rclcpp::Duration & period) = 0;
  virtual void exit() = 0;
  virtual FSMStateId checkTransition() = 0;
protected:
  CtrlInterfaces & interfaces_;
};
```

> `run(time, period)` 的参数在 Foxy 下由 `AdhereKamrController::update()`
> 内部通过 `get_node()->now()` 与上次调用时间差计算得出并传入——`FSMState` 本身
> 不直接依赖 `controller_interface::ControllerInterface::update()` 的 Foxy
> 无参签名，保持可测试性。

## 通用状态语义

| 状态 | 职责 |
|---|---|
| `PASSIVE` | 所有推力器零输出，安全默认态 |
| `PRELOAD_ATTACH` | 逐步提升 4 个法向推力器至预紧力，确认 4 垫均进入压缩（`F_N,i > 0`）后方可迁移至算法特化状态 |
| `EMERGENCY_DETACH` | 检测到持续掀离/菱形核严重违约时，主动撤离壁面（反向推力），防止碰撞损坏 |

## `FossenHydrodynamics`

实现 6-DOF 刚体水动力模型：附加质量项、Morison 二次阻力、近壁 Venturi 吸附效应、
`58.86 N` 重力/浮力偏置，以及标准 12.0s 阵风扰动 profile 的查询接口
（`steady V_y=0.3m/s` + 两段半正弦阵风）。**必须**与
`mujoco_mppi_ws/src/env/hydrodynamics.py` 的系数逐项核对对齐（只读参考）。

## 线程安全

本包内所有类均**不**自带线程同步原语；线程安全由调用方保证：
`CtrlInterfaces` 的读写发生在 `controller_manager` 的单一实时线程内，天然串行；
`FossenHydrodynamics::compute()` 为纯函数风格（无内部可变状态跨调用共享），
可被 500Hz 硬件线程与 20Hz MPPI 异步线程安全地各自持有独立实例调用。

## Phase 1 状态

仅搭建 `include/adhere_controller_common/{FSM,common,hydrodynamics}/` 与 `src/`
目录骨架，`CMakeLists.txt` 构建目标以 `# TODO(Phase 2)` 注释占位。
