# `controllers/` — 控制算法层

## 设计理念

本层目前只有一个软件包：`adhere_controller`。Phase 3.1/3.2 曾按
`quadruped_ros2_control` 的"基础设施库 + 算法插件"两层范式拆分为
`adhere_controller_common`（基础设施）与 `adhere_kamr_controller`（算法主体，
`pluginlib` 插件）两个包，但二者始终一起编译、一起部署，从未独立使用过，纯粹的
包拆分只带来跨包链接与维护开销。Phase 3.3 按 `CLAUDE.md` 第 3 节的决定将二者
合并为单一包 `adhere_controller`。

## `adhere_controller`

单一 `controller_interface::ControllerInterface` `pluginlib` 插件
（`adhere_controller/AdhereController`），同时承载：

- `CtrlInterfaces`（零拷贝 `LoanedStateInterface`/`LoanedCommandInterface` 包装）；
- `ModelParams`（运行时从 `simplified_v1_description/config/robot_control.yaml`
  加载的模型解耦物理/增益参数）与动态 `ThrusterAllocator`（运行时从
  `ModelParams` 的推力器几何构建 `6x8` 分配矩阵 `B` 及其阻尼伪逆 `B^+`）；
- `FSMState` 抽象基类与全部具体状态（`IDLE_PASSIVE_NONE`/`HOLD_PID_DEPTH`/
  `TELEOP_OPENLOOP_6DOF`/`HOLD_PID_6DOF`，未来 `ADHERE_MPPI_KAMR` 状态同样加入这里）；
- `MultiRateDecimator`（均匀整数抽取 `500Hz -> 250Hz/25Hz`）；
- `mathTypes`/`mathTools` 通用数学工具。

详见 [`adhere_controller/README.md`](adhere_controller/README.md)。

## 模型解耦原则（CLAUDE.md 第 3 节）

`adhere_controller` 的 C++ 代码中不包含任何机器人专属几何或物理常数（推力器
坐标/方向、贴垫坐标、质量/浮力/CoB、Fossen 阻力系数、各状态 PID 增益）——全部
由 `ModelParams::loadFromNode()` 在 `on_configure()` 时从
`<pkg_description>/config/robot_control.yaml` 读取。切换到几何/质量不同的新
机型只需改这一份 YAML，`adhere_controller` 零 C++ 改动、零重新编译。
