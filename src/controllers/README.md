# `controllers/` — 控制算法层

## 设计理念

本层对标 `quadruped_ros2_control` 中 `libraries/controller_common`（基础设施库）
与 `controllers/unitree_guide_controller`（具体算法插件）的两层分离范式。注意
按照本工作空间 `CLAUDE.md` 第 4 节 "工作空间目录与软件包架构" 的明确约定：
`adhere_controller_common` 是**自研基础设施**（非第三方依赖库），因此直接置于
`src/controllers/` 之下，与 `adhere_kamr_controller` 并列，**不**单独建立
`libraries/` 目录。

## 两个软件包的职责划分

| | `adhere_controller_common` | `adhere_kamr_controller` |
|---|---|---|
| **定位** | 可复用的控制基础设施库（纯 C++ 静态/共享库，不是 `ros2_control` 插件） | `controller_interface::ControllerInterface` `pluginlib` 插件（算法主体） |
| **包含内容** | `CtrlInterfaces`（零拷贝 `LoanedStateInterface`/`LoanedCommandInterface` 包装）、`FSMState` 抽象基类与通用状态（`Passive`、`PreloadAttach`、`EmergencyDetach`）、`mathTypes`/`mathTools`、`FossenHydrodynamics` | FSM 管理器（注册 `StationKeeping`、`TrajectoryTracking` 等算法特化状态）、250Hz 内环（Planar Impedance PD + Pad-Force DOB + 12D Elastic Rhombic-Kern OSQP）、25Hz 异步 CUDA MPPI 外环滚动线程 |
| **复用边界** | 未来若新增其他算法插件（例如对比基准 `Baseline MPPI` 控制器），应复用本包的 `CtrlInterfaces`/`FSMState`/水动力模型，而不是复制粘贴 | 仅服务于 `KAMR-MPPI`（`pr_wbc_mppi_v4`）这一具体算法契约；文件名/类名与 `CLAUDE.md` 第 5 节数学契约逐条对应 |
| **是否依赖 `ros2_control` 插件机制** | 否（无 `pluginlib` 导出，供其他包 `target_link_libraries`） | 是（导出 `controller_interface.xml`，由 `controller_manager` 通过 `pluginlib` 动态加载） |
| **是否被 `hardwares/adhere_mujoco_hardware` 依赖** | **是**——`AdhereMujocoHardware` 复用本包的 `FossenHydrodynamics` 计算 6-DOF 水动力+阵风，保证硬件接口与控制器对水动力模型的理解完全一致（避免 Sim-to-Real 验证时出现"同一套力学公式两份实现"的维护灾难） | 否 |

这种划分的核心收益：`FossenHydrodynamics` 作为"物理世界的真实规律"只有一份实现，
同时被 `AdhereMujocoHardware`（在 `write()` 中计算施加到仿真体的外力）和
`AdhereKamrController`（在 250Hz 内环的 6-DOF 弧矢量重构与 25Hz 外环 rollout 中
做前馈/预测）共享，从架构上杜绝数值不一致。

## FSM 分层设计

`FSMState` 基类及通用状态（`PASSIVE`、`PRELOAD_ATTACH`、`EMERGENCY_DETACH`）定义在
`adhere_controller_common`，因为它们是与具体控制算法无关的"安全兜底状态"；而算法
特化状态（`STATION_KEEPING`、`TRAJECTORY_TRACKING`，内部调用 250Hz 内环与 25Hz
外环）定义在 `adhere_kamr_controller/include/adhere_kamr_controller/fsm_states/`。

## 当前软件包

| 软件包 | 职责 |
|---|---|
| [`adhere_controller_common`](adhere_controller_common/README.md) | `CtrlInterfaces`、`FSMState` 基类与通用状态、数学工具、`FossenHydrodynamics` |
| [`adhere_kamr_controller`](adhere_kamr_controller/README.md) | KAMR-MPPI 算法主体：FSM 管理 + 250Hz 内环 + 25Hz 异步 CUDA 外环 |

## Phase 1 状态

两个包均已搭建 `package.xml`/`CMakeLists.txt`/头文件子目录骨架（`FSM/ common/
hydrodynamics/` 与 `fsm_states/ inner_wbc/ outer_mppi_cuda/`），源码留待 Phase 3/4。
