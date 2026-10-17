# `commands/` — 遥操作与基准触发层

## 设计理念

本层对标 `quadruped_ros2_control/commands`（`keyboard_input`、`joystick_input`）的
定位：提供**人机交互与实验基准触发**节点，它们不参与 500Hz/250Hz/25Hz 实时控制回路，
只通过标准 ROS 2 话题/服务（定义于 `interfaces/adhere_control_msgs`）与
`AdhereKamrController` 异步通信——这类低频、非实时的交互路径天然适合走 DDS，
无需追求 `CtrlInterfaces` 级别的零拷贝性能。

## 核心职责

- **FSM 状态切换**：发送 `PASSIVE → PRELOAD_ATTACH → STATION_KEEPING →
  TRAJECTORY_TRACKING → EMERGENCY_DETACH` 状态切换请求。
- **目标下发**：站位保持目标位姿 `(Y, Z, psi)`、轨迹跟踪的参考轨迹。
- **标准基准工况触发**：一键注入 `CLAUDE.md` 第 4 节定义的 12.0s 标准扰动
  profile（定常侧向流 + 两段半正弦阵风），用于复现论文实验数据。

## 当前软件包

| 软件包 | 职责 |
|---|---|
| [`adhere_keyboard_input`](adhere_keyboard_input/README.md) | 键盘遥操作与基准触发节点 |

## Phase 1 状态

仅搭建 `package.xml`/`CMakeLists.txt`/`include`/`src` 骨架，具体键位映射与节点逻辑
留待后续阶段实现。
