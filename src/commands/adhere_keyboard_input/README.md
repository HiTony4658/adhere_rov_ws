# `adhere_keyboard_input`

## 职责

键盘遥操作与基准实验触发节点。通过 `adhere_control_msgs` 定义的消息/服务，
以标准 ROS 2 话题/服务（非实时路径）与 `AdhereKamrController` 异步通信：

- 发送 FSM 状态切换请求（`SwitchFsmState.srv`）；
- 发送站位保持目标位姿 / 轨迹跟踪参考轨迹（`TargetPose.msg` / `TargetTrajectory.msg`）；
- 一键触发标准 12.0s 阵风扰动 profile（用于复现论文基准实验）。

## 类与文件规划

```text
include/adhere_keyboard_input/
└── keyboard_input_node.hpp     # KeyboardInputNode 类声明（rclcpp::Node 派生）

src/
└── keyboard_input_node.cpp     # 终端原始模式读键 + 发布/调用服务
```

## 规划中的键位映射（Phase 2 起实现，暂定）

| 按键 | 功能 |
|---|---|
| `1` | 切换至 `PRELOAD_ATTACH` |
| `2` | 切换至 `STATION_KEEPING` |
| `3` | 切换至 `TRAJECTORY_TRACKING` |
| `0` | 切换至 `EMERGENCY_DETACH`（紧急撤离） |
| `W/A/S/D` | 微调站位保持目标 `(Y, Z)` |
| `Q/E` | 微调目标偏航角 `psi` |
| `G` | 触发标准 12.0s 阵风扰动 benchmark profile |
| `Space` | 立即切换至 `PASSIVE`（安全停车） |

## ROS 2 接口（规划）

| 类型 | 名称 | 方向 |
|---|---|---|
| Service Client | `/adhere_kamr_controller/switch_fsm_state` | 调用 `SwitchFsmState.srv` |
| Publisher | `/adhere_kamr_controller/target_pose` | 发布 `TargetPose.msg` |
| Publisher | `/adhere_kamr_controller/target_trajectory` | 发布 `TargetTrajectory.msg` |
| Service Client | `/adhere_mujoco_hardware/trigger_benchmark_gust` | 触发标准扰动 profile（Phase 2 起由硬件接口侧提供） |

## Phase 1 状态

仅搭建 `package.xml`/`CMakeLists.txt`/`include`/`src` 骨架，节点逻辑与终端原始
输入模式处理留待后续阶段实现。
