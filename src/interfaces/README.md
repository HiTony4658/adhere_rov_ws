# `interfaces/` — 消息/服务接口层

## 设计理念

本层对标 `quadruped_ros2_control` 中 `commands/control_input_msgs` 的定位：所有跨包、
跨节点传递的 ROS 2 消息/服务类型**只允许在本层定义**，严禁在 `hardwares/`、
`controllers/`、`commands/` 任意包内私自定义跨包共享的消息类型。这样可以保证：

1. **单一事实来源**：`AdhereMujocoHardware`、`AdhereKamrController`、
   `adhere_keyboard_input` 三者若需要通过标准 ROS 2 话题/服务（而非进程内指针）交换
   数据（例如遥测、FSM 切换请求），统一依赖本层生成的类型，避免类型定义漂移。
2. **编译依赖方向清晰**：`interfaces` 不依赖任何其他本工作空间内的包，是依赖图的叶子
   节点，所有其他 5 个包都可以单向依赖它。
3. **与进程内指针通道解耦**：请注意，`AdhereMujocoHardware` ↔ `AdhereKamrController`
   之间的**主数据通路**（`hw_states_[36]`、`hw_commands_[8]`）是通过 `ros2_control`
   的 `CtrlInterfaces`（`LoanedStateInterface`/`LoanedCommandInterface` 原始指针）
   实现的零拷贝通道，**不经过**本层消息类型；本层消息类型仅用于标量频率较低的
   **监控/遥测/人机交互**场景（FSM 状态切换请求、目标轨迹下发、500Hz/20Hz 诊断量
   的 RViz/rqt 可视化）。

## 当前软件包

| 软件包 | 职责 |
|---|---|
| [`adhere_control_msgs`](adhere_control_msgs/README.md) | FSM 命令、目标位姿/轨迹、KAMR-MPPI 遥测（菱形核余量、掀离统计、Pad-Force DOB 估计）消息与服务定义 |

## Phase 1 状态

仅搭建 `package.xml`/`CMakeLists.txt`/`msg`/`srv` 空目录骨架，尚未定义具体
`.msg`/`.srv` 文件（见包内 README 的"规划中的接口列表"）。
