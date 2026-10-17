# `adhere_control_msgs`

## 职责

为推力贴附式船体巡检 ROV 的 KAMR-MPPI 控制栈提供跨节点共享的消息/服务类型：
FSM 状态切换命令、状态内数值设定点、Xbox 手柄连续遥操作指令、控制器实时反馈，
以及未来的目标轨迹/500Hz/20Hz 遥测。本包不依赖本工作空间内任何其他包，是
依赖图的叶子节点，可与 `adhere_keyboard_input` 一并部署到不安装 MuJoCo/CUDA/
`ros2_control` 的远程上位机（GCS）笔记本上。

> 这些消息类型**不**用于 `AdhereMujocoHardware` ↔ `AdhereController` 的主数据
> 通路（该通路走 `ros2_control` 的 `CtrlInterfaces` 零拷贝指针），仅用于低频
> 监控、人机交互、日志记录场景。

## 已实现的接口

### 消息（`msg/`）：4 话题解耦契约（CLAUDE.md Phase 3.2）

状态切换、状态内离散设定点、连续手柄遥操作、控制器反馈是 4 个正交关注点，
分别对应 4 个独立话题，互不混用：

| 文件 | 话题 / 发布方 | 字段摘要 | 用途 |
|---|---|---|---|
| `FsmCommand.msg` | `/rov/fsm_command`，`fsm_console_node` 或 `xbox_teleop_node` 按键边沿 | `int32 command`（`NO_CHANGE=0, PASSIVE=1, DEPTH_HOLD=2, OPEN_LOOP=3, PID_HOLD=4`） | **仅**用于切换 FSM 状态，不再携带任何数值设定点 |
| `SetpointCommand.msg` | `/rov/setpoint_cmd`，`state_cmd_console_node` | `uint8 mode`（`2`=`DEPTH_HOLD` 用 `target_z`；`4`=`PID_HOLD` 用 `target_x/y/z/yaw` 或 `lock_current_pose`）; `bool lock_current_pose`; `float64 target_x/y/z/yaw` | 离散数值设定点，与状态切换正交，不触发 `FSMState::enter()`（不重置积分器） |
| `TeleopCommand.msg` | `/rov/teleop_cmd`，`xbox_teleop_node`，50Hz | `float64 surge_x, sway_y, heave_z, yaw_rate`（归一化至 `[-1,1]`，美国手/Mode 2 映射）; `bool active` | Xbox 手柄连续遥操作指令，被 `StateOpenLoop`/`StatePidHold`/`StateDepthHold` 消费；`AdhereController` 对其施加 `0.25s` 失联看门狗 |
| `FsmState.msg` | `/rov/fsm_state`，`AdhereController`，外环速率 | `int32 current_state`; `string state_name`; `float64[4] target_pose`; `float64[4] actual_pose` | 控制器实时反馈，供 `fsm_console_node`/`state_cmd_console_node` 显示；是终端感知"当前到底是哪个状态"的唯一信源，无论状态切换来自键盘还是手柄按键 |

## 规划中的接口列表（后续阶段实现）

| 文件 | 字段摘要 | 用途 |
|---|---|---|
| `TargetTrajectory.msg` | `geometry_msgs/PoseStamped[] waypoints`, `float64[] timestamps` | 轨迹跟踪参考轨迹 |
| `RhombicKernMetrics.msg` | `float64 k_val, m_kern, p_y, p_z, f_press` | 500Hz 菱形核可行域余量遥测（对应 `CLAUDE.md` §4.4） |
| `PadContactState.msg` | `float64[4] f_normal, float64[4] v_sep, bool[4] liftoff` | 4 个接触垫法向力/分离速度/掀离标志（对应 36 维状态接口中的 `pad_contact` 段） |
| `KamrMppiTelemetry.msg` | `float64 outer_solve_latency_ms, float64 f_x_extra_preload, float64[4] d_hat_unload, float64[4] w_cmd` | 20Hz 外环求解延迟、Pad-Force DOB 卸载估计、MPPI 名义力矩 |
| `SwitchFsmState.srv` | `req: uint8 target_state` / `res: bool accepted, string reason` | 同步 FSM 状态切换请求（允许拒绝非法迁移） |

## 构建说明

`CMakeLists.txt` 的 `rosidl_generate_interfaces(${PROJECT_NAME} "msg/FsmCommand.msg"
"msg/SetpointCommand.msg" "msg/TeleopCommand.msg" "msg/FsmState.msg" DEPENDENCIES std_msgs)`
生成对应的 `adhere_control_msgs/msg/*.hpp`。其余 `.msg`/`.srv` 文件待后续阶段按上表
实现后追加到该调用的文件列表中。

## ROS 2 Foxy 注意事项

`rosidl_default_generators`/`rosidl_default_runtime` 在 Foxy 与后续版本 API 一致，
无需特殊适配；但生成的 C++ 消息头文件路径格式
（`adhere_control_msgs/msg/fsm_command.hpp`）需在依赖包 `CMakeLists.txt` 中正确
`find_package(adhere_control_msgs REQUIRED)` + `ament_target_dependencies`。
