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

### 消息（`msg/`）：5 话题解耦契约（CLAUDE.md Phase 3.2，四元数化 + 实时诊断于 Phase 3.6）

状态切换、状态内离散设定点、连续手柄遥操作、控制器反馈、实时诊断是 5 个
正交关注点，分别对应 5 个独立话题，互不混用：

| 文件 | 话题 / 发布方 | 字段摘要 | 用途 |
|---|---|---|---|
| `FsmCommand.msg` | `/rov/fsm_command`，`fsm_console_node` 或 `xbox_teleop_node` 按键边沿 | `int32 command`（`NO_CHANGE=0, PASSIVE=1, DEPTH_HOLD=2, OPEN_LOOP=3, PID_HOLD=4, ADHERE_PID=5, ADHERE_SMC=6, BASELINE_MPPI=7, KAMR_MPPI=8`） | **仅**用于切换 FSM 状态，不再携带任何数值设定点；`5`/`6` 为 Phase 3.8 新增的贴壁 PID/SMC 基线，`7`/`8` 为 Phase 4 预留状态 ID（由 Phase 3.6 的 `5`/`6` 顺延而来） |
| `SetpointCommand.msg` | `/rov/setpoint_cmd`，`state_cmd_console_node` | `uint8 command_type`（`1=MODE_DEPTH_ONLY, 2=MODE_FULL_POSE, 3=MODE_LOCK_CURRENT, 4=MODE_LEVEL_ATTITUDE`）; `geometry_msgs/Pose target_pose`（世界系位置 + 单位四元数） | 离散数值设定点，与状态切换正交，不触发 `FSMState::enter()`（不重置积分器）；Phase 3.6 由 6 个裸标量（`mode/target_x..yaw`）重构为 `command_type` + `Pose`，姿态改用四元数避免万向锁 |
| `TeleopCommand.msg` | `/rov/teleop_cmd`，`xbox_teleop_node`，50Hz | `geometry_msgs/Vector3 linear`（surge/sway/heave）; `geometry_msgs/Vector3 angular`（roll_rate=RT/RB, pitch_rate=LT/LB, yaw_rate），均归一化至 `[-1,1]`; `bool active` | Xbox 手柄连续遥操作指令，被 `StateOpenLoop`/`StatePidHold`/`StateDepthHold`/`StateAdherePid`/`StateAdhereSmc` 消费；`AdhereController` 对其施加 `0.25s` 失联看门狗；Phase 3.6 由 6 个裸标量重构为 2 个 `Vector3` |
| `ControllerState.msg` | `/rov/controller_state`，`AdhereController`，外环速率（替代 Phase 3.6 删除的 `FsmState.msg`） | `int32 current_state`; `string state_name`; `string estimation_mode`; `geometry_msgs/Pose target_pose`; `geometry_msgs/Pose estimated_pose`; `geometry_msgs/Twist estimated_twist_body`; `float64[4] contact_forces`; `float64[8] thruster_cmds`; `float64 thrust_l2_norm`; `float64 thrust_power_proxy`; `float64 thrust_slew_rate_l2`; `float64 cumulative_energy_l2`; `float64 cumulative_impulse_l1`; `uint8 saturated_thruster_count`; `uint8 active_contact_pad_count`; `float64 kern_margin` | 控制器实时反馈，供 `fsm_console_node`/`state_cmd_console_node`/`rt_monitor_node`/`benchmark_evaluator_node` 消费；**100% Sim-to-Real 纯净**——不含任何地面真值字段（地面真值改由 `adhere_mujoco_node` 单独发布在 `/rov/ground_truth_odom`），姿态全部用四元数表示；Phase 3.8 新增 7 个效率/啁啾/安全遥测字段 |
| `RuntimeDiagnostics.msg` | `/rov/runtime_diagnostics`，`AdhereController`，固定 10Hz（Phase 3.6 新增） | 9 路频率（Hz）+ 执行延迟/预算（ms）+ 超时计数 + `bool timing_healthy` | 实时频率/延迟可观测性，驱动 `/rov/diagnostics_hud_marker` RViz2 HUD 与 `rt_monitor_node` 实时终端表格 |

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
"msg/SetpointCommand.msg" "msg/TeleopCommand.msg" "msg/ControllerState.msg"
"msg/RuntimeDiagnostics.msg" DEPENDENCIES std_msgs geometry_msgs)`
生成对应的 `adhere_control_msgs/msg/*.hpp`。其余 `.msg`/`.srv` 文件待后续阶段按上表
实现后追加到该调用的文件列表中。

## ROS 2 Foxy 注意事项

`rosidl_default_generators`/`rosidl_default_runtime` 在 Foxy 与后续版本 API 一致，
无需特殊适配；但生成的 C++ 消息头文件路径格式
（`adhere_control_msgs/msg/fsm_command.hpp`）需在依赖包 `CMakeLists.txt` 中正确
`find_package(adhere_control_msgs REQUIRED)` + `ament_target_dependencies`。
