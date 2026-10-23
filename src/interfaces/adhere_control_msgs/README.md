# `adhere_control_msgs`

## 职责

为推力贴附式船体巡检 ROV 的 KAMR-MPPI 控制栈提供跨节点共享的消息/服务类型：
FSM 状态切换命令、状态内数值设定点、Xbox 手柄连续遥操作指令、控制器实时反馈，
以及未来的目标轨迹/500Hz/20Hz 遥测。本包不依赖本工作空间内任何其他包，是
依赖图的叶子节点，可与 `adhere_teleop_gcs`/`adhere_monitor` 一并部署到不安装 MuJoCo/CUDA/
`ros2_control` 的远程上位机（GCS）笔记本上。

> 这些消息类型**不**用于 `AdhereMujocoHardware` ↔ `AdhereController` 的主数据
> 通路（该通路走 `ros2_control` 的 `CtrlInterfaces` 零拷贝指针），仅用于低频
> 监控、人机交互、日志记录场景。

## 已实现的接口

### 消息（`msg/`）：8 话题解耦契约（CLAUDE.md Phase 3.2，四元数化 + 实时诊断于 Phase 3.6，贴壁目标实时重定向于 Phase 4.5，AprilTag 测量/基准遥测拆分于 Phase 5.0 Stage 6/7）

状态切换、状态内离散设定点、连续手柄遥操作、控制器反馈、实时诊断、贴壁目标
实时重定向（人工意图）、贴壁目标测量（AprilTag）、基准/效率遥测是 8 个正交
关注点，分别对应 8 个独立话题，互不混用：

| 文件 | 话题 / 发布方 | 字段摘要 | 用途 |
|---|---|---|---|
| `FsmCommand.msg` | `/rov/fsm_command`，`fsm_console_node` 或 `xbox_teleop_node` 按键边沿 | `int32 command`（`NO_CHANGE=0, IDLE_PASSIVE_NONE=1, TELEOP_OPENLOOP_6DOF=2, HOLD_PID_6DOF=3, ADHERE_PID_WBC=4, ADHERE_SMC_WBC=5, ADHERE_MPPI_BASELINE=6, ADHERE_MPPI_KAMR=7`，Phase 4.3 更名为 `FUNCTION_ALGORITHM_CATEGORY` 命名规范；Phase 5.0 删除冗余的 `HOLD_PID_DEPTH` 并将 ID 由历史的 `1-8` 压缩为 `1-7`） | **仅**用于切换 FSM 状态，不再携带任何数值设定点；`4`/`5` 为 Phase 3.8 新增的贴壁 PID/SMC 基线，`7` 为 Phase 4 预留状态 ID |
| `SetpointCommand.msg` | `/rov/setpoint_cmd`，`ros2 topic pub` 手动发布（Phase 5.0 删除了专属设定终端） | `uint8 command_type`（`1=MODE_DEPTH_ONLY, 2=MODE_FULL_POSE, 3=MODE_LOCK_CURRENT, 4=MODE_LEVEL_ATTITUDE`）; `geometry_msgs/Pose target_pose`（世界系位置 + 单位四元数） | 离散数值设定点，与状态切换正交，不触发 `FSMState::enter()`（不重置积分器）；Phase 3.6 由 6 个裸标量（`mode/target_x..yaw`）重构为 `command_type` + `Pose`，姿态改用四元数避免万向锁 |
| `TeleopCommand.msg` | `/rov/teleop_cmd`，`xbox_teleop_node`，50Hz | `geometry_msgs/Vector3 linear`（surge/sway/heave）; `geometry_msgs/Vector3 angular`（roll_rate=RT/RB, pitch_rate=LT/LB, yaw_rate），均归一化至 `[-1,1]`; `bool active` | Xbox 手柄连续遥操作指令，被 `StateOpenLoop`/`StatePidHold`/`StateAdherePid`/`StateAdhereSmc` 消费；`AdhereController` 对其施加 `0.25s` 失联看门狗；Phase 3.6 由 6 个裸标量重构为 2 个 `Vector3` |
| `ControllerState.msg` | `/rov/controller_state`，`AdhereController`，外环速率（替代 Phase 3.6 删除的 `FsmState.msg`） | `int32 current_state`; `string state_name`; `string estimation_mode`; `geometry_msgs/Pose target_pose`; `bool target_valid`（Phase 4.3，`HOLD_PID_6DOF`/`ADHERE_*` 为 `true`）; `string adhere_stage`; `geometry_msgs/Pose estimated_pose`; `geometry_msgs/Twist estimated_twist_body`; `float64[4] contact_forces`; `float64[8] thruster_cmds` | 控制器实时反馈，供 `fsm_console_node` 等消费；**100% Sim-to-Real 纯净**——不含任何地面真值字段（地面真值改由 `adhere_mujoco_node` 单独发布在 `/rov/ground_truth_odom`），姿态全部用四元数表示；Phase 5.0 Stage 7 把原 Phase 3.8 新增的 7 个效率/啁啾/安全遥测字段 + `kern_margin` 拆到 `BenchmarkMetrics.msg`，本消息保持精简 |
| `RuntimeDiagnostics.msg` | `/rov/runtime_diagnostics`，`AdhereController`，固定 10Hz（Phase 3.6 新增） | 9 路频率（Hz）+ 执行延迟/预算（ms）+ 超时计数 + `bool timing_healthy` | 实时频率/延迟可观测性，驱动 `/rov/diagnostics_hud_marker` RViz2 HUD 与 `rt_monitor_node` 实时终端表格 |
| `AdhereTargetCommand.msg` | `/rov/adhere_target_cmd`，`ros2 topic pub` 手动发布（Phase 4.5 新增，Phase 5.0 删除了专属设定终端） | `uint8 command_type`（`1=TAG_PRESET, 2=CUSTOM_Y, 3=CUSTOM_DEPTH, 4=CUSTOM_Y_DEPTH, 5=USE_CURRENT`）; `uint8 tag_id`; `float64 target_y`; `float64 target_depth_m` | **仅**用于实时改写 `ADHERE_PID_WBC`/`ADHERE_SMC_WBC`/`ADHERE_MPPI_BASELINE` 共享的贴壁目标（墙面 QR 码/AprilTag 预设或自定义 Y/深度）——**人工意图**，与 `SetpointCommand` 正交——不携带通用 6-DOF 位姿，也不触发状态切换；`AdhereController` 据此写入 `CtrlInterfaces::adhere_target_`（三个贴壁状态唯一的实时目标来源），变化超过 `adhere_retarget_threshold`（`0.06m`）时强制重新进入 `STANDOFF_HOMING` |
| `TagDetection.msg` | `/rov/tag_detection`，`perception/adhere_apriltag_detector`（Phase 5.0 Stage 6 新增） | `uint8 station_id`（检测到的 2x2 矩阵最小标签 ID / 4）; `float64 target_y`; `float64 target_depth_m`; `float64 confidence`（`[0,1]`）; `bool valid` | **测量**，与 `AdhereTargetCommand` 的人工意图严格区分；与 `AdhereTargetCommand` 共用同一个 `AdhereController::applyResolvedAdhereTarget()` 写入路径与 `retarget_threshold`/`changed_pending` 门限——后写入者生效，测得标签优先于更早的预设，直到操作员显式重新预设 |
| `BenchmarkMetrics.msg` | `/rov/benchmark_metrics`，`AdhereController`，固定 10Hz（与 `RuntimeDiagnostics` 共用同一个 "diag" `MultiRateDecimator` 通道，Phase 5.0 Stage 7 新增） | `int32 current_state`; `string state_name`（自描述）; `float64 thrust_l2_norm`; `float64 thrust_power_proxy`; `float64 thrust_slew_rate_l2`; `float64 cumulative_energy_l2`; `float64 cumulative_impulse_l1`; `uint8 saturated_thruster_count`; `uint8 active_contact_pad_count`; `float64 kern_margin` | 从 `ControllerState.msg` 拆出的效率/啁啾/安全遥测，只被 `adhere_monitor` 的 `rt_monitor_node`/`benchmark_evaluator_node` 消费 |

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
"msg/RuntimeDiagnostics.msg" "msg/AdhereTargetCommand.msg" "msg/TagDetection.msg"
"msg/BenchmarkMetrics.msg" DEPENDENCIES std_msgs geometry_msgs)`
生成对应的 `adhere_control_msgs/msg/*.hpp`。其余 `.msg`/`.srv` 文件待后续阶段按上表
实现后追加到该调用的文件列表中。

## ROS 2 Foxy 注意事项

`rosidl_default_generators`/`rosidl_default_runtime` 在 Foxy 与后续版本 API 一致，
无需特殊适配；但生成的 C++ 消息头文件路径格式
（`adhere_control_msgs/msg/fsm_command.hpp`）需在依赖包 `CMakeLists.txt` 中正确
`find_package(adhere_control_msgs REQUIRED)` + `ament_target_dependencies`。
