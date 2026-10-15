# `adhere_control_msgs`

## 职责

为推力贴附式船体巡检 ROV 的 KAMR-MPPI 控制栈提供跨节点共享的消息/服务类型：
FSM 命令、目标位姿/轨迹、500Hz/20Hz 遥测。本包不依赖本工作空间内任何其他包，
是依赖图的叶子节点。

> 这些消息类型**不**用于 `AdhereMujocoHardware` ↔ `AdhereKamrController` 的主数据
> 通路（该通路走 `ros2_control` 的 `CtrlInterfaces` 零拷贝指针），仅用于低频
> 监控、人机交互、日志记录场景。

## 规划中的接口列表（Phase 2 起实现）

### 消息（`msg/`）

| 文件 | 字段摘要 | 用途 |
|---|---|---|
| `FsmCommand.msg` | `uint8 target_state`（`PASSIVE=0, PRELOAD_ATTACH=1, STATION_KEEPING=2, TRAJECTORY_TRACKING=3, EMERGENCY_DETACH=4`） | `adhere_keyboard_input` → `AdhereKamrController` 的状态切换请求 |
| `TargetPose.msg` | `float64 y, z, psi` | 站位保持目标位姿（壁面平面内 2D 位置 + 偏航角） |
| `TargetTrajectory.msg` | `geometry_msgs/PoseStamped[] waypoints`, `float64[] timestamps` | 轨迹跟踪参考轨迹 |
| `RhombicKernMetrics.msg` | `float64 k_val, m_kern, p_y, p_z, f_press` | 500Hz 菱形核可行域余量遥测（对应 `CLAUDE.md` §4.4） |
| `PadContactState.msg` | `float64[4] f_normal, float64[4] v_sep, bool[4] liftoff` | 4 个接触垫法向力/分离速度/掀离标志（对应 36 维状态接口中的 `pad_contact` 段） |
| `KamrMppiTelemetry.msg` | `float64 outer_solve_latency_ms, float64 f_x_extra_preload, float64[4] d_hat_unload, float64[4] w_cmd` | 20Hz 外环求解延迟、Pad-Force DOB 卸载估计、MPPI 名义力矩 |

### 服务（`srv/`）

| 文件 | 请求/响应摘要 | 用途 |
|---|---|---|
| `SwitchFsmState.srv` | `req: uint8 target_state` / `res: bool accepted, string reason` | 同步 FSM 状态切换请求（允许拒绝非法迁移，如掀离中禁止直接进入 `TRAJECTORY_TRACKING`） |

## 构建说明

Phase 1 阶段 `CMakeLists.txt` 中 `rosidl_generate_interfaces()` 调用整体注释，
`colcon build` 仅生成一个空的 `ament_cmake` 包。Phase 2 将取消注释并按上表填充
真实 `.msg`/`.srv` 文件。

## ROS 2 Foxy 注意事项

`rosidl_default_generators`/`rosidl_default_runtime` 在 Foxy 与后续版本 API 一致，
无需特殊适配；但生成的 C++ 消息头文件路径格式
（`adhere_control_msgs/msg/fsm_command.hpp`）需在依赖包 `CMakeLists.txt` 中正确
`find_package(adhere_control_msgs REQUIRED)` + `ament_target_dependencies`。
