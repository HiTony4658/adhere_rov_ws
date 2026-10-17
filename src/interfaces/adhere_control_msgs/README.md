# `adhere_control_msgs`

## 职责

为推力贴附式船体巡检 ROV 的 KAMR-MPPI 控制栈提供跨节点共享的消息/服务类型：
FSM 命令、目标位姿/轨迹、500Hz/20Hz 遥测。本包不依赖本工作空间内任何其他包，
是依赖图的叶子节点。

> 这些消息类型**不**用于 `AdhereMujocoHardware` ↔ `AdhereKamrController` 的主数据
> 通路（该通路走 `ros2_control` 的 `CtrlInterfaces` 零拷贝指针），仅用于低频
> 监控、人机交互、日志记录场景。

## 已实现的接口

### 消息（`msg/`）

| 文件 | 字段摘要 | 用途 |
|---|---|---|
| `FsmCommand.msg` | `std_msgs/Header header`; `int32 command`（常量 `NO_CHANGE=0, PASSIVE=1, DEPTH_HOLD=2`）; `float64 target_z, target_y, target_yaw` | `adhere_keyboard_input`（`fsm_console_node`）→ `AdhereKamrController` 的 `/rov/fsm_command` 状态切换/定深目标请求。`target_y`/`target_yaw` 为未来平面站位保持/航向控制预留，当前未被 `StateDepthHold` 使用 |

## 规划中的接口列表（后续阶段实现）

| 文件 | 字段摘要 | 用途 |
|---|---|---|
| `TargetPose.msg` | `float64 y, z, psi` | 站位保持目标位姿（壁面平面内 2D 位置 + 偏航角） |
| `TargetTrajectory.msg` | `geometry_msgs/PoseStamped[] waypoints`, `float64[] timestamps` | 轨迹跟踪参考轨迹 |
| `RhombicKernMetrics.msg` | `float64 k_val, m_kern, p_y, p_z, f_press` | 500Hz 菱形核可行域余量遥测（对应 `CLAUDE.md` §4.4） |
| `PadContactState.msg` | `float64[4] f_normal, float64[4] v_sep, bool[4] liftoff` | 4 个接触垫法向力/分离速度/掀离标志（对应 36 维状态接口中的 `pad_contact` 段） |
| `KamrMppiTelemetry.msg` | `float64 outer_solve_latency_ms, float64 f_x_extra_preload, float64[4] d_hat_unload, float64[4] w_cmd` | 20Hz 外环求解延迟、Pad-Force DOB 卸载估计、MPPI 名义力矩 |
| `SwitchFsmState.srv` | `req: uint8 target_state` / `res: bool accepted, string reason` | 同步 FSM 状态切换请求（允许拒绝非法迁移） |

## 构建说明

`CMakeLists.txt` 的 `rosidl_generate_interfaces(${PROJECT_NAME} "msg/FsmCommand.msg" DEPENDENCIES std_msgs)`
生成 `adhere_control_msgs/msg/fsm_command.hpp`。其余 `.msg`/`.srv` 文件待后续阶段
按上表实现后追加到该调用的文件列表中。

## ROS 2 Foxy 注意事项

`rosidl_default_generators`/`rosidl_default_runtime` 在 Foxy 与后续版本 API 一致，
无需特殊适配；但生成的 C++ 消息头文件路径格式
（`adhere_control_msgs/msg/fsm_command.hpp`）需在依赖包 `CMakeLists.txt` 中正确
`find_package(adhere_control_msgs REQUIRED)` + `ament_target_dependencies`。
