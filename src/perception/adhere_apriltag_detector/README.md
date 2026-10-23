# `adhere_apriltag_detector`

## 职责

墙面贴壁目标站点的测量式检测（CLAUDE.md Phase 5.0 Stage 6），取代此前靠人工弹出
设定终端手动输入贴壁目标的做法：粗略预设（`robot_control.yaml` 的
`adhere_target_*`，或经 `ros2 topic pub /rov/adhere_target_cmd` 手动下发）只负责
把 `STANDOFF_HOMING` 大致带到正确的高度/Y；**测得的标签检测是最终权威**，一旦到达
测得值并与当前生效目标的差超过 `adhere_stage.retarget_threshold`，即覆盖粗预设
并强制重新进入 `STANDOFF_HOMING`（复用既有的 `changed_pending` 路径）。

本包发布 `TagDetection.msg` 到 `/rov/tag_detection`——与 `/rov/adhere_target_cmd`
（人工意图）严格区分，这是一条**测量**话题。`adhere_apriltag_detector_node`
本身是一个**邻近触发的点发布器，不是视觉管线**：维护与
`robot_control.yaml` 的 `adhere_wall_tags_y`/`adhere_wall_tags_depth_m`
相同的站点表（Y, 深度）二维坐标，订阅 ROV 的**估计**位姿
（`/rov/controller_state`，绝不读地面真值，保持 Sim-to-Real 纯净），当 ROV 进入
某个站点可配置的检测半径内，发布该站点坐标加一个小的可配置高斯偏移 +
`confidence`——用来演练"预设坐标 ≠ 真实贴纸实际位置"这一与真实部署一致的场景。

真实机器人上的等价节点是 `apriltag_ros`（`tag36h11` 族，每个站点一个 2x2 标签
矩阵，详见 `simplified_v1_description/README.md` 的物理贴纸规格），发布**完全
相同**的 `TagDetection` 消息到**完全相同**的 `/rov/tag_detection` 话题，因此
`AdhereController::applyResolvedAdhereTarget()` 这一侧的消费逻辑在仿真与真机之间
无需任何代码改动。

本包**零依赖** MuJoCo/CUDA/`ros2_control`/`adhere_controller`。

## ROS 参数

| 参数 | 默认值 | 说明 |
|---|---|---|
| `wall_tags_y` | `[0.00, 0.80, -0.80, 1.20]` | 各站点世界 Y 坐标（m），**必须**与 `robot_control.yaml` 的 `adhere_wall_tags_y` 保持一致 |
| `wall_tags_depth_m` | `[1.00, 1.00, 1.50, 2.00]` | 各站点水面以下深度（m），**必须**与 `adhere_wall_tags_depth_m` 保持一致 |
| `detection_range_m` | `0.5` | ROV 到站点中心的欧氏距离（Y-深度平面）小于此值即视为"检测到" |
| `position_noise_std_m` | `0.02` | 发布坐标上叠加的高斯噪声标准差（m） |
| `publish_rate_hz` | `5.0` | 发布频率 |

## 运行

```bash
ros2 run adhere_apriltag_detector adhere_apriltag_detector_node
```

ROV 不在任何站点检测半径内时，仍以 `publish_rate_hz` 持续发布
`TagDetection{valid: false}`（而非停止发布），供下游判断检测器本身是否存活。
