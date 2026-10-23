# `simplified_v1_description`

## 职责

提供推力贴附式船体巡检 ROV 的双重几何/物理描述：

1. **ROS 2 / `ros2_control` 侧**：`urdf/simplified_v1.urdf.xacro`（经 `xacro:include`
   聚合 `xacro/robot.xacro` 几何宏与 `xacro/ros2_control.xacro` 硬件接口声明），供
   `robot_state_publisher`、`rviz2` 与 `controller_manager` 解析。该 URDF 是**纯机器
   人模型**：唯一的根连杆（root link）是 `base_link`（机体 + 4 接触垫 + 8 推力器
   标记），不包含任何 `world`/`wall` 连杆——壁面是仿真环境的一部分，不是机器人
   本体的一部分。
2. **MuJoCo 物理侧**：`mjcf/rov_wall.xml`，与 Python/JAX 原型仓库
   `mujoco_mppi_ws/assets/rov_wall.xml`（只读参考）逐字节一致，由
   `adhere_mujoco_hardware` 在运行时加载；壁面几何只存在于这一侧。
3. **RViz2 可视化侧**：壁面仅通过 `adhere_mujoco_node` 发布的
   `visualization_msgs/msg/Marker`（`/rov/environment_markers`，`world` 坐标系下的
   `CUBE`）渲染，与机器人本体 URDF 完全解耦。

两侧几何参数（接触垫/推力器/壁面坐标）严格对齐，详见下表。

## 文件结构

```text
urdf/
  └── simplified_v1.urdf.xacro     # 顶层入口：include robot.xacro + ros2_control.xacro
xacro/
  ├── robot.xacro                   # 机体/4接触垫/8推力器标记几何宏（base_link 为唯一根连杆）
  └── ros2_control.xacro            # <ros2_control> 标签：8 command + 57 state interface
mjcf/
  └── rov_wall.xml                  # MuJoCo 物理描述（与原型仓库数值对齐，只读复制）
config/
  ├── visualize.rviz                # RViz2 几何检视配置（RobotModel + TF + Grid）
  ├── mujoco_sim.rviz               # RViz2 闭环仿真显示配置（Odometry + EnvironmentMarkers，TF 缩放
                                     # 0.12）；Phase 3.7 从 adhere_mujoco_hardware 迁移至此，由
                                     # launch/mujoco.launch.py 统一解析，使硬件包零模型/可视化配置文件
  └── robot_control.yaml            # 模型解耦参数：推力器/接触垫几何、Fossen 水动力参数、
                                     # 工作空间安全边界、各 FSM 状态增益（CLAUDE.md §3.3，Phase 3.7 新增
                                     # dynamics.*/contact_force_thresh）
launch/
  ├── visualize.launch.py           # 一行命令启动 robot_state_publisher + RViz2（纯几何检视，无物理仿真）
  └── mujoco.launch.py              # 模型中心化主入口（CLAUDE.md Phase 3.4）：robot_state_publisher +
                                     # adhere_mujoco_hardware 的 adhere_mujoco_node + RViz2 +
                                     # 可选 adhere_controller 自动加载 + 可选 GCS 遥操作/FSM 控制台
```

## 导出接口契约（与 `CLAUDE.md` §3 严格一致）

### 8 个 `command_interface`

| 名称 | 范围 |
|---|---|
| `thr_N1/effort` .. `thr_N4/effort` | `[-40.0, 40.0] N` |
| `thr_T5/effort` .. `thr_T8/effort` | `[-40.0, 40.0] N` |

### 57 个 `state_interface`（CLAUDE.md Phase 3.5：地面真值 + 独立的噪声多速率传感器通路；Phase 3.6 新增诊断 2 项）

| 分组 | 数量 | 接口名 | 性质 |
|---|---|---|---|
| 推力器回读 | 8 | `thr_N1/effort` .. `thr_T8/effort` | — |
| IMU (`imu_sensor`) | 10 | `orientation.w/x/y/z`, `angular_velocity.x/y/z`, `linear_acceleration.x/y/z` | 地面真值 |
| 位姿/机体速度 (`rov_odom`) | 10 | `pos.x/y/z`, `quat.w/x/y/z`, `vel_body.x/y/z` | 地面真值 |
| 接触垫 (`pad_contact`) | 8 | `pad_TL/force` .. `pad_BR/force`, `pad_TL/v_sep` .. `pad_BR/v_sep` | 地面真值 |
| 噪声 IMU (`imu_sensor_noisy`) | 10 | 与 `imu_sensor` 同名布局，500Hz | 噪声（`RovStateEstimator` 专用） |
| 噪声深度 (`depth_sensor_noisy`) | 1 | `z`，50Hz 采样保持 | 噪声 |
| 噪声 DVL (`dvl_sensor_noisy`) | 4 | `vel.x/y/z`, `range`，20Hz 采样保持 | 噪声 |
| 噪声载荷传感器 (`loadcell_noisy`) | 4 | `pad_TL/force` .. `pad_BR/force`，500Hz | 噪声 |
| 诊断 (`diagnostics`) | 2 | `hw_loop_hz`, `hw_step_exec_ms` | 镜像 `MujocoVisBridge` 物理线程 EMA 遥测（Phase 3.6） |

地面真值与噪声两组通路完全独立生成（`AdhereMujocoHardware::read()`），从不
互相派生；`AdhereController` 仅把地面真值组私有化用于评估，`FSMState` 只能
读取 `estimation_mode` 选定来源后写入的 `CtrlInterfaces::ActiveState`
（详见 [`adhere_controller/README.md`](../../controllers/adhere_controller/README.md)）。

## 关键几何参数（须与 MJCF 数值一致）

- 4 个被动接触垫（球体，半径 0.035 m）：`r_i = (-0.10, ±0.25, ±0.15) m`
- 8 个推力器安装位：`X = 0.0 m`，与接触垫共用 Y/Z 角点坐标
- 壁面：世界坐标 `X = 0` 平面（MJCF 中渲染为 `X = -0.01` 的薄壁；不在 URDF 中，
  仅存在于 `mjcf/rov_wall.xml` 与 RViz2 的 `/rov/environment_markers` Marker 中），
  `Y ∈ [-10.0, +10.0] m × Z ∈ [-10.0, +1.5] m`（20m × 11.5m，CLAUDE.md Phase 4.2：
  顶部在水面 `Z=0.0m` 以上延伸 `1.5m`，供贴壁模式 ROV 在水面附近/以上作业）
- 水箱：水面在世界 `Z = 0.0 m`，箱底在 `Z = -10.0 m`（10m 水深），水平范围
  `20.0 m × 20.0 m`；ROV 默认水下出生点 `pos = "0.125 0 -2.0"`（水面以下 2.0m）
- 机体：质量 20.0 kg（MJCF 中为含附加质量的 26.0 kg），对角惯量 `[1.35, 0.55, 1.10] kg·m²`
- 传感器安装位（CLAUDE.md Phase 3.5，`mjcf/rov_wall.xml` 的 `imu_site`/
  `depth_site`/`dvl_site`/`loadcell_*` 与 `xacro/robot.xacro` 的
  `imu_link`/`depth_link`/`dvl_link`/`loadcell_*_link` 一一对应）：IMU/深度计在
  质心 `(0,0,0)`，DVL 在机体底面 `(0,0,-0.20)`，4 个载荷传感器与接触垫同坐标
  `(-0.10, ±0.25, ±0.15)`

## `config/robot_control.yaml`：模型解耦参数文件

`src/controllers/adhere_controller` 的 C++ 代码里不包含任何机器人专属几何或
物理常数——`ModelParams::loadFromNode()` 在 `AdhereController::on_configure()`
中从本文件读取推力器几何（`thruster_pos_xyz`/`thruster_dir_xyz`/`thruster_min`/
`thruster_max`）、接触垫几何（`pad_pos_xyz`）、Fossen 水动力前馈参数
（`rho_water`/`gravity`/`vehicle_mass`/`displaced_volume`/`D_lin`/`D_quad`/...）、
工作空间安全边界（`workspace_z_min`/`workspace_z_max`/`workspace_x_min`）以及
各 FSM 状态增益（`open_loop.*`/`pid_hold.*`/`adhere_pid.*`/
`adhere_smc.*`，后两者为 Phase 3.8 新增的贴壁 PID/SMC 基线增益；`depth_hold.*` 已随
Phase 5.0 删除的 `HOLD_PID_DEPTH` 状态一并移除）。切换到几何/质量
不同的新机型只需改这一份 YAML，`adhere_controller` 零 C++ 改动、零重新编译。
本文件通过本包自己的 `launch/mujoco.launch.py`（CLAUDE.md Phase 3.4：模型/
描述包拥有自己的启动文件，`adhere_mujoco_hardware`/`adhere_controller` 均为
零启动文件的纯工具箱）作为 `--params-file` 传给 `adhere_mujoco_node`
可执行文件；由于 `ros2 launch` 的节点重映射机制，文件内所有参数必须平铺在
同一个顶层键 `adhere_mujoco_node:` 下（详见
[`adhere_controller/README.md`](../../controllers/adhere_controller/README.md)）。

模型没有任何可动的 URDF 关节——唯一的真实自由度（浮动基座）由
`AdhereMujocoHardware` 的 MuJoCo `freejoint` 驱动，而非 URDF 关节；因此本包的
RViz 检视视图无需 `joint_state_publisher`（更不需要其 GUI 变体）。`base_link` 是
整个 URDF 树的唯一根连杆（无 `world` 连杆），`check_urdf` 校验根连杆即为
`base_link`。

## 运行

本包是模型中心化启动的唯一入口（CLAUDE.md Phase 3.4）：`adhere_mujoco_hardware`
与 `adhere_controller` 均不再持有任何启动文件，切换到新机型只需在该机型自己的
描述包下复制/适配同名启动文件。

```bash
colcon build --symlink-install
source install/setup.bash

# 静态几何检视：仅 robot_state_publisher + rviz2（Fixed Frame = base_link），
# 不含任何物理仿真，也不渲染壁面（壁面不属于机器人 URDF）
ros2 launch simplified_v1_description visualize.launch.py

# 完整闭环仿真（默认行为）：adhere_mujoco_hardware 的 500Hz 物理后端 + Fossen
# 水动力学 + 可运行时切换造浪 + RViz2 + adhere_controller 自动加载/配置/激活
# （默认 IDLE_PASSIVE_NONE 态）+ Xbox 手柄遥操作 + FSM 终端控制台
ros2 launch simplified_v1_description mujoco.launch.py

# 纯开环硬件仿真：覆盖为 0 个控制器加载（u=0 被动漂移验证场景），不弹出遥操作
ros2 launch simplified_v1_description mujoco.launch.py autostart_controllers:="[]" \
    launch_teleop:=false

# CLAUDE.md Phase 3.5：切换到"仅凭噪声多速率传感器 + RovStateEstimator"的真实
# 部署模式（默认是 ground_truth，与 Phase 2-3.4 行为完全一致）
ros2 launch simplified_v1_description mujoco.launch.py estimation_mode:=sensor_fusion
```

## 墙面贴壁目标站点 AprilTag 物理贴纸规格（CLAUDE.md Phase 5.0 Stage 6）

真实部署时，每个贴壁目标站点（`robot_control.yaml` 的 `adhere_wall_tags_y`/
`adhere_wall_tags_depth_m`，默认 4 个站点）在真实墙面上粘贴一张 **2x2 AprilTag
矩阵**贴纸，由真实机器人上运行的 `apriltag_ros`（`tag36h11` 族）识别后发布
`TagDetection` 到与仿真侧 `adhere_apriltag_detector` 完全相同的
`/rov/tag_detection` 话题（详见 `perception/adhere_apriltag_detector/README.md`）。

- **标签族**：`tag36h11`；单个标签黑色方块边长 `s = 80mm`；标签中心间距
  （横/纵均相同）`p = 160mm`（相邻标签间 `80mm` 白边间隙）；矩阵黑色区域总
  外廓 `240 x 240mm`；背板 `300 x 300mm`，防水**哑光**（Matte）覆膜——严禁
  使用光面覆膜，水下点光源的镜面反光会导致识别失败。
- **编号**：4 个站点 x 每站点 4 个标签 = ID `0`-`15`，站点 `k` 使用连续 ID
  `4k..4k+3`，即 `station_id = min_id / 4`；矩阵几何中心 = 该站点在
  `adhere_wall_tags_y`/`adhere_wall_tags_depth_m` 中的 `(Y, 深度)` 目标点：

  | 站点 k | `adhere_wall_tags`（Y, 深度） | 左上 | 右上 | 左下 | 右下 |
  |---|---|---|---|---|---|
  | 0 | (0.00, 1.00) | 0 | 1 | 2 | 3 |
  | 1 | (0.80, 1.00) | 4 | 5 | 6 | 7 |
  | 2 | (-0.80, 1.50) | 8 | 9 | 10 | 11 |
  | 3 | (1.20, 2.00) | 12 | 13 | 14 | 15 |

- **`apriltag_ros` `tag_bundles` 局部坐标**（矩阵自身坐标系原点在中心，
  `x` = 水平方向/世界 Y，`y` = 竖直方向/世界上方，半间距 `0.08m`，每个标签
  `size = 0.08`）：

  ```text
  左上(4k+0): x=-0.08 y=+0.08    右上(4k+1): x=+0.08 y=+0.08
  左下(4k+2): x=-0.08 y=-0.08    右下(4k+3): x=+0.08 y=-0.08
  ```

  检测到的矩阵原点位姿即该站点的 `(Y, 深度)` 目标，以 `TagDetection`
  形式发布。
- **安装**：4 个标签共面、紧贴墙面，行/列分别对齐世界 Y（水平）与 Z
  （竖直，不旋转），矩阵中心落在站点目标点上。
- **仿真侧说明**：`adhere_apriltag_detector`（仿真版）不需要这套 2x2 几何，
  只用 4 个站点中心点——16 个 ID 在仿真里退化为 4 个点；该矩阵布局规格的
  存在仅为了让真实 `apriltag_ros` 配置随时可用，不影响仿真行为。
