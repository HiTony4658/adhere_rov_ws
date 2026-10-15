# `descriptions/` — 机器人描述层

## 设计理念

本层对标 `quadruped_ros2_control/descriptions`（各厂商四足机器人 URDF 描述包）与
`ros-controls/mujoco_ros2_control` 示例中的描述包惯例，统一存放：

1. **ROS 2 侧描述**：`urdf/` 与 `xacro/`（含 `ros2_control` 标签，声明 8 个推力器的
   `command_interface`（`effort`）与 36 个 `state_interface`，供 `controller_manager`
   解析 `robot_description` 参数并加载 `AdhereMujocoHardware` 插件）。
2. **MuJoCo 侧描述**：`mjcf/`（`rov_wall.xml`），与 Python/JAX 原型仓库
   `mujoco_mppi_ws/assets/rov_wall.xml` **数值完全对齐**（接触几何 `a=0.15m, b=0.25m`、
   Kelvin-Voigt 接触参数 `k_c, b_c`、推力器安装点、质量/惯量/浮力偏置），
   由 `AdhereMujocoHardware` 在 `configure()` 阶段通过 `mj_loadXML()` 加载。
3. **可视化与参数配置**：`meshes/`（可视化网格，若使用简化几何体可省略）、
   `config/`（RViz 配置、`ros2_control` 控制器参数 YAML：更新频率、PID/OSQP 权重、
   CUDA MPPI 超参数）、`launch/`（启动 `robot_state_publisher`、`controller_manager`、
   `rviz2` 的组合 launch 文件）。

## 关键设计约束

- **MJCF 与 URDF 必须保持几何/惯量一致**：URDF 仅用于 `ros2_control` 接口声明与
  RViz 可视化，真实物理仿真完全由 MJCF 驱动；两者的质量属性、接触几何若不一致，
  将导致 RViz 可视化与实际物理行为脱节，引发调试误导。
- **接口命名与 `CLAUDE.md` 第 3 节 "导出接口契约" 严格一致**：8 个
  `command_interface`（`thr_N1/effort` .. `thr_T8/effort`）与 36 个
  `state_interface`（推力器 8 + IMU 10 + 位姿/机体速度 10 + 接触垫 8）命名必须与
  `adhere_mujoco_hardware` 导出的接口逐字匹配，否则 `controller_manager` 装配阶段
  将报 "interface not found" 错误。

## 当前软件包

| 软件包 | 职责 |
|---|---|
| [`simplified_v1_description`](simplified_v1_description/README.md) | URDF/Xacro + MJCF + RViz/控制器参数配置 + 启动文件 |

