# `simplified_v1_description`

## 职责

提供推力贴附式船体巡检 ROV 的双重几何/物理描述：

1. **ROS 2 / `ros2_control` 侧**：`urdf/simplified_v1.urdf.xacro`（经 `xacro:include`
   聚合 `xacro/robot.xacro` 几何宏与 `xacro/ros2_control.xacro` 硬件接口声明），供
   `robot_state_publisher`、`rviz2` 与 `controller_manager` 解析。
2. **MuJoCo 物理侧**：`mjcf/rov_wall.xml`，与 Python/JAX 原型仓库
   `mujoco_mppi_ws/assets/rov_wall.xml`（只读参考）逐字节一致，由
   `adhere_mujoco_hardware` 在运行时加载。

两侧几何参数（接触垫/推力器/壁面坐标）严格对齐，详见下表。

## 文件结构

```text
urdf/
  └── simplified_v1.urdf.xacro     # 顶层入口：include robot.xacro + ros2_control.xacro
xacro/
  ├── robot.xacro                   # 机体/4接触垫/8推力器标记/壁面几何宏
  └── ros2_control.xacro            # <ros2_control> 标签：8 command + 36 state interface
mjcf/
  └── rov_wall.xml                  # MuJoCo 物理描述（与原型仓库数值对齐，只读复制）
config/
  └── visualize.rviz                # RViz2 几何检视配置（RobotModel + TF + Grid）
launch/
  └── visualize.launch.py           # 一行命令启动 robot_state_publisher + RViz2
```

## 导出接口契约（与 `CLAUDE.md` §3 严格一致）

### 8 个 `command_interface`

| 名称 | 范围 |
|---|---|
| `thr_N1/effort` .. `thr_N4/effort` | `[22.0, 40.0] N` |
| `thr_T5/effort` .. `thr_T8/effort` | `[-40.0, 40.0] N` |

### 36 个 `state_interface`

| 分组 | 数量 | 接口名 |
|---|---|---|
| 推力器回读 | 8 | `thr_N1/effort` .. `thr_T8/effort` |
| IMU (`imu_sensor`) | 10 | `orientation.w/x/y/z`, `angular_velocity.x/y/z`, `linear_acceleration.x/y/z` |
| 位姿/机体速度 (`rov_odom`) | 10 | `pos.x/y/z`, `quat.w/x/y/z`, `vel_body.x/y/z` |
| 接触垫 (`pad_contact`) | 8 | `pad_TL/force` .. `pad_BR/force`, `pad_TL/v_sep` .. `pad_BR/v_sep` |

## 关键几何参数（须与 MJCF 数值一致）

- 4 个被动接触垫（球体，半径 0.035 m）：`r_i = (-0.10, ±0.25, ±0.15) m`
- 8 个推力器安装位：`X = 0.0 m`，与接触垫共用 Y/Z 角点坐标
- 壁面：世界坐标 `X = 0` 平面（MJCF 中渲染为 `X = -0.01` 的薄壁）
- 机体：质量 20.0 kg（MJCF 中为含附加质量的 26.0 kg），对角惯量 `[1.35, 0.55, 1.10] kg·m²`

模型没有任何可动的 URDF 关节——唯一的真实自由度（浮动基座）由
`AdhereMujocoHardware` 的 MuJoCo `freejoint` 驱动，而非 URDF 关节；因此本包的
RViz 检视视图无需 `joint_state_publisher`（更不需要其 GUI 变体）。

## 运行

```bash
colcon build --packages-select simplified_v1_description
source install/setup.bash
ros2 launch simplified_v1_description visualize.launch.py
```

该命令仅做静态几何检视（`robot_state_publisher` + `rviz2`），不包含任何物理仿真；
若需查看带 Fossen 水动力学与阵风扰动的被动随波漂流效果，请使用
`adhere_mujoco_hardware` 包的 `mujoco_sim.launch.py`。
