# `adhere_rov_description`

## 职责

提供推力贴附式船体巡检 ROV 的双重几何/物理描述：

1. **ROS 2 / `ros2_control` 侧**：`xacro/adhere_rov.urdf.xacro`（含 `<ros2_control>`
   标签，声明 `hardware_interface/AdhereMujocoHardware` 插件装配参数、8 个
   `command_interface` 与 36 个 `state_interface`），供 `robot_state_publisher`
   与 `controller_manager` 解析。
2. **MuJoCo 物理侧**：`mjcf/rov_wall.xml`，与 Python/JAX 原型仓库
   `mujoco_mppi_ws/assets/rov_wall.xml` 数值严格对齐（只读参考，不可修改源仓库）。

## 规划中的文件列表

```text
urdf/           # (若需要) 纯 URDF，无 xacro 宏展开的产物，一般不手工维护
xacro/
  └── adhere_rov.urdf.xacro       # 含 <ros2_control> 标签的主 xacro 文件
  └── adhere_rov_macro.xacro      # 推力器/接触垫宏定义
mjcf/
  └── rov_wall.xml                 # MuJoCo 物理描述（壁面+ROV+8推力器+4接触垫）
meshes/
  └── (可选) 简化几何体 .stl/.dae，若直接用 MuJoCo primitive geom 可省略
config/
  └── adhere_controllers.yaml      # controller_manager 装配参数（update_rate: 500 等）
  └── rviz_adhere.rviz              # RViz 可视化配置
launch/
  └── adhere_mujoco_node.launch.py  # 启动 controller_manager + robot_state_publisher + rviz2
```

## 导出接口契约（与 `CLAUDE.md` §3.2 严格一致）

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

- 接触垫位置：`r_i = (-0.10, ±0.15, ±0.25) m`（`a=0.15m, b=0.25m`）
- 切向推力器安装偏置：`X = -0.10 m`
- 壁面：世界坐标 `X = 0` 平面

## Phase 1 状态

目录骨架已建立（`urdf/ xacro/ mjcf/ meshes/ config/ launch/`），具体 xacro/MJCF/
YAML 内容留待 Phase 2，需先与 `mujoco_mppi_ws/assets/rov_wall.xml` 逐项核对参数。
