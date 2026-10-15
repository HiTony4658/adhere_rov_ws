# `hardwares/` — `ros2_control` 硬件接口层

## 设计理念

本层对标 `quadruped_ros2_control/hardwares`（`gz_quadruped_hardware`、
`hardware_unitree_sdk2` 分别对应仿真后端与真实硬件后端的"可插拔双实现"范式）与
`ros-controls/mujoco_ros2_control` 的核心思想：**硬件接口是唯一允许直接触碰物理
仿真/真实硬件 I/O 的边界层**，`controllers/` 中的任何算法代码都不应感知自己是在
对接 MuJoCo 仿真还是真实电调/传感器——这正是 `CLAUDE.md` 中"Defensive Systems
Architect"职责要求的**严格 Sim-to-Real 一致性**的工程落地方式。

当前阶段（Phase 1-6）只实现仿真后端 `adhere_mujoco_hardware`；根据 `CLAUDE.md`
第 6 节注记，未来 Jetson AGX Xavier 真实硬件接入阶段会新增 `adhere_real_hardware`
包，与 `adhere_mujoco_hardware` 导出**完全相同的** 8 个 `command_interface` 与
36 个 `state_interface` 命名契约，使 `AdhereKamrController` 无需任何代码改动即可
切换仿真/实物。

## 核心机制：500Hz 自治循环

`AdhereMujocoHardware` 被 `controller_manager`（`update_rate: 500`）严格以
`dt = 2.0 ms` 周期驱动，其 `read()`/`write()` 内部完整封装了"传感器读取 → 水动力
计算 → 物理步进 → 状态更新"的闭环，**完全不关心**上层 `AdhereKamrController` 是以
200Hz 还是 20Hz 更新 `hw_commands_`。详见
[`adhere_mujoco_hardware/README.md`](adhere_mujoco_hardware/README.md) 的深度剖析。

## 当前软件包

| 软件包 | 职责 |
|---|---|
| [`adhere_mujoco_hardware`](adhere_mujoco_hardware/README.md) | 500Hz MuJoCo `SystemInterface` 后端 + `MujocoVisBridge` + 60Hz `MujocoViewer` 可视化前端 |

