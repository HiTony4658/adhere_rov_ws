# `adhere_mujoco_hardware`

## 职责概述

`AdhereMujocoHardware` 是本工作空间中唯一直接持有 MuJoCo `mjModel*`/`mjData*` 的
类，扮演"真实 ROV + 水下环境"的角色。它是一个 ROS 2 Foxy
`hardware_interface::SystemInterface` 插件，由 `controller_manager` 以严格
**`500Hz`**（`update_rate: 500`，`dt = 2.0 ms`）周期驱动，对上层控制器
（`AdhereKamrController`）的实际更新速率（200Hz 内环 / 20Hz 外环）完全无感知，
从而保证 `CLAUDE.md` §1.3 要求的"Defensive Systems Architect"核心约束：
**Sim-to-Real 一致性**。

## 类与文件规划

```text
include/adhere_mujoco_hardware/
├── adhere_mujoco_hardware.hpp      # AdhereMujocoHardware 主类声明
├── mujoco_vis_bridge.hpp           # MujocoVisBridge 结构体（60Hz<->500Hz 快照桥）
├── mujoco_viewer.hpp               # MujocoViewer 类声明（GLFW/OpenGL 渲染线程）
└── fossen_hydro_adapter.hpp        # 对 adhere_controller_common::FossenHydrodynamics 的薄封装

src/
├── adhere_mujoco_hardware.cpp      # 500Hz read()/write() 主体实现
├── mujoco_vis_bridge.cpp
└── mujoco_viewer.cpp

adhere_mujoco_hardware.xml           # pluginlib 插件描述文件（hardware_interface 导出）
```

## `AdhereMujocoHardware` 类职责与成员

```text
class AdhereMujocoHardware
  : public hardware_interface::BaseInterface<hardware_interface::SystemInterface>
{
  mjModel*  m_;                       // 主物理模型（configure() 阶段 mj_loadXML 加载）
  mjData*   d_;                       // 主物理数据
  FossenHydrodynamics  hydro_;        // 复用 adhere_controller_common，保证与控制器一致
  double    hw_states_[36];           // 导出为 36 个 StateInterface 的裸内存
  double    hw_commands_[8];          // 导出为 8 个 CommandInterface 的裸内存
  std::unique_ptr<MujocoVisBridge> vis_bridge_;  // 60Hz 可视化快照桥（见下）
  int       step_counter_ = 0;        // 用于 "每 8 步同步一次可视化" 的节拍计数
};
```

## ROS 2 Foxy 生命周期方法签名（严格遵守，无 `time, period` 参数）

```cpp
hardware_interface::return_type configure(const hardware_interface::HardwareInfo & info) override;
std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
hardware_interface::return_type start() override;
hardware_interface::return_type stop() override;
hardware_interface::return_type read() override;   // 无参数！Foxy 特有
hardware_interface::return_type write() override;  // 无参数！Foxy 特有
```

> 与 Humble/Jazzy 的 `read(time, period)`/`write(time, period)` 不同，Foxy 的
> `SystemInterface::read()`/`write()` **不接受任何参数**；`dt` 通过
> `controller_manager` 的 `update_rate` 参数隐式保证，`AdhereMujocoHardware`
> 内部若需要精确 `dt`，应从 `m_->opt.timestep`（MJCF 中配置为 `0.002`）读取，
> 而不是依赖外部传入的 `period`。

## 500Hz 主循环深度剖析

### `configure(info)`

1. 解析 `info.hardware_parameters["mjcf_path"]`（通过
   `ament_index_cpp::get_package_share_directory("adhere_rov_description")`
   拼接绝对路径，**禁止硬编码绝对路径**）。
2. `m_ = mj_loadXML(...)`，`d_ = mj_makeData(m_)`。
3. 校验 `info.joints`/`info.sensors` 与 MJCF 的推力器/传感器数量一致（8 推力器、
   36 态）。
4. 构造 `hydro_`（`FossenHydrodynamics`，来自 `adhere_controller_common`），加载
   质量、惯量、浮力偏置（`58.86 N`）、Morison 阻力系数等参数。

### `read()`（@ 500Hz，每次 `controller_manager` 调用一次）

```text
1. hw_states_[0:8]   = d_->ctrl[0:8]                         // 推力器回读
2. hw_states_[8:18]  = IMU: d_->sensordata[imu段]              // 姿态四元数+角速度+线加速度
3. hw_states_[18:28] = 位姿: d_->xpos/xquat + 机体速度         // pos.xyz, quat.wxyz, vel_body.xyz
4. hw_states_[28:36] = 4个接触垫: F_N,i (d_->sensordata[contact段])
                        v_sep,i = v_x,b + omega_y*z_i - omega_z*y_i   // 按 CLAUDE.md §4.2 公式实时计算
5. 计算 500Hz ground-truth 遥测：
     M_kern = 1 - K_val   （K_val 由 F_N,i 反算压力中心 p_y,p_z 得出）
     liftoff_i = (F_N,i <= 0)
   写入诊断缓冲（供可选的 adhere_control_msgs::RhombicKernMetrics 发布，走独立低频线程）
```

### `write()`（@ 500Hz，每次 `controller_manager` 调用一次）

```text
1. 读取 hw_commands_[0:8] 的"当前值"——无论这是上一次 200Hz/20Hz tick 写入的新值，
   还是维持了若干个 500Hz tick 未变的旧值，write() 都一视同仁。
   这天然模拟了真实 ESC 的零阶保持（ZOH）特性，是 Sim-to-Real 一致性的关键。
2. 对每个推力器施加一阶滞后 tau_T（可选）与死区（可选）：
     u_filtered_i = u_prev_i + (dt/tau_T) * (hw_commands_[i] - u_prev_i)
3. hydro_.compute(d_->qpos, d_->qvel, t) 计算 6-DOF Fossen 水动力
   （附加质量 + Morison 二次阻力 + 近壁 Venturi 吸附效应 + 浮力偏置）
   + 标准 12.0s 阵风扰动 profile（若处于 benchmark 模式）
   写入 d_->xfrc_applied[rov_body]。
4. 将 u_filtered_i 映射为推力器施力方向向量，叠加进 d_->xfrc_applied 或
   d_->qfrc_applied（取决于推力器是否建模为独立 body）。
5. mj_step(m_, d_)   // 推进物理 2.0 ms
6. 若 (++step_counter_ % 8 == 0)：尝试 std::mutex::try_lock()（<10us），
   将 d_->qpos/qvel/ctrl + 4个F_N,i + 菱形核CoP 拷贝进 MujocoVisBridge 快照区；
   若加锁失败（MujocoViewer 正在读取上一帧），直接跳过，绝不阻塞 500Hz 主循环。
```

## `MujocoVisBridge`：500Hz ↔ 60Hz 指针桥

```cpp
struct MujocoVisBridge {
  std::mutex mtx;                 // try_lock() 保护，非阻塞
  double qpos_snapshot[...];
  double qvel_snapshot[...];
  double ctrl_snapshot[8];
  double pad_force_snapshot[4];
  double kern_cop_snapshot[2];    // p_y, p_z
  double xfrc_perturb[6];         // 60Hz -> 500Hz 反向通道：鼠标扰动力
};
```

- **写方向（500Hz → 60Hz）**：`AdhereMujocoHardware::write()` 末尾，每 8 步写一次
  快照（见上），用 `try_lock()` 保证绝不阻塞物理主循环。
- **读方向（60Hz → 500Hz）**：`MujocoViewer` 渲染线程中若 `mjvPerturb` 检测到用户
  鼠标拖拽，将扰动力写入 `xfrc_perturb`；`AdhereMujocoHardware::write()` 的下一次
  500Hz tick 会读取该字段并叠加进 `d_->xfrc_applied`。

## `MujocoViewer`：60Hz 可视化前端

- 独立线程运行 GLFW/OpenGL 渲染循环，拥有自己的 `mjData* d_vis_ = mj_makeData(m_)`
  （与主 `d_` 完全隔离，避免渲染读取与物理写入的数据竞争）。
- 每帧：从 `MujocoVisBridge` 拷贝快照到 `d_vis_`，调用 `mj_forward(m_, d_vis_)`
  重新计算运动学/碰撞几何（**不调用 `mj_step`**，因为渲染线程不拥有权威物理状态），
  渲染 3D ROV/壁面/4 个接触垫力柱状图/菱形核 CoP 轨迹。
- 该线程的渲染延迟、VSync 等待、OpenGL 上下文切换**绝不会**影响 500Hz 主循环的
  实时性，这是本架构相较于"仿真与渲染同步阻塞"方案的核心优势。

## 线程安全机制小结

| 共享数据 | 保护机制 | 访问方 |
|---|---|---|
| `hw_states_[36]` / `hw_commands_[8]` | 无锁（单生产者单消费者，`controller_manager` 保证 `read→update→write` 串行调用顺序，天然无竞争） | `AdhereMujocoHardware` ↔ `AdhereKamrController`（via `CtrlInterfaces`） |
| `MujocoVisBridge` 快照区 | `std::mutex::try_lock()`（非阻塞，失败即跳过本次同步） | `AdhereMujocoHardware`（500Hz 写）↔ `MujocoViewer`（60Hz 读） |
| `MujocoVisBridge::xfrc_perturb` | 同上互斥锁 | `MujocoViewer`（60Hz 写）↔ `AdhereMujocoHardware`（500Hz 读） |

## ROS 2 Foxy 特有注意事项

- `pluginlib_export_plugin_description_file(hardware_interface adhere_mujoco_hardware.xml)`
  必须在 `CMakeLists.txt` 中调用，且 `package.xml` 的
  `<export><hardware_interface plugin=.../></export>` 标签名为 `hardware_interface`
  （Foxy 与后续版本一致）。
- `BaseInterface<SystemInterface>` 这一层包装是 Foxy 特有的（Humble 起已移除），
  直接继承 `hardware_interface::SystemInterface` 在 Foxy 下会导致纯虚函数未实现
  的编译错误。

## Phase 1 状态

仅搭建头文件/源文件目录骨架与注释掉的 `CMakeLists.txt` 构建目标、空的
`adhere_mujoco_hardware.xml` 插件描述占位。真实 MuJoCo/GLFW 依赖发现与源码实现
留待 Phase 2。
