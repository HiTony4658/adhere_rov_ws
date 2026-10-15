# `adhere_mujoco_hardware`

## 职责概述

`AdhereMujocoHardware` 是本工作空间中唯一直接持有 MuJoCo `mjModel*`/`mjData*` 的
类，扮演"真实 ROV + 水下环境"的角色。它是一个 ROS 2 Foxy
`hardware_interface::SystemInterface` 插件，由 `controller_manager` 以严格
**`500Hz`**（`update_rate: 500`，`dt = 2.0 ms`）周期驱动。本阶段（Phase 2）尚未
接入任何上层控制器，`hw_commands_[8]` 恒为零，用于验证"零控制输入下的被动随波
漂流"（calm 水域下的浮力/阻力漂移，以及造浪工况下的侧向/吸附漂移）。

环境水流不再是启动时静态绑定的，而是**运行时可热切换**的：节点启动时恒定从
静水（Calm Water）开始，用户可随时通过交互式终端控制台 `wave_console_node`
开启/关闭造浪（详见"运行时造浪机切换"一节）。

## 文件结构

```text
include/adhere_mujoco_hardware/
├── adhere_mujoco_hardware.hpp      # AdhereMujocoHardware 主类
├── fossen_hydrodynamics.hpp        # HydroParams / GustEvent / FossenHydrodynamics
├── flow_environment_plugin.hpp     # FlowEnvironmentPlugin 策略接口 + 2 个具体插件 + 工厂函数
├── mujoco_vis_bridge.hpp           # MujocoVisBridge：500Hz<->60Hz 快照桥（POD + mutex）
└── mujoco_viewer.hpp               # MujocoViewer：GLFW/OpenGL 60Hz 渲染线程

src/
├── adhere_mujoco_hardware.cpp
├── fossen_hydrodynamics.cpp
├── flow_environment_plugin.cpp
├── mujoco_viewer.cpp
├── adhere_mujoco_node.cpp          # 500Hz ControllerManager 循环 + 100Hz TF/Odom/Marker 发布
└── wave_console_node.cpp           # 交互式造浪机终端控制台（发布 /rov/wave_mode_cmd）

config/
├── hydro_wave_params.yaml          # controller_manager 运行参数 + 两套水动力预设数值参考
└── mujoco_sim.rviz

launch/
└── mujoco_sim.launch.py

adhere_mujoco_hardware.xml           # pluginlib 插件描述文件（hardware_interface 导出）
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

`dt` 不依赖外部传入的 `period`，而是从 `mjModel::opt.timestep`（MJCF 中为
`0.002`）读取；`BaseInterface<SystemInterface>` 这一层包装是 Foxy 特有的。

## 500Hz 主循环

- **`configure(info)`**：通过 `ament_index_cpp::get_package_share_directory("simplified_v1_description")`
  解析 `mjcf/rov_wall.xml` 的绝对路径（不硬编码路径），`mj_loadXML` + `mj_makeData`；
  同时构造好 `CalmWaterPlugin` 与 `StationKeepingGustPlugin` 两个插件实例，运行时
  模式恒定初始化为 `1`（静水），不再由任何启动参数决定。
- **`read()`**：从 `mjData` 提取 8 个推力器回读值、10 维 IMU（姿态四元数 + 机体系
  角速度 + 有限差分线加速度近似）、10 维位姿/机体速度、8 维接触垫数据（4 个
  `touch_pad_*` 传感器法向力 + 按 `CLAUDE.md` §4.2 公式
  `v_sep,i = v_x,b + omega_y*z_i - omega_z*y_i` 实时计算的分离速度）。
- **`write()`**：把 `hw_commands_[0:8]`（本阶段恒为零）写入 `d_->ctrl`；通过一个
  仅在本进程内安装的 `mjcb_passive` 回调（与 Python 原型 `setup_hydrodynamics_
  callback` 完全相同的调用时机与目标累加数组 `qfrc_passive`，用于保证数值一致性）
  注入 `FossenHydrodynamics::compute()` 算出的水动力合力/合力矩；随后 `mj_step`
  推进 2.0 ms；每 8 步通过 `std::mutex::try_lock()`（绝不阻塞主循环）向
  `MujocoVisBridge` 写入一次渲染快照。

## 水动力学与可插拔水流环境

`fossen_hydrodynamics.hpp/.cpp` 是 `~/Research/mujoco_mppi_ws/src/env/hydrodynamics.py`
的逐条等价 C++ 移植（附加质量重力偏置、Morison 耦合二次阻力、近壁 Venturi 吸附、
自由液面渐变、CoB 回正力矩），已通过 3 组随机状态与 Python 参考实现数值比对，
误差远小于 `1e-5`（详见下方"数值校验"）。

`flow_environment_plugin.hpp/.cpp` 提供策略模式的可插拔水流环境：

| 插件 | 运行时模式 | 行为 |
|---|---|---|
| `CalmWaterPlugin`（默认/启动即激活） | `1` | 环境流速恒为零，仅受浮力/阻力/Venturi 吸附影响 |
| `StationKeepingGustPlugin` | `2` | 定常 `0.3 m/s` 世界系 `+Y` 侧向流 + 两段半正弦阵风（相对造浪时钟 `τ∈[3.0,4.2]s` 与 `[7.0,8.2]s`，侧向峰值 `0.9 m/s`，法向吸附峰值 `0.2 m/s`） |

两个插件的数值常量与 Python 参考逐位一致。`config/hydro_wave_params.yaml` 同时承载
两重内容：`controller_manager` 实际加载的运行参数（`update_rate: 500`），以及两套
预设数值的人工参考记录（非运行时解析，避免引入额外 YAML 解析依赖）。

### 运行时造浪机切换（Runtime Wave-Maker Switching）

`AdhereMujocoHardware` 不再在启动时静态绑定水流环境——节点启动时恒定从**静水
（模式 `1`）**开始，`wave_mode_`（`std::atomic<int>`）与 `wave_trigger_time_`
（`std::atomic<double>`）允许从任意线程安全地热切换：

- **相对造浪时钟**：切换到模式 `2` 时，立即将 `t_trigger` 锁存为当前仿真时间
  `d->time`；之后水动力评估改用相对时间 `τ = t_sim - t_trigger >= 0` 查询
  `StationKeepingGustPlugin`，而不是原始仿真时间——这样无论用户在第几秒按下
  `2`，两段阵风都会从造浪开始的那一刻重新播放，不会被仿真已运行的时长"吃掉"。
- **切回模式 `1`** 时立即恢复零环境流速（`v_flow = 0`），不经过任何过渡。
- 通过话题 `/rov/wave_mode_cmd`（`std_msgs/msg/Int32`，`1` 或 `2`）下发指令，
  由 `adhere_mujoco_node` 转发给当前唯一存活的 `AdhereMujocoHardware` 实例
  （`AdhereMujocoHardware::activeInstance()`，与 `mjcb_passive` 单例回调复用
  同一条"单进程单实例"架构假设）。

### 交互式终端控制台 `wave_console_node`

```text
========================================================
  Underwater ROV Wave Environment Console (造浪机控制台)
========================================================
  [1] Calm Water Environment (静水环境 - 默认/停止造浪)
  [2] Wave Gust Scenario     (开启造浪: 0.3m/s横流 + 双阵风冲击)
  [q] Quit Console           (退出控制台)
--------------------------------------------------------
Current Mode: [1] Calm Water | Enter selection (1/2/q):
```

`mujoco_sim.launch.py` 默认会自动检测 `gnome-terminal`/`xterm` 并在独立终端窗口
中启动该控制台（`spawn_wave_console:=false` 可关闭自动弹窗）；也可以在任意终端
手动运行：

```bash
ros2 run adhere_mujoco_hardware wave_console_node
```

输入 `1` 切回静水、`2` 开启造浪、`q` 退出控制台（不影响仿真本身继续运行）。

## `MujocoVisBridge` 与 `MujocoViewer`：500Hz↔60Hz 双向解耦

`MujocoVisBridge` 是一个裸数据结构（`qpos`/`qvel`/`ctrl`/`pad_force`/
`flow_vel_world` + `std::mutex`），由 `AdhereMujocoHardware::write()` 每 8 步
（约 60Hz）用 `try_lock()` 写入一次快照；`MujocoViewer` 在独立线程中运行 GLFW/
OpenGL 渲染循环，拥有自己的 `mjData* d_vis_`（与主 `mjData*` 完全隔离），每帧
拷贝快照、调用 `mj_forward`（不调用 `mj_step`，因为渲染线程不拥有权威物理状态）
并渲染 ROV/壁面/4 个接触垫受力柱状图/环境流速箭头。`headless:=true` 时
`MujocoViewer` 跳过 GLFW 窗口创建，仅空转等待停止信号。

## `adhere_mujoco_node`：单进程宿主 + RViz2 同步

`adhere_mujoco_node` 在同一进程内：

1. 用传入的 `robot_description` 构造 `hardware_interface::ResourceManager` 并
   `start_components()`（触发 `AdhereMujocoHardware::configure()`/`start()`）；
2. 以该 `ResourceManager` 构造标准 `controller_manager::ControllerManager`，在专属
   线程里以 `500Hz` 循环调用 `read()`/`update()`/`write()`（本阶段无控制器挂载，
   `update()` 为空操作）；
3. 额外在 `100Hz` 定时器中，通过 `claim_state_interface()` 拿到的位姿/速度/接触力
   状态接口，发布 `tf2_ros::TransformBroadcaster`（`world -> base_link`）、
   `nav_msgs/msg/Odometry`（`/rov/odom`）、`sensor_msgs/msg/JointState`（`/joint_states`，
   因模型无可动关节而恒为空）、`visualization_msgs/msg/MarkerArray`
   （`/rov/environment_markers`：壁面 + 4 个接触垫受力柱状图），使 RViz2 与内嵌
   MuJoCo 窗口实时同步。

## 数值校验

用 3 组随机 `(com_pos_world, orientation, ang_vel_world, lin_vel_world, t)` 状态
分别调用 Python 参考实现（`mujoco_mppi_ws/src/env/hydrodynamics.py`，站位保持
阵风预设）与本包的 `FossenHydrodynamics::compute()`，输出的六维力/力矩在全部
打印精度（10 位小数）下逐位一致，满足 `CLAUDE.md` 要求的 `<1e-5` 数值一致性。

在默认静水模式（`1`）下以 `u=0` 运行：ROV 因初始姿态与壁面重叠而被接触反力
推离壁面，随后因净正浮力（约 `+1.96 N`）缓慢上浮至自由液面（`z_surface=1.0`）
附近并稳定，`Y` 方向无漂移。切换到造浪模式（`2`）后，`Y` 方向机体速度随定常
流速 `0.300 m/s`（`X`/`Z` 方向速度趋近于零）持续收敛，相对造浪时钟 `τ∈[3.0,4.2]s`
与 `[7.0,8.2]s` 的阵风窗口内可观察到额外的侧向/吸附扰动，均与预期物理行为一致；
再次切回模式 `1` 后环境流速立即归零。

## 运行

```bash
colcon build --packages-up-to adhere_mujoco_hardware --symlink-install
source install/setup.bash

# 默认：静水启动，带 MuJoCo 窗口 + RViz2，并自动弹出造浪机控制台终端
ros2 launch adhere_mujoco_hardware mujoco_sim.launch.py

# 无头模式（无 GLFW 窗口），不启动 RViz2，不自动弹出控制台终端
ros2 launch adhere_mujoco_hardware mujoco_sim.launch.py \
  headless:=true use_rviz:=false spawn_wave_console:=false

# 在任意终端手动启动造浪机控制台（若自动弹窗被关闭，或想在已运行的仿真上手动控制）
ros2 run adhere_mujoco_hardware wave_console_node

# 不使用控制台，直接发送一次性造浪切换指令
ros2 topic pub /rov/wave_mode_cmd std_msgs/msg/Int32 "{data: 2}" --once   # 开启造浪
ros2 topic pub /rov/wave_mode_cmd std_msgs/msg/Int32 "{data: 1}" --once   # 切回静水
```
