# `adhere_mujoco_hardware`

## 职责概述

`AdhereMujocoHardware` 是本工作空间中唯一直接持有 MuJoCo `mjModel*`/`mjData*` 的
类，扮演"真实 ROV + 水下环境"的角色。它是一个 ROS 2 Foxy
`hardware_interface::SystemInterface` 插件，由 `controller_manager` 以严格
**`500Hz`**（`update_rate: 500`，`dt = 2.0 ms`）周期驱动。本阶段（Phase 2）尚未
接入任何上层控制器，`hw_commands_[8]` 恒为零，用于验证"零控制输入下的被动随波
漂流"（calm 水域下的浮力/阻力漂移，以及造浪工况下的侧向/吸附漂移）。

环境水流不再是启动时静态绑定的，而是**运行时可热切换**的：节点启动时恒定从
静水（Calm Water）开始，用户可随时在内嵌 MuJoCo 窗口中按 `1`/`2` 键或点击左侧
边栏的 "Wave Environment" 单选按钮开启/关闭造浪（详见"运行时造浪机切换"一节）。

## 文件结构

```text
include/adhere_mujoco_hardware/
├── adhere_mujoco_hardware.hpp      # AdhereMujocoHardware 主类
├── fossen_hydrodynamics.hpp        # HydroParams / GustEvent / FossenHydrodynamics
├── flow_environment_plugin.hpp     # FlowEnvironmentPlugin 策略接口 + 2 个具体插件 + 工厂函数
├── mujoco_vis_bridge.hpp           # MujocoVisBridge：500Hz<->60Hz 双向快照/控制桥（mutex 快照 + 原子控制量）
└── mujoco_viewer.hpp               # MujocoViewer：GLFW/OpenGL 60Hz 交互式渲染线程（mjUI 双侧边栏）

src/
├── adhere_mujoco_hardware.cpp
├── fossen_hydrodynamics.cpp
├── flow_environment_plugin.cpp
├── mujoco_viewer.cpp
└── adhere_mujoco_node.cpp          # 500Hz ControllerManager 循环 + 100Hz TF/Odom/Marker 发布

config/
└── mujoco_sim.rviz                 # RViz2 显示配置（Odometry + EnvironmentMarkers，TF 坐标系缩放 0.12）

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
  `v_sep,i = v_x,b + omega_y*z_i - omega_z*y_i` 实时计算的分离速度，其中
  `(y_i, z_i)` 在 `configure()` 时从 `model_->site_pos[pad_*_site]` 查询得到，
  绝不硬编码）。
- **`write()`**：把 `hw_commands_[0:8]`（本阶段恒为零）与 `MujocoVisBridge::
  manual_ctrl[8]`（GUI 推力器滑条，叠加式）之和写入 `d_->ctrl`；通过一个仅在
  本进程内安装的 `mjcb_passive` 回调（与 Python 原型 `setup_hydrodynamics_
  callback` 完全相同的调用时机与目标累加数组 `qfrc_passive`，用于保证数值一致性）
  注入 `FossenHydrodynamics::compute()` 算出的水动力合力/合力矩 **加上**
  `MujocoViewer` 的 `Ctrl+拖拽` 交互扰动力/力矩；随后调用 `mj_step2` +
  `mj_step1`（零延迟两段式步进，见下方"零延迟两段式 `mj_step` 契约"）；每 8 步
  通过 `std::mutex::try_lock()`（绝不阻塞主循环）向 `MujocoVisBridge` 写入一次
  渲染快照。`write()` 开头还会消费 GUI 发出的复位 / 造浪模式切换请求。

### 零延迟两段式 `mj_step` 契约

单次调用整体式 `mj_step()` 会让 `xipos`/`xquat`/`cvel`/`sensordata`（接触力/
分离速度）比 `qpos`/`qvel` 落后整整一个 `2.0 ms` 周期才更新——`read()` 读到的
衍生量永远是"上一拍"的。本阶段改为 MuJoCo 官方推荐的两段式调用：

- `start()`：`mj_resetData` 后立即 `mj_step1`，为第一次 `read()` 预置好初始
  衍生量/传感器数据。
- `write()`：`mj_step2`（完成当前拍的积分）紧跟 `mj_step1`（为新状态重新计算
  衍生量/传感器），使 `read()` 读到的永远是零延迟的"当前拍"数据。
- `Backspace`（GUI 复位）走相同的 `mj_resetData` + `mj_step1` 序列。

## 水动力学与可插拔水流环境

`fossen_hydrodynamics.hpp/.cpp` 是 `~/Research/mujoco_mppi_ws/src/env/hydrodynamics.py`
的逐条等价 C++ 移植（附加质量重力偏置、Morison 耦合二次阻力、近壁 Venturi 吸附、
自由液面渐变、CoB 回正力矩），已通过 3 组随机状态与 Python 参考实现数值比对，
误差远小于 `1e-5`（详见下方"数值校验"）。

### 双模式水动力引擎：`HydroModelMode`

`FossenHydrodynamics::compute()` 支持两种互斥模式，由 `MujocoVisBridge::hydro_mode`
（`MujocoViewer` 左侧边栏 "Hydro Model" 单选，`Lumped`/`Stonefish-Mesh`）在运行时
切换，`AdhereMujocoHardware::write()` 每拍读取：

- **`kParametricLumped`**（默认）：上文描述的单点质心集中参数模型，与 Python
  参考逐位一致，作为永久保留的数值基准。
- **`kStonefishGeometryDriven`**（仿 *Stonefish* 仿真器的几何/网格驱动模型）：
  `HydroGeometryModel::buildFromMjModel()` 在 `configure()` 时自动遍历
  `rov_base` 自身的全部几何体（`mjGEOM_BOX`/`SPHERE`/`CYLINDER`/`CAPSULE`/
  `ELLIPSOID`/`MESH`），按面/经纬网格/三角面自动切分出一组表面微元
  `(centroid_body, normal_body, area)`，若原始微元数超过 256 个上限则按
  空间体素网格做面积加权合并（当前模型 96(箭体6面×4×4) + 4×48(垫球) = 288
  个原始微元会触发一次合并）。每拍 500Hz 对这组微元做：
  - 法向分布式二次压阻 + 切向蒙皮摩擦（`Cd_normal`/`Cf_tangent`）；
  - 仅对朝壁（法向 `x<-0.5`）微元按各自局部 `x` 坐标计算的近壁 Venturi 吸附 +
    挤膜阻尼（`C_venturi`/`C_squeeze`）；
  - 用 Lamb 椭球近似（由该模型的包围盒半轴 `bbox_half_extents` 通过数值积分
    得到各主轴附加质量/附加惯量系数）注入非定常 Froude-Krylov 力与 Munk
    回正力矩。
  - 浮力/重力偏置与自由液面渐变复用集中参数模型的同一组公式（整体效应，
    与微元分布无关）。

  这一路径对几何**完全无感知假设**：换成任意 CAD 三角网格（未来
  `full_cad_description` 包）后 `buildFromMjModel()` 会自动识别
  `mjGEOM_MESH` 并切分三角面，不需要改动任何水动力代码。单次 500Hz 评估
  （≤256 个微元的定长循环，无堆分配）远低于 `30us` 预算。

两种模式的开关、当前激活的微元数（`hydro_patch_count`）与 Rhombic Kern 遥测
都会实时显示在 `MujocoViewer` 的左上角浮层中。

`flow_environment_plugin.hpp/.cpp` 提供策略模式的可插拔水流环境：

| 插件 | 运行时模式 | 行为 |
|---|---|---|
| `CalmWaterPlugin`（默认/启动即激活） | `1` | 环境流速恒为零，仅受浮力/阻力/Venturi 吸附影响 |
| `StationKeepingGustPlugin` | `2` | 定常 `0.3 m/s` 世界系 `+Y` 侧向流 + 两段半正弦阵风（相对造浪时钟 `τ∈[3.0,4.2]s` 与 `[7.0,8.2]s`，侧向峰值 `0.9 m/s`，法向吸附峰值 `0.2 m/s`） |

两个插件的数值常量与 Python 参考逐位一致。两套水流预设的完整数值参考如下
（由 `flow_environment_plugin.cpp` 在 C++ 侧硬编码构造，本表仅作人工比对记录，
不被任何运行时代码解析）：

| 字段 | 单位 | Calm Water (`1`) | Station-Keeping Gust (`2`) |
|---|---|---|---|
| `current_steady_world` | m/s | `[0, 0, 0]` | `[0.0, 0.3, 0.0]` |
| `gust_events` | — | 无 | `t∈[3.0,4.2]s` 与 `[7.0,8.2]s`，峰值 `peak_y=0.9, peak_x=0.2` |

共用 `HydroParams`（两套预设均继承自静水默认值）：

| 字段 | 值 | 字段 | 值 |
|---|---|---|---|
| `rho_water` | `1000.0 kg/m^3` | `D_lin` | `[30.0, 20.0, 25.0]` |
| `gravity` | `9.81 m/s^2` | `D_quad` | `[140.0, 50.0, 70.0]` |
| `vehicle_mass` | `20.0 kg` | `D_ang_lin` | `[8.0, 8.0, 10.0]` |
| `augmented_mass` | `26.0 kg` | `D_ang_quad` | `[15.0, 15.0, 20.0]` |
| `displaced_volume` | `0.0202 m^3` | `C_venturi` | `0.45` |
| `cob_offset_body` | `[0, 0, 0.05] m` | `h_gap_nominal` | `0.025 m` |
| `z_surface` | `1.0 m` | `A_plate` | `0.24 m^2` |
| `z_transition` | `0.4 m` | `h0_decay` | `0.05 m` |
| — | — | `wall_x_offset` | `0.1 m` |

`controller_manager` 运行参数（`update_rate: 500`）不通过 YAML 文件加载，而是由
`adhere_mujoco_node` 内部的 `physics_thread`（`std::chrono::steady_clock::sleep_until`
严格 `2.0 ms` 周期）直接保证，详见下方"500Hz 实时时序"一节。该周期还会按
`MujocoVisBridge::sim_speed_factor`（`<=1.0`，由 `MujocoViewer` 左侧边栏
"Sim Speed" 单选控制，`1x/0.5x/0.2x/0.1x`）成倍拉长，用于观察毫秒级接触瞬态
而不改变 `mjModel::opt.timestep` 本身，因此不影响任何数值一致性约束。

## 统一几何约定与 Rhombic Kern

4 个接触垫站点 `pad_TL/TR/BL/BR` 的机体系坐标在 `AdhereMujocoHardware::configure()`
时从 `model_->site_pos` 一次性解析（`pad_yz_`、`MujocoVisBridge::pad_site_pos_body`），
是整条代码路径——`read()` 的分离速度公式、`write()` 的 Rhombic Kern 遥测、
`adhere_mujoco_node` 的受力箭头标记、`MujocoViewer` 的 Kern 菱形可视化——唯一
共用的数据来源，绝不在多处各自硬编码。对当前 `rov_wall.xml`，水平半宽
`L_y = max(|y_i|) = 0.25m`，竖直半高 `L_z = max(|z_i|) = 0.15m`（横向更宽、
纵向更窄，与机体 box 几何 `0.2 x 0.6 x 0.4m` 一致）；若 MJCF 几何调整，`L_y`/`L_z`
会自动跟随变化，无需修改任何 C++ 常量。

Rhombic Kern 可行性判据按 `CLAUDE.md` §4.4：

```
p_y = sum(F_Ni * y_i) / sum(F_Ni),  p_z = sum(F_Ni * z_i) / sum(F_Ni)
K_val = |p_y| / L_y + |p_z| / L_z,  M_kern = 1 - K_val
```

`write()` 每次快照 tick 都会算出 `kern_val`/`m_kern`/`liftoff_ratio`（4 个垫中
法向力 `<=0` 的比例）/`cop_y`/`cop_z`，写入 `MujocoVisBridge` 供
`MujocoViewer` 的实时浮层、Kern 菱形可视化（见下文）与未来的 OSQP 控制器
直接复用。

### 运行时造浪机切换（Runtime Wave-Maker Switching）

`AdhereMujocoHardware` 不再在启动时静态绑定水流环境——节点启动时恒定从**静水
（模式 `1`）**开始，`wave_mode_`（`std::atomic<int>`）与 `wave_trigger_time_`
（`std::atomic<double>`）允许从任意线程安全地热切换：

- **相对造浪时钟**：切换到模式 `2` 时，立即将 `t_trigger` 锁存为当前仿真时间
  `d->time`；之后水动力评估改用相对时间 `τ = t_sim - t_trigger >= 0` 查询
  `StationKeepingGustPlugin`，而不是原始仿真时间——这样无论用户在第几秒按下
  `2`，两段阵风都会从造浪开始的那一刻重新播放，不会被仿真已运行的时长"吃掉"。
- **切回模式 `1`** 时立即恢复零环境流速（`v_flow = 0`），不经过任何过渡。
- 两个等价的指令入口，都会调用同一个 `AdhereMujocoHardware::setWaveMode(int)`：
  - **`MujocoViewer`**（主要入口）：按 `1`/`2` 键或点击左侧边栏 "Wave Environment"
    单选按钮，经 `MujocoVisBridge::wave_mode_cmd`（边沿触发的原子请求槽位）
    直接送达 `write()`，无需经过 ROS 话题/DDS。
  - 话题 `/rov/wave_mode_cmd`（`std_msgs/msg/Int32`，`1` 或 `2`）：供脚本化/
    远程场景下发指令，由 `adhere_mujoco_node` 转发给当前唯一存活的
    `AdhereMujocoHardware` 实例（`AdhereMujocoHardware::activeInstance()`，
    与 `mjcb_passive` 单例回调复用同一条"单进程单实例"架构假设），例如：
    ```bash
    ros2 topic pub /rov/wave_mode_cmd std_msgs/msg/Int32 "{data: 2}" --once   # 开启造浪
    ros2 topic pub /rov/wave_mode_cmd std_msgs/msg/Int32 "{data: 1}" --once   # 切回静水
    ```

## `MujocoVisBridge` 与 `MujocoViewer`：500Hz↔60Hz 双向解耦 + 完整交互式 GUI

`MujocoVisBridge` 是 500Hz(`AdhereMujocoHardware`) <-> 60Hz(`MujocoViewer`) 的
双向桥，两类字段各自用不同的同步策略：

- **硬件 -> GUI 的批量快照**（`qpos`/`qvel`/`ctrl`/`pad_force`/`pad_v_sep`/
  `flow_vel_world`）：由 `write()` 每 8 步（约 60Hz）用 `std::mutex::try_lock()`
  写入，绝不阻塞 500Hz 主循环。
- **GUI -> 硬件的控制请求**（`paused`/`reset_request`/`wave_mode_cmd`/
  `manual_ctrl[8]`/`xfrc_perturb[6]`）与**硬件 -> GUI 的标量遥测**
  （`actual_physics_hz`/`sim_time`/`wave_mode_active`/`wave_tau`）：全部是
  `std::atomic`，任意线程可无锁读写。

### `MujocoViewer`：仿照官方 `simulate` 的完整交互式窗口

`MujocoViewer` 在独立线程中运行 GLFW/OpenGL 渲染循环，拥有自己的
`mjData* d_vis_`（与主 `mjData*` 完全隔离），每帧拷贝快照、调用 `mj_forward`
（不调用 `mj_step`，因为渲染线程不拥有权威物理状态）。`headless:=true` 时跳过
GLFW 窗口创建，仅空转等待停止信号。

### 鼠标与键盘操作一览

**鼠标（中央 3D 视口内）**：

| 操作 | 效果 |
|---|---|
| 左键拖拽 | 相机环绕（orbit） |
| 右键拖拽 | 相机平移（pan） |
| 滚轮 | 缩放（zoom） |
| 左键双击 | 选中物体（`mjv_select`）并初始化 `mjvPerturb` |
| `Ctrl` + 左键拖拽 | 对已选中物体施加外部力矩（torque） |
| `Ctrl` + 右键拖拽 | 对已选中物体施加外部力（force） |

`Ctrl` 拖拽产生的力/力矩会实时写回 `MujocoVisBridge::xfrc_perturb`，并在
`AdhereMujocoHardware::applyHydrodynamics()` 中与水动力合并注入
`qfrc_passive`——松开鼠标按键后自动归零。左右两侧边栏内的点击/拖拽（滑条、
复选框、单选按钮、按钮）与 3D 视口的相机/扰动操作互不干扰，由
`MujocoViewer` 按点击位置落在哪个 `mjrRect` 自动路由。

**键盘（任意焦点下）**：

| 按键 | 效果 |
|---|---|
| `Space` | 运行/暂停切换 |
| `S` | 单步执行一个 `2.0 ms` 物理 tick（若当前未暂停，会先自动暂停，再前进一拍——与左侧边栏
  "Step Once" 按钮等价，适合逐帧调试接触力建立/碰撞响应过程） |
| `Backspace` | 复位仿真（`mj_resetData` + `mj_step1`，回到初始状态） |
| `1` / `2` | 切换静水 / 造浪模式（与左侧边栏 "Wave Environment" 单选按钮等价） |
| `Tab` | 隐藏/显示左侧边栏（腾出视口空间） |

内容已尽量对齐官方 `simulate` 应用暴露的可视化/调试开关，并拆分到左右两侧，
避免单侧内容超过一屏高度（`mjui_resize()` 在内容过高时的滚动行为不可靠，详见
`mujoco_viewer.hpp` 中 `ui0_scroll_`/`ui1_scroll_` 的注释）。除标注"自定义"外，
其余控件均直接绑定到 `mjvOption opt_`/`mjvScene scn_` 内部字段（如
`&opt_.flags[mjVIS_ACTUATOR]`、`&scn_.flags[mjRND_SHADOW]`、`&opt_.frame`、
`&opt_.label`、`&opt_.geomgroup[i]`），不经过额外镜像变量，`mjv_defaultOption()`/
`mjv_defaultScene()` 已为它们设好 MuJoCo 自身的默认值。

**左侧边栏（`ui0`）**：

| 分组 | 控件 | 说明 |
|---|---|---|
| Simulation | `Running` 复选框 | 与 `Space` 等价 |
| | `Step Once` 按钮 | 与 `S` 等价 |
| | `Reset` 按钮 | 与 `Backspace` 等价 |
| | `Sim Speed` 单选 | `1x/0.5x/0.2x/0.1x`，拉长 500Hz 物理线程的墙钟周期做慢动作观察，不改变 `mjModel::opt.timestep` |
| | `Default Orbit`/`Wall YZ View`/`Top XY View`/`Side XZ View` 按钮 | 4 个预设相机视角（环绕默认视角 / 正对壁面 Y-Z 平面 / 俯视 X-Y 平面 / 侧视 X-Z 平面） |
| | `Camera Follow` 复选框 | 勾选后每帧将相机 `lookat` 锁定到 ROV 质心，可与任意预设/手动环绕叠加使用 |
| | `Reset Camera View` 按钮 | 相机环绕/拖拽"迷路"后一键恢复默认视角（`distance=2.8, azimuth=-135°, elevation=-20°`） |
| | `Screenshot` 按钮 | 将当前整个窗口帧保存为 `/tmp/adhere_mujoco_screenshot_<ms>.ppm`（PPM 格式，GIMP/大多数看图软件可直接打开；若需要 PNG 可用 `convert shot.ppm shot.png` 转换），终端会打印保存路径 |
| Wave & Hydro Model | `Mode` 单选 | Calm Water / Station-Keeping Gust，与 `1`/`2` 键等价 |
| | `Hydro Model` 单选 | `Lumped`（集中参数）/ `Stonefish-Mesh`（几何驱动）水动力模型切换，见上文"双模式水动力引擎" |
| | `Kern Diamond` 复选框 | 壁面上的 Rhombic Kern 菱形 + 实时压力中心（CoP）标记开关，颜色随 `K_val`/是否脱离壁面变化（绿/黄/红） |
| ROV Overlays（自定义） | `Pad Contact Forces` | 4 个接触垫受力柱状图开关（绿色胶囊，长度正比于法向力） |
| | `Flow Velocity Arrow` | 环境流速箭头开关（蓝色，从机体质心指向流速方向） |
| Visualize（默认折叠） | `Transparent Hull` (`mjVIS_TRANSPARENT`) | 船体半透明——**可直接看到内部的接触垫/惯量箱/质心标记** |
| | `Joint`/`Actuator`/`Activation` | 关节、执行器、激活状态的原生可视化标记 |
| | `Camera`/`Light` | 相机/光源位置标记 |
| | `Constraint` | 等式约束标记（本模型无等式约束，预留） |
| | `Contact Point`/`Contact Force`/`Contact Split` | MuJoCo 原生接触点 + 接触力箭头（可拆分法向/切向分量） |
| | `Perturb Force`/`Perturb Object` | `Ctrl`+拖拽扰动时的力箭头/高亮物体渲染 |
| | `Center of Mass` | 显示每个刚体的质心标记（验证浮力/重力作用点） |
| | `Select Point` | 双击选中后的选中点标记 |
| | `Static Body` | 以不同样式标出静态刚体（如壁面） |
| | `Inertia Box`/`Scale Inertia w/ Mass` | 等效惯量长方体（核对 `diaginertia`），后者按质量缩放箱体视觉大小 |
| | `Auto Connect` | 关节-刚体连接引导线 |
| | `Coord Frames` | None/Body/Geom/Site/Camera/Light/Contact/World 八态单选——标记长度/线宽已
  整体缩小至 MuJoCo 默认值的 `1/50`（见下方"坐标系缩放"一节） |

**右侧边栏（`ui1`）**：

| 分组 | 控件 | 说明 |
|---|---|---|
| Thrusters (Manual)（自定义） | `Zero All` 按钮 | 一键将全部 8 个滑条归零 |
| | `thr_N1..4` / `thr_T5..8` 滑条 | `[-40, 40] N`，叠加写入
  `MujocoVisBridge::manual_ctrl`（与上层控制器指令相加），可在未挂载控制器时
  手动测试单个推力器——例如把 4 个 `thr_N*` 拉到正值可将 ROV 压向壁面，
  用于验证接触力/`Pad Contact Forces` 可视化是否随指令动态响应 |
| Live Scope（自定义） | `Scope Plot` 单选 | `Off` / `Pad Forces F_N` / `Kern Margin & minF_N` /
  `Body Vel vs Flow` / `Thruster Commands`——在视口右下角嵌入一个 `mjvFigure` 实时滚动
  示波器（约 400 帧滚动历史窗口），对应曲线由每帧的 `cached_*` 快照驱动 |
| Render Flags（默认折叠） | `Wireframe`/`Shadow`/`Reflection`/`Skybox`/`Fog`/`Haze` | 场景级渲染效果开关（`mjtRndFlag`），纯视觉，不影响物理 |
| Label（默认折叠） | `Mode` 单选 | None/Body/Joint/Geom/Site/Camera/Light/Tendon/Actuator/Constraint/
  Flex/Skin/Selection/SelPoint/ContactPt/ContactForce/Island——在 3D 视口内
  叠加对应类型的文字标签 |
| Visibility Groups（默认折叠） | `Geom Group 0`/`Site Group 0` | 整体隐藏/显示所有几何体/站点（`rov_wall.xml`
  目前所有元素都在默认组 `0`，故只有这一组有实际效果；`opt_.geomgroup[1..5]`/
  `sitegroup[1..5]` 仍可通过代码按需启用，这里暂不在 UI 上重复暴露） |
| Telemetry（自定义） | （说明性条目，指向视口左上角的实时数值浮层） | 浮层内容：物理频率（Hz）、
  仿真时间、造浪模式与 `τ`、环境流速、4 个接触垫受力、ROV 位姿
  `(X,Y,Z,φ,θ,ψ)`、机体系线速度 `(vx,vy,vz)`（已按当前姿态旋转到机体系）、
  机体角速度 `(ωx,ωy,ωz)`、当前水动力模型与微元数、Rhombic Kern 裕度
  `M_kern`、脱离壁面比例 |

`Visualize`/`Render Flags`/`Label`/`Visibility Groups` 四个原生 MuJoCo 开关组
默认折叠（点击分组标题即可展开），为新增的 `Sim Speed`/相机预设/`Hydro Model`/
`Live Scope` 等常用控件腾出屏幕高度，避免任一侧边栏默认展开内容超过一屏。

### 坐标系（Coord Frames）缩放在哪里调

`Coord Frames`/`Label` 等标记的长度/线宽不是 UI 可调参数，而是
`mjModel::vis.scale.framelength`/`framewidth` 这两个模型级字段，在
`AdhereMujocoHardware::configure()`（`adhere_mujoco_hardware.cpp`，紧跟在
`mj_loadXML` 成功之后）被覆写为 `0.02`/`0.002`（MuJoCo 默认值为 `1.0`/`0.1`，
相对这个 `~0.2-0.6m` 尺度的 ROV 明显过大）。这是运行时对已加载 `mjModel` 的
覆写，而不是编辑 `mjcf/rov_wall.xml`（该文件按约定与只读参考仓库保持字节
一致）；需要进一步调整时直接改这两行代码即可，不涉及 MJCF 或任何 UI 控件。

## `adhere_mujoco_node`：单进程宿主 + RViz2 同步

`adhere_mujoco_node` 在同一进程内：

1. 构造 `hardware_interface::ResourceManager`（其构造函数即解析 `robot_description`
   并运行 `AdhereMujocoHardware::configure()`）；随后通过
   `AdhereMujocoHardware::activeInstance()` 取得这个唯一实例，调用
   `setHeadless()` 传入 `headless` 节点参数（`ros2_control` 的 `<hardware>` 标签
   不再携带 `headless`，见下方"纯 Xacro 解耦"），最后才调用
   `resource_manager->start_components()`（触发 `start()`，此时才真正按
   `headless_` 构造 `MujocoViewer`）；
2. 以该 `ResourceManager` 构造标准 `controller_manager::ControllerManager`，在专属
   线程里以严格 `500Hz` 循环调用 `read()`/`update()`/`write()`（本阶段无控制器
   挂载，`update()` 为空操作）。该线程用 `std::chrono::steady_clock::sleep_until`
   而非 `rclcpp::Rate`，并对每拍耗时做指数滑动平均写入
   `MujocoVisBridge::actual_physics_hz`，用于证明 `MujocoViewer`（60Hz GL）与
   RViz2（100Hz 话题）都不会拖慢物理循环；
3. 额外在 `100Hz` 定时器中，从 `AdhereMujocoHardware::visBridge()` 的快照（而非
   `claim_state_interface()`，后者与 `controller_manager` 所属的 `ResourceManager`
   跨线程竞争同一资源是不安全的）读取位姿/速度/接触力/环境流速，发布
   `tf2_ros::TransformBroadcaster`（`world -> base_link`）、
   `nav_msgs/msg/Odometry`（`/rov/odom`）、`sensor_msgs/msg/JointState`（`/joint_states`，
   因模型无可动关节而恒为空）、`visualization_msgs/msg/MarkerArray`
   （`/rov/environment_markers`：壁面 + 4 个接触垫受力柱状图 + 环境流速箭头，
   后者与 `MujocoViewer` 自己画的箭头保持同一份数据来源），使 RViz2 与内嵌
   MuJoCo 窗口实时同步；
4. `robot_state_publisher` 作为独立节点与 `adhere_mujoco_node` 一起由
   `mujoco_sim.launch.py` 启动，发布 `/robot_description` 与完整静态 TF 树
   （`base_link -> pad_*`/`thr_*`）。

### 纯 Xacro 解耦（Pure-Xacro Decoupling）

`simplified_v1.urdf.xacro`/`ros2_control.xacro` 不再携带 `headless` 这个
与"是否弹出仿真器窗口"相关的仿真参数——机器人描述文件必须 100% 与仿真器 GUI
是否启动解耦。`headless` 现在是 `adhere_mujoco_node` 的标准 ROS 2 节点参数，由
`mujoco_sim.launch.py` 传入，经 `AdhereMujocoHardware::setHeadless()` 在
`configure()` 之后、`start()` 之前（也就是 `MujocoViewer` 真正被构造之前）
到达硬件插件。

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

# 默认：静水启动，带 MuJoCo 窗口 + RViz2；在 MuJoCo 窗口中按 1/2 切换造浪
ros2 launch adhere_mujoco_hardware mujoco_sim.launch.py

# 无头模式（无 GLFW 窗口），不启动 RViz2
ros2 launch adhere_mujoco_hardware mujoco_sim.launch.py headless:=true use_rviz:=false

# 无 GUI 窗口时，用话题一次性切换造浪（例如配合 headless:=true 做脚本化测试）
ros2 topic pub /rov/wave_mode_cmd std_msgs/msg/Int32 "{data: 2}" --once   # 开启造浪
ros2 topic pub /rov/wave_mode_cmd std_msgs/msg/Int32 "{data: 1}" --once   # 切回静水
```
