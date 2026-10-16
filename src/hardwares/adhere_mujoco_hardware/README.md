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
  manual_ctrl[8]`（GUI 推力器滑条，叠加式）之和**先钳位到 `±40.0 N` 硬件轨道
  极限**（`CLAUDE.md` §4.3），再可选经过一阶 ESC 惯性延迟滤波
  （`tau_T = 0.015s`，由右侧边栏 "Thruster Lag (15ms)" 复选框 /
  `MujocoVisBridge::enable_thruster_lag` 开关，默认关闭=直通），最终写入
  `d_->ctrl`；通过一个仅在本进程内安装的 `mjcb_passive` 回调（与 Python 原型
  `setup_hydrodynamics_callback` 完全相同的调用时机与目标累加数组
  `qfrc_passive`，用于保证数值一致性）注入 `FossenHydrodynamics::compute()`
  算出的水动力合力/合力矩 **加上** `MujocoViewer` 的 `Ctrl+拖拽` 交互扰动
  力/力矩；随后调用 `mj_step2` + `mj_step1`（零延迟两段式步进，见下方"零延迟
  两段式 `mj_step` 契约"）。**每一次真实物理步（而非每 8 步的渲染快照节流）**
  都会重新计算接触垫法向力、Rhombic Kern 判据与脱离/角三角陷阱分类，累加进
  500Hz 真实时基的统计计数器（见下文"Rhombic Kern 遥测"），确保瞬态 `<5ms`
  的接触事件不会被 60Hz 快照节流漏采；每 8 步再通过 `std::mutex::try_lock()`
  （绝不阻塞主循环）向 `MujocoVisBridge` 写入一次批量渲染快照（`qpos`/`qvel`/
  `ctrl`/`pad_force` 等）。`write()` 开头还会消费 GUI 发出的复位 / 统计复位 /
  造浪模式切换请求。

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
  `rov_base` 自身的几何体（`mjGEOM_BOX`/`SPHERE`/`CYLINDER`/`CAPSULE`/
  `ELLIPSOID`/`MESH`），按面/经纬网格/三角面自动切分出一组表面微元
  `(centroid_body, normal_body, area)`。4 个微小的 `pad_*` 接触垫球体
  **不参与**这一流体表面切分（它们已经由 MuJoCo 自身的 Kelvin-Voigt 接触
  求解器单独建模，详见 `CLAUDE.md` §4.2）——把它们混入水动力外壳只会污染
  切分预算并通过 `decimatePatches()` 的体素合并污染箭体正面法向/面积。排除
  后当前模型恰好是箭体 6 面 × 4×4 = **96** 个原始微元，不超过 256 上限、
  无需任何有损合并，正面贴壁板（`x=-0.10m`）精确对应 **16 个微元、总面积
  严格等于 `A_plate=0.24 m^2`、法向严格为 `[-1,0,0]^T`**（由单元测试
  `FrontPlateAreaAndNormalsExact` 锁定）。若未来几何改动导致微元数仍然超过
  256，才会触发"空间体素 × 主导法向方向（6 类：`+X/-X/+Y/-Y/+Z/-Z`）"联合键
  面积加权合并（法向方向参与分桶键，防止闭合曲面法向抵消导致错误合并）。
  每拍 500Hz 对这组微元 `p` 做：

  记微元在机体系的相对流速 `v_p^B = v_rel,CoM^B + ω^B × r_p^B`，法向分量
  `v_n = v_p^B · n_p`，切向分量 `v_t^B = v_p^B - v_n n_p`：

  - **迎流侧单面法向压阻**（钝体 `Cd_normal` 本就定义在单面迎流投影面积上，
    迎流+背风双面施加会计重两次）：
    `F_drag,n = -0.5 ρ Cd_normal A_p v_n^2 n_p`（仅当 `v_n > 0`，背风侧为零）；
  - **切向蒙皮摩擦**（双向）：`F_skin,t = -0.5 ρ Cf_tangent A_p |v_t^B| v_t^B`；
  - **朝壁微元**（法向 `n_p·x̂ < -0.5`）的近壁 Venturi 吸附 + 挤膜阻尼：取该
    微元自身世界系 `x` 坐标 `x_world,p`（微元质心本就位于箭体外表面，**不再
    二次减去** `wall_x_offset`）得到间隙 `h_gap,p = max(x_world,p, 0.005)`，
    衰减因子 `decay = exp(-max(0, h_gap,p - h_gap_nominal) / h0_decay)`：
    ```
    F_suction,n = +0.5 ρ C_venturi A_p (v_t,y^2 + v_t,z^2) decay n_p   (吸附，指向壁面)
    F_squeeze,n = -C_squeeze A_p (h_gap_nominal / h_gap,p)^3 v_n decay n_p  (阻尼，双向负反馈)
    ```
    `n_p ≈ [-1,0,0]`：吸附力恒为 `-X`（吸向壁面）；挤膜阻尼在靠近壁面
    （`v_n>0`）时为 `+X`（阻碍靠近），远离壁面（`v_n<0`）时为 `-X`
    （阻碍远离），两种情形都是负反馈阻尼而非正反馈。`(h_gap_nominal/h_gap,p)^3`
    是无量纲比值，名义间隙处恒为 `1.0`；配套系数 `C_squeeze = 64.0 N·s/m`
    （而非早期未归一化版本遗留的 `1e-6`），使名义间隙处单板总阻尼
    `C_squeeze * A_plate = 15.36 N·s/m` 落在物理量级，而不是约 `2.4e-8 N`
    的近似于零的力（单元测试 `StonefishVenturiAndSqueezePhysicalSigns` 锁定
    靠近/远离壁面时的阻尼力符号与 `>0.5N` 量级）。
  - **整体线性+二次角阻尼**（弥补纯二次微元阻力在 `v_rel=0` 处导数为零、定点
    悬停时缺低速阻尼刚度的问题；二次角阻尼项与集中参数模型的
    `tau_drag_body` 完全对称，因为微元杠杆臂很小（`r_p~0.15m => r_p^3~
    0.0034 m^3`），仅靠微元法向阻力本身量级上低估钝体旋转涡脱阻尼一个数量级）：
    `F_lin = -D_lin ⊙ v_rel,CoM^B`，
    `T_lin = -D_ang,lin ⊙ ω^B - D_ang,quad ⊙ (ω^B ⊙ |ω^B|)`；
  - **非定常 Froude-Krylov + 附加质量反力**（环境流体加速度 `a_fluid^B`，来自
    半正弦阵风的解析导数）：
    `F_FK+AM = (ρ V_disp I_3 + diag(M_A,lin)) a_fluid^B`；
  - **Munk 回正力矩**（经典理想流体附加质量各向异性耦合，含平动与转动两项）：
    `T_Munk = -v_rel,CoM^B × (M_A,lin ⊙ v_rel,CoM^B) - ω^B × (M_A,ang ⊙ ω^B)`；
  - 用 Lamb 椭球近似（由该模型的包围盒半轴 `bbox_half_extents` 通过数值积分
    得到各主轴附加质量/附加惯量系数 `M_A,lin`/`M_A,ang`，仅在 `buildFromMjModel()`
    时算一次）为上述两项提供系数；
  - 浮力/重力偏置与自由液面渐变复用集中参数模型的同一组公式（整体效应，
    与微元分布无关）。

  `compute()` 还接受一个可选输出指针 `HydroWrenchBreakdown * breakdown`
  （默认 `nullptr`，不影响任何既有调用点），逐项回填上述压阻+摩擦+线性
  阻尼（`f_drag_body`）、Venturi 吸附（`f_venturi_body`）、挤膜阻尼
  （`f_squeeze_body`）、非定常 Froude-Krylov+附加质量（`f_fk_am_body`）、
  浮力（`f_buoy_world`）、Munk 力矩（`t_munk_body`）六项分量（外加一个
  耗散性旋转阻尼分量 `t_drag_body`，供 Tier 4 被动性校验与实时
  `Hydro Breakdown (N)` 示波器使用），`AdhereMujocoHardware::
  applyHydrodynamics()` 每拍 500Hz 调用一次并写入
  `MujocoVisBridge::hydro_f_*` 原子量供 `MujocoViewer` 的
  `Live Scope → Hydro Breakdown (N)` 选项实时绘制（见下方"`MujocoVisBridge`
  与 `MujocoViewer`"一节的控件表）。

  这一路径对几何**完全无感知假设**：换成任意 CAD 三角网格（未来
  `full_cad_description` 包）后 `buildFromMjModel()` 会自动识别
  `mjGEOM_MESH` 并切分三角面，不需要改动任何水动力代码。重网格化/体素合并/
  Lamb 椭球积分只在 `configure()` 时跑一次（实测约 `1.7ms`），500Hz 热路径
  只剩 `≤256` 个微元的定长循环（无堆分配）：实测单次 `compute()` 调用约
  `4.2us`（66 个微元），远低于 `2000us`/拍（500Hz）预算。

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

`write()` 在**每一次真实物理步**（而非每 8 步的渲染快照节流）都会算出
`kern_val`/`m_kern`/`cop_y`/`cop_z`，并把该拍的脱离（`F_min<=1e-3N`）/角三角
陷阱（`ΣF_N,i>1e-3N` 且 `|p_y|<=L_y`、`|p_z|<=L_z` 且 `K_val>1.0`）判定结果
累加进 `total_eval_steps_`/`liftoff_steps_`/`liftoff_events_`（上升沿计数）/
`corner_trap_steps_` 计数器，换算成累积比例写入 `MujocoVisBridge::
liftoff_ratio`/`corner_trap_ratio`/`liftoff_events`/`active_pads`——这是
`CLAUDE.md` §4 基准表要求的"500Hz 累积脱离时间比例/离散脱离事件数/角三角
陷阱时间比例"，不会漏采任何瞬态 `<5ms` 接触事件。供 `MujocoViewer` 的实时
浮层、Kern 菱形/CoP 可视化（见下文）与未来的 OSQP 控制器直接复用；
`Backspace`（完整复位）或 `Reset Stats` 按钮均会清零这些累积计数器。

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
| 左键拖拽 | 相机环绕（orbit），默认竖直变体（上下拖拽调俯仰） |
| 左键拖拽 + `Shift` | 相机环绕，水平变体（左右拖拽调方位角） |
| 右键拖拽 | 相机平移（pan），默认竖直变体（上下拖拽沿世界 `Z` 轴平移，方便沿壁面上下移动视角） |
| 右键拖拽 + `Shift` | 相机平移，水平变体 |
| 滚轮 | 缩放（zoom） |
| 左键双击 | 选中物体（`mjv_select`）并初始化 `mjvPerturb` |
| `Ctrl` + 左键拖拽 | 对已选中物体施加外部力矩（torque），默认竖直变体 `mjMOUSE_ROTATE_V` |
| `Ctrl` + 右键拖拽 | 对已选中物体施加外部力（force），默认竖直变体 `mjMOUSE_MOVE_V` |
| `Ctrl` + `Shift` + 拖拽 | 同上，但切换为水平变体（`mjMOUSE_ROTATE_H`/`mjMOUSE_MOVE_H`） |

默认竖直变体让用户可以直接沿世界 `Z` 轴在壁面上下拖拽/旋转 ROV（最常用的
贴壁位姿调试场景），按住 `Shift` 切到水平变体用于其他方向的调试。每次
`Ctrl`+点击都会重新调用 `mjv_initPerturb()` 用 ROV 当前位姿重新播种扰动基准
（而非复用上一次双击选中时的旧位姿），避免 ROV 已经移动后拖拽方向错位。

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
| | `Reset Stats` 按钮 | 仅清零 500Hz 累积统计计数器（脱离/角三角陷阱累积时间比例、离散脱离事件数），不影响 ROV 位姿/速度，适合在不重置物理状态的情况下重新开始一轮基准统计 |
| | `Sim Speed` 单选 | `1x/0.5x/0.2x/0.1x`，拉长 500Hz 物理线程的墙钟周期做慢动作观察，不改变 `mjModel::opt.timestep` |
| | `Default Orbit`/`Wall YZ View`/`Top XY View`/`Side XZ View` 按钮 | 4 个预设相机视角（环绕默认视角 / 正对壁面 Y-Z 平面 / 俯视 X-Y 平面 / 侧视 X-Z 平面） |
| | `Camera Follow` 复选框 | 勾选后每帧将相机 `lookat` 锁定到 ROV 质心，可与任意预设/手动环绕叠加使用 |
| | `Reset Camera View` 按钮 | 相机环绕/拖拽"迷路"后一键恢复默认视角（`distance=2.8, azimuth=-135°, elevation=-20°`） |
| | `Screenshot` 按钮 | 将当前整个窗口帧保存为 `/tmp/adhere_mujoco_screenshot_<ms>.ppm`（PPM 格式，GIMP/大多数看图软件可直接打开；若需要 PNG 可用 `convert shot.ppm shot.png` 转换），终端会打印保存路径 |
| Wave & Hydro Model | `Mode` 单选 | Calm Water / Station-Keeping Gust，与 `1`/`2` 键等价 |
| | `Hydro Model` 单选 | `Lumped`（集中参数）/ `Stonefish-Mesh`（几何驱动）水动力模型切换，见上文"双模式水动力引擎" |
| | `Kern Diamond` 复选框 | 壁面上的灰色支撑矩形（`±L_y × ±L_z`，角三角陷阱区边界）+ **固定青色**（`{0.1,0.85,1.0}`）Rhombic Kern 菱形边界 + 实时压力中心（CoP）标记开关；菱形边界颜色恒定不变，CoP 球颜色随可行性状态变化——**绿**（4 垫均接触、`K_val<=1.0`）/**黄**（4 垫均接触但 `K_val>1.0`，角三角陷阱）/**红**（任意一垫实际脱离）；完全脱壁（`ΣF_N,i<=1e-3N`）时不绘制 CoP 球（无物理意义） |
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
| | `Thruster Lag (15ms)` 复选框 | 默认关闭（直通，钳位后立即写入 `ctrl`）；勾选后对钳位后的目标推力额外叠加一阶 ESC 惯性延迟滤波（`tau_T=0.015s`，`alpha_T = 1-exp(-dt/tau_T)`），仅在物理确实前进一拍时积分，暂停时保持上次值不变 |
| | `thr_N1..4` / `thr_T5..8` 滑条 | `[-40, 40] N`，叠加写入
  `MujocoVisBridge::manual_ctrl`（与上层控制器指令相加）后统一钳位到
  `±40.0 N` 硬件轨道极限，可在未挂载控制器时手动测试单个推力器——例如把 4 个
  `thr_N*` 拉到正值可将 ROV 压向壁面，用于验证接触力/`Pad Contact Forces`
  可视化是否随指令动态响应 |
| Live Scope（自定义） | `Scope Plot` 单选 | `Off` / `Pad Forces F_N` / `Kern Margin & minF_N` /
  `Body Vel vs Flow` / `Thruster Commands` / `Hydro Breakdown (N)`——在视口右下角嵌入一个
  `mjvFigure` 实时滚动示波器（约 400 帧滚动历史窗口），对应曲线由每帧的 `cached_*` 快照
  （或 `Hydro Breakdown` 通道的 `MujocoVisBridge::hydro_f_*` 原子量）驱动；切换通道会
  清空历史缓冲，避免旧量级（如 `40N` 推力器）残留压扁新通道（如 `0.3m/s` 速度）的
  自动量程。`Hydro Breakdown (N)` 通道绘制 `FossenHydrodynamics::compute()` 返回的
  `HydroWrenchBreakdown` 四条代表性分量：`F_drag_y`（压阻+摩擦+线性阻尼）、
  `F_venturi_x`（近壁 Venturi 吸附）、`F_squeeze_x`（近壁挤压膜阻尼）、`F_fk_am_y`
  （非定常 Froude-Krylov + 附加质量反力），均在 `AdhereMujocoHardware::
  applyHydrodynamics()` 内随 500Hz 水动力求解同步写入，可直接读出每一项水动力效应
  当前的真实牛顿量级，便于逐项核验/调参 |
| Render Flags（默认折叠） | `Wireframe`/`Shadow`/`Reflection`/`Skybox`/`Fog`/`Haze` | 场景级渲染效果开关（`mjtRndFlag`），纯视觉，不影响物理 |
| Label（默认折叠） | `Mode` 单选 | None/Body/Joint/Geom/Site/Camera/Light/Tendon/Actuator/Constraint/
  Flex/Skin/Selection/SelPoint/ContactPt/ContactForce/Island——在 3D 视口内
  叠加对应类型的文字标签 |
| Visibility Groups（默认折叠） | `Geom Group 0`/`Site Group 0` | 整体隐藏/显示所有几何体/站点（`rov_wall.xml`
  目前所有元素都在默认组 `0`，故只有这一组有实际效果；`opt_.geomgroup[1..5]`/
  `sitegroup[1..5]` 仍可通过代码按需启用，这里暂不在 UI 上重复暴露） |
| Telemetry（自定义） | （说明性条目，指向视口左上角的实时数值浮层） | 浮层内容：物理频率（Hz，
  附带 `work:` 括注的纯计算耗时 `step_work_ms`——`read()+update()+write()` 的 EMA
  毫秒数，用于量化 `Stonefish-Mesh` 相对 `Lumped` 的真实 CPU 开销）、
  仿真时间、造浪模式与 `τ`、环境流速、4 个接触垫受力、ROV 位姿
  `(X,Y,Z,φ,θ,ψ)`、机体系线速度 `(vx,vy,vz)`（已按当前姿态旋转到机体系）、
  机体角速度 `(ωx,ωy,ωz)`、当前水动力模型与微元数、Rhombic Kern 裕度
  `M_kern`、当前接触垫数（`Active Pads`，0-4）、**500Hz 真实时基累积**的
  脱离时间比例（`Liftoff Ratio`）、角三角陷阱时间比例（`Corner-Trap Ratio`）
  与离散脱离事件数（`Liftoff Events`，上升沿计数）——均每个物理步累加，不受
  60Hz 渲染快照节流影响，可被 `Reset Stats` 按钮清零重新统计 |

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

## 数值校验与五级水动力精确性验证体系（5-Tier V&V）

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

除上述整体一致性校验外，`test/test_fossen_hydrodynamics_geometry.cpp` 另包含一套
面向 Elsevier *Ocean Engineering* 发表级精确性要求的**五级水动力精确性验证体系
（5-Tier V&V）**，逐级从解析极限、退化等价、带几何细节的解析积分基准、热力学
被动性定律直到闭环 MuJoCo 动态轨迹，系统性证明 `kStonefishGeometryDriven`
（`Stonefish-Mesh`）与 `kParametricLumped`（`Lumped`）两套水动力引擎在各自适用
范围内均物理自洽：

1. **Tier 1（Lamb 球解析极限，`HydroVV_Tier1`）**：构造一个半径 `R=0.20m` 的纯球形
   内存 MJCF，验证 `HydroGeometryModel::buildFromMjModel()` 基于 `ellipsoidAlpha()`
   积分给出的线性附加质量 `M_A = 0.5 * ρ * (4/3)πR³`（`k1=k2=k3=0.5`）与经典
   Lamb 椭球公式相对误差 `<0.1%`；同时验证旋转附加惯量 `I_A` 严格为零——球体在
   无界理想流体中绕任意轴旋转不扰动流场，是 Lamb《Hydrodynamics》Art.114 的经典
   结论，`buildFromMjModel()` 据此对近似球形壳体（三轴半长近似相等）特判返回
   `I_A=0`，而非对所有壳体通用但在 `a=b=c` 处本身不收敛于零的 0.2m_Aᵢ(Σ其余两轴
   半长²) 启发式近似公式（该公式仅对非球形壳体保留，继续标注为"待未来调参"的
   占位近似）。
2. **Tier 2（微元-集总模型退化精确等价，`HydroVV_Tier2_PatchToLumpedExactEquivalence`）**：
   零转速、单位姿态下，`Stonefish-Mesh` 对前壁板 16 个微元的 Venturi 吸附力逐元
   求和必须与 `Lumped` 单点公式的吸附力严格相等（`<1e-12 N`）；纯轴向相对速度
   `v_y=0.4m/s` 下，总侧向力必须与解析正面压阻 + 摩擦积分
   `F_y = -(D_lin,y·v_y + 0.5ρ(C_{d,n}·A_迎风 + C_{f,t}·A_切向)·v_y²)` 严格相等
   （`<1e-11 N`，`A_迎风`/`A_切向` 直接从 `geo.patches` 法向筛选求和，不依赖硬编码
   箱体尺寸）。
3. **Tier 3（倾斜/旋转下的解析曲面积分基准，`HydroVV_Tier3_AnalyticalPitchSqueezeAndVenturiTorque`）**：
   在额定间隙（`h_gap=h_gap_nominal`，衰减因子恒为 `1`）下施加纯俯仰角速度
   `ω_y=0.5rad/s`（临时置零 `Cd_normal`/`Cf_tangent`/`D_ang_lin`/`D_ang_quad` 以
   单独隔离挤压膜力矩项），离散挤压膜俯仰力矩 `τ_y = -C_squeeze·Σ(A_p·z_p²)·ω_y`
   必须与代码内部逐元求和结果逐位一致（`<1e-9`），并与连续曲面积分闭式解
   `τ_y ≈ -C_squeeze·A_plate·L_z²/3·ω_y`（`L_z` 取自 `geo.bbox_half_extents`，
   非硬编码）相差在 `10%` 量化误差带内；俯仰倾角 `θ=0.05rad` 叠加 `V_y=0.5m/s`
   横流时，前壁板沿 `Z` 的两半因间隙不对称（更近一侧衰减更弱、吸附更强）产生
   `Stonefish-Mesh` 独有的非零俯仰力矩，而 `Lumped` 模式（零 CoB 偏置以剔除
   静水回正力矩干扰后）该力矩严格为零——验证的是"几何驱动模型能捕捉集总模型
   天然缺失的姿态耦合效应"这一物理论断本身，符号方向由测试内部按两侧间隙大小
   动态判定（而非预先硬编码），避免押注未经独立验证的旋转约定符号。
4. **Tier 4（热力学被动性 / 能量耗散定律，`HydroVV_Tier4_ThermodynamicPassivityAndWorklessMunk`）**：
   以固定种子 `std::mt19937(42)` 生成 200 组伪随机 6-DOF 状态
   （`x_CoM∈[0.105,0.5]m`，`v^B∈[-1,1]³m/s`，`ω^B∈[-2,2]³rad/s`，静水），对每组
   验证耗散功率严格为负：
   `P_diss = v^B·(f_drag^B+f_squeeze^B) + ω^B·τ_drag^B < 0`
   （`f_drag_body`/`f_squeeze_body`/`t_drag_body` 均来自新增的
   `HydroWrenchBreakdown` 输出，刚体功率恒等式 `ΣF_i·v_i = v_cog·ΣF_i+ω·Σr_i×F_i`
   保证该式与逐微元耗散功率求和严格相等），且旋转自耦合 Munk 项严格无功：
   `ω^B·(-ω^B×I_A ω^B) = 0`（`<1e-12 W`，三重积恒等式 `a·(a×b)≡0` 的直接验证，
   与另一项耦合平动速度的 `-v^B×(M_lin v^B)` Munk 子项——该子项对 `ω^B` 一般
   **不**无功，不包含在此断言范围内——相互独立）。
5. **Tier 5（闭环 MuJoCo 动态轨迹验证，`HydroVV_Tier5_ClosedLoopTerminalVelocityAndRollStability`）**：
   手动复现 `AdhereMujocoHardware::applyHydrodynamics()` 的"读状态 → 计算 Fossen
   力矩 → 写入 `xfrc_applied` → `mj_step`"闭环（中性浮力参数，剔除与本测试无关
   的既有 `+1.96N` 上浮漂移；远离壁面以避免接触力干扰），在 `0.3m/s` 定常横流下
   验证 `v_y(t)` 单调（容差 `1e-6`）收敛至 `0.300±0.01 m/s`（12s/6000 步）；
   `10°` 初始横滚（绕机体 `X` 轴，即垂直于壁面的法向轴）在 CoB 偏置静水回正
   力矩 + 角阻尼联合作用下全程不超过 `15°`（验证不发散）并在 8s/4000 步内
   稳定衰减至 `<0.5°`。

执行 `colcon test --packages-select adhere_mujoco_hardware --event-handlers
console_direct+` 即可同时跑完既有一致性回归测试与上述 5-Tier V&V 套件。

## 跨模型零重编扩展指南（如何无缝切换新的 ROV 模型包）

`adhere_mujoco_hardware` 本身不包含任何与 `simplified_v1_description`/
`rov_wall.xml` 绑定的硬编码假设——换到一个全新的 ROV 模型包（例如未来的全 CAD
网格模型 `full_cad_description`，拥有不同的壳体尺寸、焊接子刚体、质量/浮力或
推力器量程）只需要在启动时指定三个 launch 参数，**无需修改任何 C++ 源码或
重新编译**：

```bash
ros2 launch adhere_mujoco_hardware mujoco_sim.launch.py \
    pkg_description:=full_cad_description \
    mjcf_file:=mjcf/full_cad.xml \
    xacro_file:=urdf/full_cad.urdf.xacro
```

不传任何参数时三者分别默认为 `simplified_v1_description`/`mjcf/rov_wall.xml`/
`urdf/simplified_v1.urdf.xacro`，与改动前完全一致。支撑这一点的 4 项机制：

1. **启动时模型来源解析**：`mujoco_sim.launch.py` 把 `pkg_description`/
   `mjcf_file` 作为 `adhere_mujoco_node` 的 ROS 2 节点参数传入；该节点在构造
   `hardware_interface::ResourceManager`（其构造函数内部同步调用
   `AdhereMujocoHardware::configure()`）**之前**调用静态方法
   `AdhereMujocoHardware::setModelSourceOverride(pkg, mjcf)` 把这两个值记入
   一对静态变量——之所以必须是静态方法而不是实例方法，是因为此时
   `AdhereMujocoHardware` 实例尚未被 pluginlib 构造出来，根本没有
   `activeInstance()` 可调用。`configure()` 内部的解析优先级为：
   ① `setModelSourceOverride` 的静态覆盖值 → ② `<hardware><param
   name="pkg_description"/"mjcf_file">`（`HardwareInfo::hardware_parameters`，
   供脱离 `adhere_mujoco_node`、直接用 pluginlib 的场景使用）→
   ③ 硬编码默认值 `simplified_v1_description`/`mjcf/rov_wall.xml`。
2. **焊接子刚体递归（`HydroGeometryModel::buildFromMjModel()`）**：不再只收集
   `root_body_id` 自身的直属 `<geom>`，而是先对 `mjModel` 做一次正向遍历
   （MuJoCo 保证 `body_parentid[b] < b`，单次遍历即可），判定每个刚体 `b`
   是否属于 `root_body_id` 的"刚性壳体子树"——即 `b` 本身零自由度
   （`body_jntnum[b]==0`，真正被"焊接"而非挂关节）且其父节点也在该子树内；
   再沿焊接父链累乘 `body_pos`/`body_quat` 得到每个子刚体相对 `root_body_id`
   的复合位姿，把该子刚体每个 `<geom>` 的局部位姿与之复合后才送入
   `tessellateBox`/`tessellateMesh` 等切分函数。对 `rov_wall.xml`
   （`rov_base` 无子刚体）该复合变换恒为单位变换，输出与改动前逐位一致的
   `96` 个微元；对未来带焊接整流罩/浮力泡沫子刚体的 CAD 模型，这些子刚体的
   几何会被自动拉回 `root_body_id` 坐标系参与水动力切分。所有微元质心最终
   还会再整体减去 `root_body_id` 自身的 `<inertial pos>`
   （`m->body_ipos[root_body_id]`），确保微元坐标严格以 CoM（`d->xipos`，
   `FossenHydrodynamics::compute()` 唯一使用的参考点）而非刚体关节原点
   （`d->xpos`）为原点——`rov_wall.xml` 声明 `<inertial pos="0 0 0"/>`，
   两者重合，故这一步对现有模型是零改动；只有未来 CAD 模型若声明非零
   `<inertial pos>` 时才会产生实际偏移。
3. **根刚体名称回退链**：`AdhereMujocoHardware::configure()` 依次尝试
   `"rov_base"` → `"base_link"` → 模型中第一个拥有自由关节
   （`mjJNT_FREE`）的刚体，不再要求新模型必须沿用 `"rov_base"` 这个具体
   名字。该解析结果会同步写入 `MujocoVisBridge::root_body_id`
   （一次写入、全程只读，与 `pad_site_pos_body` 同一模式），`MujocoViewer`
   构造时直接复用这个已解析好的根刚体索引，而不是在自己的构造函数里重新
   执行一遍硬编码的 `"rov_base"` 名称查找——否则换成命名为 `"base_link"`
   的新模型时，`MujocoViewer::body_id_` 会静默变为 `-1`，导致相机跟随/
   流速箭头/Rhombic Kern 菱形/`Ctrl+拖拽`扰动注入全部失效。
4. **推力器轨道上限动态解析 + MJCF `<custom><numeric>` 水动力参数覆盖**：
   `configure()` 为每个推力器从其自身 `model_->actuator_ctrlrange` 读取
   量程（声明了 `ctrlrange` 则采用之，否则回退到 `±40N`），写入
   `MujocoVisBridge::thruster_ctrl_min/max`（与 `pad_site_pos_body` 相同的
   "一次写入、全程只读"模式），`write()` 的钳位与 `MujocoViewer` 右侧边栏的
   8 个滑条量程均读取这对数组，而不是硬编码的 `"-40 40"`。同时
   `HydroParams::applyMjModelOverrides(m, params)` 会扫描 MJCF 的
   `<custom><numeric name="hydro_..." data="..."/>` 条目（支持
   `hydro_vehicle_mass`/`hydro_augmented_mass`/`hydro_displaced_volume`/
   `hydro_cob_offset_body`/`hydro_z_surface`/`hydro_D_lin`/`hydro_D_quad`/
   `hydro_D_ang_lin`/`hydro_D_ang_quad`/`hydro_C_venturi`/
   `hydro_h_gap_nominal`/`hydro_A_plate`/`hydro_C_squeeze`/`hydro_Cd_normal`/
   `hydro_Cf_tangent` 共 14 个名字），MJCF 未声明的名字保持 `HydroParams`
   默认值不变（`rov_wall.xml` 目前一个都未声明，因此零回归）；未来模型可以
   直接在自己的 MJCF 里覆盖任意子集的水动力参数，完全不需要改动这个包的
   头文件默认值。

## 运行

```bash
colcon build --packages-up-to adhere_mujoco_hardware --symlink-install
source install/setup.bash

# 默认：静水启动，带 MuJoCo 窗口 + RViz2；在 MuJoCo 窗口中按 1/2 切换造浪
ros2 launch adhere_mujoco_hardware mujoco_sim.launch.py

# 无头模式（无 GLFW 窗口），不启动 RViz2
ros2 launch adhere_mujoco_hardware mujoco_sim.launch.py headless:=true use_rviz:=false

# 切换到另一个模型描述包，零 C++ 重编译（见上方"跨模型零重编扩展指南"）
ros2 launch adhere_mujoco_hardware mujoco_sim.launch.py \
    pkg_description:=full_cad_description mjcf_file:=mjcf/full_cad.xml \
    xacro_file:=urdf/full_cad.urdf.xacro

# 无 GUI 窗口时，用话题一次性切换造浪（例如配合 headless:=true 做脚本化测试）
ros2 topic pub /rov/wave_mode_cmd std_msgs/msg/Int32 "{data: 2}" --once   # 开启造浪
ros2 topic pub /rov/wave_mode_cmd std_msgs/msg/Int32 "{data: 1}" --once   # 切回静水
```
