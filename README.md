# adhere_rov_ws — 过驱动推力贴附式船体巡检 ROV 多速率控制工作空间

> **KAMR-MPPI**（Kernel-Admissible Rhombic-Margin MPPI）过驱动推力贴附 ROV 控制框架的
> ROS 2 Foxy + MuJoCo C++ + CUDA 工程实现。目标期刊：Elsevier *Ocean Engineering*；
> 目标部署平台：NVIDIA Jetson AGX Xavier。

---

## 1. 项目背景与目标

本项目研究**过驱动推力贴附式船体巡检 ROV**（Overactuated Thrust-Adhered Hull-Inspection ROV）
在竖直船体壁面（`X = 0`，世界坐标系）上的平面站位保持与轨迹跟踪问题。ROV 依靠 4 个法向
推力器将机体压向壁面，由 4 个被动接触垫（corner pads）承受法向反力，同时必须满足：

1. **零掀离（Zero Liftoff）约束**：任意时刻 4 个接触垫法向力 `min_i F_{N,i}(t) > 0`；
2. **菱形核（Rhombic Kern）可行域约束**：压力中心（CoP）必须落在由接触几何与临界力
   `F_crit` 共同决定的菱形安全域内，`K_val <= 1 - 4*F_crit/F_press`；
3. **多速率混叠抑制**：500Hz 物理 / 250Hz 内环 / 25Hz 外环 / 60Hz 可视化之间不产生
   非预期的离散化伪影或代数环（如 `z = -K_f` 死循环），且内外环均为 500Hz 主时钟的
   **整数倍精确抽取**（`inner_decimation=2`、`outer_decimation=20`），不存在任何
   非均匀交替子步调度。

在 Python/JAX（MJX）原型 `pr_wbc_mppi_v4` 中，我们已经验证了算法层面的有效性
（`0.9 m/s` 阵风扰动下 `5.83%` 掀离率、`11.6 mm` 位置 RMSE），但暴露出两个工程瓶颈：

- **求解延迟过高**：Python/MJX 控制回路单步 `1381.5 ms`，远超 `50 ms` 外环预算（慢 28 倍）；
- **仿真与控制耦合过紧**：MuJoCo 物理步进被硬编码在控制器脚本内部，无法保证
  Sim-to-Real 的架构一致性。

`adhere_rov_ws` 的目标是用 **C++17 + ROS 2 Foxy + `ros2_control` + MuJoCo C++ API + CUDA**
重新实现该控制栈，使其满足实时性要求，并为未来向 Jetson AGX Xavier 的真实硬件迁移
预留严格对齐的接口。

---

## 2. 物理平台与坐标系约定

- **机体坐标系（Body Frame）**：原点位于 ROV 几何中心，`+X` 指向船艏（远离壁面方向），
  `-X` 方向压向壁面；`+Y` 指向右舷；`+Z` 垫直指向上方。壁面位于世界坐标系 `X = 0` 平面。
- **4 个被动接触垫**（`pad_TL`, `pad_TR`, `pad_BL`, `pad_BR`）布置在机体坐标
  `r_i = (-0.10, ±a, ±b) m`，其中 `a = 0.15 m`（Y 方向半宽），`b = 0.25 m`（Z 方向半高）。
- **8 个推力器**：
  - `thr_N1..N4`（法向推力器，索引 `0:4`）：与 4 个接触垫共位，产生 `-X` 压紧力与
    `T_y, T_z` 力矩，限幅 `[-40.0, 40.0] N`；
  - `thr_T5..T8`（切向推力器，索引 `4:8`）：产生 `F_y, F_z, T_x`，限幅 `[-40.0, 40.0] N`，
    安装于 `X = -0.10 m` 偏置处，因此会耦合产生寄生倾覆力矩 `T_y, T_z`。
- **菱形核判据**：压力中心 `p_z = T_y / F_press`、`p_y = -T_z / F_press` 须满足
  `K_val = |p_y|/a + |p_z|/b <= 1 - 4*F_crit/F_press`，带符号安全余量
  `M_kern = 1 - K_val`。
- **标准扰动工况（12.0s 基准）**：定常侧向流 `V_y = 0.3 m/s` + 两段半正弦阵风
  （`t ∈ [3.0, 4.2] s` 与 `t ∈ [7.0, 8.2] s`，侧向流峰值 `0.9 m/s`，法向吸附流峰值 `0.2 m/s`）。

---

## 3. KAMR-MPPI 多速率架构总览

整个仿真运行在**单一宿主进程**（`adhere_mujoco_node`）内，`AdhereMujocoHardware`、
`AdhereKamrController`、`MujocoViewer` 三者通过**进程内指针**直接互联，
**零 DDS 序列化开销**：

```text
┌──────────────────────────────────────────────────────────────────────────┐
│                      adhere_mujoco_node（单进程，宿主一切）                 │
│                                                                            │
│  ┌────────────────────────────┐        ┌──────────────────────────────┐  │
│  │   MujocoViewer (60Hz)      │        │  Async CUDA MPPI Thread (25Hz) │  │
│  │   GLFW/OpenGL 渲染线程      │        │  w_cmd = [Fx,Fy,Fz,Tx]^T 采样  │  │
│  │   d_vis_ = mj_makeData(m_) │        │  < 30ms 目标求解延迟           │  │
│  └─────────────▲──────────────┘        └───────────────▲────────────────┘  │
│                │ try_lock() <10us 快照拷贝               │ realtime_tools:: │
│                │ (qpos,qvel,ctrl,F_N,i,CoP)              │ RealtimeBuffer   │
│         ┌──────┴───────────┐                     ┌──────┴───────────────┐ │
│         │  MujocoVisBridge  │                     │   CtrlInterfaces      │ │
│         │ (60Hz<->500Hz桥)  │                     │ (零拷贝 double* 指针) │ │
│         └──────┬───────────┘                     └──────┬───────────────┘ │
│                │                                          │                │
│  ┌─────────────▼─────────────────────┐    ┌───────────────▼─────────────┐ │
│  │   AdhereMujocoHardware (500Hz)     │    │  AdhereKamrController       │ │
│  │   hardware_interface::SystemIf     │◄───┤  controller_interface       │ │
│  │   read(): d_ -> hw_states_[36]     │    │  FSM: PASSIVE/PRELOAD/      │ │
│  │   write(): hw_commands_[8] ->      │    │       STATION_KEEP/TRAJ/    │ │
│  │     Fossen水动力+阵风 -> mj_step    │    │       EMERGENCY_DETACH      │ │
│  │   owns: mjModel* m_, mjData* d_,   │    │  Inner(250Hz,抽取比2): PD+  │ │
│  │   FossenHydrodynamics,             │    │    DOB+12D Rhombic-Kern OSQP│ │
│  │   hw_states_[36], hw_commands_[8]  │    │  Outer(25Hz,抽取比20): 与   │ │
│  └─────────────────────────────────────┘    │    CUDA线程交换(qpos,qvel..)│ │
│                                              └─────────────────────────────┘ │
│  controller_manager 以 update_rate: 500 严格驱动 read()/write()/update() │
└──────────────────────────────────────────────────────────────────────────┘
```

### 3.1 `AdhereMujocoHardware`（500Hz，`dt = 2.0 ms`）

扮演"真实 ROV + 水下环境"的角色，对上层控制器节奏完全无感知：

- **`read()`**：从 `d_` 中提取 IMU（姿态/角速度/加速度）、6-DOF 位姿/机体速度、
  4 个接触垫法向力 `F_N,i` 与分离速度 `v_sep,i`，写入 `hw_states_[36]`；同步记录
  500Hz ground-truth 掀离与菱形核余量 `M_kern` 遥测。
- **`write()`**：读取当前内存中 `hw_commands_[0..7]`（无论控制器是在 `250Hz` 还是
  `25Hz` 更新它、抑或本 tick 未更新——天然模拟硬件 ESC 零阶保持 ZOH！），
  施加可选推力器滞后 `tau_T` 与死区，计算 6-DOF Fossen 水动力 + 阵风并写入
  `d_->xfrc_applied`，调用 `mj_step(m_, d_)` 推进 `2.0 ms`。

### 3.2 `AdhereKamrController` 均匀整数抽取多速率解耦（`CtrlInterfaces` 零拷贝）

`controller_manager` 与 `AdhereMujocoHardware` 均以严格 500Hz（`dt = 2.0 ms`）驱动
`read()`/`write()`/`update()`。`AdhereKamrController::update()` 内部用
`MultiRateDecimator`（`adhere_controller_common`）对 500Hz 主时钟做**纯整数取模抽取**，
彻底取代早期 `[3,3,3,3,3,2,2,2,2,2]` 交替子步调度（该调度为凑出 `200Hz` 而在
`4ms`/`6ms` 间抖动，给 Pad-Force DOB 等离散滤波器引入 `±20%` 采样周期抖动）：

- **内环（`inner_decimation=2` → 精确 250Hz，`dt = 4.0 ms` 恒定不抖动）**：
  `tick % 2 == 0` 时触发 `FSMState::run(time, 4ms)` → 平面阻抗 PD + 接触力 DOB
  （`tau_dob = 0.02 s`）+ 12 维弹性菱形核 OSQP，更新 `hw_commands_[0..7]`；
  非触发 tick 上原样返回，`hw_commands_` 保持不变（硬件 ESC 零阶保持 ZOH）。
- **外环（`outer_decimation=20` → 精确 25Hz，`dt = 40.0 ms` 恒定，每 10 个内环步
  触发一次；也可配置为 `outer_decimation=25` 得到 `20Hz`）**：通过
  `realtime_tools::RealtimeBuffer` 与异步 CUDA MPPI 滚动线程交换
  `(qpos, qvel, F_x_extra_preload)` 与名义 4 维力矩
  `w_cmd = [F_x, F_y, F_z, T_x]^T`（目标求解延迟 `< 30 ms`；Phase 4/5 前为占位钩子）。
- `cm_update_rate`/`inner_decimation`/`outer_decimation` 均为 `AdhereKamrController` 的
  ROS 参数（见 `simplified_v1_description/config/robot_control.yaml`），默认
  `500`/`2`/`20`。

### 3.3 `MujocoViewer`（60Hz，异步可视化前端）

- `AdhereMujocoHardware` 将只读 `const mjModel* m_` 传给 `MujocoViewer`，后者在独立
  渲染线程分配 `mjData* d_vis_ = mj_makeData(m_)`；
- 每 8 个物理步（约 60Hz）在 `write()` 末尾，`AdhereMujocoHardware` 用
  `std::mutex::try_lock()`（`< 10 us`）将 `qpos/qvel/ctrl`、4 个垫力、菱形核 CoP
  拷入 `MujocoVisBridge`；`MujocoViewer` 拷贝快照到 `d_vis_` 并调用
  `mj_forward()` 渲染，OpenGL VSync 绝不阻塞 500Hz 主循环；
- 用户在 `MujocoViewer` 中的鼠标扰动（`mjvPerturb`）通过 `MujocoVisBridge::xfrc_perturb`
  反向注入 `AdhereMujocoHardware::write()` 的下一个 500Hz tick。

---

## 4. 硬件目标

| 平台 | 用途 | GPU | CUDA Compute Capability |
|---|---|---|---|
| Dell Precision 7920 | 仿真工作站（当前阶段） | NVIDIA Quadro RTX 5000 (Turing) | `sm_75` |
| NVIDIA Jetson AGX Xavier | Sim-to-Real 边缘部署目标（未来阶段） | Volta iGPU | `sm_72` |

所有 CUDA 目标在 CMake 中统一配置 `CMAKE_CUDA_ARCHITECTURES "72;75"`，确保同一套代码
无需修改即可在两种平台上原生编译。操作系统统一为 **Ubuntu 20.04 LTS + ROS 2 Foxy Fitzroy**。

---

## 5. 工作空间目录结构

```text
adhere_rov_ws/
├── CLAUDE.md                 # 架构与数学宪章（英文，约束 AI 协作与工程规范）
├── README.md                 # 本文件
└── src/
    ├── interfaces/            # 消息/服务接口层
    │   └── adhere_control_msgs/
    ├── descriptions/          # URDF/Xacro/MJCF 机器人描述层
    │   └── simplified_v1_description/
    ├── hardwares/             # ros2_control 硬件接口层（MuJoCo 500Hz 后端）
    │   └── adhere_mujoco_hardware/
    ├── controllers/           # 控制算法层（基础设施 + KAMR-MPPI 插件）
    │   ├── adhere_controller_common/
    │   └── adhere_kamr_controller/
    └── commands/              # 遥操作与基准触发层
        └── adhere_keyboard_input/
```

各层级与各软件包的详细设计说明，请参阅对应目录下的 `README.md`：

- [`src/interfaces/README.md`](src/interfaces/README.md)
- [`src/descriptions/README.md`](src/descriptions/README.md)
- [`src/hardwares/README.md`](src/hardwares/README.md)
- [`src/controllers/README.md`](src/controllers/README.md)
- [`src/commands/README.md`](src/commands/README.md)

---

## 6. 构建与运行

```bash
# 1. 安装 ROS 2 Foxy 依赖（假设已 source /opt/ros/foxy/setup.bash）
cd ~/Research/adhere_rov_ws
rosdep install --from-paths src --ignore-src -r -y

# 2. 构建整个工作空间（MuJoCo 需预先安装在 $MUJOCO_DIR / ~/.mujoco/mujoco-* / /opt/mujoco）
colcon build --symlink-install
source install/setup.bash

# 3a. 静态几何检视（RobotModel + TF，无物理仿真）
ros2 launch simplified_v1_description visualize.launch.py

# 3b. 纯开环硬件仿真：adhere_mujoco_hardware 对任何控制器/控制台零感知，
# 默认 0 个控制器加载（u=0 被动漂移验证场景）
ros2 launch adhere_mujoco_hardware mujoco_sim.launch.py

# 3c. 完整 FSM 闭环仿真：AdhereKamrController 自动加载/配置/激活（默认 PASSIVE 态，
# 250Hz 均匀内环 / 25Hz 均匀外环），MuJoCo 窗口 + RViz2 双可视化同步，并自动弹出
# FSM 终端控制台（输入 1 切 PASSIVE、输入 2 并指定目标深度切 DEPTH_HOLD 做闭环定深）
ros2 launch adhere_kamr_controller mujoco.launch.py
```

各命令的完整参数说明见 [`src/descriptions/simplified_v1_description/README.md`](src/descriptions/simplified_v1_description/README.md)、
[`src/hardwares/adhere_mujoco_hardware/README.md`](src/hardwares/adhere_mujoco_hardware/README.md)、
[`src/controllers/adhere_kamr_controller/README.md`](src/controllers/adhere_kamr_controller/README.md)
与 [`src/commands/adhere_keyboard_input/README.md`](src/commands/adhere_keyboard_input/README.md)
（FSM 终端控制台用法）。

---

## 7. 分阶段路线图

| 阶段 | 内容 | 状态 |
|---|---|---|
| **Phase 1** | 工作空间骨架、6 个包清单、全量中文 README | ✅ 完成 |
| **Phase 2** | `AdhereMujocoHardware` 500Hz 读写循环 + Fossen 水动力 + 可插拔水流环境 + `MujocoViewer`/RViz2 双可视化 | ✅ 完成 |
| **Phase 3.1** | `adhere_controller_common`（`CtrlInterfaces`/FSM/数学工具）+ `PASSIVE`/`DEPTH_HOLD` 两态 + `AdhereKamrController` 插件 + FSM 终端控制台 | ✅ 完成 |
| **Phase 3.1.1** | 均匀整数抽取多速率（500Hz→250Hz 内环/25Hz 外环，`MultiRateDecimator`）+ `adhere_mujoco_hardware`/`adhere_kamr_controller` 严格包解耦（独立 `mujoco_sim.launch.py` / `mujoco.launch.py`） | ✅ 当前阶段 |
| **Phase 4** | `AdhereKamrController` 250Hz 内环（PD + DOB + 12D OSQP） | 待开始 |
| **Phase 5** | 25Hz 异步 CUDA MPPI 外环滚动线程（`sm_72`/`sm_75` 双架构） | 待开始 |
| **Phase 6** | 与 Python/JAX 基准（`mujoco_mppi_ws`）数值对齐验证 + 论文实验复现 | 待开始 |
| **Phase 7** | Jetson AGX Xavier Sim-to-Real 真实硬件接口 `adhere_real_hardware` | 未来阶段 |

---

## 8. 参考工程

- [`ros-controls/mujoco_ros2_control`](https://github.com/ros-controls/mujoco_ros2_control) —
  MuJoCo `ros2_control` 硬件接口的架构参考。
- `quadruped_ros2_control`（本地参考仓库，只读）— `CtrlInterfaces`/`FSM`/多包分层
  工程范式参考。
- `mujoco_mppi_ws`（本地 Python/JAX 原型仓库，**只读**）— 数学公式与水动力/接触参数
  对齐基准，**绝不修改**该仓库任何文件。
