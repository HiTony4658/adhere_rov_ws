# `adhere_kamr_controller`

## 职责

`AdhereKamrController` 是 `controller_interface::ControllerInterface` 的
`pluginlib` 插件。当前（Phase 3.1）承载 FSM 管理本身：`PASSIVE`/`DEPTH_HOLD`
两个状态（实现于 `adhere_controller_common`，本包只负责生命周期管理、接口装配
与状态切换）。未来阶段将在本包内继续承载 `KAMR-MPPI`（`pr_wbc_mppi_v4`）算法的
完整实现：200Hz 内环（Planar Impedance PD + Pad-Force DOB + 12D Elastic
Rhombic-Kern OSQP）、20Hz 异步 CUDA MPPI 外环（见下方"未来 KAMR-MPPI 数学契约"
各节，这些是尚未实现的设计契约，不是已完成的功能）。详见
[`src/controllers/README.md`](../README.md) 中与 `adhere_controller_common` 的
职责划分表。

## 当前实现（Phase 3.1）

```text
include/adhere_kamr_controller/
├── adhere_kamr_controller.hpp     # AdhereKamrController 声明
├── fsm_states/                    # 占位：未来 StateStationKeeping/StateTrajectoryTracking
├── inner_wbc/                     # 占位：未来 PlanarImpedancePD/PadForceDOB/RhombicKernQP
└── outer_mppi_cuda/                # 占位：未来 MppiRolloutThread/mppi_rollout_kernel

src/adhere_kamr_controller.cpp      # 生命周期方法 + FSM 切换逻辑 + pluginlib 导出
config/controllers.yaml             # controller_manager 参数服务器描述（文档用途，见下）
```

`AdhereKamrController` 持有 `adhere_controller_common::CtrlInterfaces`，在
`on_configure()` 中订阅 `/rov/fsm_command`（`adhere_control_msgs::msg::FsmCommand`），
在 `on_activate()` 中按固定名称（如 `thr_N1/effort`、`imu_sensor/orientation.w`、
`rov_odom/pos.x`、`pad_contact/pad_TL/force`）从 `command_interfaces_`/
`state_interfaces_` 中逐一查找并装入 `CtrlInterfaces` 的对应插槽（不依赖
`controller_manager` 内部装配顺序与请求顺序一致这一假设）。默认在 `PASSIVE`
态启动。`update()` 每次调用时，通过 `get_node()->now()` 与上一次调用时刻的差值
计算实际 `period`，再转发给当前 `FSMState::run()`。

`adhere_mujoco_node.cpp` 以编程方式调用
`controller_manager->load_controller(name, type)` /
`configure_controller(name)` / `switch_controller(...)` 完成本控制器的加载/
配置/激活，不依赖 `controllers.yaml` 的参数服务器查找路径（该 YAML 文件仅作为
未来 `spawner` 式启动方式的文档参考）。`switch_controller()` 在 Foxy 下会
阻塞等待下一次 `ControllerManager::update()` 真正执行切换，因此该调用被安排在
500Hz 物理线程启动**之后**，避免与尚未开始调用 `update()` 的早期阶段死锁。

## 未来 KAMR-MPPI 数学契约（Phase 1/5 设计冻结文档，尚未实现）

## 类与文件规划

```text
include/adhere_kamr_controller/
├── fsm_states/
│   ├── StateStationKeeping.hpp
│   └── StateTrajectoryTracking.hpp
├── inner_wbc/
│   ├── PlanarImpedancePD.hpp
│   ├── PadForceDOB.hpp
│   └── RhombicKernQP.hpp
└── outer_mppi_cuda/
    ├── MppiRolloutThread.hpp      # 宿主侧线程管理 + RealtimeBuffer 交换
    └── mppi_rollout_kernel.cuh    # CUDA kernel 声明

src/
├── fsm_states/
│   ├── StateStationKeeping.cpp
│   └── StateTrajectoryTracking.cpp
├── inner_wbc/
│   ├── PlanarImpedancePD.cpp
│   ├── PadForceDOB.cpp
│   └── RhombicKernQP.cpp
└── outer_mppi_cuda/
    ├── MppiRolloutThread.cpp
    └── mppi_rollout_kernel.cu
```

## ROS 2 Foxy `ControllerInterface` 生命周期签名

```cpp
controller_interface::return_type init(const std::string & controller_name) override;
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State & previous_state) override;
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & previous_state) override;
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
controller_interface::return_type update() override;  // 无参数！period 内部通过 get_node()->now() 计算
```

`update()` 内部伪代码：

```cpp
controller_interface::return_type AdhereKamrController::update() {
  const auto now = get_node()->now();
  const auto period = now - last_update_time_;
  last_update_time_ = now;
  fsm_manager_.getCurrentState()->run(now, period);   // 转发给当前 FSMState
  return controller_interface::return_type::OK;
}
```

## 速率解耦实现机制

`controller_manager` 以 500Hz 调用 `update()`，内部用计数器触发内环/外环：

```text
inner_trigger_pattern = [3,3,3,3,3,2,2,2,2,2]   // 25 个 500Hz tick 内按此序列触发内环
                                                  // 平均周期 = 25/(ticks总和) ≈ 1/200Hz
outer_trigger_period  = 25 ticks (50 ms)         // 每 25 个 500Hz tick 触发一次外环交换
```

非触发 tick 上，`hw_commands_[0..7]` 保持上一次内环写入的值不变（由
`AdhereMujocoHardware::write()` 的 ZOH 语义天然承接）。

## 200Hz 内环数学契约

### 平面阻抗 PD

```text
K_p,yz = 50.0, K_d,yz = 20.0, K_p,psi = 5.0, K_d,psi = 2.0
```

### Pad-Force DOB（打破 `z = -K_f` 代数环）

```text
d_raw,k        = F_N,meas,k - A_contact * u_{k-1}
d_hat_ext,k    = (1 - alpha_dob) * d_hat_ext,k-1 + alpha_dob * d_raw,k
d_hat_unload,k = max(0, -d_hat_ext,k)
tau_dob = 0.02 s,  alpha_dob = 1 - exp(-0.005/0.02)
F_x,extra_preload = sum_{i=1}^{4} d_hat_unload,k,i   # 导出给外环
```

> **关键设计动机**：直接代数求解 `z_k = -K_f(z_k)` 会形成瞬时代数环（本时刻的
> 控制输出依赖本时刻的测量估计，而估计又依赖本时刻的控制输出）。DOB 通过使用
> `u_{k-1}`（上一拍的控制量，而非 `u_k`）计算 `d_raw,k`，把代数环转换为一个
> 严格因果的一阶低通递推，物理对应 `tau_dob = 0.02 s` 的扰动观测器带宽。

### 6-DOF 弧矢量重构

```text
F_x,cmd^full = min(F_x,cmd, -4*F_critical)
             - D_vx * max(0, v_x,b)
             - alpha_gust * sqrt((F_y,cmd^inner)^2 + (F_z,cmd^inner)^2)
             - gamma_dob * sum(d_hat_unload,i)

T_y,cmd^full = -D_omega*omega_y + k_ff*F_z,cmd^inner
             - k_moment,cbf * sum(d_hat_unload,i * z_i)

T_z,cmd^full = -D_omega*omega_z - k_ff*F_y,cmd^inner
             + k_moment,cbf * sum(d_hat_unload,i * y_i)

F_critical=26.0N, D_vx=20.0, alpha_gust=0.1, gamma_dob=0.9, k_moment,cbf=0.02
```

### 12 维弹性菱形核 OSQP

决策变量 `x = [u^T, s^T]^T ∈ R^12`（`u ∈ R^8` 推力器指令, `s ∈ R^4 >= 0` 松弛变量）：

```text
min  ||B*u - w_cmd^full||_W^2 + rho*||u||_2^2 + rho_du*||u - u_prev||_2^2 + M_slack*||s||_2^2
s.t. box bounds on u (见根 README §2 推力器限幅表)
     A_contact*u + s >= F_critical + D_pad*max(0, v_sep,i) + gamma_dob*d_hat_unload,i
     C_kern * u <= 0                     # 菱形核线性化约束，epsilon_margin = 0.20

W = diag(1,1,1,1,0.2,0.2), rho=1e-4, rho_du=1e-3, M_slack=1e4, D_pad=300.0
```

## 20Hz 异步 CUDA MPPI 外环数学契约

### 协方差 pushforward 与 C^∞ Softplus 内部收缩

```text
sigma_w = sqrt((B_cmd^2) * sigma_u^2)
  sigma_u = [40,40,40,40,10,10,10,10]^T N,  sigma_F_x = 80.0 N

F_x,floor,live = 4*F_min + F_x,extra_preload      # F_min = 27.0 N
c = -(F_x,floor,live + delta_int)                  # delta_int = 6.0 N
F_x^smooth = c - tau_s * ln(1 + exp((c - F_x^raw)/tau_s))     # tau_s = 8.0 N
```

> 该 Softplus 收缩保证采样轨迹永远不会穿越压紧力下界的"硬边界"，以
> **C^∞ 光滑**方式替代 hard clamp，避免 MPPI 代价函数梯度在边界处不连续
> （"Atom-Free" 即不引入离散指示函数/原子操作）。

### 内环对齐 rollout 前馈

```text
tau_body = [T_x, -D_omega*omega_y + k_ff*F_z, -D_omega*omega_z - k_ff*F_y]^T
D_omega = 20.0, k_ff = 0.1 m
```
Rollout 期间推力器 `ctrl` 置零，仅注入 `tau_body` 广义力，使 GPU 端前向动力学与
200Hz 内环的 6-DOF 重构公式保持结构一致。

### 代价权重

```text
Q_pos=[800,800], Q_psi=20.0, Q_vel=[8,8], Q_omega=1.0,
R_u=1e-3 (中心平移至 -F_x,floor,live), R_du=5e-3,
Q_contact=0.05, F_safe=10.0N, lambda=25.0
```

## CUDA 构建注意事项

```cmake
set(CMAKE_CUDA_ARCHITECTURES "72;75")   # sm_72 = Jetson AGX Xavier, sm_75 = Quadro RTX 5000
```
`outer_mppi_cuda/mppi_rollout_kernel.cu` 需同时在两种架构下编译通过，禁止使用
仅 Turing (`sm_75`) 支持而 Volta (`sm_72`) 不支持的指令集特性。

## 线程安全：`realtime_tools::RealtimeBuffer`

200Hz 内环所在的 `controller_manager` 实时线程与 20Hz 异步 CUDA rollout 线程之间，
通过 `realtime_tools::RealtimeBuffer<MppiExchangeData>` 交换数据：

- 实时线程写入 `(qpos, qvel, F_x_extra_preload)`，非阻塞；
- CUDA 线程读取最新快照，求解后将 `w_cmd = [F_x,F_y,F_z,T_x]^T` 写回另一个
  `RealtimeBuffer`，供下一次 200Hz 内环迭代的 6-DOF 重构使用；
- 外环求解延迟 `< 30 ms` 目标，若超过 50ms 窗口未完成，内环沿用上一次有效
  `w_cmd`（图优雅降级，不阻塞实时线程）。

## 尚未实现部分的当前状态

`include/adhere_kamr_controller/{fsm_states,inner_wbc,outer_mppi_cuda}/` 与
`launch/` 目前仍是目录骨架；`CMakeLists.txt` 中 OSQP/CUDA 依赖发现与对应构建
目标以 `# TODO(Phase 3.2+/KAMR-MPPI)` 注释占位，待 200Hz 内环/20Hz 外环里程碑
启动时再实现。
