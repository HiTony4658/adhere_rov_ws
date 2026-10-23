# `adhere_controller`

## 职责

`AdhereController` 是唯一的 `controller_interface::ControllerInterface`
`pluginlib` 插件（`adhere_controller/AdhereController`，Phase 3.3 合并自此前的
`adhere_controller_common` + `adhere_kamr_controller` 两个包）。承载 FSM 管理、
生命周期管理、接口装配、均匀整数抽取多速率调度、4 话题订阅/发布、Xbox 手柄
`0.25s` 失联看门狗与状态切换，以及 `IDLE_PASSIVE_NONE`/`TELEOP_OPENLOOP_6DOF`/
`HOLD_PID_6DOF`/`ADHERE_PID_WBC`/`ADHERE_SMC_WBC`/`ADHERE_MPPI_BASELINE` 六个 FSM 状态的完整实现，
外加集中式推力器效率/啁啾/安全遥测（`ControllerMetrics.h`）。`ADHERE_MPPI_BASELINE`
（Phase 4.1）是第一个纯 CUDA 采样式规划器：`algorithms/baseline_mppi/` 下的
`BaselineMppiController` 在既有的 25Hz 异步外环工作线程上跑完整的 MPPI
采样/代价/软最大值加权/分配流水线，`StateBaselineMppi` 在 250Hz 内环侧只做
严格的零阶保持（Zero-Order Hold）直通，不含任何 PID/DOB/QP。未来阶段将在
本包内继续承载 `KAMR-MPPI`（`pr_wbc_mppi_v4`）算法的完整实现：250Hz 内环
（Planar Impedance PD + Pad-Force DOB + 12D Elastic Rhombic-Kern OSQP）、
复用同一套异步外环工作线程架构的接触耦合 CUDA MPPI 外环。

## 目录结构

```text
include/adhere_controller/
├── adhere_controller.hpp        # AdhereController 声明
├── CtrlInterfaces.h             # 零拷贝命令包装 + ActiveState（唯一的 FSM 可见状态） + ControlInputs +
│                                # pending_thruster_cmd（输出安全闸门写入点，Phase 3.7）
├── dynamics/
│   ├── ModelParams.h            # 运行时加载的模型解耦物理/增益/传感器安装参数（CLAUDE.md Phase 3.3/3.5/3.7）
│   └── FossenDynamics.h         # 标称 Fossen 刚体+附加质量模型：M/C(ν)/D(ν)/g(q)/正向动力学（Phase 3.7）
├── allocation/
│   └── ThrusterAllocator.h      # 6x8 推力分配矩阵 B/B^+ + Pre-MPPI 的 G/G^+/A_contact/C_kern（Phase 3.6）
├── estimator/
│   └── RovStateEstimator.h      # 500Hz 多速率传感器融合 + 接触辅助墙面约束更新（Phase 3.5/3.7）
├── algorithms/
│   ├── pid/
│   │   ├── SixDofPidController.h    # 解耦 6-DOF 位置 + SO(3) 姿态 PID（从 StatePidHold 抽取，Phase 3.7）
│   │   └── AdherePidController.h    # 贴壁法向力 PI 调节器，覆盖 Fx 分量（Phase 3.8）
│   ├── smc/
│   │   └── SixDofSmcController.h    # 贴壁 6-DOF 滑模控制器（Phase 3.8）
│   └── baseline_mppi/             # 纯 CUDA 外环 MPPI（Phase 4.1），零 Eigen/ROS 依赖的 GPU 类型独立成子目录
│       ├── BaselineCudaTypes.h       # POD float 结构体：CudaState6Dof/CudaBaselineModelParams/CudaBaselineRuntimeConfig
│       ├── BaselineMppiConfig.h      # host 侧超参数 + loadFromNode()（独立于 ModelParams 的解析器）
│       ├── BaselineMppiKernels.h     # host<->nvcc 共享的核函数包装声明（仅 cudaStream_t + POD 类型）
│       └── BaselineMppiController.h  # 引擎：init()/warmStart()/solve()，持久化设备/锁页缓冲区
├── FSM/
│   ├── FSMState.h               # 抽象基类（checkFsmCommandChange()/targetPose() 共享实现）
│   ├── StatePassive.h
│   ├── StateOpenLoop.h
│   ├── StatePidHold.h           # 仅保留设定点积分，误差->力矩律委托给 SixDofPidController（Phase 3.7）
│   ├── StateAdherePid.h         # 三阶段贴壁 PID 基线：STANDOFF_HOMING 定位 + SOFT_DOCKING 位置保持 + WALL_ADHERED 法向力 PI（Phase 3.8）
│   ├── StateAdhereSmc.h         # 三阶段贴壁 SMC 基线：全程单一滑模控制器（含 Y/Z 积分项）+ 贴壁后压墙偏置（Phase 3.8）
│   └── StateBaselineMppi.h      # 250Hz 严格零阶保持直通 + StatePidHold 同构的设定点/摇杆积分（Phase 4.1）
└── common/
    ├── enumClass.h              # FSMStateName（IDLE_PASSIVE_NONE/TELEOP_OPENLOOP_6DOF/HOLD_PID_6DOF/ADHERE_PID_WBC/
    │                            # ADHERE_SMC_WBC/ADHERE_MPPI_BASELINE）/ FSMMode
    ├── mathTypes.h              # Eigen 类型别名（Vec3/Vec6/Vec8/Mat68/Mat86/Mat34/Mat43/Mat48/RotMat）
    ├── mathTools.h              # quatToRotMat / rpyToRotMat / rotMatToRPY / quatMul / quatInverse /
    │                            # quatFromRPY / quatError / smallAngleQuat / saturation /
    │                            # alignQuaternionHemisphere / clampStepVec3 / clampSlewQuat（Phase 3.7）
    ├── MultiRateDecimator.h     # 均匀整数抽取多速率调度（CLAUDE.md Phase 3.1.1）
    ├── LoopProfiler.h           # 频率(Hz)/执行延迟(ms)/超时计数实时观测（CLAUDE.md Phase 3.6）
    ├── OutputSafety.h           # sanitizeAndWriteThrusterCommands()：NaN/Inf 全零 + 饱和写出闸门（Phase 3.7）
    └── ControllerMetrics.h      # ControllerMetricsAccumulator：效率/啁啾/安全遥测（Phase 3.8）

src/
├── adhere_controller.cpp        # 生命周期方法 + 均匀多速率调度 + 500Hz 状态估计/融合 + 4 话题订阅/发布 +
│                                # FSM 切换逻辑 + 集中式输出安全闸门调用点 + pluginlib 导出
├── dynamics/
│   ├── ModelParams.cpp
│   └── FossenDynamics.cpp
├── allocation/
│   └── ThrusterAllocator.cpp
├── estimator/
│   └── RovStateEstimator.cpp
├── algorithms/
│   ├── pid/
│   │   ├── SixDofPidController.cpp
│   │   └── AdherePidController.cpp
│   ├── smc/
│   │   └── SixDofSmcController.cpp
│   └── baseline_mppi/
│       ├── BaselineMppiConfig.cpp      # loadFromNode() 实现
│       ├── BaselineMppiController.cpp  # 引擎：设备/锁页缓冲区管理 + 4 个核函数的编排调用（纯 g++ 编译）
│       └── baseline_mppi_kernels.cu    # 4 个 CUDA 核函数 + stepBaselineFossen6Dof 设备端动力学（nvcc 编译）
└── FSM/
    ├── StatePassive.cpp
    ├── StateOpenLoop.cpp
    ├── StatePidHold.cpp
    ├── StateAdherePid.cpp
    ├── StateAdhereSmc.cpp
    └── StateBaselineMppi.cpp

launch/mujoco.launch.py          # 闭环启动文件：robot_state_publisher + adhere_mujoco_node
                                  # (autostart_controllers=['adhere_controller']) + RViz2
                                  # + launch_teleop:=true 时自动包含 adhere_teleop_gcs 的
                                  # teleop_gcs.launch.py（joy_node + xbox_teleop_node + fsm_console_node）

test/
├── test_multirate_decimator.cpp      # 纯逻辑 gtest，不依赖 rclcpp/hardware_interface
├── test_thruster_allocator.cpp       # 纯 Eigen gtest：B*B^+ ~= I_6，零寄生力矩，A_contact/C_kern（Phase 3.6）
├── test_fsm_states.cpp               # 闭环 gtest：假 CtrlInterfaces + 质点平面"虚拟物理"验证 3 态收敛性
├── test_sensor_fusion_and_teleop.cpp # DVL 杠杆臂补偿收敛性 + sensor_fusion 闭环跟踪 + LT/RT 映射（CLAUDE.md Phase 3.5）
├── test_loop_profiler.cpp            # 纯逻辑 gtest：Hz EMA 收敛性 + 执行延迟/超时计数（CLAUDE.md Phase 3.6）
├── test_dynamics_and_pid.cpp         # FossenDynamics/SixDofPidController/接触辅助估计/输出安全闸门（Phase 3.7）
├── test_adhere_wall_states.cpp       # 假单边弹簧贴墙模型：StateAdherePid/StateAdhereSmc 收敛性 +
│                                      # Rhombic Kern 约束 + PID-vs-SMC 抗扰/啁啾对比（Phase 3.8）
├── test_controller_metrics.cpp       # 纯逻辑 gtest：ControllerMetricsAccumulator 能耗/啁啾/
│                                      # 饱和/接触垫计数契约（Phase 3.8）
└── test_baseline_mppi.cpp             # 真实 GPU gtest（本工作站 RTX5000，非 CPU mock）：
                                        # K=1024/H=30 确定性+延迟+推力器界限、目标方向代价梯度
                                        # 合理性、StateBaselineMppi ZOH 闭环位置跟踪（Phase 4.1）
```

注：`config/controllers.yaml`（历史上的 `controller_manager` 参数文档）与
`adhere_mujoco_hardware/config/mujoco_sim.rviz` 均已在 Phase 3.7 移除/迁移——
前者确认从未被 `launch/mujoco.launch.py` 实际加载，所有真实参数唯一来源于
`simplified_v1_description/config/robot_control.yaml`；后者迁移到
`simplified_v1_description/config/mujoco_sim.rviz`，使 `adhere_mujoco_hardware`
真正做到零模型/可视化配置文件。

## 模型解耦：`ModelParams` + 动态 `ThrusterAllocator`（CLAUDE.md 第 3 节）

`adhere_controller` 的 C++ 代码里不包含任何机器人专属几何或物理常数。
`ModelParams::loadFromNode(get_node())` 在 `AdhereController::on_configure()`
中被调用一次，从 `simplified_v1_description/config/robot_control.yaml`（经由
`launch/mujoco.launch.py` 作为 `--params-file` 传给 `adhere_mujoco_node`）读取：

- `thruster_names/thruster_pos_xyz/thruster_dir_xyz/thruster_min/thruster_max`
  （8 个推力器的站点坐标、推力方向单位向量与限幅，body 坐标系，相对质心）；
- `pad_names/pad_pos_xyz`（4 个贴垫坐标，供未来 WBC/Rhombic Kern 使用）；
- `rho_water/gravity/vehicle_mass/augmented_mass/displaced_volume/
  cob_offset_body/z_surface/z_transition/D_lin/D_quad/D_ang_lin/D_ang_quad`
  （Fossen 水动力前馈参数）；
- `workspace_z_min/workspace_z_max/workspace_x_min`（世界坐标安全边界）；
- `open_loop.*`/`pid_hold.*`/`adhere_pid.*`/`adhere_smc.*`
  （各 FSM 状态的增益/限幅；后两者为 Phase 3.8 新增的贴壁基线参数；`depth_hold.*`
  已随 Phase 5.0 删除的 `HOLD_PID_DEPTH` 状态一并移除）；
- `adhere_target_depth_m/adhere_target_y/adhere_standoff_x/adhere_depth_tol/
  adhere_y_tol/adhere_vz_tol/adhere_vel_tol/adhere_approach_vx_damp/
  adhere_retarget_threshold`（均属 `ModelParams::adhere_stage`，`ADHERE_PID_WBC`/
  `ADHERE_SMC_WBC`/`ADHERE_MPPI_BASELINE` 三者共享的三阶段定位-贴壁目标/
  门限/软着陆逼近阻尼/实时重定向阈值；`adhere_target_depth_m`/`adhere_target_y`
  本阶段起只是初值，运行期实际目标由 `/rov/adhere_target_cmd` 实时改写）；
- `adhere_wall_tags_y/adhere_wall_tags_depth_m`（本阶段新增，并行扁平数组，
  墙面 QR 码/AprilTag 预设表，按下标配对成 `ModelParams::AdhereWallTag`）。

`ThrusterAllocator.h::buildThrusterAllocationMatrix(pos, dir)` 对每个推力器
通用地构造 `b_i = [dir_i; pos_i x dir_i] ∈ R^6`（叉乘公式，不再硬编码角点/
`zaxis` 几何），拼成 `B ∈ R^{6x8}`；`buildThrusterAllocationPseudoInverse()`
计算阻尼 Moore-Penrose 伪逆 `B^+ = B^T(BB^T + 1e-6*I_6)^-1`。切换到几何/质量
不同的新机型只需改 `robot_control.yaml`，本包零 C++ 改动、零重新编译。

> **对 `CLAUDE.md` 第 4.3 节占位几何的一处偏离（Phase 3.2 起延续）**：
> `robot_control.yaml` 的 `thruster_pos_xyz`/`thruster_dir_xyz` 编码的是真实
> `rov_wall.xml` 几何（全部 8 个站点在 body `X=0.0`，与质心同一 `X` 坐标，
> 零杠杆臂），而非 `CLAUDE.md` 第 4.3 节给出的示意性 `X=-0.10m` 杠杆臂占位
> 常数（二者描述的推力器安装偏移不同，已与用户确认按真实几何实现）。

## `CtrlInterfaces`：零拷贝命令包装 + 严格地面真值隔离（CLAUDE.md Phase 3.5）

```cpp
struct ActiveState {
  double pos_w[3];        // 世界坐标系质心位置 [x, y, z]
  double quat_w[4];       // 世界坐标系姿态四元数 [w, x, y, z]（Phase 3.6）
  double vel_body[3];     // 机体坐标系线速度
  double omega_body[3];   // 机体坐标系角速度
  double pad_force[4];    // TL/TR/BL/BR 法向接触力
  double v_sep[4];         // TL/TR/BL/BR 分离速度
};

struct CtrlInterfaces {
  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
    thruster_command_interface_;   // 8: thr_N1..thr_N4, thr_T5..thr_T8
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
    thruster_state_interface_;     // 8（推力器力反馈，非位姿/速度，不受隔离约束）
  ActiveState active_state;        // 唯一允许 FSMState 读取的位姿/速度/接触力来源
  ControlInputs control_inputs_;
  int frequency_ = 500;
};
```

**严格地面真值隔离**：`imu_state_interface_`/`odom_state_interface_`/
`pad_contact_state_interface_`（地面真值）与 4 组噪声多速率传感器状态接口
**均不是** `CtrlInterfaces` 的成员——它们是 `AdhereController` 自己的私有成员
（`gt_*_state_interface_`/`noisy_*_state_interface_`），任何 `FSMState` 都
无法访问。`AdhereController::update()` 在每个 500Hz tick 的内环门控**之前**，
依据 `estimation_mode`（见下）把地面真值或 `RovStateEstimator` 的融合输出
写入 `ctrl_interfaces_.active_state`——这是 4 个 `FSMState` 读取位姿/速度/
接触力的唯一入口。

`ControlInputs` 对应 CLAUDE.md Phase 3.2 的 4 话题解耦契约，持有 3 组正交字段
（均为 `std::atomic<T>`，由 `AdhereController` 的订阅回调——ROS executor
线程——写入，由当前活跃 `FSMState::run()`——500Hz 物理线程——无锁读取）：
`command`（`/rov/fsm_command`）、`setpoint_*` + `setpoint_seq`
（`/rov/setpoint_cmd`，沿边检测，Phase 3.5 新增 `setpoint_target_roll/pitch`）、
`teleop_*` + `teleop_last_rx_time_sec`（`/rov/teleop_cmd`，`0.25s` 失联看门狗
判据，Phase 3.5 新增 `teleop_pitch_rate`/`teleop_roll_rate`）。

## `RovStateEstimator`：500Hz 多速率传感器融合（CLAUDE.md Phase 3.5）

`model_params_.estimation_mode`（`robot_control.yaml`/`mujoco.launch.py` 的
`estimation_mode` 启动参数；注意该参数**不**写在 `robot_control.yaml` 里，
见该文件内的说明注释）控制 `ActiveState` 的来源：

- `"ground_truth"`（默认）：直接拷贝 `AdhereMujocoHardware` 的无噪声
  `imu_sensor`/`rov_odom`/`pad_contact` 状态接口。
- `"sensor_fusion"`：每个 500Hz tick 调用 `RovStateEstimator::predict()`
  （融合噪声 IMU 陀螺/加计）与 `correctAttitude()`（融合噪声 IMU 姿态四元数），
  并在噪声 `depth_sensor_noisy`/`dvl_sensor_noisy` 采样保持值发生变化时调用
  `correctDepth()`/`correctDvl()`（含精确杠杆臂补偿
  `y = v_dvl_meas - omega_hat x r_dvl`）。`RovStateEstimator` 是纯数学头文件
  （无 ROS/MuJoCo 依赖），实现的是**按通道解耦的固定增益（稳态）
  Kalman/互补滤波**，而非单一联合协方差 EKF——增益依据 CLAUDE.md Phase 3.5
  给定的噪声标准差，用指数滑动平均稳态方差公式
  `Var(y_hat) = K/(2-K) * Var(meas)` 解析选取，并在
  `test_sensor_fusion_and_teleop.cpp` 中验证满足 `<0.015 m/s`/`<15mm`/
  `<1.0deg` 的 RMSE 契约。

无论哪种模式，地面真值都会被读入 `AdhereController::update()` 的局部变量，
仅用于首次 `reset()` 估计器与填充 `ground_truth` 模式的 `active_state`
——Phase 3.6 起**不再**通过本包发布地面真值（`/rov/fsm_state` 已删除）：
地面真值改由 `adhere_mujoco_node` 单独发布在 `/rov/ground_truth_odom`
（`nav_msgs/Odometry`），仅供离线评估参考。

## 当前状态（6 态，CLAUDE.md Phase 3.2/3.3/3.5/3.8/4.1，`HOLD_PID_DEPTH` 已于 Phase 5.0 删除并压缩 ID）

| 状态 | ID | 职责 |
|---|---|---|
| `IDLE_PASSIVE_NONE` (`state_idle_passive_none`) | 1 | `enter()`/`run()` 均将 8 个推力器指令置零，安全默认态 |
| `TELEOP_OPENLOOP_6DOF` (`state_teleop_openloop_6dof`) | 2 | 无记忆地把 `/rov/teleop_cmd` 的 6 维遥操作输入（含 Phase 3.5 新增的 `pitch_rate`/`roll_rate`）线性映射为机体力矩（增益来自 `open_loop.max_wrench`），用于验证 `B`/`B^+` 分配零寄生耦合 |
| `HOLD_PID_6DOF` (`state_hold_pid_6dof`) | 3 | `enter()` 锁存当前 `(x,y,z,roll,pitch,yaw)` 为保持点；摇杆/扳机推动期间按机体对齐速度平移/旋转该设定点（增益来自 `pid_hold.*`，roll/pitch 目标钳制在 `±max_tilt_deg`），松手瞬间冻结；6-DOF PID + 浮力/科氏力/阻力前馈 |
| `ADHERE_PID_WBC` (`state_adhere_pid_wbc`) | 4 | 贴壁 PID 基线（Phase 3.8，三阶段序列）：`enter()` 后先进入 `STANDOFF_HOMING` 阶段，Y/Z/X 分别锁定到实时目标 `CtrlInterfaces::adhere_target_`/`adhere_pid.x_wall + adhere_standoff_x`，待 `|y-Y_target|<=adhere_y_tol` 且 `|z-Z_target|<=adhere_depth_tol` 且平面速度 `<=adhere_vel_tol` 后粘滞进入 `SOFT_DOCKING`/`WALL_ADHERED`；该阶段内未贴壁时 `AdherePidController` 之外的 Y/Z/姿态+X 全部由 `SixDofPidController` 驱动，X 向设定点以 `adhere_pid.approach_speed` 匀速逼近 `adhere_pid.x_wall`；一旦 4 垫估计法向力之和超过 `adhere_pid.dock_force_thresh` 且位置进入 `adhere_pid.x_dock_eps`，粘滞切换为贴壁态：X 向改由专用 `AdherePidController` 的法向力 PI 律覆盖，Y/Z/姿态仍由 `SixDofPidController` 保持；`/rov/adhere_target_cmd` 实时改变目标且越过阈值时强制退回 `STANDOFF_HOMING` |
| `ADHERE_SMC_WBC` (`state_adhere_smc_wbc`) | 5 | 贴壁 SMC 基线（Phase 3.8，三阶段序列）：`STANDOFF_HOMING`/`SOFT_DOCKING`/`WALL_ADHERED` 门限与 `ADHERE_PID_WBC` 完全一致；全程只用一个 `SixDofSmcController` 实例驱动全部 6DOF（不为逼近阶段单独构造位置控制器），滑模面叠加了 Y/Z 积分项（`adhere_smc.ki`）以对抗贴壁后的静摩擦，贴壁判定与 `ADHERE_PID_WBC` 相同，贴壁后额外叠加常值压墙偏置 `adhere_smc.wall_preload_fx` |
| `ADHERE_MPPI_BASELINE` (`state_adhere_mppi_baseline`) | 6 | 纯 CUDA 外环 MPPI 基线（Phase 4.1，三阶段序列，无独立 `SOFT_DOCKING` 子阶段）：`enter()` 锁存当前位姿为目标并调用 `BaselineMppiController::warmStart()`；`STANDOFF_HOMING` 阶段同样先锁定 Y/Z/X standoff 目标（实时读取 `CtrlInterfaces::adhere_target_`），对齐后直接进入 `WALL_ADHERED` 把目标 X 改为 `BaselineMppiController::xWall()`（实际逼近/压墙由 CUDA 滚动代价里的墙面接触惩罚 + `preload_fx` 完成）；设定点/摇杆设定点积分与 `HOLD_PID_6DOF` 同构，但在 `STANDOFF_HOMING` 期间被目标覆盖 |

### 三阶段贴壁序列（`STANDOFF_HOMING` → `SOFT_DOCKING` → `WALL_ADHERED`，Phase 4.2，重命名/扩展于本阶段）

`ADHERE_PID_WBC`/`ADHERE_SMC_WBC`/`ADHERE_MPPI_BASELINE` 三个贴壁 FSM 状态共享同一套 `ModelParams::
AdhereStageParams`（`adhere_stage`，由 `robot_control.yaml` 的 `adhere_target_depth_m`/
`adhere_target_y`/`adhere_standoff_x`/`adhere_depth_tol`/`adhere_y_tol`/`adhere_vz_tol`/
`adhere_vel_tol`/`adhere_approach_vx_damp`/`adhere_retarget_threshold` 零重编译参数驱动），而非
各算法私有的 `adhere_pid.*`/`adhere_smc.*`/`baseline_mppi.*` 块 —— 三者统一先定位、再贴壁。
`adhere_target_depth_m`/`adhere_target_y` 本阶段起只是 `on_activate()` 的初值种子，运行期实际
目标改由新话题 `/rov/adhere_target_cmd`（`AdhereTargetCommand.msg`）实时写入
`CtrlInterfaces::adhere_target_`（墙面 QR 码/AprilTag 预设表 `adhere_wall_tags_y`/
`adhere_wall_tags_depth_m` 或自定义 Y/深度）。

`STANDOFF_HOMING`：目标锁定到 `(x_wall+standoff_x, target_y, -target_depth_m)`，不触碰墙面；一旦
**Y 误差、深度误差与平面速度三者同时**收敛到容差内（本阶段把原先仅检查深度的门限扩展为同时检查
Y/Z/速度 —— 正是"ROV 贴壁后被卡在错误 Y 位置"问题的根本修复：原门限只看深度就允许压墙，若此时
Y 还未收敛，贴壁后的库仑摩擦力常常压过 Y 向 PID 的剩余权限），粘滞切换为 `SOFT_DOCKING`/
`WALL_ADHERED`。若贴壁后 `/rov/adhere_target_cmd` 送来新目标且与当前目标的 Y 或深度差超过
`adhere_retarget_threshold`（`0.06m`），立即退回 `STANDOFF_HOMING` 重新定位，而不是带着压墙力
横向/纵向拖动 ROV。各 `FSMState::adhereStageName()` 实现（非贴壁状态返回空字符串）把该阶段字符
串（`STANDOFF_HOMING`/`SOFT_DOCKING`/`WALL_ADHERED`）写入 `ControllerState.msg::adhere_stage`，
供 `adhere_mujoco_hardware` 的嵌入式 MuJoCo Viewer 叠加层/绿色目标球体以及 RViz2
`/rov/environment_markers` 消费。

## `MultiRateDecimator`：均匀整数抽取多速率调度

```cpp
class MultiRateDecimator {
public:
  MultiRateDecimator(int cm_update_rate, int inner_decimation, int outer_decimation);
  struct TickResult { bool inner_due; bool outer_due; };
  TickResult tick();                     // 对 500Hz 主时钟做纯整数取模抽取
  double innerPeriodSeconds() const;     // inner_decimation / cm_update_rate（恒定）
  double outerPeriodSeconds() const;     // outer_decimation / cm_update_rate（恒定）
  void reset();                          // 下次 tick() 令 inner_due/outer_due 同时为真
};
```

默认 `inner_decimation=2`、`outer_decimation=20` 使内环精确 `250Hz`
（`dt=4.0ms` 恒定）、外环精确 `25Hz`（`dt=40.0ms` 恒定），`outer_decimation`
也可配置为 `25` 得到 `20Hz` 外环。

## Pre-MPPI 基础设施：`A_contact`/`C_kern` 预计算与异步外环工作线程骨架（CLAUDE.md Phase 3.6）

`ThrusterAllocator.h` 在 `B`/`B^+` 之外新增 4 个函数，供 `AdhereController::
on_activate()`（与各 `FSMState` 构造 `B`/`B^+` 同一时机）预计算：

- `buildPadNormalMap(pad_pos)` → `G ∈ R^{3x4}`：第 `i` 列为
  `[+1, -z_i, y_i]^T`，把 4 个贴垫法向力 `F_N,i >= 0`（沿 body `+X`）映射到
  `[Fx, Ty, Tz]^T`（**注意**：代码按数值算例取 `+1`，与 `CLAUDE.md` §4.4
  文字公式字面写的 `-1` 不一致，详见 `ThrusterAllocator.h` 内联注释）；
- `buildPadNormalMapPseudoInverse(G)` → `G^+ ∈ R^{4x3}`（阻尼伪逆）；
- `buildContactMap(G^+, B)` → `A_contact = -G^+ B_{[Fx,Ty,Tz]} ∈ R^{4x8}`，
  估计 `F_N_hat ≈ A_contact * u`；
- `buildKernConstraintMatrix(B, Ly, Lz, eps_margin)` → `C_kern ∈ R^{4x8}`
  （Rhombic Kern 4 条边约束，尚未接入任何实时约束，留给 Phase 4 的 12D
  OSQP）。

`a_contact_`/`c_kern_` 用于把 `/rov/controller_state` 的 `contact_forces`
（`kern_margin` 已于 Phase 5.0 Stage 7 随效率/安全遥测一起移至
`/rov/benchmark_metrics`，见下文）由**已下达的推力指令**估计出来（而非
直接读测量贴垫力传感器），与未来 KAMR-MPPI 约束用的是同一套准静态预测
模型。

## 纯 CUDA 外环 Baseline MPPI：`algorithms/baseline_mppi/`（CLAUDE.md Phase 4.1）

`AdhereController::on_activate()` 用与 `a_contact_`/`c_kern_` 相同的
`B`/`B^+`（新增成员 `b_`/`b_pinv_`）调用一次
`BaselineMppiController::init()`，一次性分配全部设备/锁页宿主内存
（`cudaMalloc`/`cudaHostAlloc`，K×H×6 的 `eps`、K 的 `costs`/`weights`、
H×6 的 `tau_bar`/`tau_bar_new`、8 维 `u0`），`solve()` 之后再无任何运行时
分配。`algorithms/baseline_mppi/BaselineCudaTypes.h` 定义零 Eigen/ROS 依赖
的 `float` POD 结构体，供 `baseline_mppi_kernels.cu`（nvcc 编译）与
`BaselineMppiController.cpp`（纯 g++ 编译，仅通过 `<cuda_runtime.h>` 的
host API 操作设备指针/流）共同使用；两者之间唯一的共享头是
`BaselineMppiKernels.h`，故意不出现任何 `curand_kernel.h` 类型或核函数
启动语法——Philox RNG 状态数组对 host 侧始终是不透明的 `void*`。

四个核函数按 `BaselineMppiController::solve()` 固定顺序在一条
`cudaStreamNonBlocking` 流上依次提交，最终 `cudaStreamSynchronize()`
一次性等待并把结果拷回锁页内存（`solve()` 本身是同步调用，但调用方始终
是已经异步的 25Hz 外环工作线程，从不阻塞 500Hz/250Hz 的 ROS 线程）：

1. `rolloutBaselineMppiKernel`（每 rollout 一个线程，`K` 路并行）：用
   `curand_normal` 采样 `eps_{k,t} ~ N(0, diag(sigma_tau))`，经
   `B^+`/`clamp`/`B` 往返得到 8 推力器指令与"已分配"机体力矩，RK2（显式
   中点法）积分 `stepBaselineFossen6Dof`（对角 Fossen 刚体+附加质量+
   科氏力+阻尼+静水恢复力模型，外加单边 Kelvin-Voigt 式贴墙接触罚函数），
   累积四元数代价（位置/姿态/速度误差 + 控制量/控制增量代价 + MPPI
   信息论交叉项）。**数值安全网**：每步后速度/角速度钳制在 `±50`，终值
   `isfinite()` 兜底替换为大而有限的哨兵值——极端采样下某条 rollout 的
   RK2 积分发散绝不会把 `NaN`/`Inf` 带进后续所有 `K` 条 rollout 共享的
   `softmax`/加权归约。
2. `softmaxWeightsKernel`（单 block）：共享内存 + warp shuffle 两段式
   归约求 `rho=min(S)`、`eta=sum(exp(-(S-rho)/lambda))`，写出归一化权重
   `w_k`。
3. `updateNominalKernel`（每个时域步一个 block）：对 `K` 条 rollout 做
   `sum_k w_k * eps_{k,t}` 的网格跨步归约，更新 `tau_bar_new[t] = tau_bar[t]
   + (...)`。
4. `finalizeNominalKernel`（单 block，`H` 线程）：3 点平滑 `tau_bar_new`，
   热启动右移一拍写回 `tau_bar`（下次 `solve()` 的初值），并用平滑后的
   `tau_bar_new[0]` 经 `B^+`/`clamp` 算出 `u_0*` 写入设备端 `u0` 缓冲——
   CUDA 核函数之间没有跨 block 的全局屏障，这是把"平滑+右移+求输出"拆成
   独立第 4 个核函数（而非任务原文的 3 个）的原因。

`BaselineMppiConfig`（`K/H/dt/lambda/sigma_tau/Q_*/R_*/x_wall/wall_k/
wall_b/preload_fx/seed`）故意**不**挂在 `ModelParams` 下——它是算法专属
超参数，由独立的 `BaselineMppiConfig::loadFromNode()` 解析
`robot_control.yaml` 的 `baseline_mppi.*` 键，与 `ModelParams` 只保留物理
真值的原则一致，方便未来 `algorithms/kamr_mppi/` 落地时拥有自己的一套
配置/动力学表示而不触碰这里的任何代码。`BaselineMppiController::solve()`
直接返回 8 维推力器指令（不经过 WBC 分配器；Phase 3.7 曾新增两个纯数据
契约占位抽象类 `MppiPlannerBase`/`WbcAllocatorBase`，零子类，已于
Phase 5.0 作为死代码删除——KAMR-MPPI 真正落地时会连同其具体实现一起
重新引入接口，而不是提前预留）——Baseline MPPI 明确更简单，
是未来 KAMR-MPPI 的消融对照基线，从不经过 WBC 分配器。

## 效率/啁啾/安全遥测：`ControllerMetrics.h`（CLAUDE.md Phase 3.8）

`common/ControllerMetrics.h::ControllerMetricsAccumulator`（头文件式实现，
无 ROS 依赖）在每个 250Hz 内环拍，从经 `sanitizeAndWriteThrusterCommands()`
清洗/饱和后、通过 `thruster_command_interface_[i].get_value()` 回读的 8 路
**真实发出**推力指令计算出：

- `thrust_l2_norm = ‖u‖_2`、`thrust_power_proxy = Σ|u_i|^1.5`（CLAUDE.md
  第 5 节的功率代理公式）；
- `thrust_slew_rate_l2 = ‖(u[k]-u[k-1])/dt‖_2`（啁啾指标，首拍恒为 0）；
- `cumulative_energy_l2 = ∫‖u‖_2^2 dt`、`cumulative_impulse_l1 = ∫‖u‖_1 dt`
  （FSM 状态切换时调用 `reset()` 清零）；
- `saturated_thruster_count`（按每个推力器自己的 `thruster_max[i]` 判定
  `>=98%`，而非硬编码额定值）、`active_contact_pad_count`（按
  `contact_force_thresh` 判定）。

这 7 个字段连同 `kern_margin` 共 8 个，发布在独立的 `/rov/benchmark_metrics`
话题上（`BenchmarkMetrics.msg`，Phase 5.0 Stage 7 从 `ControllerState.msg`
拆出，与 `/rov/runtime_diagnostics` 共用同一个 10Hz "diag" `MultiRateDecimator`
通道）——`/rov/controller_state` 只保留精简的 FSM/位姿反馈，供
`fsm_console_node`（显示效率摘要时也订阅 `/rov/benchmark_metrics`）；
`/rov/benchmark_metrics` 本身只被 `adhere_monitor` 的
`rt_monitor_node`/`benchmark_evaluator_node` 消费。

`AdhereController` 有一个真实的异步外环工作**线程**（`outerWorkerLoop()`，
`on_activate()` 启动、`on_deactivate()`/析构函数 `join()`），通过
`mutex`+`condition_variable` 保护的 `OuterLoopInput`/`OuterLoopOutput`
双缓冲结构体与 500Hz 物理线程非阻塞握手：每个 `tick.outer_due` 把当前
`active_state`+`targetPose()` 快照（及一个 `run_baseline_mppi` 布尔位,
由 `current_state_->state_name` 决定）写入输入缓冲并 `notify_one()`。
`ADHERE_MPPI_BASELINE` 激活时 worker 调用 `baseline_mppi_.solve()`（Phase 4.1,
见下一节）；其余状态仍是"只回传序号，不做真实求解"的占位处理——未来
`ADHERE_MPPI_KAMR` 落地时同样只需在这个 `if` 分支旁再加一条分支，线程生命周期/
握手架构本身始终不用再改。`OuterLoopOutput::u_star`/`solve_latency_ms`
由 `update()` 每个 500Hz tick 以同一把互斥锁拷贝进
`ctrl_interfaces_.outer_loop_cmd_`（`StateBaselineMppi::run()` 的无锁读取
桥）与 `baseline_mppi_solve_ms_`（下方 `outer_planner_solve_ms` 的数据
来源）。

## 实时诊断：`LoopProfiler` + `RuntimeDiagnostics`（CLAUDE.md Phase 3.6）

`LoopProfiler.h`（纯头文件，无 ROS/Eigen 依赖，风格与 `MultiRateDecimator.h`
一致）提供 `tickMarker()`（基于 `steady_clock` 调用间隔的 EMA 估计触发
频率）与 `beginExec()`/`endExec()`（测最新/EMA均值/最大执行耗时，并对
`budget_ms` 计超时次数）。`AdhereController` 为估计器、内环（预算
`4.0ms`）、外环（预算 `40.0ms`）、IMU/称重传感器（逐 tick）、深度/DVL
（采样保持值变化时边沿触发）各建一个 `LoopProfiler` 实例，每 50 个 500Hz
tick（固定 10Hz，独立于 `inner_decimation_`/`outer_decimation_`）汇总发布
`/rov/runtime_diagnostics`，并发布一个绿/红
`visualization_msgs::Marker`（`TEXT_VIEW_FACING`）到
`/rov/diagnostics_hud_marker`，浮在 ROV 正上方。`hw_loop_hz`/
`hw_step_exec_ms` 两路直接读取 `AdhereMujocoHardware` 新增的
`diagnostics/hw_loop_hz`/`hw_step_exec_ms` 状态接口（镜像其
`MujocoVisBridge` 既有的物理线程 EMA 遥测，本包不反向依赖
`adhere_mujoco_hardware` 内部类型）。`estimator_hz` 在
`estimation_mode=="ground_truth"` 时合法地报 `0Hz`（估计器本就没跑），
因此被排除在 `timing_healthy` 的频率检查之外。`outer_planner_solve_ms`
（Phase 4.1 新增字段）在没有任何 GPU 外环规划器激活时恒为 `0`，
`ADHERE_MPPI_BASELINE` 激活时反映 `BaselineMppiController::solve()` 的真实 GPU
墙钟延迟（典型 `<0.5ms`，远低于 `40ms` 外环预算）。

### `robot_control.yaml` 参数文件与节点重映射陷阱

闭环参数由 `simplified_v1_description/config/robot_control.yaml` 提供，通过
`simplified_v1_description/launch/mujoco.launch.py`（CLAUDE.md Phase 3.4：本包
不再持有任何启动文件）作为 `--params-file` 传给 `adhere_mujoco_node` 可执行文件。
**关键陷阱**：`ros2 launch` 的 `Node` 动作总会附加一条无作用域的
`-r __node:=<name>` 重映射规则（此处 `<name>` 即该可执行文件被赋予的名字
`adhere_mujoco_node`），对进程内所有使用默认 `NodeOptions` 构造的节点都生效
——不仅是顶层 `rclcpp::Node`，连内部 `ControllerManager` 节点、以及
`AdhereController` 自己的 `controller_interface` 节点，三者在 ROS 图里都显示
为同一个 `/adhere_mujoco_node`。因此 `robot_control.yaml` 里
`controller_manager` 与 `AdhereController` 各自的 `ros__parameters`
**必须合并在同一个顶层键 `adhere_mujoco_node:` 下**。

## 线程安全

本包所有类均**不**自带线程同步原语，仅 `ControlInputs`/`OuterLoopCommand`
（Phase 4.1 新增，供 `StateBaselineMppi::run()` 无锁读取异步工作线程输出）
使用 `std::atomic<T>`；`CtrlInterfaces` 的读写发生在 `controller_manager`
的单一物理线程（500Hz）内，天然串行；`BaselineMppiController::solve()`
仅由专用的 `outerWorkerLoop()` 线程调用，与 500Hz 线程之间只通过
`outer_worker_mutex_` 做极短暂的快照拷贝同步（GPU 计算本身在锁外进行）。

## CUDA 构建依赖（CLAUDE.md Phase 4.1）

`algorithms/baseline_mppi/baseline_mppi_kernels.cu` 需要 `nvcc`
（`CMAKE_CUDA_ARCHITECTURES "72;75"`，匹配 Jetson AGX Xavier/RTX5000）。本
工作站 CMake 3.16 早于 `FindCUDAToolkit`（需 3.17+），`CMakeLists.txt`
改用传统 `find_package(CUDA REQUIRED)`；若 `nvcc` 不在 `$PATH`（本机
`/usr/local/cuda-11.4/bin/nvcc`，而 CLAUDE.md 第 0 节的构建命令只前置了
`/usr/bin`），`CMakeLists.txt` 会在 `CMAKE_CUDA_COMPILER` 未被外部设置时
自动回退到 `/usr/local/cuda/bin/nvcc`，因此 Phase 4.1 之后的
`colcon build` 不需要修改既有构建命令。

## 运行

本包（CLAUDE.md Phase 3.4）不再持有任何启动文件——是一个纯粹的、通用的
`controller_interface` 插件工具箱。实际启动入口在
[`simplified_v1_description/launch/mujoco.launch.py`](../../descriptions/simplified_v1_description/README.md)：

```bash
colcon build --packages-up-to adhere_controller --symlink-install
source install/setup.bash

ros2 launch simplified_v1_description mujoco.launch.py
ros2 launch simplified_v1_description mujoco.launch.py headless:=true use_rviz:=false \
    launch_teleop:=false
```

默认行为即把 `autostart_controllers:=['adhere_controller']` 与
`simplified_v1_description/config/robot_control.yaml` 一并传给同一个通用
`adhere_mujoco_node` 可执行文件；传入 `autostart_controllers:="[]"` 可还原为纯
开环、0 个控制器加载的行为。`launch_teleop:=false` 时的上位机/下位机分布式
部署方式见 [`adhere_teleop_gcs/README.md`](../../commands/adhere_teleop_gcs/README.md)。
