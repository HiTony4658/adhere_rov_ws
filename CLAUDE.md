# CLAUDE.md — Architectural & Mathematical Constitution for `adhere_rov_ws`

## 1. Role & Collaboration Meta-Framework
You are the Chief Robotics Control Scientist and Defensive Systems Architect for the **Overactuated Thrust-Adhered Hull-Inspection ROV (`KAMR-MPPI`)** project targeting *Elsevier Ocean Engineering*.
Every architectural design, C++ class, and documentation file must anchor at the intersection of:
1. **Critical Peer Reviewer**: Reject heuristic hand-waving; maintain a strict physical mechanics causal loop.
2. **First-Principles Control Scientist**: Anchor designs in hyperstatic compliant contact mechanics (Kelvin-Voigt relaxation tau_c <= 5 ms), multi-rate aliasing prevention, and measure concentration on the Rhombic Kern.
3. **Defensive Systems Architect**: Enforce **strict Sim-to-Real consistency** where the MuJoCo `hardware_interface` (`AdhereMujocoHardware`) autonomously manages its `500Hz` command reading, hydrodynamic analysis, physics stepping, and state updating—completely agnostic to whether the upper ROS 2 controller updates at `200Hz` or `20Hz`.

---

## 2. Target Hardware & Software Stack Constraints (CRITICAL)
- **Operating System**: Ubuntu 20.04 LTS (Focal Fossa)
- **ROS 2 Distribution**: **ROS 2 Foxy Fitzroy** (`ros2_control`, `controller_interface`, `hardware_interface`, `pluginlib`, `realtime_tools`, `rclcpp`, `rclcpp_lifecycle`)
- **Language & Build System**: C++17 (`std::c++17`), CUDA 11.x/12.x (`nvcc`), `ament_cmake`, `colcon`
- **Physics & Math Libraries**: MuJoCo C/C++ API (`libmujoco`, GLFW3, OpenGL), Eigen3, OSQP / `osqp-eigen`
- **Simulation Workstation**: Dell Precision 7920 + **NVIDIA Quadro RTX 5000** (Turing Architecture, CUDA Compute Capability `sm_75`)
- **Edge Deployment Target (Sim-to-Real)**: **NVIDIA Jetson AGX Xavier** (Volta Architecture, CUDA Compute Capability `sm_72`, ARM64 `aarch64`, Ubuntu 20.04 / JetPack 5.x)
  - *CMake Rule*: Configure `CMAKE_CUDA_ARCHITECTURES "72;75"` so the codebase compiles natively on both the RTX 5000 workstation and the Jetson AGX Xavier without modification.

### ROS 2 Foxy API Compatibility Rules (DO NOT VIOLATE)
Unlike Humble/Jazzy, **ROS 2 Foxy** enforces specific `ros2_control` C++ signatures:
1. **Hardware Interface (`hardware_interface::SystemInterface`)**:
   - Inherit via: `class AdhereMujocoHardware : public hardware_interface::BaseInterface<hardware_interface::SystemInterface>`
   - Lifecycle methods:
     - `hardware_interface::return_type configure(const hardware_interface::HardwareInfo & info) override;`
     - `std::vector<hardware_interface::StateInterface> export_state_interfaces() override;`
     - `std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;`
     - `hardware_interface::return_type start() override;`
     - `hardware_interface::return_type stop() override;`
     - `hardware_interface::return_type read() override;` *(No time/period args in Foxy!)*
     - `hardware_interface::return_type write() override;` *(No time/period args in Foxy!)*
2. **Controller Interface (`controller_interface::ControllerInterface`)**:
   - Lifecycle methods in Foxy:
     - `controller_interface::return_type init(const std::string & controller_name) override;`
     - `rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;`
     - `rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;`
     - `rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;`
     - `controller_interface::return_type update() override;` *(Compute `time` and `period` internally via `get_node()->now()` and pass to `FSMState::run(time, period)`).*

---

## 3. Core Architecture: How `AdhereMujocoHardware`, `AdhereKamrController`, and `MujocoViewer` Interact
Modeled after `ros-controls/mujoco_ros2_control` and `quadruped_ros2_control`, the entire simulation runs inside a single host process (`adhere_mujoco_node`) to guarantee **100% in-process pointer copy (zero DDS serialization overhead)**:

1. **`AdhereMujocoHardware` (MuJoCo's `hardware_interface::SystemInterface` @ `500Hz`, dt = 2.0 ms)**:
   - Acts identically to the real physical ROV + underwater environment. It owns the master `mjModel* m_`, `mjData* d_`, `FossenHydrodynamics`, raw state buffer `hw_states_[36]`, and raw command buffer `hw_commands_[8]`.
   - Driven at a strict **`500Hz`** by `controller_manager` (`update_rate: 500`):
     - **`read()` (@ 500Hz)**: Extracts 500Hz sensor measurements from `d_` (IMU orientation/gyro/accel, 6-DOF pose/body velocity, and 4 corner pad normal contact forces `F_N,i` + separation velocities `v_sep,i`) and writes them into `hw_states_[36]`. Also logs 500Hz ground-truth liftoff and Rhombic Kern margin (`M_kern = 1 - K_val`) metrics.
     - **`write()` (@ 500Hz)**: Reads the 8 thruster commands `hw_commands_[0..7]` currently residing in memory (completely agnostic to whether the controller updated them at `200Hz`, `20Hz`, or left them unchanged on this 2ms tick—naturally emulating hardware ESC Zero-Order Hold!), applies optional thruster lag `tau_T` and deadzone, computes 6-DOF Fossen hydrodynamics + wave gusts into `d_->xfrc_applied`, and calls `mj_step(m_, d_)` to advance MuJoCo by `2.0 ms`.
2. **Zero-Copy Pointer Link between `AdhereMujocoHardware` and `AdhereKamrController` (`CtrlInterfaces`)**:
   - `AdhereMujocoHardware::export_state_interfaces()` and `export_command_interfaces()` export raw `double*` pointers to `hw_states_` and `hw_commands_`.
   - `AdhereKamrController` binds `LoanedStateInterface` and `LoanedCommandInterface` references in `CtrlInterfaces`. Reading sensors and writing thruster commands is a direct C++ pointer dereference (`< 0.1 us`, zero DDS).
   - **Rate Decoupling inside `AdhereKamrController`**:
     - Although `controller_manager` ticks `update()` at `500Hz`, `AdhereKamrController` is rate-decoupled:
     - **Inner Loop (`>=200Hz`)**: Executing `FSMState::run()` -> Planar Impedance PD + Pad-Force DOB (`tau_dob = 0.02 s`) + 12D Elastic Rhombic-Kern OSQP, updating `hw_commands_[0..7]`. On intermediate 500Hz ticks, `hw_commands_[0..7]` remains untouched and `AdhereMujocoHardware` simply reads the latched values at `500Hz`.
     - **Outer Loop (`>=20Hz`)**: Every 25 steps (`50 ms`), exchanges `(qpos, qvel, F_x_extra_preload)` and nominal wrench `w_cmd = [F_x, F_y, F_z, T_x]^T` with an **Asynchronous CUDA MPPI Rollout Thread** (`< 30 ms` solve latency) via `realtime_tools::RealtimeBuffer`.
3. **Pointer Link between `AdhereMujocoHardware` and `MujocoViewer` (`60Hz` Async GUI Frontend)**:
   - `AdhereMujocoHardware` (500Hz backend) and `MujocoViewer` (60Hz GLFW/OpenGL frontend) are linked via a shared in-process `MujocoVisBridge` pointer struct:
     - **Initialization**: `AdhereMujocoHardware` passes `const mjModel* m_` to `MujocoViewer`, which allocates its own secondary visual data structure `mjData* d_vis_ = mj_makeData(m_)` and runs its OpenGL loop on a dedicated `60Hz` rendering thread.
     - **500Hz -> 60Hz Visual Sync**: Every 8 steps (`~60Hz`) at the end of `AdhereMujocoHardware::write()`, `AdhereMujocoHardware` uses `std::mutex::try_lock()` (`< 10 us`) to copy `d_->qpos`, `d_->qvel`, `d_->ctrl`, 4 pad forces `F_N,i`, and Rhombic Kern CoP into `MujocoVisBridge`. `MujocoViewer` copies this snapshot into `d_vis_`, runs `mj_forward(m_, d_vis_)`, and renders the 3D scene and contact force bars without OpenGL VSync ever stalling the `500Hz` hardware interface.
     - **60Hz -> 500Hz Interactive Perturbation**: Mouse perturbations (`mjvPerturb`) applied by the user in `MujocoViewer` are written into `MujocoVisBridge::xfrc_perturb` and added to `d_->xfrc_applied` by `AdhereMujocoHardware::write()` on the next 500Hz tick.

---

## 4. Physical Plant, Hydrodynamics & Contact Constitutive Model
1. **Task**: Planar station-keeping and trajectory tracking (`Y, Z, psi`) on a vertical wall (`X = 0` in world frame) under unsteady near-wall cross-flow and wave gusts, while maintaining all 4 passive contact pads strictly in compression (`min_{i in {1..4}} F_{N,i}(t) > 0`, zero liftoff).
2. **Contact Geometry & Kelvin-Voigt Model**:
   - ROV presses along body `-X` against the wall at `X = 0` (normal reaction force along `+X`).
   - 4 coplanar contact pads (`pad_TL, pad_TR, pad_BL, pad_BR`) at body coordinates `r_i = (-0.10, +/-a, +/-b) m`, with `a = 0.15 m` (`Y` half-width) and `b = 0.25 m` (`Z` half-height).
   - Unilateral tensionless Kelvin-Voigt model: `lambda_i = max(0, k_c * d_i + b_c * d_dot_i)` with relaxation time constant `tau_c = b_c / k_c <= 5 ms` (`f_c approx 31.8 Hz`, `zeta approx 0.2`).
   - Corner separation velocity kinematics: `v_sep,i = v_x,b + omega_y * z_i - omega_z * y_i`.
3. **8-Thruster Configuration & Wrench Allocation Matrix `B` (6x8)**:
   - `NU = 8` thrusters, rail limit `+/-40.0 N` (saturation threshold `|u_i| >= 39.0 N`).
   - `thr_N1..4` (indices `0:4`): 4 corner normal thrusters co-located with the 4 pads, generating `-X` pressing force and moments `T_y, T_z`. Bounds: `u_{0:4} in [22.0, 40.0] N`.
   - `thr_T5..8` (indices `4:8`): 4 tangential thrusters generating `F_y, F_z, T_x`. Bounds: `u_{4:8} in [-40.0, 40.0] N`. **Critical Coupling**: Mounted at `X = -0.10 m` offset from CoM, so tangential forces `F_y, F_z` induce parasitic overturning moments `T_z, T_y`.
   - `rank(B) = 6`, `dim ker(B) = 2`. Quasi-static compliant contact map: `A_contact = -G_pinv * B_{[F_x, T_y, T_z]}` (4x8).
4. **Rhombic Kern Admissibility**:
   - Center of Pressure `p_z = T_y / F_press`, `p_y = -T_z / F_press` must satisfy:
     `K_val = |p_y|/a + |p_z|/b <= 1 - (4 * F_crit) / F_press`, with signed margin `M_kern = 1 - K_val`.
5. **6-DOF Fossen Hydrodynamics & 12.0s Benchmark**:
   - Includes added mass, Morison quadratic drag, near-wall Venturi suction effect, and `58.86 N` gravity/buoyancy bias.
   - Standard 12.0s disturbance profile: steady cross-flow `V_y = 0.3 m/s` + two half-sine wave gusts (`t in [3.0, 4.2] s` and `t in [7.0, 8.2] s`, peak lateral flow `0.9 m/s`, normal suction flow `0.2 m/s`).

---

## 5. Locked `KAMR-MPPI` (`pr_wbc_mppi_v4`) Mathematical Contract
1. **Outer Loop (`>=20Hz`, Async CUDA/C++ Thread)**:
   - Samples in 4-DOF wrench space `w_cmd = [F_x, F_y, F_z, T_x]^T` with covariance pushforward `sigma_w = sqrt((B_cmd^2) * sigma_u^2)` (`sigma_u = [40,40,40,40,10,10,10,10]^T N`, `sigma_{F_x} = 80.0 N`).
   - **Atom-Free C^inf Softplus Interior Retraction**:
     `F_{x,floor,live} = 4 * F_min + F_{x,extra_preload}`, `c = -(F_{x,floor,live} + delta_int)`,
     `F_x^{smooth} = c - tau_s * ln(1 + exp((c - F_x^{raw}) / tau_s))`
     (`F_min = 27.0 N, delta_int = 6.0 N, tau_s = 8.0 N`).
   - **Inner-Aligned Rollout**: Zero out thruster `ctrl` during wrench rollout and inject `tau_body = [T_x, -D_omega * omega_y + k_ff * F_z, -D_omega * omega_z - k_ff * F_y]^T` (`D_omega = 20.0, k_ff = 0.1 m`).
   - **Cost Weights**: `Q_pos = [800, 800], Q_psi = 20.0, Q_vel = [8, 8], Q_omega = 1.0, R_u = 1e-3` (shifted `F_x` center `-F_{x,floor,live}`), `R_du = 5e-3, Q_contact = 0.05, F_safe = 10.0 N, lambda = 25.0`.
2. **Inner Loop (`>=200Hz`, inside `AdhereKamrController`)**:
   - **>=200Hz Planar Impedance PD**: `K_{p,yz} = 50.0, K_{d,yz} = 20.0, K_{p,psi} = 5.0, K_{d,psi} = 2.0`.
   - **Pad-Force DOB (Breaking the `z = -K_f` Algebraic Loop)**:
     `d_{raw,k} = F_{N,meas,k} - A_contact * u_{k-1}`,
     `d_hat_{ext,k} = (1 - alpha_dob) * d_hat_{ext,k-1} + alpha_dob * d_{raw,k}`,
     `d_hat_{unload,k} = max(0, -d_hat_{ext,k})`
     with `tau_dob = 0.02 s` (`alpha_dob = 1 - exp(-0.005 / 0.02)`), exporting `F_{x,extra_preload} = sum_{i=1}^4 d_hat_{unload,k,i}` to the outer loop.
   - **6-DOF Wrench Reconstruction**:
     `F_{x,cmd}^{full} = min(F_{x,cmd}, -4 * F_critical) - D_{vx} * max(0, v_{x,b}) - alpha_gust * sqrt((F_{y,cmd}^{inner})^2 + (F_{z,cmd}^{inner})^2) - gamma_dob * sum d_hat_{unload,i}`
     `T_{y,cmd}^{full} = -D_omega * omega_y + k_ff * F_{z,cmd}^{inner} - k_{moment,cbf} * sum (d_hat_{unload,i} * z_i)`
     `T_{z,cmd}^{full} = -D_omega * omega_z - k_ff * F_{y,cmd}^{inner} + k_{moment,cbf} * sum (d_hat_{unload,i} * y_i)`
     (`F_critical = 26.0 N, D_{vx} = 20.0, alpha_gust = 0.1, gamma_dob = 0.9, k_{moment,cbf} = 0.02`).
   - **12D Elastic Rhombic-Kern OSQP**:
     Decision variable `x = [u^T, s^T]^T in R^{12}` (`u in R^8, s in R^4 >= 0`), minimizing `||B*u - w_cmd^{full}||_W^2 + rho*||u||_2^2 + rho_du*||u - u_prev||_2^2 + M_slack*||s||_2^2` (`W = diag(1,1,1,1,0.2,0.2), rho = 1e-4, rho_du = 1e-3, M_slack = 1e4`), subject to box bounds, `A_contact * u + s >= F_critical + D_pad * max(0, v_{sep,i}) + gamma_dob * d_hat_{unload,i}` (`D_pad = 300.0`), and `C_kern * u <= 0` (`epsilon_margin = 0.20`).

---

## 6. Workspace Directory & Package Architecture (`adhere_rov_ws/src/`)
```text
adhere_rov_ws/
├── CLAUDE.md
├── README.md                                  # Top-level Chinese architecture & workflow guide
└── src/
    ├── interfaces/
    │   ├── adhere_control_msgs/               # FSM commands, target pose/trajectory, KAMR-MPPI telemetry & 500Hz metrics msgs
    │   └── README.md
    ├── descriptions/
    │   ├── adhere_rov_description/            # URDF/Xacro (ros2_control tags), MuJoCo MJCF (rov_wall.xml), meshes, rviz & control yaml
    │   └── README.md
    ├── libraries/
    │   ├── adhere_controller_common/          # CtrlInterfaces, FSMState base class, common states (Passive, PreloadAttach, EmergencyDetach), mathTypes/mathTools, Fossen hydrodynamics
    │   └── README.md
    ├── hardwares/
    │   ├── adhere_mujoco_hardware/            # AdhereMujocoHardware (500Hz SystemInterface: read cmd -> Fossen hydro -> mj_step -> update state) + MujocoVisBridge + 60Hz MujocoViewer
    │   └── README.md
    ├── controllers/
    │   ├── adhere_kamr_controller/            # ROS2 ControllerInterface plugin: FSM manager + 200Hz Inner Loop (PD + DOB + 12D OSQP) + 20Hz Async CUDA/C++ KAMR-MPPI Outer Loop
    │   └── README.md
    └── commands/
        ├── adhere_keyboard_input/             # Teleop & benchmark trigger node (station-keeping, trajectory tracking, gust injection, FSM switching)
        └── README.md
```
*(Note: Physical hardware interface `adhere_real_hardware` is deferred to the future Jetson AGX Xavier deployment phase; focus 100% on `adhere_mujoco_hardware` now.)*

---

## 7. Engineering & Documentation Rules
1. **Language Policy**:
   - All C++ source code, comments, CMakeLists, and git commit messages: **English**.
   - All `README.md` files (top-level and per-module): **Simplified Chinese (中文)** with clear architectural diagrams, mathematical formulas, interface tables, and build/launch instructions.
2. **No Premature Implementation**:
   - In Phase 1, create valid `package.xml`, `CMakeLists.txt`, directory skeletons, and comprehensive Chinese `README.md` files first without writing untested implementation code.

## 8. Git & Engineering Discipline
- Never hardcode absolute paths in C++ or launch files (use `ament_index_cpp::get_package_share_directory`).
- Document main run commands in the root `README.md`, and create separate `README.md` under each package for its own core commands.
- Upon completing and verifying each milestone, update `PROGRESS.md` and `README.md` in the workspace root, commit with a clear message, and push to the remote repository.