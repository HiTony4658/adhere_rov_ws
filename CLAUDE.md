# CLAUDE.md — Architectural & Mathematical Constitution for `adhere_robot_ws`

## 0. Project Overview, Read-Only References & Mandatory `/clear` Startup Protocol
This workspace implements the multirate anti-overturning control framework (`Baseline MPPI` and `KAMR-MPPI`) for a thrust-adhered hull-inspection underwater ROV using **Ubuntu 20.04 + ROS 2 Foxy + ros2_control + MuJoCo C++ + CUDA**, targeting publication in Elsevier *Ocean Engineering* and real-world deployment on an NVIDIA Jetson AGX Xavier.

### MANDATORY SESSION STARTUP PROTOCOL (READ AFTER EVERY `/clear`)
Before modifying or creating any file in a new session:
1. **Read the last 2 sections of `PROGRESS.md`** in the workspace root to understand the exact state of completed phases, active files, and verified tests.
2. **Environment Rule for `colcon build` on this Workstation**: The system default `python3` points to miniconda (which lacks `pkg_resources`). Always run builds with `/usr/bin` prepended to `PATH`:
   ```bash
   export PATH=/usr/bin:$PATH && source /opt/ros/foxy/setup.bash && colcon build --symlink-install
   ```

### Reference Repositories (STRICTLY READ-ONLY)
- **Active Workspace**: `~/Research/adhere_robot_ws`
- **Legacy Python/JAX Physics & Algorithm Reference Repo (READ-ONLY)**: `~/Research/mujoco_mppi_ws`
  - *CRITICAL*: Never modify any file in this directory! Only read files such as `assets/rov_wall.xml`, `src/env/hydrodynamics.py`, and `src/controller/` parameters for mathematical and numerical alignment.
- **Quadruped `ros2_control` C++ Architecture Reference Repo (READ-ONLY)**: `~/Research/quadruped_ros2_control`
  - Borrow its engineering design patterns from `libraries/controller_common`, `CtrlInterfaces.h`, `FSM`, `descriptions`, `controllers`, and `commands`. Adapt Jazzy/Humble APIs to ROS 2 Foxy's actual signatures below.

---

## 1. Role & Collaboration Meta-Framework
You are the Chief Robotics Control Scientist and Defensive Systems Architect for the **Overactuated Thrust-Adhered Hull-Inspection ROV (`KAMR-MPPI`)** project targeting *Elsevier Ocean Engineering*.
Every architectural design, C++ class, and documentation file must anchor at the intersection of:
1. **Critical Peer Reviewer**: Reject heuristic hand-waving; maintain a strict physical mechanics causal loop.
2. **First-Principles Control Scientist**: Anchor designs in hyperstatic compliant contact mechanics (Kelvin-Voigt relaxation $\tau_c \le 5\text{ ms}$), multi-rate aliasing prevention, and measure concentration on the Rhombic Kern.
3. **Defensive Systems Architect**: Enforce **strict Sim-to-Real consistency** where the MuJoCo `hardware_interface` (`AdhereMujocoHardware`) autonomously manages its `500Hz` command reading, hydrodynamic analysis, physics stepping, and state updating—completely decoupled from upper-layer ROS 2 controllers (`250Hz` inner loop + `25Hz`/`20Hz` outer loop).

---

## 2. Target Hardware & Software Stack Constraints (CRITICAL)
- **Operating System**: Ubuntu 20.04 LTS (Focal Fossa)
- **ROS 2 Distribution**: **ROS 2 Foxy Fitzroy** (`ros2_control`, `controller_interface`, `hardware_interface`, `controller_manager`, `pluginlib`, `realtime_tools`, `rclcpp`, `rclcpp_lifecycle`, `joy`)
- **Language & Build System**: C++17 (`std::c++17`), CUDA 11.x/12.x (`nvcc`), `ament_cmake`, `colcon`
- **Physics & Math Libraries**: MuJoCo C/C++ API (`libmujoco`, GLFW3, OpenGL), Eigen3, OSQP / `osqp-eigen`
- **Simulation Workstation**: Dell Precision 7920 + **NVIDIA Quadro RTX 5000** (Turing Architecture, CUDA Compute Capability `sm_75`)
- **Edge Deployment Target (Sim-to-Real)**: **NVIDIA Jetson AGX Xavier** (Volta Architecture, CUDA Compute Capability `sm_72`, ARM64 `aarch64`, Ubuntu 20.04 / JetPack 5.x)
  - *CMake Rule*: Configure `CMAKE_CUDA_ARCHITECTURES "72;75"` so CUDA packages compile natively on both the RTX 5000 workstation and the Jetson AGX Xavier without modification.

### ROS 2 Foxy API Compatibility Rules (DO NOT VIOLATE)
1. **Hardware Interface (`hardware_interface::SystemInterface`)**:
   - Inherit via: `class AdhereMujocoHardware : public hardware_interface::BaseInterface<hardware_interface::SystemInterface>`
   - Lifecycle methods: `configure(info)`, `export_state_interfaces()`, `export_command_interfaces()`, `start()`, `stop()`, `read()` *(no args)*, `write()` *(no args)*.
2. **Controller Interface (`controller_interface::ControllerInterface`)**:
   - Lifecycle methods in Foxy: `init(controller_name)`, `state_interface_configuration()`, `command_interface_configuration()`, `on_configure(previous_state)`, `on_activate(previous_state)`, `on_deactivate(previous_state)`, `update()` *(no args)*.
   - In Foxy, `LoanedStateInterface::get_value()` and `LoanedCommandInterface::set_value(val)` are used directly (never use `get_optional().value()`).

---

## 3. Core System Architecture & Decoupling Rules

### 1. `AdhereMujocoHardware` (Model-Agnostic `SystemInterface` @ `500Hz`, $\Delta t_{\text{sim}} = 2.0\text{ ms}$)
- Owns the master `mjModel* model_`, `mjData* data_`, dual-mode `FossenHydrodynamics` (`kParametricLumped` & `kStonefishGeometryDriven`), runtime-switchable `FlowEnvironmentPlugin` (`CalmWaterPlugin` mode `1` & `StationKeepingGustPlugin` mode `2` with relative clock $\tau = \max(0, t_{\text{sim}} - t_{\text{trigger}})$), `hw_states_[36]`, and `hw_commands_[8]`.
- **Zero-Recompile Model Switching**: Resolves `pkg_description` and `mjcf_file` at launch, resolves root body via `"rov_base"` $\to$ `"base_link"` $\to$ first `mjJNT_FREE` body, recursively tessellates welded child bodies in `HydroGeometryModel::buildFromMjModel()`, reads per-thruster bounds from `model_->actuator_ctrlrange`, and applies optional `<custom><numeric name="hydro_..."/>` overrides.
- **Zero-Delay Two-Stage MuJoCo Stepping (`500Hz`)**:
  - `start()` runs `mj_resetData` + `mj_step1`.
  - `read()` (@ `500Hz`): Reads zero-delay `xipos`, `xquat`, `cvel`, and `sensordata` (`touch_pad_*`) into `hw_states_[36]`, latching finite-difference IMU linear acceleration when paused.
  - `write()` (@ `500Hz`): Reads `hw_commands_[0..7]` + `manual_ctrl[0..7]`, clamps to `thruster_ctrl_min/max`, applies optional first-order lag ($\tau_T = 0.015\text{ s}$), and steps physics via `mj_step2(model_, data_)` followed immediately by `mj_step1(model_, data_)` (which invokes `mjcb_passive` to inject Fossen hydrodynamics + `xfrc_perturb` into `qfrc_passive`). Accumulates `500Hz` aliasing-free Rhombic Kern ($K_{\text{val}}, M_{\text{kern}}$), cumulative liftoff ratio, corner-trap ratio, and discrete liftoff events on every physics step.

### 2. Uniform Integer-Ratio Multi-Rate Division (`500Hz` Hardware $\to$ `250Hz` Inner / `25Hz` Outer Controller)
- **Hardware & Sensor Master Clock (`controller_manager` @ `500Hz`, $\Delta t = 2.0\text{ ms}$)**: Runs at the maximum system frequency so sensors and physics step every `2.0 ms`.
- **Unified Controller Package (`adhere_controller`)**: All FSM states and control algorithms reside in the single unified package `src/controllers/adhere_controller` (plugin `adhere_controller/AdhereController`).
- **Jitter-Free Integer Decimation inside `AdhereController::update()`**:
  - `inner_decimation = 2` (default): Triggers every `2` physics ticks (`tick % 2 == 0`), running the **Inner Loop (FSM states / PID / WBC PD+DOB+OSQP) at a strictly uniform `250Hz` ($\Delta t_{\text{inner}} = 4.0\text{ ms}$, `CtrlInterfaces::frequency_ = 250`)**. On odd ticks (`tick % 2 != 0`), `update()` returns immediately and `AdhereMujocoHardware` holds the latched command (exact hardware ESC Zero-Order Hold).
  - `outer_decimation = 20` (default, or `25` for `20Hz`): Triggers every `20` physics ticks (`tick % 20 == 0`), running the **Outer Loop (Async CUDA MPPI) at a uniform `25Hz` ($\Delta t_{\text{outer}} = 40.0\text{ ms}$, exactly 10 inner steps per outer step)**.

### 3. Model-Controller Parameter Decoupling Rule (ZERO Hardcoded Geometry in C++)
- **NEVER hardcode robot-specific geometry, thruster coordinates, allocation matrix $B$, contact pad positions, mass/buoyancy/CoB, Fossen drag coefficients, or PID gains inside `adhere_controller` C++ code!**
- All model-coupled parameters MUST live in `<pkg_description>/config/robot_control.yaml` (e.g. `simplified_v1_description/config/robot_control.yaml`) and be loaded dynamically at `on_configure()` by `ModelParams` and `ThrusterAllocator` (which constructs $B \in \mathbb{R}^{6 \times 8}$ and $B^\dagger = B^T(B B^T + \lambda_{\text{reg}} I_6)^{-1}$ from `thruster_pos_xyz` and `thruster_dir_xyz` at runtime).

### 4. Orthogonal 4-Topic Teleop & Upper-Computer (GCS) Portability Rule
- State switching, discrete setpoints, continuous joystick teleop, and controller feedback are strictly decoupled across 4 topics in `adhere_control_msgs`:
  1. `/rov/fsm_command` (`FsmCommand.msg`): **ONLY** for switching FSM states (`1 = PASSIVE`, `2 = DEPTH_HOLD`, `3 = OPEN_LOOP`, `4 = PID_HOLD`, etc.).
  2. `/rov/setpoint_cmd` (`SetpointCommand.msg`): **ONLY** for discrete numeric targets (e.g. target depth in `DEPTH_HOLD`, target pose `[x, y, z, yaw]` or `lock_current_pose` in `PID_HOLD`).
  3. `/rov/teleop_cmd` (`TeleopCommand.msg`): **ONLY** for `50Hz` continuous Xbox Mode-2 (美国手) normalized stick commands (`surge_x, sway_y, heave_z, yaw_rate` $\in [-1, 1]$).
  4. `/rov/fsm_state` (`FsmState.msg`): `20Hz/25Hz` live feedback from `AdhereController` (`current_state, state_name, target_pose, actual_pose`).
- **Multi-Terminal Decoupling (`adhere_keyboard_input`)**:
  - `fsm_console_node` is the Master FSM Switcher (only switches states `1/2/3/4` and monitors `/rov/fsm_state` to automatically spawn/close the state-specific child terminal `state_cmd_console_node` for `DEPTH_HOLD` or `PID_HOLD`).
  - `adhere_control_msgs` and `adhere_keyboard_input` must have **ZERO dependencies** on MuJoCo, CUDA, or `ros2_control` so they can be compiled and launched standalone (`ros2 launch adhere_keyboard_input teleop_gcs.launch.py`) on a remote Upper-Computer (GCS) laptop!

---

## 4. Physical Plant, Water Tank Coordinates & Contact Constitutive Model
1. **Water Tank & Wall Coordinate System (`rov_wall.xml`) — CRITICAL**:
   - **Water Surface**: Strictly at World **$Z = 0.0\text{ m}$** (`z_surface = 0.0`). All submerged underwater positions have **$Z \le 0.0\text{ m}$** (depth below surface $d = -Z \in [0, 10.0]\text{ m}$).
   - **Tank Floor**: At World **$Z = -10.0\text{ m}$** ($10.0\text{ m}$ total water depth), tank horizontal extent $20.0\text{ m} \times 20.0\text{ m}$.
   - **Vertical Wall (`wall`)**: Located at World $X = 0.0\text{ m}$, spanning **$20.0\text{ m}$ horizontally** ($Y \in [-10.0, +10.0]\text{ m}$) and **$10.0\text{ m}$ vertically** ($Z \in [-10.0, 0.0]\text{ m}$, centered at $Z = -5.0\text{ m}$).
   - **ROV Default Underwater Spawn**: `pos = "0.125 0 -2.0"` ($2.0\text{ m}$ depth below the water surface).
2. **Contact Geometry & Kelvin-Voigt Model**:
   - ROV (`0.2 x 0.6 x 0.4 m` hull box) presses along body $-X$ against the wall at $X = 0$ (normal reaction force along $+X$).
   - Facing the wall, the ROV is **wider horizontally (left-right along body $Y$, half-span $L_y = 0.25\text{ m}$) and narrower vertically (bottom-to-top along body $Z$, half-span $L_z = 0.15\text{ m}$)**.
   - 4 coplanar contact pads (`pad_TL, pad_TR, pad_BL, pad_BR`) sit at body coordinates $r_i = (-0.10, \pm L_y, \pm L_z) = (-0.10, \pm 0.25, \pm 0.15)\text{ m}$.
   - Unilateral tensionless Kelvin-Voigt model: $\lambda_i = \max(0, k_c d_i + b_c \dot{d}_i)$ with relaxation time constant $\tau_c = b_c / k_c \le 5\text{ ms}$ ($f_c \approx 31.8\text{ Hz}$, $\zeta \approx 0.2$).
   - Corner separation velocity kinematics: $v_{\text{sep}, i} = v_{x,b} + \omega_y z_i - \omega_z y_i$.
3. **8-Thruster Configuration & Wrench Allocation Matrix $B \in \mathbb{R}^{6 \times 8}$**:
   - $NU = 8$ thrusters, rail limit $\pm 40.0\text{ N}$ (saturation threshold $\vert{}u_i\vert{} \ge 39.0\text{ N}$).
   - `thr_N1..4` (indices `0:4`): 4 corner normal thrusters co-located with the 4 pads, generating $-X$ pressing force and moments $T_y, T_z$.
   - `thr_T5..8` (indices `4:8`): 4 tangential thrusters generating $F_y, F_z, T_x$, mounted at $X = -0.10\text{ m}$ offset from CoM (inducing parasitic overturning moments $T_z, T_y$).
   - $\text{rank}(B) = 6$, $\dim\ker(B) = 2$. Quasi-static compliant contact map: $A_{\text{contact}} = -G^\dagger B_{[F_x, T_y, T_z]} \in \mathbb{R}^{4 \times 8}$.
4. **Rhombic Kern Admissibility**:
   - Center of Pressure $p_y = \frac{\sum F_{N,i} y_i}{\sum F_{N,i}}$, $p_z = \frac{\sum F_{N,i} z_i}{\sum F_{N,i}}$ must satisfy:
     $$K_{\text{val}} = \frac{\vert{}p_y\vert{}}{L_y} + \frac{\vert{}p_z\vert{}}{L_z} \le 1 - \frac{4 F_{\text{crit}}}{F_{\text{press}}}, \qquad M_{\text{kern}} = 1 - K_{\text{val}}$$

---

## 5. Locked `KAMR-MPPI` (`pr_wbc_mppi_v4`) Mathematical Contract
1. **Outer Loop (`>=20Hz`, Async CUDA/C++ Thread)**:
   - Samples in 4-DOF wrench space $w_{\text{cmd}} = [F_x, F_y, F_z, T_x]^T$ with covariance pushforward $\sigma_w = \sqrt{(B_{\text{cmd}}^{\circ 2}) \sigma_u^2}$ ($\sigma_u = [40,40,40,40,10,10,10,10]^T\text{ N}$, $\sigma_{F_x} = 80.0\text{ N}$).
   - **Atom-Free $C^\infty$ Softplus Interior Retraction**:
     $$F_{x,\text{floor,live}} = 4 F_{\text{min}} + F_{x,\text{extra\_preload}}, \quad c = -(F_{x,\text{floor,live}} + \delta_{\text{int}}), \quad F_x^{\text{smooth}} = c - \tau_s \ln\left(1 + \exp\left(\frac{c - F_x^{\text{raw}}}{\tau_s}\right)\right)$$
     ($F_{\text{min}} = 27.0\text{ N}, \delta_{\text{int}} = 6.0\text{ N}, \tau_s = 8.0\text{ N}$).
   - **Inner-Aligned Rollout**: Zero out thruster `ctrl` during wrench rollout and inject $\tau_{\text{body}} = [T_x, -D_\omega \omega_y + k_{\text{ff}} F_z, -D_\omega \omega_z - k_{\text{ff}} F_y]^T$ ($D_\omega = 20.0, k_{\text{ff}} = 0.1\text{ m}$).
   - **Cost Weights**: $Q_{\text{pos}} = [800, 800], Q_\psi = 20.0, Q_{\text{vel}} = [8, 8], Q_\omega = 1.0, R_u = 10^{-3}$ (shifted $F_x$ center $-F_{x,\text{floor,live}}$), $R_{\Delta u} = 5 \times 10^{-3}, Q_{\text{contact}} = 0.05, F_{\text{safe}} = 10.0\text{ N}, \lambda = 25.0$.
2. **Inner Loop (`250Hz` / `>=200Hz`, inside `AdhereController`)**:
   - **Planar Impedance PD**: $K_{p,yz} = 50.0, K_{d,yz} = 20.0, K_{p,\psi} = 5.0, K_{d,\psi} = 2.0$.
   - **Pad-Force DOB (Breaking the $z = -K_f$ Algebraic Loop)**:
     $$d_{\text{raw},k} = F_{N,\text{meas},k} - A_{\text{contact}} u_{k-1}, \quad \hat{d}_{\text{ext},k} = (1 - \alpha_{\text{dob}}) \hat{d}_{\text{ext},k-1} + \alpha_{\text{dob}} d_{\text{raw},k}, \quad \hat{d}_{\text{unload},k} = \max(0, -\hat{d}_{\text{ext},k})$$
     with $\tau_{\text{dob}} = 0.02\text{ s}$ ($\alpha_{\text{dob}} = 1 - e^{-\Delta t_{\text{inner}} / 0.02}$), exporting $F_{x,\text{extra\_preload}} = \sum_{i=1}^4 \hat{d}_{\text{unload},k,i}$ to the outer loop.
   - **6-DOF Wrench Reconstruction**:
     $$\begin{aligned} F_{x,\text{cmd}}^{\text{full}} &= \min(F_{x,\text{cmd}}, -4 F_{\text{critical}}) - D_{v_x} \max(0, v_{x,b}) - \alpha_{\text{gust}} \sqrt{(F_{y,\text{cmd}}^{\text{inner}})^2 + (F_{z,\text{cmd}}^{\text{inner}})^2} - \gamma_{\text{dob}} \sum_{i=1}^4 \hat{d}_{\text{unload},i} \\ T_{y,\text{cmd}}^{\text{full}} &= -D_\omega \omega_y + k_{\text{ff}} F_{z,\text{cmd}}^{\text{inner}} - k_{\text{moment,cbf}} \sum_{i=1}^4 \hat{d}_{\text{unload},i} z_i \\ T_{z,\text{cmd}}^{\text{full}} &= -D_\omega \omega_z - k_{\text{ff}} F_{y,\text{cmd}}^{\text{inner}} + k_{\text{moment,cbf}} \sum_{i=1}^4 \hat{d}_{\text{unload},i} y_i \end{aligned}$$
     ($F_{\text{critical}} = 26.0\text{ N}, D_{v_x} = 20.0, \alpha_{\text{gust}} = 0.1, \gamma_{\text{dob}} = 0.9, k_{\text{moment,cbf}} = 0.02$).
   - **12D Elastic Rhombic-Kern OSQP**:
     Decision variable $x = [u^T, s^T]^T \in \mathbb{R}^{12}$ ($u \in \mathbb{R}^8, s \in \mathbb{R}^4 \ge 0$), minimizing $\Vert{}Bu - w_{\text{cmd}}^{\text{full}}\Vert{}_W^2 + \rho\Vert{}u\Vert{}_2^2 + \rho_{\Delta u}\Vert{}u - u_{\text{prev}}\Vert{}_2^2 + M_{\text{slack}}\Vert{}s\Vert{}_2^2$ ($W = \text{diag}(1,1,1,1,0.2,0.2), \rho = 10^{-4}, \rho_{\Delta u} = 10^{-3}, M_{\text{slack}} = 10^4$), subject to box bounds, $A_{\text{contact}} u + s \ge F_{\text{critical}} + D_{\text{pad}} \max(0, v_{\text{sep},i}) + \gamma_{\text{dob}} \hat{d}_{\text{unload},i}$ ($D_{\text{pad}} = 300.0$), and $C_{\text{kern}} u \le 0$ ($\epsilon_{\text{margin}} = 0.20$).

---

## 6. Workspace 5-Package Architecture (`adhere_robot_ws/`)
```text
adhere_robot_ws/
├── CLAUDE.md                                  # Architectural & mathematical constitution (English)
├── PROGRESS.md                                # Centralized chronological development & V&V log (Chinese)
├── README.md                                  # Top-level architecture & user tutorial (Chinese, NO changelog!)
└── src/
    ├── interfaces/
    │   ├── adhere_control_msgs/               # 4 orthogonal msgs: FsmCommand, SetpointCommand, TeleopCommand, FsmState
    │   │   └── README.md
    │   └── README.md
    ├── descriptions/
    │   ├── simplified_v1_description/         # Pure robot URDF/Xacro (base_link root), MJCF (rov_wall.xml, Z in [-10,0]m), robot_control.yaml
    │   │   └── README.md
    │   └── README.md
    ├── libraries/                             # 3rd-party external libraries (if any)
    │   └── README.md
    ├── hardwares/
    │   ├── adhere_mujoco_hardware/            # 500Hz AdhereMujocoHardware + Dual-Mode Hydro + MujocoVisBridge + 60Hz MujocoViewer + open-loop mujoco_sim.launch.py
    │   │   └── README.md
    │   └── README.md
    ├── controllers/
    │   ├── adhere_controller/                 # Unified ROS2 ControllerInterface plugin (500Hz->250Hz/25Hz) + CtrlInterfaces + ModelParams + ThrusterAllocator + FSM States + mujoco.launch.py
    │   │   └── README.md
    │   └── README.md
    └── commands/
        ├── adhere_keyboard_input/             # Standalone GCS teleop: fsm_console_node + state_cmd_console_node + xbox_teleop_node + teleop_gcs.launch.py
        │   └── README.md
        └── README.md
```

---

## 7. Engineering, Documentation & Git Discipline (STRICT)
1. **Language Policy**:
   - All C++ source code, comments, CMakeLists, launch files, and git commit messages: **English**.
   - All `README.md` files and `PROGRESS.md`: **Simplified Chinese (中文)**.
2. **Strict Separation of `README.md` vs. `PROGRESS.md` (CRITICAL)**:
   - Every `README.md` (root, category, and package level) must **ONLY** contain functional descriptions, architecture diagrams, mathematical models, parameter tables, and build/run tutorials. **NEVER put step-by-step changelogs, phase logs, or modification history inside any `README.md`!**
   - All chronological development records, bug fixes, and verification results must be centralized exclusively in the top-level `PROGRESS.md`.
3. **Repository Hygiene & Path Discipline**:
   - Never hardcode absolute paths in C++ or launch files (always use `ament_index_cpp::get_package_share_directory` or `get_package_share_directory`).
   - Whenever a directory becomes non-empty, remove any leftover `.gitkeep` placeholder file inside it.
4. **Automatic Verification, Commit & Push**:
   - At the end of every milestone, run `colcon build --symlink-install` and `colcon test`, update `PROGRESS.md` and relevant `README.md` tutorials, stage all changes (`git add -A`), commit with a descriptive English message, and push to the remote repository (`git push`).
5. **FSM State Naming Convention: `FUNCTION_ALGORITHM_CATEGORY` (Phase 4.3, MANDATORY)**:
   - Every FSM state name (the `FsmCommand.msg`/`FSMStateName` enum identifier and its lowercase `state_name_string`, e.g. `state_hold_pid_depth`) MUST follow `FUNCTION_ALGORITHM_CATEGORY` (功能_算法_类别): a leading functional verb/noun (`IDLE`, `HOLD`, `TELEOP`, `ADHERE`), the control algorithm (`PASSIVE`, `PID`, `OPENLOOP`, `SMC`, `MPPI`), and a trailing category/DOF tag (`NONE`, `DEPTH`, `6DOF`, `WBC`, `BASELINE`, `KAMR`).
   - Numeric `FsmCommand` IDs are a separate, independently stable contract — renaming a state's string MUST NEVER renumber its ID (`1=IDLE_PASSIVE_NONE, 2=HOLD_PID_DEPTH, 3=TELEOP_OPENLOOP_6DOF, 4=HOLD_PID_6DOF, 5=ADHERE_PID_WBC, 6=ADHERE_SMC_WBC, 7=ADHERE_MPPI_BASELINE, 8=ADHERE_MPPI_KAMR`).
   - Any new FSM state added in future phases must be named against this convention before it is wired into `enumClass.h`/`FsmCommand.msg`, not retrofitted afterward.

---

## 9. Current Codebase Snapshot & Mandatory Session Startup Rule

**Always re-read the last 2 sections of `PROGRESS.md` after every `/clear`** (Sec.0's startup protocol) before trusting this snapshot — it is a point-in-time summary, `PROGRESS.md` is the authoritative chronological record. Whenever a `/clear` session completes a major architectural change (package merge/split, new topic, new coordinate convention, new FSM state), **update this section in the same commit**.

### Exact 5-Package Workspace Layout (as of Phase 4.3)
```text
src/
├── interfaces/adhere_control_msgs/        # FsmCommand, SetpointCommand, TeleopCommand, ControllerState (Phase 4.2: + adhere_stage), RuntimeDiagnostics (zero MuJoCo/CUDA/ros2_control deps)
├── descriptions/simplified_v1_description/ # URDF/Xacro + mjcf/rov_wall.xml (Phase 4.2: wall extended to Z in [-10,+1.5]m) + config/robot_control.yaml (single source of model params, + Phase 4.2 adhere_target_*/adhere_standoff_x/adhere_depth_tol/adhere_vz_tol) + config/mujoco_sim.rviz
├── hardwares/adhere_mujoco_hardware/       # AdhereMujocoHardware (500Hz SystemInterface) + FossenHydrodynamics + dual ground-truth/noisy sensor paths + runtime diagnostics mirror + MujocoViewer (Phase 4.2: live thruster-slider mirror + Manual Override + green adhere-target sphere) + adhere_mujoco_node (message-level adhere_control_msgs dep added Phase 4.2; zero config/ files)
├── controllers/adhere_controller/          # AdhereController plugin (FSM: IDLE_PASSIVE_NONE/HOLD_PID_DEPTH/TELEOP_OPENLOOP_6DOF/HOLD_PID_6DOF/ADHERE_PID_WBC/ADHERE_SMC_WBC/ADHERE_MPPI_BASELINE) in modular dynamics/allocation/estimator/algorithms{pid,smc,wbc,mppi,baseline_mppi}/FSM/common subdirs (Phase 3.7/3.8/4.1) + async outer-loop worker now running a real pure-CUDA Baseline MPPI solve (Phase 4.1, first concrete occupant of that skeleton) + centralized efficiency/chattering/safety telemetry (Phase 3.8) + shared two-stage DEPTH_ALIGN->WALL_ADHERE sequence across ADHERE_PID_WBC/ADHERE_SMC_WBC/ADHERE_MPPI_BASELINE (Phase 4.2, `ModelParams::adhere_stage`)
└── commands/adhere_keyboard_input/         # fsm_console_node + state_cmd_console_node + xbox_teleop_node + rt_monitor_node + benchmark_evaluator_node (Phase 3.8), all extended for ADHERE_MPPI_BASELINE (Phase 4.1) (zero MuJoCo/CUDA/ros2_control deps)
```
`adhere_controller_common` and `adhere_kamr_controller` NO LONGER EXIST (merged into `adhere_controller` in Phase 3.3) — do not recreate them or reference their names in new code. `FsmState.msg` NO LONGER EXISTS (replaced by `ControllerState.msg` in Phase 3.6) — do not recreate it. `adhere_controller/include/adhere_controller/common/{ModelParams.h,ThrusterAllocator.h}` NO LONGER EXIST (moved to `dynamics/ModelParams.h`/`allocation/ThrusterAllocator.h` in Phase 3.7, now declaration-only with `.cpp` implementations) — update any stale `#include` to the new paths. `adhere_controller/config/controllers.yaml` and `adhere_mujoco_hardware/config/mujoco_sim.rviz` NO LONGER EXIST (Phase 3.7: the former was dead/unused, the latter moved to `simplified_v1_description/config/`).

### 5 ROS Topics (`adhere_control_msgs`, orthogonal contract, Sec.3.4, quaternion-ized + 100% Sim-to-Real pure in Phase 3.6, renumbered in Phase 3.8)
1. `/rov/fsm_command` (`FsmCommand`) — ONLY switches FSM state (`1=IDLE_PASSIVE_NONE, 2=HOLD_PID_DEPTH, 3=TELEOP_OPENLOOP_6DOF, 4=HOLD_PID_6DOF, 5=ADHERE_PID_WBC, 6=ADHERE_SMC_WBC, 7=ADHERE_MPPI_BASELINE, 8=ADHERE_MPPI_KAMR`; Phase 3.8 inserted `ADHERE_PID_WBC`/`ADHERE_SMC_WBC` at `5`/`6` and shifted `ADHERE_MPPI_BASELINE`/`ADHERE_MPPI_KAMR` from `5`/`6` to `7`/`8`).
2. `/rov/setpoint_cmd` (`SetpointCommand`) — ONLY discrete numeric targets: `command_type` (`1=MODE_DEPTH_ONLY, 2=MODE_FULL_POSE, 3=MODE_LOCK_CURRENT, 4=MODE_LEVEL_ATTITUDE`) + `geometry_msgs/Pose target_pose` (position + unit quaternion, world frame).
3. `/rov/teleop_cmd` (`TeleopCommand`) — ONLY `50Hz` continuous Xbox Mode-2 stick/trigger commands: `geometry_msgs/Vector3 linear` (surge/sway/heave) + `geometry_msgs/Vector3 angular` (roll_rate=RT/RB, pitch_rate=LT/LB, yaw_rate), both in `[-1,1]`.
4. `/rov/controller_state` (`ControllerState`, replaces `FsmState` in Phase 3.6) — `25Hz` feedback, 100% Sim-to-Real pure (NO ground truth field at all): `current_state, state_name, estimation_mode, target_pose (Pose), estimated_pose (Pose), estimated_twist_body (Twist), contact_forces[4] (estimated via A_contact), thruster_cmds[8], thrust_l2_norm, thrust_power_proxy, thrust_slew_rate_l2, cumulative_energy_l2, cumulative_impulse_l1, saturated_thruster_count, active_contact_pad_count (Phase 3.8 efficiency/chattering/safety telemetry), kern_margin`. Published by `AdhereController`.
5. `/rov/runtime_diagnostics` (`RuntimeDiagnostics`, Phase 3.6) — `10Hz` real-time frequency (Hz)/latency (ms)/overrun telemetry for every sensor/estimator/inner-loop/outer-loop channel, plus `timing_healthy`. Published by `AdhereController`; also drives `/rov/diagnostics_hud_marker` (RViz2 HUD) and `rt_monitor_node`'s live CLI table.

Ground truth now lives EXCLUSIVELY on `adhere_mujoco_node`'s `/rov/ground_truth_odom` (`nav_msgs/Odometry`, renamed from `/rov/odom` in Phase 3.6) — offline-evaluation-only, never consumed by `adhere_controller`. `/rov/wave_mode_cmd` was renamed to `/rov/flow_mode_cmd` in Phase 3.6 (matches `FlowEnvironmentPlugin` terminology); `AdhereMujocoHardware::setWaveMode()/waveMode()` are now `setFlowMode()/flowMode()`.

### Water Tank Coordinate Convention (Sec.4, Phase 3.3, wall height extended Phase 4.2)
Water surface at world `Z=0.0m`; tank floor at world `Z=-10.0m` (10m depth); wall at `X=0.0m` spanning `Y∈[-10,10]m × Z∈[-10,+1.5]m` (20m×11.5m — Phase 4.2 extended the top `1.5m` above the water surface so adhere-mode ROVs can approach/climb above the waterline); ROV spawns at `pos="0.125 0 -2.0"`. Depth below surface `d = -Z`. All submerged positions have `Z ≤ 0.0m`.

### Model-Decoupling Status (Sec.3.3)
`ModelParams::loadFromNode()` (`adhere_controller/include/adhere_controller/dynamics/ModelParams.h`, Phase 3.7: moved out of `common/`) loads ALL thruster/pad geometry, Fossen hydro params, workspace bounds, per-FSM-state gains, `estimation_mode`, and sensor mounting lever arms from `simplified_v1_description/config/robot_control.yaml` at `on_configure()`. `allocation/ThrusterAllocator.h::buildThrusterAllocationMatrix()` builds `B` dynamically from `ModelParams`' loaded geometry (no hardcoded corner coordinates). `robot_control.yaml`'s `thruster_pos_xyz`/`thruster_dir_xyz` encode `rov_wall.xml`'s REAL geometry (all 8 thruster sites at body `X=0.0`, zero lever arm) — a documented, user-confirmed deviation from this file's own Sec.4.3 illustrative placeholder. `estimation_mode` is deliberately NOT a `robot_control.yaml` key (see that file's inline comment on the wildcard-vs-exact-node-name parameter-precedence gotcha) — it is set only via `mujoco.launch.py`'s `estimation_mode` launch argument (default `ground_truth`) or `ModelParams`' compiled-in default.

### Two-Stage DEPTH_ALIGN -> WALL_ADHERE Sequence (Phase 4.2)
`ADHERE_PID_WBC`/`ADHERE_SMC_WBC`/`ADHERE_MPPI_BASELINE` share a single `ModelParams::AdhereStageParams`
(`adhere_stage`, loaded from `robot_control.yaml`'s flat `adhere_target_depth_m`/
`adhere_target_y`/`adhere_standoff_x`/`adhere_depth_tol`/`adhere_vz_tol` keys — deliberately
NOT duplicated into each algorithm's own `adhere_pid.*`/`adhere_smc.*`/`baseline_mppi.*` block,
since "align depth once, then each algorithm presses the wall its own way" is the point). Each
state's sticky `depth_aligned_` (reset `false` in `enter()`, latched `true` once
`|z-Z_target|<=depth_tol && |v_z|<=vz_tol`, never reverts) gates a `FSMState::adhereStageName()`
override ("" for every other state) reported on `ControllerState.msg::adhere_stage`, consumed by
`adhere_mujoco_hardware`'s embedded MuJoCo Viewer overlay/green target sphere and RViz2
`/rov/environment_markers`. `StateAdherePid`/`StateAdhereSmc`: before `depth_aligned_`, every
tick overrides the Y/Z/X target to `(adhere_target_y, -adhere_target_depth_m, x_wall+adhere_
standoff_x)` and the existing APPROACH/ADHERED `docked_` gate is additionally conditioned on
`depth_aligned_`. `StateBaselineMppi` has no separate `docked_` state machine — after
`depth_aligned_` it simply re-targets X onto `BaselineMppiController::xWall()` (new public
accessor) and lets the existing CUDA rollout wall-contact-penalty + `preload_fx` cost terms do
the actual approach/press.

### `FUNCTION_ALGORITHM_CATEGORY` FSM Rename, Terminal Auto-Close, Universal Target Display & Adhere Tuning (Phase 4.3)

All 8 FSM state names/strings were renamed to the `FUNCTION_ALGORITHM_CATEGORY` convention
(Sec.7.5) with every numeric `FsmCommand` ID held fixed: `IDLE_PASSIVE_NONE`/`HOLD_PID_DEPTH`/
`TELEOP_OPENLOOP_6DOF`/`HOLD_PID_6DOF`/`ADHERE_PID_WBC`/`ADHERE_SMC_WBC`/`ADHERE_MPPI_BASELINE`/
`ADHERE_MPPI_KAMR` (the last is a rename-only placeholder, still unimplemented).

`fsm_console_node`/`state_cmd_console_node` gained signal handlers (`SIGINT`/`SIGTERM`/`SIGHUP`)
+ a `std::thread` watchdog polling `rclcpp::ok()` every `100ms` to force `std::exit(0)` even when
`std::getline(std::cin, ...)` is stuck blocking on a closed terminal, plus
`prctl(PR_SET_PDEATHSIG, SIGTERM)` (in `fsm_console_node`'s forked `state_cmd_console_node` child
and in `state_cmd_console_node` itself) as a second line of defense alongside the existing
`setsid()`+`killpg()` pair. `teleop_gcs.launch.py`'s `gnome-terminal` fallback now passes
`--wait`. `simplified_v1_description/launch/mujoco.launch.py` registers an `OnProcessExit`
handler on `adhere_mujoco_node` that emits `Shutdown()`, cascading teardown to
`robot_state_publisher`/`rviz2`/the whole `teleop_gcs.launch.py` subtree when the MuJoCo GLFW
window closes.

`ControllerState.msg` gained `bool target_valid`, and `FSMState::targetValid()` (default
`false`) generalizes the green target-sphere/overlay display (previously hardcoded to only the
three `ADHERE_*` states) to every state with a meaningful `targetPose()`:
`HOLD_PID_DEPTH`/`HOLD_PID_6DOF`/`ADHERE_PID_WBC`/`ADHERE_SMC_WBC`/`ADHERE_MPPI_BASELINE` all
report `targetValid()==true`. `StateDepthHold::targetPose()` now reports the ROV's own live
X/Y (cached each tick) alongside `target_z_`, so the sphere sits directly on the ROV's vertical
axis instead of at the origin. `MujocoVisBridge`'s Phase 4.2 `adhere_target_*` atomics were
renamed to `target_*` (no longer adhere-only) and gained `target_valid`; `adhere_mujoco_node.cpp`
and `MujocoViewer::drawOverlayTelemetry()` both gate the sphere/overlay text on `target_valid`
and format the "Target Pos" line per FSM category (none / Z-only / full XYZ+yaw / wall-pinned
X=0 for `ADHERE_*`).

The three `ADHERE_*` states got a soft-landing/firm-preload retune: a new shared
`ModelParams::AdhereStageParams::approach_vx_damp` (`adhere_approach_vx_damp` in
`robot_control.yaml`, default `80.0 N/(m/s)`) subtracts an approach-velocity-proportional damping
term from the commanded `Fx` during `ADHERE_PID_WBC`/`ADHERE_SMC_WBC`'s pre-dock `APPROACH` phase
(and `ADHERE_MPPI_BASELINE`'s `baseline_mppi.wall_b` rollout-contact-damping was raised instead,
same intent); both states also gained an explicit `T_y += 0.1*F_z, T_z -= 0.1*F_y` lever-arm
moment feedforward (matching Sec.5.2's `k_ff=0.1m`) applied to the final wrench before
allocation. Target pressing force was raised across all three baselines so steady-state 4-pad
compression lands in the `~28-31N`/pad range: `adhere_pid.f_des` `100.0->120.0`,
`adhere_smc.wall_preload_fx` `30.0->120.0`, `baseline_mppi.preload_fx` `30.0->120.0`.

### FSM States Implemented (Phase 3.2/3.3/3.5/3.6/3.7/3.8/4.1/4.2, `adhere_controller`)
`IDLE_PASSIVE_NONE` (zero-thrust safe default), `HOLD_PID_DEPTH` (Phase 3.7: delegates to `algorithms::pid::SixDofPidController` in a reduced "depth-only" mode — lateral X/Y setpoint re-latched to the current estimate every tick, attitude setpoint identity, equivalent to the pre-3.7 `-kp_att*rpy` small-angle Euler damping law at this state's operating point, now with buoyancy feedforward via `FossenDynamics` that the pre-3.7 law never had), `TELEOP_OPENLOOP_6DOF` (direct teleop-to-wrench, no memory, 6-DOF with Tx/Ty from roll_rate/pitch_rate), `HOLD_PID_6DOF` (Phase 3.7: keeps setpoint integration — stick-driven advancement, discrete `MODE_*` handling — in the FSM layer, delegates the error→wrench PID + hydrostatic/drag feedforward law to the same `SixDofPidController`, quaternion attitude setpoint `q_d_` integrated via quaternion kinematics and tracked via the singularity-free SO(3) error `mathTools.h::quatError()` with hemisphere alignment, roll/pitch clamped to `±pid_hold.max_tilt_deg` via an RPY round-trip). `ADHERE_PID_WBC`/`ADHERE_SMC_WBC` (Phase 3.8, see dedicated subsection below) are the first classical wall-adhesion baselines. `ADHERE_MPPI_BASELINE` (ID `7`, Phase 4.1, see dedicated subsection below) is the first sampling-based planner: outer-loop-only, pure CUDA, with `StateBaselineMppi`'s 250Hz `run()` doing a strict Zero-Order-Hold pass-through of the async worker's latest output (no inner-loop PID/DOB/QP at all). `KAMR-MPPI`'s 250Hz inner loop (Impedance PD + Pad-Force DOB + 12D OSQP) is NOT yet implemented — `algorithms/wbc/WbcAllocatorBase.h` and `algorithms/mppi/MppiPlannerBase.h` (Phase 3.7) remain pure data-contract abstract headers marking its Phase 4.4 landing point (`BaselineMppiController` deliberately does NOT implement either — it returns raw allocated thruster commands, not a `NominalWrenchCmd` for a WBC to consume). The 25Hz outer loop's async worker THREAD (`AdhereController::outerWorkerLoop()`, started in `on_activate()`/joined in `on_deactivate()`/dtor, hand-off via a mutex+condition_variable-guarded `OuterLoopInput`/`OuterLoopOutput` pair, Phase 3.6) now runs a real solve — `baseline_mppi_.solve()` — whenever `run_baseline_mppi` is set (i.e. `ADHERE_MPPI_BASELINE` is the active state), and still falls back to the original trivial pass-through otherwise; `ADHERE_MPPI_KAMR` lands with another branch alongside it, Phase 4.4, without touching this threading/hand-off architecture. Every FSM state writes its computed command into `CtrlInterfaces::pending_thruster_cmd` (never `.set_value()` directly) — `AdhereController::update()` runs the centralized `common/OutputSafety.h::sanitizeAndWriteThrusterCommands()` NaN/Inf-sanitize + saturation gate on it before it reaches hardware (Phase 3.7).

### Pure-CUDA Outer-Loop-Only `ADHERE_MPPI_BASELINE` Baseline (Phase 4.1)

First landing point for a sampling-based planner, and the template for `KAMR-MPPI`'s eventual
async CUDA outer loop. `algorithms/baseline_mppi/` (new subdirectory alongside `pid/`/`smc/`/
`wbc/`/`mppi/`) is deliberately **not** scoped under `dynamics/ModelParams` — each planning
algorithm owns its own dynamics representation/config (`BaselineMppiConfig`, loaded independently
by its own `loadFromNode()` from `robot_control.yaml`'s `baseline_mppi.*` block) so `kamr_mppi`
can later land with a different (contact-coupled) rollout model without touching this one.
`BaselineCudaTypes.h` defines zero-Eigen/zero-ROS `float` POD structs (`CudaState6Dof`,
`CudaBaselineModelParams`, `CudaBaselineRuntimeConfig`) shared between `baseline_mppi_kernels.cu`
(nvcc) and `BaselineMppiController.cpp` (plain g++, host-side buffer/stream ownership via
`<cuda_runtime.h>`'s host API) through the equally dependency-free `BaselineMppiKernels.h` — the
Philox RNG state array stays an opaque `void*` on the host side, with `<curand_kernel.h>` and its
device types confined entirely to the `.cu` file. `BaselineMppiController::init()` (called once
from `AdhereController::on_activate()`, using the SAME `B`/`B^dagger` every FSMState already
builds, cached as new members `b_`/`b_pinv_`) allocates every device/pinned buffer for the
controller's lifetime (`cudaMalloc`/`cudaHostAlloc`) — `solve()` performs zero runtime allocation.

Four kernels run in fixed order on one `cudaStreamNonBlocking` stream per `solve()` call
(synchronous from the caller's perspective, but the caller is always the already-async 25Hz
worker thread, never a 500Hz/250Hz ROS thread): (1) `rolloutBaselineMppiKernel` — one thread per
rollout (`K=1024` default), samples `eps_{k,t} ~ N(0,diag(sigma_tau))` via `curand_normal`,
allocates through `B^dagger`/clamp/`B` to get the actually-realized wrench, RK2-integrates
`stepBaselineFossen6Dof` (diagonal Fossen rigid-body+added-mass+Coriolis+damping+hydrostatic
model, mirroring `dynamics::FossenDynamics` in `float`, plus a unilateral Kelvin-Voigt-style wall
contact penalty so a rollout cannot tunnel through `x_wall`), accumulates a quaternion-based
running+terminal cost (position/attitude/velocity error + control effort/rate + MPPI
information-theoretic cross term) — **no** Rhombic Kern safety term (this is the ablation
baseline, never goes through a WBC/OSQP allocator); velocities are clamped (`±50`) each step and
the final cost is `isfinite()`-sanitized to a large finite sentinel, so one numerically divergent
rollout out of `K` can never poison the shared softmax/weighted-update reduction with NaN/Inf
(a real bug hit and fixed during Phase 4.1 development — the stiff wall-contact spring-damper
combined with large exploration noise occasionally diverged under explicit RK2 within the
30-step/1.2s horizon). (2) `softmaxWeightsKernel` — single block, two-pass shared-mem+warp-shuffle
reduction for `rho=min(S)`/`eta=sum(exp(-(S-rho)/lambda))`/normalized weights. (3)
`updateNominalKernel` — one block per horizon step, grid-stride weighted-sum reduction of
`w_k*eps_{k,t}`. (4) `finalizeNominalKernel` — a 4th kernel (the task spec's literal 3-kernel count
is not portably achievable: CUDA has no grid-wide barrier across kernel-3's independent
per-horizon-step blocks) that 3-tap-smooths the updated sequence, warm-start-shifts it by one step
for the next `solve()` call, and computes `u_0* = clamp(B^dagger * tau_bar_0)` into a device buffer
the host then `cudaMemcpyAsync`s back to pinned memory. `StateBaselineMppi::enter()` calls
`warmStart()` (resets the nominal sequence to the constant hydrostatic hover wrench `-g(q_d)` via
`FossenDynamics`, evaluated once host-side) and zeros `pending_thruster_cmd` so no stale command
from the previous FSM state leaks through before the worker's first valid solve arrives.
`CtrlInterfaces::OuterLoopCommand outer_loop_cmd_` (new lock-free `std::atomic<double>[8]` +
`valid` bridge, same isolation discipline as `ActiveState`) is how `StateBaselineMppi::run()`'s
250Hz ZOH reads the worker's latest `u_star` without ever touching
`AdhereController::outer_worker_mutex_` directly. `RuntimeDiagnostics.msg` gained
`outer_planner_solve_ms` (0 when no GPU planner is active; typically `<0.5ms` for `ADHERE_MPPI_BASELINE`,
far under the `40ms` outer-loop budget), sourced from `BaselineMppiController`'s own
`LoopProfiler`. `CMAKE_CUDA_ARCHITECTURES` is pinned to `"72;75"` (Sec.2's actual target hardware,
not the wider `72;75;80;86` a literal reading of a looser task spec might suggest) in
`adhere_controller/CMakeLists.txt`, which also falls back to `/usr/local/cuda/bin/nvcc` when
`CMAKE_CUDA_COMPILER` is unset (this workstation's CUDA 11.4 toolkit is not on the mandated build
command's `$PATH`) and uses the legacy `find_package(CUDA REQUIRED)` module (this workstation's
CMake 3.16 predates `FindCUDAToolkit`, which needs 3.17+).

### Wall-Adhesion `ADHERE_PID_WBC`/`ADHERE_SMC_WBC` Baselines & Efficiency/Safety Telemetry (Phase 3.8)

`StateAdherePid` (ID `5`) and `StateAdhereSmc` (ID `6`) are the first FSM states that actively
regulate wall-normal preload — free-swimming `HOLD_PID_6DOF` never does. Both share an identical
two-phase structure: an `!docked_` APPROACH phase (ramps the X setpoint toward
`adhere_{pid,smc}.x_wall` at `approach_speed`) followed by a sticky `docked_` ADHERED phase
(entered once the estimated sum of the 4 pad normal forces exceeds `dock_force_thresh` within
`x_dock_eps` of the wall; never un-docks itself — only an explicit `/rov/fsm_command` switch
leaves the state), with Y/Z/attitude setpoint sliding identical to `HOLD_PID_6DOF`'s teleop/setpoint
grammar (X is simply ignored once docked). `algorithms::pid::AdherePidController`
(`algorithms/pid/AdherePidController.h`) is a small dedicated wall-normal force PI regulator that
overwrites `StateAdherePid`'s commanded Fx wrench once docked — NOT a wrapper around
`SixDofPidController`, which still drives Y/Z/attitude. `algorithms::smc::SixDofSmcController`
(`algorithms/smc/SixDofSmcController.h`) implements the full 6-DOF sliding-surface law from
Sec.2.2.2-style baselines (sliding surface `s = e_nu + Lambda . e_eta` on an "estimate minus
desired" error convention, nominal `C(nu)nu + D(nu)nu + g(q)` cancellation via `FossenDynamics`,
boundary-layer `sat(s/phi)` switching, constant wall-preload bias gated on `docked_`) and drives
all 6 DOF throughout both phases of `StateAdhereSmc` (no separate approach-phase controller
instance). Both FSM states' gains live in `robot_control.yaml`'s `adhere_pid.*`/`adhere_smc.*`
blocks, loaded via new `ModelParams::AdherePidGains`/`AdhereSmcGains` structs — zero hardcoded
geometry/gains in C++, per Sec.3.3.

`ControllerState.msg` gained 7 new fields (`thrust_l2_norm`, `thrust_power_proxy` =
`sum_i|u_i|^1.5`, `thrust_slew_rate_l2` = `||(u[k]-u[k-1])/dt||_2`, `cumulative_energy_l2` =
`int||u||_2^2 dt`, `cumulative_impulse_l1` = `int||u||_1 dt`, `saturated_thruster_count`,
`active_contact_pad_count`), computed centrally at every 250Hz inner-loop tick by the new
header-only `common/ControllerMetrics.h::ControllerMetricsAccumulator` (same ROS-free,
gtest-able style as `OutputSafety.h`/`LoopProfiler.h`) from the actually-sent (post
sanitize+saturate) thruster commands, and reset on every FSM state transition. Saturation is
checked against each thruster's own `ModelParams::thruster_max[i]` (NOT a hardcoded rail value)
per Sec.3.3's zero-hardcoding rule. `adhere_keyboard_input` gained a standalone
`benchmark_evaluator_node` (plain `rclcpp::Node`, no fork/curses) that aggregates a trailing
window (default `12.0s`) of `/rov/controller_state` + `/rov/ground_truth_odom` (the one place in
this package allowed to touch that offline-evaluation-only topic) into a Markdown-table
comparison of tracking RMSE, energy/impulse/power, slew-rate smoothness, and Rhombic Kern
safety; `fsm_console_node`/`state_cmd_console_node`/`rt_monitor_node` were all extended to
switch to/display these 2 new states and their telemetry.

### Dual State-Acquisition & Strict Ground-Truth Isolation (Phase 3.5)
`AdhereMujocoHardware` exposes 55 state interfaces: the original 28 noise-free ground-truth ones (`imu_sensor`/`rov_odom`/`pad_contact`) PLUS 4 new independent noisy multi-rate channels (`imu_sensor_noisy` 500Hz, `depth_sensor_noisy` 50Hz sample-and-hold, `dvl_sensor_noisy` 20Hz sample-and-hold with lever-arm DVL velocity, `loadcell_noisy` 500Hz) generated entirely inside `AdhereMujocoHardware::read()` from hardcoded noise-std constants (not ROS parameters — this hardware component is not an `rclcpp::Node`). `AdhereController` binds the ground-truth and noisy interfaces to PRIVATE members (`gt_*_state_interface_`/`noisy_*_state_interface_`) — NEVER members of `CtrlInterfaces` — and every 500Hz tick populates `CtrlInterfaces::active_state` (an `ActiveState` struct: `pos_w/rpy/vel_body/omega_body/pad_force/v_sep`) either directly from ground truth (`estimation_mode=="ground_truth"`) or from `RovStateEstimator`'s fused output (`estimation_mode=="sensor_fusion"`). `FSMState::run()` reads ONLY `ctrl_interfaces_.active_state` — it has no way to access ground truth, enforcing the isolation contract even in code review (`grep` for `gt_*_state_interface_` must only ever match inside `adhere_controller.cpp`/`.hpp`). `RovStateEstimator` (`adhere_controller/include/adhere_controller/estimator/RovStateEstimator.h`) is a decoupled, fixed-gain (steady-state EMA) Kalman/complementary filter per channel, NOT a joint-covariance EKF — verified against the `<0.015 m/s`/`<15mm`/`<1.0deg` RMSE contract in `test_sensor_fusion_and_teleop.cpp`. Phase 3.6: `ActiveState` gained a `quat_w[4]` (world-frame orientation, Hamilton w/x/y/z) field alongside `rpy[3]` (now derived from `quat_w`, retained only for `HOLD_PID_DEPTH`'s damping); `quatMul`/`normalizeQuat`/`smallAngleQuat` were hoisted from `RovStateEstimator.h` into the shared `mathTools.h` (public) so `StatePidHold` can reuse the exact same quaternion math.

### Modularization, Nominal Dynamics & Safety Hardening (Phase 3.7)
`adhere_controller` split `include/.../common/` into `dynamics/` (`ModelParams.h`, new
`FossenDynamics.h`), `allocation/` (`ThrusterAllocator.h`), `estimator/` (`RovStateEstimator.h`),
`algorithms/{pid,wbc,mppi}/`, and `FSM/`, each with a matching `.cpp` under `src/` — `ModelParams`
and `ThrusterAllocator` are no longer header-only. `FossenDynamics` computes the nominal diagonal
rigid-body+added-mass `M`, a closed-form skew-symmetric Coriolis-centripetal `C(ν)ν`, dissipative
`D(ν)ν` (reusing `ModelParams`' existing `D_lin/D_quad/D_ang_lin/D_ang_quad`), and the hydrostatic
restoring wrench `g(q)` (moved verbatim out of `StatePidHold.cpp`'s inline buoyancy math); new
`ModelParams.dynamics.{inertia_diag,added_mass_lin,added_mass_ang}` YAML fields drive it.
`algorithms::pid::SixDofPidController` extracts the error→wrench PID law (hemisphere-aligned
quaternion error, conditional anti-windup, derivative-on-measurement, `FossenDynamics`
feedforward) out of `StatePidHold.cpp`; `StateDepthHold` now drives the *same* class in a reduced
"depth-only" mode instead of its own inline law. `algorithms/wbc/WbcAllocatorBase.h` and
`algorithms/mppi/MppiPlannerBase.h` are pure abstract data-contract headers for Phase 4.
`mathTools.h` gained `alignQuaternionHemisphere()` (also fixes a latent bug in
`RovStateEstimator::correctAttitude()`'s nlerp, which previously hand-rolled its own sign check)
and generic `clampStepVec3()`/`clampSlewQuat()` slew-rate-limit primitives.
`RovStateEstimator::correctContact()` (Phase 3.7) is a new pseudo-measurement update: once all 4
EMA-filtered loadcell channels exceed `ModelParams.contact_force_thresh`, it blends
`v_hat_body_.x() -> 0` and `p_hat_world_.x() -> workspace_x_min` to arrest low-frequency DVL
integration drift during prolonged wall adhesion. `CtrlInterfaces` gained
`pending_thruster_cmd[8]`: every `FSMState::run()` writes there instead of calling
`.set_value()` directly, and `AdhereController::update()` runs the new
`common/OutputSafety.h::sanitizeAndWriteThrusterCommands()` (NaN/Inf → zero all 8 + counter,
else clamp to `[thruster_min,thruster_max]`) as the single centralized write-out gate.
`adhere_controller/config/controllers.yaml` (confirmed dead — never loaded by
`mujoco.launch.py`, all real params live in `robot_control.yaml`) was removed, and
`adhere_mujoco_hardware/config/mujoco_sim.rviz` moved to
`simplified_v1_description/config/mujoco_sim.rviz` so the hardware toolbox package owns zero
model/visualization config files.

### Runtime Diagnostics & Pre-MPPI Infrastructure (Phase 3.6)
`LoopProfiler` (`adhere_controller/include/adhere_controller/common/LoopProfiler.h`, header-only, same style as `MultiRateDecimator.h`) tracks trigger frequency (EMA of inter-call period via `tickMarker()`) and exec latency/overrun-vs-budget (`beginExec()`/`endExec()`). `AdhereMujocoHardware` exposes 2 new state interfaces (`diagnostics/hw_loop_hz`, `diagnostics/hw_step_exec_ms`, now 57 total state interfaces) that simply mirror its existing `MujocoVisBridge::actual_physics_hz`/`step_work_ms` EMA telemetry — `adhere_mujoco_hardware` does NOT depend on `adhere_controller`'s `LoopProfiler`, preserving the hardware/controller package decoupling rule. `AdhereController` instruments sensor sample rates (IMU/loadcell every tick; depth/DVL edge-detected on their sample-and-hold value changing), the estimator, and the inner/outer loops, publishing `RuntimeDiagnostics` at a fixed 10Hz (independent of `inner_decimation_`/`outer_decimation_`) plus a green/red `visualization_msgs::Marker` (`TEXT_VIEW_FACING`) HUD on `/rov/diagnostics_hud_marker`. `estimator_hz` legitimately reads `0.0` whenever `estimation_mode=="ground_truth"` (the estimator simply isn't driven) and is excluded from the `timing_healthy` frequency check for that reason.

`ThrusterAllocator.h` gained `buildPadNormalMap()`/`buildPadNormalMapPseudoInverse()`/`buildContactMap()`/`padHalfSpans()`/`buildKernConstraintMatrix()` building `G∈R^{3x4}`, `G^dagger∈R^{4x3}`, `A_contact∈R^{4x8}`, and `C_kern∈R^{4x8}` from `ModelParams::pad_pos`/`kern_eps_margin` (new field, default `0.20`) — precomputed once in `AdhereController::on_activate()` (members `a_contact_`/`c_kern_`/`kern_l_y_`/`kern_l_z_`), not yet consumed by any live constraint (lands with Phase 4's 12D OSQP), but already used to estimate `ControllerState::contact_forces`/`kern_margin` from the commanded thruster forces. **Sign note**: `G`'s first row uses `+1` (not the `-1` a literal reading of Sec.4.4's prose formula would suggest) — verified against the documented worked example (`u[0:4]=25N` pressing thrust → `A_contact*u=[25,25,25,25]^T N`) in `test_thruster_allocator.cpp`; see the inline comment in `ThrusterAllocator.h::buildPadNormalMap()` before "fixing" this sign back.
