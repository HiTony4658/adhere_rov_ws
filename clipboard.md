# Task Phase 3.1: Build the FSM Control Framework (`StatePassive` & PID `StateDepthHold`), ROS 2 Foxy Controller Plugin, and Interactive Terminal FSM Console

**IMPORTANT PRE-CHECK**: Read `CLAUDE.md` first to load the frozen architectural and mathematical constitution. **DO NOT modify `CLAUDE.md`**.

---

## Stage 1: Phenomenon Paradox Opening
With our `500Hz` `AdhereMujocoHardware` plant and `Stonefish-Mesh` / `Lumped` hydrodynamics verified under open-loop passive drift, we now need to close the control loop through a modular Finite State Machine (FSM) architecture modeled after `~/Research/quadruped_ros2_control`.

A common engineering pitfall in underwater ROV control is tightly coupling test controllers directly inside the hardware node or hardcoding static setpoints at launch. Instead, we must establish a clean, extensible `ros2_control` FSM pipeline across four packages (`adhere_control_msgs`, `adhere_controller_common`, `adhere_kamr_controller`, and `adhere_keyboard_input`) featuring:
1. **`StatePassive` (`state_passive`)**: Sets all 8 thruster commands (`thr_N1..4`, `thr_T5..8`) to `0.0 N` for safe idle/free-drift operation.
2. **`StateDepthHold` (`state_depth_hold` / 定深控制)**: A closed-loop PID depth controller that maintains the ROV at a user-specified target vertical coordinate / depth (floating-point value in meters), automatically compensating for the net buoyancy bias (`~ +1.96 N` in `rov_wall.xml`) and cross-axis thruster moments.
3. **Interactive Terminal FSM Console (`fsm_console_node`)**: An interactive terminal window that prompts the user which FSM state to enter (`1: Passive`, `2: Depth Hold`), asks for the target depth in meters (float) when `Depth Hold` is selected, and allows seamless runtime switching between states or updating the target depth on the fly.

---

## Stage 2: Mechanism Attribution & Architectural Positioning

### 1. Custom FSM Command Interface (`src/interfaces/adhere_control_msgs`)
- Create `msg/FsmCommand.msg`:
  ```text
  std_msgs/Header header
  int32 command          # 0 = NO_CHANGE, 1 = PASSIVE, 2 = DEPTH_HOLD
  float64 target_z       # Target vertical position Z in world frame (m)
  float64 target_y       # Reserved for future planar station-keeping (m)
  float64 target_yaw     # Reserved for future heading control (rad)


⚬ Configure CMakeLists.txt and package.xml with rosidl_default_generators / rosidl_default_runtime so C++ headers (adhere_control_msgs/msg/fsm_command.hpp) are generated and exported cleanly in ROS 2 Foxy.

2. Core FSM Infrastructure & States (src/controllers/adhere_controller_common)

Borrow the clean design from ~/Research/quadruped_ros2_control/libraries/controller_common and adapt it to our 8-thruster ROV and ROS 2 Foxy API:

⚬ include/adhere_controller_common/common/enumClass.h:
  ⚬ enum class FSMStateName { INVALID, PASSIVE, DEPTH_HOLD };
  ⚬ enum class FSMMode { NORMAL, CHANGE };
⚬ include/adhere_controller_common/common/mathTypes.h & mathTools.h:
  ⚬ Eigen typedefs (Vec3, Vec4, Vec6, Vec8, Quat, Mat3, Mat6, Mat68, RotMat) and helper functions (quatToRotMat, rotMatToRPY, saturation).
⚬ include/adhere_controller_common/CtrlInterfaces.h:
  ⚬ Holds std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> thruster_command_interface_; (size 8: thr_N1..4, thr_T5..8).
  ⚬ Holds std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> for:
    ⚬ thruster_state_interface_ (8)
    ⚬ imu_state_interface_ (10: quat w,x,y,z, gyro x,y,z, accel x,y,z)
    ⚬ odom_state_interface_ (10: pos x,y,z, quat w,x,y,z, vel_body x,y,z)
    ⚬ pad_contact_state_interface_ (8: force TL..BR, v_sep TL..BR)
  ⚬ Holds the latest control input struct (int command = 0; double target_z = 0.5;) and int frequency_ = 500;.
  ⚬ Foxy API Note: In ROS 2 Foxy, LoanedStateInterface::get_value() and LoanedCommandInterface::set_value(val) are used directly (do NOT use Jazzy's get_optional().value()).
⚬ include/adhere_controller_common/FSM/FSMState.h:
  ⚬ Abstract base class with virtual void enter() = 0;, virtual void run(const rclcpp::Time& time, const rclcpp::Duration& period) = 0;, virtual void exit() = 0;, and virtual FSMStateName checkChange() { return FSMStateName::INVALID; }.
⚬ StatePassive (StatePassive.h/.cpp):
  ⚬ On enter() and run(): sets all 8 thruster_command_interface_ values to 0.0.
  ⚬ In checkChange(): transitions to FSMStateName::DEPTH_HOLD when ctrl_interfaces_.control_inputs_.command == 2.
⚬ StateDepthHold (StateDepthHold.h/.cpp):
  ⚬ Inspect simplified_v1_description/mjcf/rov_wall.xml to verify the exact thruster geometry and gear directions of thr_N1..4 and thr_T5..8.
  ⚬ PID Depth Control Law:
    ⚬ Read current World Z position z = \text{odom\_state\_interface\_}[2].\text{get\_value}(), orientation quaternion q = (w, x, y, z), body linear velocity \boldsymbol{v}_b, and body angular velocity \boldsymbol{\omega}_b. Compute World vertical velocity v_{z,W} = (\boldsymbol{R} \boldsymbol{v}_b)_z.
    ⚬ Target vertical coordinate z_{\text{des}} = \text{ctrl\_interfaces\_}.\text{target\_z} (updated live whenever the user enters a new target depth in the terminal, without resetting the integrator unless entering from PASSIVE!).
    ⚬ Compute error e_z = z_{\text{des}} - z, update clamped integral I_z \gets \text{clamp}(I_z + e_z \Delta t, -I_{\max}, I_{\max}) (anti-windup), and compute desired World vertical force using derivative-on-measurement (to prevent derivative kick on setpoint changes):
      
      $$F_{z,\text{world}}^{\text{des}} = K_{p,z} e_z + K_{i,z} I_z - K_{d,z} v_{z,W}$$
    ⚬ Include gentle attitude damping / stabilization on roll, pitch, and yaw (and parasitic pitch moment compensation if tangential thrusters have an X = -0.10\text{ m} lever arm offset from CoM) and map the desired wrench into the 8 thruster commands u_i \in [-40.0, 40.0]\text{ N} via the exact thruster allocation matrix B.
    ⚬ In checkChange(): transitions to FSMStateName::PASSIVE when ctrl_interfaces_.control_inputs_.command == 1.

3. ROS 2 Foxy Controller Plugin (src/controllers/adhere_kamr_controller)

⚬ Implement AdhereFsmController (or AdhereKamrController in adhere_kamr_controller) inheriting from controller_interface::ControllerInterface:
  ⚬ Follow ROS 2 Foxy's exact lifecycle signatures from CLAUDE.md Section 2 (init, state_interface_configuration, command_interface_configuration, on_configure, on_activate, on_deactivate, update()).
  ⚬ Export the plugin via pluginlib_export_plugin_description_file(controller_interface adhere_kamr_controller.xml).
  ⚬ In on_configure(), create a subscription to /rov/fsm_command (adhere_control_msgs::msg::FsmCommand) using a thread-safe atomic/mutex structure so user commands from the terminal console update ctrl_interfaces_ safely.
  ⚬ Initialize the FSM in FSMStateName::PASSIVE by default.
  ⚬ In update(), compute time and period using get_node()->now(), execute the active FSMState::run(time, period), check checkChange(), and perform clean exit() -> enter() state transitions when requested.
⚬ In adhere_mujoco_node.cpp and launch/mujoco_sim.launch.py:
  ⚬ Ensure controller_manager loads, configures, and activates adhere_kamr_controller automatically (either via spawner node in mujoco_sim.launch.py or programmatic load/configure/activate in adhere_mujoco_node + config YAML) so that launching mujoco_sim.launch.py immediately brings up the active FSM controller in PASSIVE state!

4. Interactive Terminal FSM Console (src/commands/adhere_keyboard_input)

⚬ Implement src/fsm_console_node.cpp in adhere_keyboard_input:
  ⚬ Subscribes to /rov/odom (nav_msgs/msg/Odometry) to display the ROV's current Z coordinate when prompting.
  ⚬ Provides a clean, interactive command-line menu:
    ============================================================
      Underwater ROV FSM Control Console (有限状态机终端控制台)
    ============================================================
      [1] state_passive    (推进器全零输出 u = 0 N，自由漂流)
      [2] state_depth_hold (PID 定深控制，输入目标深度 Z 坐标)
      [q] Quit Console     (退出控制台)
    ------------------------------------------------------------
    Current State: [1] state_passive | Current Z: 0.500 m
    Select state (1/2/q):
    
  ⚬ When the user enters 1: publishes FsmCommand{command = 1} to /rov/fsm_command and prints confirmation that all thrusters are zeroed (state_passive).
  ⚬ When the user enters 2: prompts the user:
    Enter target depth Z in meters (float, e.g. 0.50; water surface is 1.00): 
    Reads the floating-point number, validates it, and publishes FsmCommand{command = 2, target_z = val} to /rov/fsm_command.
⚬ In launch/mujoco_sim.launch.py:
  ⚬ Add a launch argument spawn_fsm_console (default: "true") that automatically opens fsm_console_node in a dedicated terminal window (gnome-terminal or xterm), while also allowing manual execution via ros2 run adhere_keyboard_input fsm_console_node.

Stage 3: Deterministic Timing & Mathematical Contract

1. Zero-Output Contract in StatePassive:
  ⚬ When in StatePassive, all 8 thruster commands must be identically 0.0 N (|u_i| == 0.0), and the ROV must exhibit the exact same passive free-drift behavior verified in Phase 2.
2. Closed-Loop Depth Tracking Contract in StateDepthHold:
  ⚬ Given any reasonable target depth z_{\text{des}} \in [0.15, 0.85]\text{ m} (e.g. z_{\text{des}} = 0.40\text{ m} or z_{\text{des}} = 0.70\text{ m}), the PID controller must smoothly drive the ROV's vertical coordinate z(t) \to z_{\text{des}} with zero steady-state offset (\vert{}z(\infty) - z_{\text{des}}\vert{} < 5\text{ mm}), minimal overshoot, and stable attitude.
  ⚬ Switching back to 1 (StatePassive) must immediately zero all 8 thrusters (u_i = 0.0 N), causing the ROV to resume passive drift under buoyancy.

Stage 4: Defensive Sandbox Evolution & Implementation Steps

1. Build src/interfaces/adhere_control_msgs:
  ⚬ Define msg/FsmCommand.msg, update CMakeLists.txt and package.xml, and remove any leftover .gitkeep in msg/.
2. Build src/controllers/adhere_controller_common:
  ⚬ Implement enumClass.h, mathTypes.h, mathTools.h, CtrlInterfaces.h, FSMState.h, StatePassive.h/.cpp, and StateDepthHold.h/.cpp.
  ⚬ Remove .gitkeep placeholders from populated directories.
3. Build src/controllers/adhere_kamr_controller:
  ⚬ Implement the ROS 2 Foxy controller_interface::ControllerInterface plugin managing the FSM (StatePassive & StateDepthHold), export the plugin XML, and configure config/controllers.yaml.
4. Build src/commands/adhere_keyboard_input & Integrate into mujoco_sim.launch.py:
  ⚬ Implement fsm_console_node.cpp and integrate controller activation + terminal console spawning into adhere_mujoco_hardware/launch/mujoco_sim.launch.py.

Stage 5: Closed-Loop Verification, Documentation Discipline & Git Push

1. Build & Closed-Loop Verification:
  ⚬ Run colcon build --symlink-install across all 6 packages in the workspace with zero errors or warnings.
  ⚬ Run an automated closed-loop test verifying the complete state machine lifecycle:
    1. Startup in default StatePassive (t \in [0, 2]\text{ s}): verify all 8 thrusters read 0.0 N and ROV drifts upward under net positive buoyancy;
    2. Switch to StateDepthHold with z_{\text{des}} = 0.40\text{ m} (t \in [2, 8]\text{ s}): verify thrusters engage and z(t) converges accurately to 0.400 \pm 0.005\text{ m};
    3. Update target depth to z_{\text{des}} = 0.75\text{ m} (t \in [8, 14]\text{ s}): verify z(t) tracks smoothly to 0.750 \pm 0.005\text{ m};
    4. Switch back to StatePassive (t \in [14, 16]\text{ s}): verify all 8 thrusters immediately return to 0.0 N.
2. Strict Documentation Audit (README.md vs. PROGRESS.md):
  ⚬ Update the Chinese README.md files in the root workspace, adhere_control_msgs, adhere_controller_common, adhere_kamr_controller, adhere_keyboard_input, and adhere_mujoco_hardware with clear functional explanations, PID depth-hold architecture, and run tutorials. Ensure zero changelog/modification history is placed in any README.md!
  ⚬ Record all Phase 3.1 implementation details and closed-loop depth-hold tracking telemetry in top-level PROGRESS.md.
3. Git Commit & Push:
  ⚬ Stage all changes (git add -A), commit with a descriptive English message, and run git push.