# Live controller feedback for terminals (Phase 3.6, 100% Sim-to-Real pure: replaces FsmState.msg
# -- ground truth no longer appears here at all, it is published separately by adhere_mujoco_node
# on /rov/ground_truth_odom for offline evaluation only). Published at the outer-loop rate by
# AdhereController on /rov/controller_state. fsm_console_node and state_cmd_console_node both
# subscribe to this (never to raw hardware/odom topics) so they work identically whether the
# state was switched via keyboard or via Xbox gamepad buttons.
std_msgs/Header header

int32 current_state                      # 1=IDLE_PASSIVE_NONE, 2=HOLD_PID_DEPTH, 3=TELEOP_OPENLOOP_6DOF, 4=HOLD_PID_6DOF, 5=ADHERE_PID_WBC, 6=ADHERE_SMC_WBC, 7=ADHERE_MPPI_BASELINE, 8=ADHERE_MPPI_KAMR
string state_name
string estimation_mode                   # "ground_truth" or "sensor_fusion" (CtrlInterfaces::active_state source)
geometry_msgs/Pose target_pose           # Active reference pose (World frame, Quaternion), meaning is per-state -- only meaningful when target_valid
bool target_valid                        # Phase 4.3: true for HOLD_PID_DEPTH/HOLD_PID_6DOF/ADHERE_*, false for IDLE_PASSIVE_NONE/TELEOP_OPENLOOP_6DOF (no setpoint)
string adhere_stage                      # "" except for ADHERE_PID_WBC/ADHERE_SMC_WBC/ADHERE_MPPI_BASELINE: "DEPTH_ALIGN" or "WALL_ADHERE" (Phase 4.2)
geometry_msgs/Pose estimated_pose        # Active pose used by the controller (World frame, Quaternion)
geometry_msgs/Twist estimated_twist_body # Active body-frame linear (m/s) & angular (rad/s) velocity
float64[4] contact_forces                # Active 4 pad normal forces [TL, TR, BL, BR] (N), estimated via A_contact*u
float64[8] thruster_cmds                 # Commanded 8 thruster forces u[0..7] (N)

# Phase 3.8 efficiency/chattering/safety telemetry (CLAUDE.md Phase 3.8): computed centrally by
# AdhereController::update() at every 250Hz inner-loop tick from the actually-sent (sanitized +
# saturated) thruster commands, published here at the outer-loop rate. cumulative_* integrators
# reset to 0 on every FSM state transition (common/ControllerMetrics.h).
float64 thrust_l2_norm                   # Instantaneous ||u||_2 (N)
float64 thrust_power_proxy               # Instantaneous hydrodynamic power proxy sum_i |u_i|^1.5 (N^1.5)
float64 thrust_slew_rate_l2              # Instantaneous ||(u[k] - u[k-1]) / dt||_2 (N/s) -- chattering indicator
float64 cumulative_energy_l2             # Integrated int ||u||_2^2 dt since state entry (N^2 * s)
float64 cumulative_impulse_l1            # Integrated int ||u||_1 dt since state entry (N * s)
uint8 saturated_thruster_count           # Number of thrusters currently at >= 98% of their per-thruster thruster_max
uint8 active_contact_pad_count           # Number of pads with estimated contact_forces[i] > contact_force_thresh

float64 kern_margin                      # Rhombic kern safety margin M_kern = 1 - K_val
