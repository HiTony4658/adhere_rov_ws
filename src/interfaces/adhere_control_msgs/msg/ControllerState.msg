# Live controller feedback for terminals (Phase 3.6, 100% Sim-to-Real pure: replaces FsmState.msg
# -- ground truth no longer appears here at all, it is published separately by adhere_mujoco_node
# on /rov/ground_truth_odom for offline evaluation only). Published at the outer-loop rate by
# AdhereController on /rov/controller_state. fsm_console_node and state_cmd_console_node both
# subscribe to this (never to raw hardware/odom topics) so they work identically whether the
# state was switched via keyboard or via Xbox gamepad buttons.
std_msgs/Header header

int32 current_state                      # 1=PASSIVE, 2=DEPTH_HOLD, 3=OPEN_LOOP, 4=PID_HOLD, 5=BASELINE_MPPI, 6=KAMR_MPPI
string state_name
string estimation_mode                   # "ground_truth" or "sensor_fusion" (CtrlInterfaces::active_state source)
geometry_msgs/Pose target_pose           # Active reference pose (World frame, Quaternion), meaning is per-state
geometry_msgs/Pose estimated_pose        # Active pose used by the controller (World frame, Quaternion)
geometry_msgs/Twist estimated_twist_body # Active body-frame linear (m/s) & angular (rad/s) velocity
float64[4] contact_forces                # Active 4 pad normal forces [TL, TR, BL, BR] (N), estimated via A_contact*u
float64[8] thruster_cmds                 # Commanded 8 thruster forces u[0..7] (N)
float64 kern_margin                      # Rhombic kern safety margin M_kern = 1 - K_val
