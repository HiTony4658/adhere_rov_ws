# Live controller feedback for terminals (Phase 3.6, 100% Sim-to-Real pure: replaces FsmState.msg
# -- ground truth no longer appears here at all, it is published separately by adhere_mujoco_node
# on /rov/ground_truth_odom for offline evaluation only). Published at the outer-loop rate by
# AdhereController on /rov/controller_state. fsm_console_node (adhere_teleop_gcs) subscribes to
# this (never to raw hardware/odom topics) so it works identically whether the state was
# switched via keyboard or via Xbox gamepad buttons. Phase 5.0 Stage 7 split the former
# efficiency/chattering/safety telemetry fields out into BenchmarkMetrics.msg (/rov/
# benchmark_metrics) -- this message stays lean FSM/pose feedback.
std_msgs/Header header

int32 current_state                      # 1=IDLE_PASSIVE_NONE, 2=TELEOP_OPENLOOP_6DOF, 3=HOLD_PID_6DOF, 4=ADHERE_PID_WBC, 5=ADHERE_SMC_WBC, 6=ADHERE_MPPI_BASELINE, 7=ADHERE_MPPI_KAMR
string state_name
string estimation_mode                   # "ground_truth" or "sensor_fusion" (CtrlInterfaces::active_state source)
geometry_msgs/Pose target_pose           # Active reference pose (World frame, Quaternion), meaning is per-state -- only meaningful when target_valid
bool target_valid                        # Phase 4.3: true for HOLD_PID_6DOF/ADHERE_*, false for IDLE_PASSIVE_NONE/TELEOP_OPENLOOP_6DOF (no setpoint)
string adhere_stage                      # "" except for ADHERE_PID_WBC/ADHERE_SMC_WBC/ADHERE_MPPI_BASELINE: "STANDOFF_HOMING"/"SOFT_DOCKING"/"WALL_ADHERED" (Phase 4.2, renamed/extended Phase 4.5)
geometry_msgs/Pose estimated_pose        # Active pose used by the controller (World frame, Quaternion)
geometry_msgs/Twist estimated_twist_body # Active body-frame linear (m/s) & angular (rad/s) velocity
float64[4] contact_forces                # Active 4 pad normal forces [TL, TR, BL, BR] (N), estimated via A_contact*u
float64[8] thruster_cmds                 # Commanded 8 thruster forces u[0..7] (N)
