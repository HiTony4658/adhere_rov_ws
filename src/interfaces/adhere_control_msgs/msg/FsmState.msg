# Live controller feedback for terminals (CLAUDE.md Phase 3.2 Stage 2.1), published at the
# outer-loop rate by AdhereController on /rov/fsm_state. fsm_console_node and
# state_cmd_console_node both subscribe to this (never to raw hardware/odom topics) so they work
# identically whether the state was switched via keyboard or via Xbox gamepad buttons.
std_msgs/Header header

int32 current_state    # 1 = PASSIVE, 2 = DEPTH_HOLD, 3 = OPEN_LOOP, 4 = PID_HOLD
string state_name
float64[4] target_pose # [target_x, target_y, target_z, target_yaw], meaning is per-state (see FSMState::targetPose())
float64[4] actual_pose # [x, y, z, yaw]
