# Discrete state-specific numeric setpoint (CLAUDE.md Phase 3.2 Stage 2.1), published by
# state_cmd_console_node on /rov/setpoint_cmd. Orthogonal to FsmCommand: never switches states,
# only updates the active state's reference while mode matches its own FSM state ID.
std_msgs/Header header

uint8 mode             # 2 = DEPTH_HOLD (use target_z), 4 = PID_HOLD (use target_x/y/z/roll/pitch/yaw or lock_current_pose)
bool lock_current_pose # If true, re-latch the ROV's current pose as the hold point instead of using target_*
float64 target_x       # World X target (m)
float64 target_y       # World Y target (m)
float64 target_z       # World Z target (m)
float64 target_roll    # Roll target (rad), PID_HOLD only
float64 target_pitch   # Pitch target (rad), PID_HOLD only
float64 target_yaw     # World Yaw target (rad)
