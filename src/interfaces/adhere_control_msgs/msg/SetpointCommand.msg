# Discrete state-specific numeric setpoint (Phase 3.6 quaternion refactor), published by
# state_cmd_console_node on /rov/setpoint_cmd. Orthogonal to FsmCommand: never switches states,
# only updates the active state's reference while command_type matches its own FSM state ID.
std_msgs/Header header

uint8 MODE_DEPTH_ONLY = 1      # Update target_pose.position.z only (HOLD_PID_DEPTH / HOLD_PID_6DOF)
uint8 MODE_FULL_POSE = 2       # Update full 6-DOF target_pose (position + quaternion orientation)
uint8 MODE_LOCK_CURRENT = 3    # Latch current estimated pose as target_pose
uint8 MODE_LEVEL_ATTITUDE = 4  # Reset target roll & pitch to 0 while keeping current target yaw & position

uint8 command_type
geometry_msgs/Pose target_pose # Position (m) in world frame + unit Quaternion (x,y,z,w) in world frame
