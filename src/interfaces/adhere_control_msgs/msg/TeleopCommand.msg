# Mode-2 (美国手) continuous joystick teleop command (CLAUDE.md Phase 3.2 Stage 2.1), published
# at 50Hz by xbox_teleop_node on /rov/teleop_cmd. Normalized inputs in [-1.0, 1.0]; consumed by
# StateOpenLoop/StatePidHold/StateDepthHold, with a 0.25s staleness watchdog enforced centrally
# by AdhereController.
std_msgs/Header header

float64 surge_x        # Right stick up/down: +1.0 = forward toward wall (-X body), -1.0 = backward (+X body)
float64 sway_y         # Right stick left/right: lateral velocity command
float64 heave_z        # Left stick up/down: +1.0 = ascend (+Z), -1.0 = descend (-Z)
float64 yaw_rate       # Left stick left/right: +1.0 = CCW yaw (+omega_z), -1.0 = CW yaw (-omega_z)
bool active            # True when at least one stick axis is outside its deadzone
