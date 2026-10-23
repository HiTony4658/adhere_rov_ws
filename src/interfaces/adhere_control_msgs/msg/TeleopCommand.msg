# Mode-2 (美国手) continuous joystick teleop command (Phase 3.6 quaternion refactor), published
# at 50Hz by xbox_teleop_node on /rov/teleop_cmd. Normalized inputs in [-1.0, 1.0]; consumed by
# StateOpenLoop/StatePidHold/StateAdhere*, with a 0.25s staleness watchdog enforced centrally
# by AdhereController.
std_msgs/Header header

geometry_msgs/Vector3 linear   # Normalized [-1.0, 1.0]: x = surge (+forward toward wall), y = sway, z = heave
geometry_msgs/Vector3 angular  # Normalized [-1.0, 1.0]: x = roll_rate (RT/RB), y = pitch_rate (LT/LB), z = yaw_rate
bool active                    # True when any axis/trigger is outside deadzone
