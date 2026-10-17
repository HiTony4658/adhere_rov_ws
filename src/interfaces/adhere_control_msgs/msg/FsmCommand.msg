# FSM state-change request, published by fsm_console_node (or any future teleop/benchmark
# node) and consumed by AdhereKamrController's /rov/fsm_command subscription.
std_msgs/Header header

int32 NO_CHANGE = 0
int32 PASSIVE = 1
int32 DEPTH_HOLD = 2

int32 command          # 0 = NO_CHANGE, 1 = PASSIVE, 2 = DEPTH_HOLD
float64 target_z       # Target vertical position Z in world frame (m)
float64 target_y       # Reserved for future planar station-keeping (m)
float64 target_yaw     # Reserved for future heading control (rad)
