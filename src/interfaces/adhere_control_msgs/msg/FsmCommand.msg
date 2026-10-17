# FSM state-switch request ONLY (CLAUDE.md Phase 3.2 Stage 2.1): published by fsm_console_node
# or xbox_teleop_node's gamepad buttons, consumed by AdhereController's /rov/fsm_command
# subscription. Intra-state setpoints/teleop now live on SetpointCommand/TeleopCommand instead.
std_msgs/Header header

int32 NO_CHANGE = 0
int32 PASSIVE = 1
int32 DEPTH_HOLD = 2
int32 OPEN_LOOP = 3
int32 PID_HOLD = 4

int32 command          # 0 = NO_CHANGE, 1 = PASSIVE, 2 = DEPTH_HOLD, 3 = OPEN_LOOP, 4 = PID_HOLD
