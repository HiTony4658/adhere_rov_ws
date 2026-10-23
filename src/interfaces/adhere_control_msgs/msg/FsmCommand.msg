# FSM state-switch request ONLY (CLAUDE.md Phase 3.2 Stage 2.1): published by fsm_console_node
# or xbox_teleop_node's gamepad buttons, consumed by AdhereController's /rov/fsm_command
# subscription. Intra-state setpoints/teleop now live on SetpointCommand/TeleopCommand instead.
# Phase 4.3: state names follow the FUNCTION_ALGORITHM_CATEGORY convention. Phase 5.0: HOLD_PID_
# DEPTH (redundant with HOLD_PID_6DOF) was deleted and IDs compacted to 1-7 -- a one-time,
# owner-confirmed relaxation of the former "renaming MUST NEVER renumber" rule (CLAUDE.md
# Sec.7.5); IDs are a stable contract again going forward from this point.
std_msgs/Header header

int32 NO_CHANGE = 0
int32 IDLE_PASSIVE_NONE = 1
int32 TELEOP_OPENLOOP_6DOF = 2
int32 HOLD_PID_6DOF = 3
int32 ADHERE_PID_WBC = 4
int32 ADHERE_SMC_WBC = 5
int32 ADHERE_MPPI_BASELINE = 6
int32 ADHERE_MPPI_KAMR = 7

int32 command          # 0=NO_CHANGE, 1=IDLE_PASSIVE_NONE, 2=TELEOP_OPENLOOP_6DOF, 3=HOLD_PID_6DOF, 4=ADHERE_PID_WBC, 5=ADHERE_SMC_WBC, 6=ADHERE_MPPI_BASELINE, 7=ADHERE_MPPI_KAMR
