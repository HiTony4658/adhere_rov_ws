# Live wall-target retargeting for the ADHERE_* states ONLY (new topic, this phase): never
# switches FSM state (see FsmCommand) and never carries a generic 6-DOF pose (see
# SetpointCommand) -- scoped strictly to mutating the shared AdhereStageParams target_y/
# target_depth_m at runtime, e.g. from a wall QR-code/AprilTag preset or a console-typed custom
# value. Published by state_cmd_console_node on /rov/adhere_target_cmd, consumed by
# AdhereController, which resolves TAG_PRESET against robot_control.yaml's adhere_wall_tags
# table and writes the result into CtrlInterfaces' live AdhereTargetState.
std_msgs/Header header

uint8 TAG_PRESET = 1       # Resolve tag_id against the robot_control.yaml adhere_wall_tags table
uint8 CUSTOM_Y = 2         # Update target_y only, keep current target_depth_m
uint8 CUSTOM_DEPTH = 3     # Update target_depth_m only, keep current target_y
uint8 CUSTOM_Y_DEPTH = 4   # Update both target_y and target_depth_m
uint8 USE_CURRENT = 5      # Latch the ROV's current estimated (y, depth) as the new target

uint8 command_type
uint8 tag_id            # Valid only when command_type == TAG_PRESET (0..3 by default)
float64 target_y        # World-frame Y (m), valid for CUSTOM_Y / CUSTOM_Y_DEPTH
float64 target_depth_m  # Depth below the water surface (m), valid for CUSTOM_DEPTH / CUSTOM_Y_DEPTH
