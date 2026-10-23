# Measured wall-tag station detection (CLAUDE.md Phase 5.0 Stage 6), published on
# /rov/tag_detection by adhere_apriltag_detector -- kept strictly distinct from
# /rov/adhere_target_cmd (human/preset intent): this message is a MEASUREMENT. A fresh valid
# detection differing from AdhereController's active resolved target by more than
# adhere_stage.retarget_threshold supersedes the coarse preset and forces a fresh
# STANDOFF_HOMING pass (reusing the existing changed_pending path) -- a received tag governs
# over a later disagreeing preset until the operator explicitly re-presets via
# /rov/adhere_target_cmd. station_id = min tag id of the detected 2x2 bundle / 4, indexing the
# same robot_control.yaml adhere_wall_tags_y/adhere_wall_tags_depth_m table
# /rov/adhere_target_cmd's TAG_PRESET already resolves against.
std_msgs/Header header

uint8 station_id        # min tag id of the bundle / 4 (0..3 by default), indexes adhere_wall_tags
float64 target_y        # World-frame Y (m) of the detected bundle centre
float64 target_depth_m  # Depth below the water surface (m) of the detected bundle centre
float64 confidence      # [0,1]: detection quality / fraction of bundle tags seen
bool valid              # false when the ROV is outside detection range of every station
