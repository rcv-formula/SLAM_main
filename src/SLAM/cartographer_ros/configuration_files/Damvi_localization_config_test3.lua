include "Damvi_localization_config_test1.lua"

-- Test 3: offset-decay comparison mode.
-- /odom stays on the raw local-matcher path, /filtered_odom exposes the direct
-- frozen-matcher candidate, and /offset_odom exposes the decayed offset result.
TRAJECTORY_BUILDER_2D.frozen_submap_scan_matcher.apply_mode = "OFFSET_DECAY"
TRAJECTORY_BUILDER_2D.frozen_submap_scan_matcher.test_mode_publish_filtered_odom = true
TRAJECTORY_BUILDER_2D.frozen_submap_scan_matcher.filtered_odom_publish_only_on_accept = true

-- A deliberately gentle starting point: accepted frozen matches move the
-- filtered pose 10% toward the new offset each scan, then the offset fades as
-- time and traveled distance grow.
TRAJECTORY_BUILDER_2D.frozen_submap_scan_matcher.offset_decay_blend_alpha = 0.10
TRAJECTORY_BUILDER_2D.frozen_submap_scan_matcher.offset_decay_time_constant_sec = 5.0
TRAJECTORY_BUILDER_2D.frozen_submap_scan_matcher.offset_decay_distance_constant_m = 8.0
TRAJECTORY_BUILDER_2D.frozen_submap_scan_matcher.offset_decay_reset_on_global_optimization = false

return options
