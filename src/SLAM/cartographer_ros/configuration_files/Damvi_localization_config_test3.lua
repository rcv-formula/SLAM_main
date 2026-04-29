include "Damvi_localization_config_test1.lua"

-- Test 3: publish-only comparison mode.
-- /odom stays on the raw local-matcher path and /filtered_odom publishes the
-- frozen-matcher corrected path for side-by-side evaluation.
TRAJECTORY_BUILDER_2D.frozen_submap_scan_matcher.test_mode_publish_filtered_odom = true

return options
