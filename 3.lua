-- Rank 3
-- Mode: PUBLISH_ONLY
-- Source: retune_publish_center_strong_both (2 repeated runs)
-- Summary:
--   mean_accepted_ratio = 0.0583948874
--   worst_longest_no_candidate_streak = 124
--   worst_no_candidate_ratio = 0.4330918795
--   worst_min_best_score = 0.1720
--   worst_p05_best_score = 0.374180
-- Notes:
--   Publish-only frozen matcher profile with looser correction caps.
--   Guardrails: near-miss

TRAJECTORY_BUILDER_2D.frozen_submap_scan_matcher.enabled = true
TRAJECTORY_BUILDER_2D.frozen_submap_scan_matcher.apply_mode = "PUBLISH_ONLY"
TRAJECTORY_BUILDER_2D.frozen_submap_scan_matcher.search_radius = 1.60
TRAJECTORY_BUILDER_2D.frozen_submap_scan_matcher.max_submaps_to_match = 60
TRAJECTORY_BUILDER_2D.frozen_submap_scan_matcher.min_realtime_correlative_score = 0.61
TRAJECTORY_BUILDER_2D.frozen_submap_scan_matcher.min_score_margin = 0.0100
TRAJECTORY_BUILDER_2D.frozen_submap_scan_matcher.score_variance_top_k = 3
TRAJECTORY_BUILDER_2D.frozen_submap_scan_matcher.min_score_variance = 0.000400
TRAJECTORY_BUILDER_2D.frozen_submap_scan_matcher.max_translation_correction = 0.140
TRAJECTORY_BUILDER_2D.frozen_submap_scan_matcher.max_rotation_correction = math.rad(0.50)
TRAJECTORY_BUILDER_2D.frozen_submap_scan_matcher.use_realtime_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.frozen_submap_scan_matcher.use_ceres_scan_matching = true
TRAJECTORY_BUILDER_2D.frozen_submap_scan_matcher.tuning_log_enabled = true
TRAJECTORY_BUILDER_2D.frozen_submap_scan_matcher.tuning_log_log_rejections = true
TRAJECTORY_BUILDER_2D.frozen_submap_scan_matcher.tuning_log_log_acceptances = true
TRAJECTORY_BUILDER_2D.frozen_submap_scan_matcher.tuning_log_detail_every_n_scans = 0
TRAJECTORY_BUILDER_2D.frozen_submap_scan_matcher.tuning_log_summary_every_n_scans = 50
TRAJECTORY_BUILDER_2D.frozen_submap_scan_matcher.tuning_log_top_candidates = 3
TRAJECTORY_BUILDER_2D.frozen_submap_scan_matcher.real_time_correlative_scan_matcher.linear_search_window = 1.40
TRAJECTORY_BUILDER_2D.frozen_submap_scan_matcher.real_time_correlative_scan_matcher.angular_search_window = math.rad(1.00)
TRAJECTORY_BUILDER_2D.frozen_submap_scan_matcher.real_time_correlative_scan_matcher.translation_delta_cost_weight = 1.00
TRAJECTORY_BUILDER_2D.frozen_submap_scan_matcher.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1.00
TRAJECTORY_BUILDER_2D.frozen_submap_scan_matcher.ceres_scan_matcher.occupied_space_weight = 60.00
TRAJECTORY_BUILDER_2D.frozen_submap_scan_matcher.ceres_scan_matcher.translation_weight = 10.00
TRAJECTORY_BUILDER_2D.frozen_submap_scan_matcher.ceres_scan_matcher.rotation_weight = 10.00
TRAJECTORY_BUILDER_2D.frozen_submap_scan_matcher.ceres_scan_matcher.ceres_solver_options.use_nonmonotonic_steps = true
TRAJECTORY_BUILDER_2D.frozen_submap_scan_matcher.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 50
TRAJECTORY_BUILDER_2D.frozen_submap_scan_matcher.ceres_scan_matcher.ceres_solver_options.num_threads = 8
