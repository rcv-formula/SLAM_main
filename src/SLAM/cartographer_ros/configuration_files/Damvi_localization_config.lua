include "map_builder.lua"
include "trajectory_builder.lua"


local fast_correlative_score_distribution_csv_path =
    os.getenv("FAST_CORRELATIVE_SCORE_DISTRIBUTION_CSV_PATH") or
    "/home/rcv/SLAM_local-loss/global_constraint_score_distributions/fast_correlative_score_distribution.csv"

-- 기본 설정
options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "imu",
  published_frame = "base_link",
  odom_frame = "odom",
  provide_odom_frame = true,
  use_odometry = false,
  use_nav_sat = false,
  use_landmarks = false,
  publish_frame_projected_to_2d = true,
  use_pose_extrapolator = true,
  publish_to_tf = true,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.1,
  pose_publish_period_sec = 0.05,
  trajectory_publish_period_sec = 0.1,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  rangefinder_sampling_ratio = 1.0,
  odometry_sampling_ratio = 1.0,
  imu_sampling_ratio = 1.0,
  fixed_frame_pose_sampling_ratio = 1.0,
  landmarks_sampling_ratio = 1.0,
}

-- 2D Trajectory 설정
MAP_BUILDER.use_trajectory_builder_2d = true
TRAJECTORY_BUILDER_2D.use_imu_data = true

-- 해상도 설정
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.05

-- Pure Localization 공통 설정
POSE_GRAPH.constraint_builder.min_score = 0.95
TRAJECTORY_BUILDER.pure_localization_trimmer = {
  max_submaps_to_keep = 5,
}
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1

-- Pose graph constrained matcher 설정
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 0.05
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(1.0)
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.log_score_distribution_to_csv = true
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.score_distribution_csv_path =
    fast_correlative_score_distribution_csv_path

-- 실시간 local matcher 설정
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.05
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(1.0)
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 25.0
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 25.0

-- LiDAR 관련
TRAJECTORY_BUILDER_2D.min_range = 0.1
TRAJECTORY_BUILDER_2D.max_range = 20.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5.0
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_length = 5.0
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.min_num_points = 350
TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.05

-- Ceres 기반 Scan Matcher 설정
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 15.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 30.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 30.0

-- IMU 설정
TRAJECTORY_BUILDER_2D.imu_gravity_time_constant = 12.0

MAP_BUILDER.num_background_threads = 8

-- 기타 posegraph 관련
POSE_GRAPH.optimize_every_n_nodes = 1
POSE_GRAPH.constraint_builder.max_constraint_distance = 15.0
POSE_GRAPH.constraint_builder.use_prior_based_ambiguity_filter = true
POSE_GRAPH.constraint_builder.ambiguity_top2_margin = 0.015
POSE_GRAPH.constraint_builder.ambiguity_max_near_top_0p02_candidates = 4
POSE_GRAPH.constraint_builder.ambiguity_max_prior_translation = 0.15
POSE_GRAPH.constraint_builder.ambiguity_max_prior_rotation = math.rad(3.0)
POSE_GRAPH.constraint_builder.loop_closure_translation_weight = 100.0
POSE_GRAPH.constraint_builder.loop_closure_rotation_weight = 100.0
POSE_GRAPH.constraint_builder.sampling_ratio = 0.78

-- 시작 pose graph 값
POSE_GRAPH.initial_global_localization_min_score = 0.5
POSE_GRAPH.initial_global_constraint_search_after_n_seconds = 0
POSE_GRAPH.initial_global_sampling_ratio = 0.05

-- 시작 이후 tracking 값
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.6
POSE_GRAPH.global_constraint_search_after_n_seconds = 5
POSE_GRAPH.global_sampling_ratio = 0.006

return options
