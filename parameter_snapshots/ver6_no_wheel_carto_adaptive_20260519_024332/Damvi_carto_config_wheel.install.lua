include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "imu", -- Ensure imu frame is used as tracking frame
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
  submap_publish_period_sec = 0.025,
  pose_publish_period_sec = 0.025,
  trajectory_publish_period_sec = 0.025,
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

-- online correlative scan matcher
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = false

MAP_BUILDER.use_trajectory_builder_2d = true
TRAJECTORY_BUILDER_2D.use_imu_data = true
TRAJECTORY_BUILDER_2D.skip_submap_insertion_for_outliers = true
TRAJECTORY_BUILDER_2D.outlier_min_correlative_score = 0.0
TRAJECTORY_BUILDER_2D.outlier_max_translation_residual = 0.10
TRAJECTORY_BUILDER_2D.outlier_max_rotation_residual = 0.02
TRAJECTORY_BUILDER_2D.outlier_min_num_filtered_points = 0
TRAJECTORY_BUILDER_2D.outlier_required_failures = 1
TRAJECTORY_BUILDER_2D.outlier_medium_translation_residual = 0.06
TRAJECTORY_BUILDER_2D.outlier_medium_rotation_residual = 0.012
TRAJECTORY_BUILDER_2D.outlier_medium_required_consecutive = 3

-- LiDAR settings
TRAJECTORY_BUILDER_2D.min_range = 0.1
TRAJECTORY_BUILDER_2D.max_range = 25.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5.0

-- LiDAR filter and precision adjustments
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_length = 5.0
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.min_num_points = 200
TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.05

TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 1.0
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(10.0)
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 5.0
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 5.0

-- IMU settings
TRAJECTORY_BUILDER_2D.imu_gravity_time_constant = 30.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 50.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 20.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 20.0

TRAJECTORY_BUILDER_2D.log_local_quality_metrics_to_csv = true
TRAJECTORY_BUILDER_2D.local_quality_metrics_csv_path = "/tmp/ver6_cartographer_local_quality.csv"

-- Optimization and mapping settings
MAP_BUILDER.num_background_threads = 4
POSE_GRAPH.optimize_every_n_nodes = 2 -- Optimization cycle adjustment
POSE_GRAPH.constraint_builder.min_score = 0.65
POSE_GRAPH.constraint_builder.sampling_ratio = 0.0001
POSE_GRAPH.global_sampling_ratio = 0.005
POSE_GRAPH.constraint_builder.max_constraint_distance = 15.0

POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 1.5
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(10.0)

-- Loop closure improvements
POSE_GRAPH.constraint_builder.loop_closure_translation_weight = 2000.0
POSE_GRAPH.constraint_builder.loop_closure_rotation_weight = 2000.0

return options
