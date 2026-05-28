include "map_builder.lua"
include "trajectory_builder.lua"

local function env_bool_or_default(name, default)
  local value = os.getenv(name)
  if value == nil or value == "" then
    return default
  end
  return value == "1" or value == "true" or value == "TRUE" or
      value == "on" or value == "ON"
end

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "imu",
  published_frame = "base_link",
  odom_frame = "odom",
  provide_odom_frame = true,
  use_odometry = true,
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
  wheel_odom_twist_only = true,
  wheel_odom_linear_scale = 2.6,
  wheel_odom_yaw_weight = 0.0,
  adaptive_odometry_blend = true,
  adaptive_odometry_full_weight_yaw_rate = 0.05,
  adaptive_odometry_zero_weight_yaw_rate = 0.20,
  adaptive_odometry_min_weight = 0.0,
  adaptive_odometry_max_weight = 1.0,
  adaptive_odometry_mismatch_override = true,
  adaptive_odometry_mismatch_ratio = 0.35,
  adaptive_odometry_min_forward_delta = 0.005,
  adaptive_odometry_mismatch_force_weight = 1.0,
  adaptive_odometry_longitudinal_only = true,
}

TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching =
    env_bool_or_default("CARTOGRAPHER_USE_ONLINE_CORRELATIVE_SCAN_MATCHING",
                        false)

MAP_BUILDER.use_trajectory_builder_2d = true
TRAJECTORY_BUILDER_2D.use_imu_data = true
TRAJECTORY_BUILDER_2D.log_local_quality_metrics_to_csv =
    (os.getenv("LOCAL_QUALITY_METRICS_CSV_PATH") or "") ~= ""
TRAJECTORY_BUILDER_2D.local_quality_metrics_csv_path =
    os.getenv("LOCAL_QUALITY_METRICS_CSV_PATH") or ""
TRAJECTORY_BUILDER_2D.skip_submap_insertion_for_outliers = true
TRAJECTORY_BUILDER_2D.outlier_min_correlative_score = 0.0
TRAJECTORY_BUILDER_2D.outlier_max_translation_residual = 0.08
TRAJECTORY_BUILDER_2D.outlier_max_rotation_residual = 0.035
TRAJECTORY_BUILDER_2D.outlier_min_num_filtered_points = 0
TRAJECTORY_BUILDER_2D.outlier_required_failures = 1
TRAJECTORY_BUILDER_2D.outlier_medium_translation_residual = 0.04
TRAJECTORY_BUILDER_2D.outlier_medium_rotation_residual = 0.026
TRAJECTORY_BUILDER_2D.outlier_medium_required_consecutive = 2

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
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.longitudinal_translation_weight = 0.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.longitudinal_translation_min_speed = 0.05
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.longitudinal_translation_max_yaw_rate = 0.60
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.longitudinal_prior_wheel_delta_scale = 1.0

-- Optimization and mapping settings
MAP_BUILDER.num_background_threads = 4
POSE_GRAPH.optimize_every_n_nodes = 2
POSE_GRAPH.constraint_builder.min_score = 0.65
POSE_GRAPH.constraint_builder.sampling_ratio = 0.0001
POSE_GRAPH.global_sampling_ratio = 0.005
POSE_GRAPH.constraint_builder.max_constraint_distance = 15.0

POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 1.5
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(10.0)

-- Loop closure improvements
POSE_GRAPH.constraint_builder.loop_closure_translation_weight = 2000.0
POSE_GRAPH.constraint_builder.loop_closure_rotation_weight = 2000.0

-- Wheel odom is useful as a weak translation prior, but its yaw can corrupt
-- corners in mapping when it is weighted as strongly as scan/local SLAM.
POSE_GRAPH.optimization_problem.odometry_translation_weight = 30.0
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 0.0

return options
