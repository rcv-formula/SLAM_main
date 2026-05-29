include "map_builder.lua"
include "trajectory_builder.lua"

local function env_bool(name, default)
  local value = os.getenv(name)
  if value == nil or value == "" then
    return default
  end
  value = string.lower(value)
  if value == "1" or value == "true" or value == "on" then
    return true
  end
  if value == "0" or value == "false" or value == "off" then
    return false
  end
  return default
end

local function env_double(name, default)
  local value = os.getenv(name)
  if value == nil or value == "" then
    return default
  end
  local number = tonumber(value)
  if number == nil then
    return default
  end
  return number
end

local local_quality_metrics_csv_path =
    os.getenv("LOCAL_QUALITY_METRICS_CSV_PATH") or
    "/home/rcv/SLAM_local-loss/local_quality_metrics/local_quality_metrics.csv"
local fast_correlative_score_distribution_csv_path =
    os.getenv("FAST_CORRELATIVE_SCORE_DISTRIBUTION_CSV_PATH") or
    "/home/rcv/SLAM_local-loss/global_constraint_score_distributions/fast_correlative_score_distribution.csv"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "imu", -- Ensure imu frame is used as tracking frame
  published_frame = "base_link",
  odom_frame = "odom",
  provide_odom_frame = true,
  use_odometry = env_bool("CARTOGRAPHER_USE_ODOMETRY", true),
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
  longitudinal_prior_occupied_space_weight_scale = 0.35,
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

MAP_BUILDER.use_trajectory_builder_2d = true
TRAJECTORY_BUILDER_2D.use_imu_data = true
TRAJECTORY_BUILDER_2D.log_local_quality_metrics_to_csv = true
TRAJECTORY_BUILDER_2D.local_quality_metrics_csv_path =
    local_quality_metrics_csv_path
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
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 25.0

-- LiDAR filter and precision adjustments
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_length = 1.0
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.min_num_points = 200
TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.05

TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = false
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 1.0
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(10.0)
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 5.0
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 5.0

-- IMU settings
TRAJECTORY_BUILDER_2D.imu_gravity_time_constant = 3.0 --30.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 25.0 --20.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 25.0 --50.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 20.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.longitudinal_translation_weight =
    env_double("CARTOGRAPHER_LONGITUDINAL_TRANSLATION_WEIGHT", 3.0)
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.longitudinal_translation_min_speed =
    env_double("CARTOGRAPHER_LONGITUDINAL_TRANSLATION_MIN_SPEED", 0.05)
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.longitudinal_translation_max_yaw_rate =
    env_double("CARTOGRAPHER_LONGITUDINAL_TRANSLATION_MAX_YAW_RATE", 0.40)
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.longitudinal_prior_wheel_delta_scale =
    env_double("CARTOGRAPHER_LONGITUDINAL_PRIOR_WHEEL_DELTA_SCALE", 1.0)

-- Optimization and mapping settings
MAP_BUILDER.num_background_threads = 4
POSE_GRAPH.optimize_every_n_nodes =
    env_double("CARTOGRAPHER_POSE_GRAPH_OPTIMIZE_EVERY_N_NODES", 2)
POSE_GRAPH.constraint_builder.min_score = 0.65
POSE_GRAPH.constraint_builder.sampling_ratio = 0.01
POSE_GRAPH.global_sampling_ratio = 0.005
POSE_GRAPH.constraint_builder.max_constraint_distance = 15.0

POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 1.5
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(10.0)
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.log_score_distribution_to_csv = true
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.score_distribution_csv_path =
    fast_correlative_score_distribution_csv_path

-- Loop closure improvements
POSE_GRAPH.constraint_builder.loop_closure_translation_weight = 1e4 -- 2000.0
POSE_GRAPH.constraint_builder.loop_closure_rotation_weight = 1e4 --2000.0
POSE_GRAPH.optimization_problem.odometry_translation_weight =
    env_double("CARTOGRAPHER_POSE_GRAPH_ODOMETRY_TRANSLATION_WEIGHT", 100.0)
POSE_GRAPH.optimization_problem.odometry_rotation_weight =
    env_double("CARTOGRAPHER_POSE_GRAPH_ODOMETRY_ROTATION_WEIGHT", 0.0)

return options
