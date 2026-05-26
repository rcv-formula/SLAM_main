include "map_builder.lua"
include "trajectory_builder.lua"

local function load_numeric_config()
  local values = {}
  local config_path = os.getenv("WHEEL_ODOM_CONFIG")
  if config_path == nil or config_path == "" then
    config_path = os.getenv("POSE_EXTRAPOLATOR_CONFIG")
  end
  if config_path == nil or config_path == "" then
    return values
  end

  local file = io.open(config_path, "r")
  if file == nil then
    return values
  end
  local content = file:read("*a")
  file:close()

  for line in string.gmatch(content, "[^\r\n]+") do
    local without_comment = string.gsub(line, "#.*$", "")
    local key, value = string.match(without_comment, "^%s*([%w_]+)%s*:%s*([^%s]+)")
    local numeric_value = tonumber(value)
    if key ~= nil and numeric_value ~= nil then
      values[key] = numeric_value
    end
  end
  return values
end

local WHEEL_ODOM_CONFIG = load_numeric_config()

local function wheel_config_or_default(key, default)
  if WHEEL_ODOM_CONFIG[key] ~= nil then
    return WHEEL_ODOM_CONFIG[key]
  end
  return default
end

local function wheel_config_bool_or_default(key, default)
  if WHEEL_ODOM_CONFIG[key] ~= nil then
    return WHEEL_ODOM_CONFIG[key] ~= 0
  end
  return default
end

local local_quality_metrics_csv_path =
    "/tmp/cartographer_localization_quality_metrics.csv"
local fast_correlative_score_distribution_csv_path =
    "/tmp/fast_correlative_score_distribution.csv"
local pose_graph_constraint_metrics_csv_path =
    "/tmp/pose_graph_constraint_metrics.csv"

-- 기본 설정
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
  use_landmarks = false, -- 안 씀
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

options.damvi_runtime_options = {
  wheel_odom_twist_only = true,
  wheel_odom_linear_scale =
      wheel_config_or_default("wheel_odom_linear_scale", 2.6),
  wheel_odom_yaw_weight = 0.0,
  imu_yaw_weight = 0.4,

  clamp_local_lateral_residual = true,
  local_lateral_residual_max = 0.03,
  reject_local_slam_outliers = false,

  pose_graph_constraint_metrics_csv_path =
      pose_graph_constraint_metrics_csv_path,

  -- Recovery relocalization ambiguity guards.
  pose_graph_ambiguous_constraint_downweight = true,
  pose_graph_ambiguous_constraint_reject = true,
  pose_graph_ambiguous_apply_to_tracking = true,
  pose_graph_ambiguous_apply_to_initial = false,
  pose_graph_ambiguous_apply_to_recovery = true,
  pose_graph_ambiguous_constraint_reject_min_score = 0.45,
  pose_graph_ambiguous_constraint_min_translation = 1.0,
  pose_graph_ambiguous_constraint_max_score_margin = 0.01,
  pose_graph_ambiguous_constraint_min_near_top_count = 20.0,
  pose_graph_reject_ambiguous_full_submap = true,
  pose_graph_ambiguous_full_submap_max_score_margin = 0.01,
  pose_graph_ambiguous_full_submap_reject_min_score = 0.45,
  pose_graph_ambiguous_full_submap_min_near_top_count = 80.0,

  pose_graph_bound_relocalization_to_prior = true,
  pose_graph_relocalization_prior_min_score = 0.45,
  pose_graph_relocalization_max_translation_correction = 0.45,
  pose_graph_relocalization_max_yaw_correction = 0.30,
  pose_graph_recovery_constraint_weight_scale = 0.15,

  adaptive_odometry_blend = true,
  adaptive_odometry_full_weight_yaw_rate =
      wheel_config_or_default("adaptive_odometry_full_weight_yaw_rate", 0.20),
  adaptive_odometry_zero_weight_yaw_rate =
      wheel_config_or_default("adaptive_odometry_zero_weight_yaw_rate", 0.80),
  adaptive_odometry_min_weight =
      wheel_config_or_default("adaptive_odometry_min_weight", 0.0),
  adaptive_odometry_max_weight =
      wheel_config_or_default("adaptive_odometry_max_weight", 0.45),
  adaptive_odometry_mismatch_override =
      wheel_config_bool_or_default("adaptive_odometry_mismatch_override", false),
  adaptive_odometry_mismatch_ratio =
      wheel_config_or_default("adaptive_odometry_mismatch_ratio", 0.35),
  adaptive_odometry_min_forward_delta =
      wheel_config_or_default("adaptive_odometry_min_forward_delta", 0.005),
  adaptive_odometry_mismatch_force_weight =
      wheel_config_or_default("adaptive_odometry_mismatch_force_weight", 0.25),
  adaptive_odometry_longitudinal_only = true,
}

-- 2D Trajectory 설정
MAP_BUILDER.use_trajectory_builder_2d = true
TRAJECTORY_BUILDER_2D.use_imu_data = true
TRAJECTORY_BUILDER_2D.log_local_quality_metrics_to_csv =
    local_quality_metrics_csv_path ~= ""
TRAJECTORY_BUILDER_2D.local_quality_metrics_csv_path =
    local_quality_metrics_csv_path

-- 해상도 설정 (GridResolution). 0.1 = 10cm 단위, 여기서는 5cm
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.05
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 45

-- Pure Localization 모드 관련 설정
  -- ◆ [1]전역 매칭(루프 클로저) 최소 점수
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.65 --0.75
  -- ◆ [1]로컬 매칭(일반 스캔 매칭) 최소 점수
POSE_GRAPH.constraint_builder.min_score = 0.82

POSE_GRAPH.global_constraint_search_after_n_seconds = 1.5
TRAJECTORY_BUILDER.pure_localization_trimmer = {
  max_submaps_to_keep = 5,
}
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.skip_submap_insertion_for_outliers = true
TRAJECTORY_BUILDER_2D.outlier_max_translation_residual = 0.08
TRAJECTORY_BUILDER_2D.outlier_max_rotation_residual =
    wheel_config_or_default("outlier_max_rotation_residual", 0.05)
TRAJECTORY_BUILDER_2D.outlier_required_failures =
    wheel_config_or_default("outlier_required_failures", 2)
TRAJECTORY_BUILDER_2D.outlier_medium_translation_residual = 0.04
TRAJECTORY_BUILDER_2D.outlier_medium_rotation_residual =
    wheel_config_or_default("outlier_medium_rotation_residual", 0.02)
TRAJECTORY_BUILDER_2D.outlier_medium_required_consecutive =
    wheel_config_or_default("outlier_medium_required_consecutive", 8)
TRAJECTORY_BUILDER_2D.outlier_min_correlative_score =
    wheel_config_or_default("outlier_min_correlative_score", 0.0)
TRAJECTORY_BUILDER_2D.outlier_min_num_filtered_points =
    wheel_config_or_default("outlier_min_num_filtered_points", 0)

-- ◆ [GLOBAL]
-- 초기 위치에 대한 설정. 아래 두 값은 초기 위치가 크게 벗어날 가능성이 높으면 큰 값을 지정
  -- [2]global Fast Correlative 매칭에서 x-y 평면상 탐색 범위 (m), 고정. 작을수록 좋음
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 0.5
  -- [2]global Fast Correlative 매칭에서 회전(각도) 탐색 범위 (라디안), 고정. 작을수록 좋음
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(2.5)
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.log_score_distribution_to_csv =
    fast_correlative_score_distribution_csv_path ~= ""
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.score_distribution_csv_path =
    fast_correlative_score_distribution_csv_path
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.min_score_distribution_margin = 0.0
  -- [1] global 전역 매칭(큰 오프셋 수정 등) 시 스캔을 추출하여 매칭 시도할 확률 (0 ~ 1). 연산량 tradeoff가 존재. 0.0036-0.004 사이. 0.0001 단위로 조절
POSE_GRAPH.global_sampling_ratio = 0.3
POSE_GRAPH.initial_global_sampling_ratio = 0.9
POSE_GRAPH.initial_global_constraint_search_after_n_seconds = 0.0
POSE_GRAPH.initial_global_localization_min_score = 0.45

-- ◆ [LOCAL]
-- real time 변수 설정
  -- [2]실시간 Local Correlative 매칭에서 x-y 평면상 탐색 범위 (m)
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.12
-- [2]실시간 Local Correlative 매칭에서 회전(각도) 탐색 범위 (라디안), 얼마나 허용할 지
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(3.5)

TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 25.0
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 12.0

-- LiDAR 관련
TRAJECTORY_BUILDER_2D.min_range = 0.1
TRAJECTORY_BUILDER_2D.max_range = 20.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5.0
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_length = 5.0
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.min_num_points = 350
TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.05

-- Ceres 기반 Scan Matcher 설정, Lidar 데이터로 이전 서브맵과의 비교를 수행, pose&orientation 파악
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 20.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 45.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight =30.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.longitudinal_translation_weight =
    wheel_config_or_default("longitudinal_translation_weight", 1.0)
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.longitudinal_translation_min_speed =
    wheel_config_or_default("longitudinal_translation_min_speed", 0.05)
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.longitudinal_translation_max_yaw_rate =
    wheel_config_or_default("longitudinal_translation_max_yaw_rate", 1.20)
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.longitudinal_prior_wheel_delta_scale =
    wheel_config_or_default("longitudinal_prior_wheel_delta_scale", 1.0)

--[드리프트 심할 때 키우세요] IMU 설정
  -- 급격한 steering이 있을 경우에는 time_constant와 rotation_weight 증가 고려
TRAJECTORY_BUILDER_2D.imu_gravity_time_constant = 12.0

MAP_BUILDER.num_background_threads = 8

-- ◆ 기타 posegraph 관련
  --n개의 노드(스캔)이 쌓일 때마다 전역 최적화(Loop Closure 등) 실행. 적을수록 빠르게 최적화가 일어남. 1개가 적절
POSE_GRAPH.optimize_every_n_nodes = 1

-- [대회장 길이에 맞추어 조절] 전역 매칭을 위한 Submap 간 최대 거리
POSE_GRAPH.constraint_builder.max_constraint_distance = 15.0
POSE_GRAPH.relocalization_trigger_sec = 15.0
POSE_GRAPH.relocalization_recovery_required_successes = 2
POSE_GRAPH.relocalization_recovery_grace_sec = 12.0

-- Loop clousre 관련 변수
POSE_GRAPH.constraint_builder.loop_closure_translation_weight = 2e4
POSE_GRAPH.constraint_builder.loop_closure_rotation_weight = 1e4
POSE_GRAPH.optimization_problem.odometry_translation_weight =
    wheel_config_or_default("odometry_translation_weight", 30.0)
POSE_GRAPH.optimization_problem.odometry_rotation_weight =
    wheel_config_or_default("odometry_rotation_weight", 0.0)

POSE_GRAPH.constraint_builder.sampling_ratio = 0.78

return options
