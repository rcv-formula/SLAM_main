include "map_builder.lua"
include "trajectory_builder.lua"

local local_quality_metrics_csv_path =
    os.getenv("LOCAL_QUALITY_METRICS_CSV_PATH") or
    "/home/symoon/Desktop/F1/Local_SLAM_Complete/local_SLAM_imu_ver2/local_quality_metrics/0501_local_quality_metrics.csv"
local fast_correlative_score_distribution_csv_path =
    os.getenv("FAST_CORRELATIVE_SCORE_DISTRIBUTION_CSV_PATH") or
    "/home/symoon/Desktop/F1/Local_SLAM_Complete/local_SLAM_imu_ver2/global_constraint_score_distributions/0501_fast_correlative_score_distribution.csv"

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

local TUNING_CONFIG = load_numeric_config()

local function config_or_default(key, default)
  if TUNING_CONFIG[key] ~= nil then
    return TUNING_CONFIG[key]
  end
  return default
end

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

-- 2D Trajectory 설정
MAP_BUILDER.use_trajectory_builder_2d = true
TRAJECTORY_BUILDER_2D.use_imu_data = true
TRAJECTORY_BUILDER_2D.log_local_quality_metrics_to_csv = true
TRAJECTORY_BUILDER_2D.local_quality_metrics_csv_path =
    local_quality_metrics_csv_path

-- 해상도 설정 (GridResolution). 0.1 = 10cm 단위, 여기서는 5cm
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.05

-- Pure Localization 모드 관련 설정
  -- ◆ [1]전역 매칭(루프 클로저) 최소 점수
POSE_GRAPH.constraint_builder.global_localization_min_score =
    config_or_default("global_localization_min_score", 0.58)
  -- ◆ [1]로컬 매칭(일반 스캔 매칭) 최소 점수
POSE_GRAPH.constraint_builder.min_score =
    config_or_default("constraint_min_score", 0.65)

POSE_GRAPH.global_constraint_search_after_n_seconds =
    config_or_default("global_constraint_search_after_n_seconds", 0.0)
TRAJECTORY_BUILDER.pure_localization_trimmer = {
  max_submaps_to_keep = config_or_default("max_submaps_to_keep", 5),
}
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1

-- Outlier filter: localization에서 갑작스러운 local mismatch를 metric으로 남기고 submap 삽입을 막는다.
TRAJECTORY_BUILDER_2D.skip_submap_insertion_for_outliers = true
TRAJECTORY_BUILDER_2D.outlier_max_translation_residual =
    config_or_default("outlier_max_translation_residual", 0.15)
TRAJECTORY_BUILDER_2D.outlier_max_rotation_residual =
    config_or_default("outlier_max_rotation_residual", 0.03)
TRAJECTORY_BUILDER_2D.outlier_required_failures =
    config_or_default("outlier_required_failures", 2)
TRAJECTORY_BUILDER_2D.outlier_medium_translation_residual =
    config_or_default("outlier_medium_translation_residual", 0.10)
TRAJECTORY_BUILDER_2D.outlier_medium_rotation_residual =
    config_or_default("outlier_medium_rotation_residual", 0.02)
TRAJECTORY_BUILDER_2D.outlier_medium_required_consecutive =
    config_or_default("outlier_medium_required_consecutive", 4)
TRAJECTORY_BUILDER_2D.outlier_min_correlative_score = 0.0
TRAJECTORY_BUILDER_2D.outlier_min_num_filtered_points = 0

-- ◆ [GLOBAL]
-- 초기 위치에 대한 설정. 아래 두 값은 초기 위치가 크게 벗어날 가능성이 높으면 큰 값을 지정
  -- [2]global Fast Correlative 매칭에서 x-y 평면상 탐색 범위 (m), 고정. 작을수록 좋음
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window =
    config_or_default("global_linear_search_window", 1.0)
  -- [2]global Fast Correlative 매칭에서 회전(각도) 탐색 범위 (라디안), 고정. 작을수록 좋음
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window =
    math.rad(config_or_default("global_angular_search_window_deg", 8.0))
  -- [1] global 전역 매칭(큰 오프셋 수정 등) 시 스캔을 추출하여 매칭 시도할 확률 (0 ~ 1). 연산량 tradeoff가 존재. 0.0036-0.004 사이. 0.0001 단위로 조절
POSE_GRAPH.global_sampling_ratio =
    config_or_default("global_sampling_ratio", 0.006)
POSE_GRAPH.initial_global_sampling_ratio =
    config_or_default("initial_global_sampling_ratio", 0.05)
POSE_GRAPH.initial_global_constraint_search_after_n_seconds =
    config_or_default("initial_global_constraint_search_after_n_seconds", 0.0)
POSE_GRAPH.initial_global_localization_min_score =
    config_or_default("initial_global_localization_min_score", 0.50)
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.log_score_distribution_to_csv = true
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.score_distribution_csv_path =
    fast_correlative_score_distribution_csv_path
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.min_score_distribution_margin =
    config_or_default("min_score_distribution_margin", 0.0)

-- 휠 오돔은 전역 최적화에서 이동량 사전값으로만 사용한다. yaw는 기본적으로 끈다.
POSE_GRAPH.optimization_problem.odometry_translation_weight =
    config_or_default("odometry_translation_weight", 3000.0)
POSE_GRAPH.optimization_problem.odometry_rotation_weight =
    config_or_default("odometry_rotation_weight", 0.0)

-- ◆ [LOCAL]
-- real time 변수 설정
  -- [2]실시간 Local Correlative 매칭에서 x-y 평면상 탐색 범위 (m)
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window =
    config_or_default("local_linear_search_window", 0.20)
-- [2]실시간 Local Correlative 매칭에서 회전(각도) 탐색 범위 (라디안), 얼마나 허용할 지
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window =
    math.rad(config_or_default("local_angular_search_window_deg", 5.0))

TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight =
    config_or_default("local_translation_delta_cost_weight", 25.0)
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight =
    config_or_default("local_rotation_delta_cost_weight", 25.0)

-- LiDAR 관련
TRAJECTORY_BUILDER_2D.min_range = 0.1
TRAJECTORY_BUILDER_2D.max_range = 20.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5.0
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_length = 5.0
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.min_num_points = 350
TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.05

-- Ceres 기반 Scan Matcher 설정, Lidar 데이터로 이전 서브맵과의 비교를 수행, pose&orientation 파악
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 15.0 
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 30.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 30.0

--[드리프트 심할 때 키우세요] IMU 설정
  -- 급격한 steering이 있을 경우에는 time_constant와 rotation_weight 증가 고려
TRAJECTORY_BUILDER_2D.imu_gravity_time_constant = 12.0

MAP_BUILDER.num_background_threads = config_or_default("num_background_threads", 8)

-- ◆ 기타 posegraph 관련
  --n개의 노드(스캔)이 쌓일 때마다 전역 최적화(Loop Closure 등) 실행. 적을수록 빠르게 최적화가 일어남. 1개가 적절
POSE_GRAPH.optimize_every_n_nodes = config_or_default("optimize_every_n_nodes", 1)

-- [대회장 길이에 맞추어 조절] 전역 매칭을 위한 Submap 간 최대 거리
POSE_GRAPH.constraint_builder.max_constraint_distance =
    config_or_default("max_constraint_distance", 15.0)

-- Loop clousre 관련 변수
POSE_GRAPH.constraint_builder.loop_closure_translation_weight =
    config_or_default("loop_closure_translation_weight", 20000.0)
POSE_GRAPH.constraint_builder.loop_closure_rotation_weight =
    config_or_default("loop_closure_rotation_weight", 20000.0)

POSE_GRAPH.relocalization_trigger_sec =
    config_or_default("relocalization_trigger_sec", 6.0)
POSE_GRAPH.relocalization_recovery_required_successes =
    config_or_default("relocalization_recovery_required_successes", 3)
POSE_GRAPH.relocalization_recovery_grace_sec =
    config_or_default("relocalization_recovery_grace_sec", 4.0)

POSE_GRAPH.constraint_builder.sampling_ratio =
    config_or_default("constraint_builder_sampling_ratio", 0.78)

return options
