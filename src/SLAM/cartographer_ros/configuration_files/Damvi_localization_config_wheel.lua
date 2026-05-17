include "map_builder.lua"
include "trajectory_builder.lua"

-- 포즈 예측기(PoseExtrapolator)와 포즈 그래프가 같이 쓰는 휠 오돔 튜닝값을 읽는다.
-- config.yaml 형식은 "파라미터_이름: 숫자값" 형태의 한 줄 숫자 값만 지원한다.
local function load_numeric_config()
  local values = {}
  local config_path = os.getenv("WHEEL_ODOM_CONFIG")
  if config_path == nil or config_path == "" then
    config_path = os.getenv("POSE_EXTRAPOLATOR_CONFIG")
  end
  if config_path == nil or config_path == "" then
    return values
  end

  local content = nil
  local file = io.open(config_path, "r")
  if file ~= nil then
    content = file:read("*a")
    file:close()
  end
  if content == nil then
    return values
  end

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

-- 2D Trajectory 설정
MAP_BUILDER.use_trajectory_builder_2d = true
TRAJECTORY_BUILDER_2D.use_imu_data = true
local LOCAL_QUALITY_METRICS_CSV = os.getenv("LOCAL_QUALITY_METRICS_CSV") or ""
TRAJECTORY_BUILDER_2D.log_local_quality_metrics_to_csv =
    LOCAL_QUALITY_METRICS_CSV ~= ""
TRAJECTORY_BUILDER_2D.local_quality_metrics_csv_path =
    LOCAL_QUALITY_METRICS_CSV

-- 해상도 설정 (GridResolution). 0.1 = 10cm 단위, 여기서는 5cm
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.05

-- 순수 위치추정 모드 관련 설정
  -- ◆ [1]전역 매칭(루프 클로저) 최소 점수
POSE_GRAPH.constraint_builder.global_localization_min_score =
    wheel_config_or_default("global_localization_min_score", 0.58)
  -- ◆ [1]로컬 매칭(일반 스캔 매칭) 최소 점수
POSE_GRAPH.constraint_builder.min_score =
    wheel_config_or_default("constraint_min_score", 0.65)

-- 시작 직후부터 전역 제약 탐색을 바로 시도한다.
POSE_GRAPH.global_constraint_search_after_n_seconds =
    wheel_config_or_default("global_constraint_search_after_n_seconds", 0)
TRAJECTORY_BUILDER.pure_localization_trimmer = {
  max_submaps_to_keep = wheel_config_or_default("max_submaps_to_keep", 4),
}
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1

-- Outlier filter: wheel metric localization path에도 새 local SLAM outlier 로직을 적용한다.
TRAJECTORY_BUILDER_2D.skip_submap_insertion_for_outliers = true
TRAJECTORY_BUILDER_2D.outlier_max_translation_residual =
    wheel_config_or_default("outlier_max_translation_residual", 0.15)
TRAJECTORY_BUILDER_2D.outlier_max_rotation_residual =
    wheel_config_or_default("outlier_max_rotation_residual", 0.03)
TRAJECTORY_BUILDER_2D.outlier_required_failures =
    wheel_config_or_default("outlier_required_failures", 2)
TRAJECTORY_BUILDER_2D.outlier_medium_translation_residual =
    wheel_config_or_default("outlier_medium_translation_residual", 0.10)
TRAJECTORY_BUILDER_2D.outlier_medium_rotation_residual =
    wheel_config_or_default("outlier_medium_rotation_residual", 0.02)
TRAJECTORY_BUILDER_2D.outlier_medium_required_consecutive =
    wheel_config_or_default("outlier_medium_required_consecutive", 4)
TRAJECTORY_BUILDER_2D.outlier_min_correlative_score =
    wheel_config_or_default("outlier_min_correlative_score", 0.0)
TRAJECTORY_BUILDER_2D.outlier_min_num_filtered_points =
    wheel_config_or_default("outlier_min_num_filtered_points", 0)

-- ◆ [전역 매칭]
-- 초기 위치에 대한 설정. 아래 두 값은 초기 위치가 크게 벗어날 가능성이 높으면 큰 값을 지정
  -- [2]전역 Fast Correlative 매칭에서 x-y 평면상 탐색 범위 (m), 고정. 작을수록 좋음
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window =
    wheel_config_or_default("global_linear_search_window", 1.5)
  -- [2]전역 Fast Correlative 매칭에서 회전(각도) 탐색 범위 (라디안), 고정. 작을수록 좋음
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window =
    math.rad(wheel_config_or_default("global_angular_search_window_deg", 10.0))
  -- [1] 전역 매칭(큰 오프셋 수정 등) 시 스캔을 추출하여 매칭 시도할 확률 (0 ~ 1). 연산량 tradeoff가 존재. 0.0036-0.004 사이. 0.0001 단위로 조절
-- 초기 재위치 보정 시도가 너무 드물지 않도록 전역 후보 샘플링을 올린 값.
POSE_GRAPH.global_sampling_ratio =
    wheel_config_or_default("global_sampling_ratio", 0.005) -- 정반대 일 떄

-- 휠 오돔은 움직임 사전값으로만 약하게 쓰는 것이 목적이다.
-- 휠이 미끄러지거나 튈 때 스캔 매칭/전역 위치추정보다 강하면 맵이 밀릴 수 있다.
POSE_GRAPH.optimization_problem.odometry_translation_weight =
    wheel_config_or_default("odometry_translation_weight", 1e3)
POSE_GRAPH.optimization_problem.odometry_rotation_weight =
    wheel_config_or_default("odometry_rotation_weight", 0.0)

-- ◆ [로컬 매칭]
-- 실시간 변수 설정
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = false
  -- [2]실시간 로컬 Correlative 매칭에서 x-y 평면상 탐색 범위 (m)
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.05
-- [2]실시간 로컬 Correlative 매칭에서 회전(각도) 탐색 범위 (라디안), 얼마나 허용할 지
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

-- Ceres 기반 스캔 매처 설정. 라이다 데이터로 이전 서브맵과 비교하여 포즈와 방향을 추정한다.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight =
    wheel_config_or_default("ceres_occupied_space_weight", 50.0)
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight =
    wheel_config_or_default("ceres_translation_weight", 20.0)
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight =
    wheel_config_or_default("ceres_rotation_weight", 20.0)

--[드리프트 심할 때 키우세요] IMU 설정
  -- 급격한 조향이 있을 경우에는 time_constant와 rotation_weight 증가 고려
TRAJECTORY_BUILDER_2D.imu_gravity_time_constant = 12.0

MAP_BUILDER.num_background_threads = 4

-- ◆ 기타 포즈 그래프 관련
  -- n개의 노드(스캔)가 쌓일 때마다 전역 최적화(루프 클로저 등)를 실행한다. 적을수록 빠르게 최적화가 일어난다. 1개가 적절
POSE_GRAPH.optimize_every_n_nodes =
    wheel_config_or_default("optimize_every_n_nodes", 2)

-- [대회장 길이에 맞추어 조절] 전역 매칭을 위한 서브맵 간 최대 거리
POSE_GRAPH.constraint_builder.max_constraint_distance =
    wheel_config_or_default("max_constraint_distance", 15.0)

-- 루프 클로저 관련 변수
POSE_GRAPH.constraint_builder.loop_closure_translation_weight =
    wheel_config_or_default("loop_closure_translation_weight", 2000.0)
POSE_GRAPH.constraint_builder.loop_closure_rotation_weight =
    wheel_config_or_default("loop_closure_rotation_weight", 2000.0)


POSE_GRAPH.constraint_builder.sampling_ratio =
    wheel_config_or_default("constraint_builder_sampling_ratio", 0.0001)

return options
