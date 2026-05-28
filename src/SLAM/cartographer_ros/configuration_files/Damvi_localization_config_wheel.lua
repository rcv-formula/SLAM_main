include "map_builder.lua"
include "trajectory_builder.lua"

local slam_main_dir = "/home/rcv/SLAM_main-SLAM_IMU_WHEEL_tun_upg"
local local_quality_metrics_csv_path =
    slam_main_dir .. "/cartographer_metrics/local_quality_metrics_" ..
    os.date("%Y%m%d_%H%M%S") .. ".csv"
local pbstream_file =
    slam_main_dir ..
    "/src/SLAM/cartographer_ros/pbstream/latest.pbstream"
local fast_correlative_score_distribution_csv_path =
    os.getenv("FAST_CORRELATIVE_SCORE_DISTRIBUTION_CSV_PATH") or ""

-- 기본 설정
options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  use_sim_time = false,
  collect_metrics = true,
  fusion_extrapolator = true,
  load_state_filename = pbstream_file,
  load_frozen_state = true,
  start_trajectory_with_default_topics = true,
  publish_odom = true,
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
  wheel_odom_twist_only = true,
  wheel_odom_linear_scale = 2.6,
  wheel_odom_yaw_weight = 0.0,
  adaptive_odometry_blend = true,
  adaptive_odometry_full_weight_yaw_rate = 0.15, --0.05,
  adaptive_odometry_zero_weight_yaw_rate = 0.75, --0.3,
  adaptive_odometry_min_weight = 0.0,
  adaptive_odometry_max_weight = 1.0,
  adaptive_odometry_mismatch_override = true,
  adaptive_odometry_mismatch_ratio = 0.65, --0.35,
  adaptive_odometry_min_forward_delta = 0.005,
  adaptive_odometry_mismatch_force_weight = 1.0,
  adaptive_odometry_longitudinal_only = true,
  clamp_local_lateral_residual = true,
  local_lateral_residual_max = 0.03, --0.03
  imu_yaw_weight = 0.35,
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
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.85
-- POSE_GRAPH.constraint_builder.global_localization_min_score = 0.7

  -- ◆ [1]로컬 매칭(일반 스캔 매칭) 최소 점수
POSE_GRAPH.constraint_builder.min_score = 0.6
-- POSE_GRAPH.constraint_builder.min_score = 0.7


-- Disable post-initial global relocalization jumps. After the initial frozen-map
-- connection, keep using local constraints against the loaded map instead of
-- falling back to full-submap global searches.
POSE_GRAPH.global_constraint_search_after_n_seconds = 1e9
TRAJECTORY_BUILDER.pure_localization_trimmer = {
  max_submaps_to_keep = 5,
}
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = false
TRAJECTORY_BUILDER_2D.skip_submap_insertion_for_outliers = true
TRAJECTORY_BUILDER_2D.outlier_max_translation_residual = 0.08
TRAJECTORY_BUILDER_2D.outlier_max_rotation_residual = 0.035
TRAJECTORY_BUILDER_2D.outlier_required_failures = 1
TRAJECTORY_BUILDER_2D.outlier_medium_translation_residual = 0.04
TRAJECTORY_BUILDER_2D.outlier_medium_rotation_residual = 0.026
TRAJECTORY_BUILDER_2D.outlier_medium_required_consecutive = 2
TRAJECTORY_BUILDER_2D.outlier_min_correlative_score = 0.0
TRAJECTORY_BUILDER_2D.outlier_min_num_filtered_points = 0

-- ◆ [GLOBAL]
-- 초기 위치에 대한 설정. 아래 두 값은 초기 위치가 크게 벗어날 가능성이 높으면 큰 값을 지정
  -- [2]global Fast Correlative 매칭에서 x-y 평면상 탐색 범위 (m), 고정. 작을수록 좋음
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 0.1 --0.07
  -- [2]global Fast Correlative 매칭에서 회전(각도) 탐색 범위 (라디안), 고정. 작을수록 좋음
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(1.5)--math.rad(2.5)
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.log_score_distribution_to_csv =
    fast_correlative_score_distribution_csv_path ~= ""
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.score_distribution_csv_path =
    fast_correlative_score_distribution_csv_path
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.min_score_distribution_margin = 0.0
  -- [1] global 전역 매칭(큰 오프셋 수정 등) 시 스캔을 추출하여 매칭 시도할 확률 (0 ~ 1). 연산량 tradeoff가 존재. 0.0036-0.004 사이. 0.0001 단위로 조절
POSE_GRAPH.global_sampling_ratio = 0.0
POSE_GRAPH.initial_global_sampling_ratio = 0.2
POSE_GRAPH.initial_global_constraint_search_after_n_seconds = 0.0
POSE_GRAPH.initial_global_localization_min_score = 0.45
-- POSE_GRAPH.initial_global_localization_min_score = 0.45

-- ◆ [LOCAL]
-- real time 변수 설정
  -- [2]실시간 Local Correlative 매칭에서 x-y 평면상 탐색 범위 (m)
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.08
-- [2]실시간 Local Correlative 매칭에서 회전(각도) 탐색 범위 (라디안), 얼마나 허용할 지
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(2.8)

TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 25.0 --25.0
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 12.0 --12.0
--TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 11.0

-- LiDAR 관련
TRAJECTORY_BUILDER_2D.min_range = 0.1
TRAJECTORY_BUILDER_2D.max_range = 20.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5.0
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_length = 5.0
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.min_num_points = 350
TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.05

-- Ceres 기반 Scan Matcher 설정, Lidar 데이터로 이전 서브맵과의 비교를 수행, pose&orientation 파악
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 40.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 30.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 20.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.longitudinal_translation_weight = 2.0 --0.5
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.longitudinal_translation_min_speed = 0.05
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.longitudinal_translation_max_yaw_rate = 0.75 --0.60
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.longitudinal_prior_wheel_delta_scale = 1.0

--[드리프트 심할 때 키우세요] IMU 설정
  -- 급격한 steering이 있을 경우에는 time_constant와 rotation_weight 증가 고려
TRAJECTORY_BUILDER_2D.imu_gravity_time_constant = 12.0

MAP_BUILDER.num_background_threads = 8

-- ◆ 기타 posegraph 관련
  --n개의 노드(스캔)이 쌓일 때마다 전역 최적화(Loop Closure 등) 실행. 적을수록 빠르게 최적화가 일어남. 1개가 적절
POSE_GRAPH.optimize_every_n_nodes = 1

-- [대회장 길이에 맞추어 조절] 전역 매칭을 위한 Submap 간 최대 거리
POSE_GRAPH.constraint_builder.max_constraint_distance = 15.0
POSE_GRAPH.relocalization_trigger_sec = 0.0
POSE_GRAPH.relocalization_recovery_required_successes = 5
POSE_GRAPH.relocalization_recovery_grace_sec = 12.0

-- Loop clousre 관련 변수
POSE_GRAPH.constraint_builder.loop_closure_translation_weight = 2e4
POSE_GRAPH.constraint_builder.loop_closure_rotation_weight = 1e4
POSE_GRAPH.optimization_problem.odometry_translation_weight = 30.0
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 0.0

POSE_GRAPH.constraint_builder.sampling_ratio = 0.78
-- POSE_GRAPH.constraint_builder.sampling_ratio = 0.78

return options
