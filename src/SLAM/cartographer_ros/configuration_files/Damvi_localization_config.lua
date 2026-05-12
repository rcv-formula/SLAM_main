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
  pose_publish_period_sec = 0.025,
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

MAP_BUILDER.use_trajectory_builder_2d = true
TRAJECTORY_BUILDER_2D.use_imu_data = true
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.05
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 45

POSE_GRAPH.constraint_builder.global_localization_min_score = 0.60
POSE_GRAPH.constraint_builder.min_score = 0.65
POSE_GRAPH.constraint_builder.log_matches = false
POSE_GRAPH.log_residual_histograms = false
POSE_GRAPH.global_constraint_search_after_n_seconds = 5.0
TRAJECTORY_BUILDER.pure_localization_trimmer = {
  max_submaps_to_keep = 5,
}
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = false

-- Outlier filter: localization mode용으로 threshold를 mapping보다 넉넉하게 설정
TRAJECTORY_BUILDER_2D.skip_submap_insertion_for_outliers = true
TRAJECTORY_BUILDER_2D.outlier_max_translation_residual = 0.15
TRAJECTORY_BUILDER_2D.outlier_max_rotation_residual = 0.03
TRAJECTORY_BUILDER_2D.outlier_required_failures = 2
TRAJECTORY_BUILDER_2D.outlier_medium_translation_residual = 0.10
TRAJECTORY_BUILDER_2D.outlier_medium_rotation_residual = 0.02
TRAJECTORY_BUILDER_2D.outlier_medium_required_consecutive = 4
TRAJECTORY_BUILDER_2D.outlier_min_correlative_score = 0.0
TRAJECTORY_BUILDER_2D.outlier_min_num_filtered_points = 0

POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 0.20 --0.05
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(5.0)
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.log_score_distribution_to_csv = true
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.score_distribution_csv_path =
    fast_correlative_score_distribution_csv_path

POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.min_score_distribution_margin = 0.0 --0.05

POSE_GRAPH.global_sampling_ratio = 0.006
POSE_GRAPH.initial_global_sampling_ratio = 0.05
POSE_GRAPH.initial_global_constraint_search_after_n_seconds = 0.0
POSE_GRAPH.initial_global_localization_min_score = 0.5

-- 고속 주행에서 pose prediction 오차 수용 + 진동으로 인한 active submap 오차 허용
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.15
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(5.0)
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 25.0
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 25.0

TRAJECTORY_BUILDER_2D.min_range = 0.1
TRAJECTORY_BUILDER_2D.max_range = 20.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5.0
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_length = 5.0
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.min_num_points = 350
TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.05

TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 15.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 30.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 30.0

-- Frozen finished submap과 한 번 더 정합해서 pure-localization pose를 보정하는 옵션입니다.
-- 아래 파라미터는 "기존 local scan matching 결과"를 frozen map으로 한 번 더 검증/보정할지 결정합니다.

-- 기능 on/off입니다.
-- 기본 원본 설정에서는 frozen matcher를 끄고, test 전용 launch에서만 별도 설정을 덮어씁니다.
TRAJECTORY_BUILDER_2D.frozen_submap_scan_matcher.enabled = false

-- 보정된 pose를 어디까지 반영할지 정합니다.
-- "FULL_PIPELINE": local SLAM 내부 pose, extrapolator, submap 삽입, publish까지 모두 반영
-- "PUBLISH_ONLY": 내부 local SLAM은 기존 pose 유지, 최종 publish/odom 성격의 pose만 보정
TRAJECTORY_BUILDER_2D.frozen_submap_scan_matcher.apply_mode = "FULL_PIPELINE"

-- 현재 map pose 주변에서 frozen submap 후보를 찾는 반경입니다. 단위는 m입니다.
TRAJECTORY_BUILDER_2D.frozen_submap_scan_matcher.search_radius = 1.6

-- 위 반경 안에 후보가 많아도 최대 이 개수까지만 평가합니다.
TRAJECTORY_BUILDER_2D.frozen_submap_scan_matcher.max_submaps_to_match = 60

-- RTC 최고 점수가 이 값보다 낮으면 frozen 보정을 버립니다.
TRAJECTORY_BUILDER_2D.frozen_submap_scan_matcher.min_realtime_correlative_score = 0.66

-- 1등 후보와 2등 후보 점수 차가 이 값보다 작으면 애매한 매칭으로 보고 버립니다.
TRAJECTORY_BUILDER_2D.frozen_submap_scan_matcher.min_score_margin = 0.01

-- 상위 몇 개 후보 점수로 분산을 계산할지 정합니다.
TRAJECTORY_BUILDER_2D.frozen_submap_scan_matcher.score_variance_top_k = 3

-- 상위 후보들의 점수 분산이 이 값보다 작으면 긴 복도처럼 구분이 안 된다고 보고 버립니다.
TRAJECTORY_BUILDER_2D.frozen_submap_scan_matcher.min_score_variance = 0.0004

-- raw local pose 대비 frozen 보정 translation이 이 값보다 크면 버립니다. 단위는 m입니다.
TRAJECTORY_BUILDER_2D.frozen_submap_scan_matcher.max_translation_correction = 0.12

-- raw local pose 대비 frozen 보정 rotation이 이 값보다 크면 버립니다. 단위는 rad입니다.
TRAJECTORY_BUILDER_2D.frozen_submap_scan_matcher.max_rotation_correction = math.rad(0.40)

-- frozen 후보를 찾을 때 Real-Time Correlative Scan Matcher를 먼저 사용할지 정합니다.
TRAJECTORY_BUILDER_2D.frozen_submap_scan_matcher.use_realtime_correlative_scan_matching = true

-- RTC 결과를 Ceres로 한 번 더 refine할지 정합니다.
TRAJECTORY_BUILDER_2D.frozen_submap_scan_matcher.use_ceres_scan_matching = true

-- 비교용 test mode입니다.
-- true면 frozen matcher는 내부적으로 항상 PUBLISH_ONLY처럼 동작하고,
-- 기존 /odom 경로는 raw pose 기준으로 유지하면서 /filtered_odom 비교 경로를 따로 제공합니다.
TRAJECTORY_BUILDER_2D.frozen_submap_scan_matcher.test_mode_publish_filtered_odom = false

-- frozen matcher 튜닝용 로그 전체 on/off입니다.
TRAJECTORY_BUILDER_2D.frozen_submap_scan_matcher.tuning_log_enabled = false

-- rejected scan의 상세 로그를 모두 남길지 정합니다. 기본값은 false로 두고 summary와 샘플 로그를 봅니다.
TRAJECTORY_BUILDER_2D.frozen_submap_scan_matcher.tuning_log_log_rejections = false

-- accepted scan의 상세 로그를 모두 남길지 정합니다. 많이 시끄러우면 false로 둡니다.
TRAJECTORY_BUILDER_2D.frozen_submap_scan_matcher.tuning_log_log_acceptances = false

-- 매 N번째 attempted scan마다 샘플 상세 로그를 추가로 남깁니다. 0이면 끕니다.
TRAJECTORY_BUILDER_2D.frozen_submap_scan_matcher.tuning_log_detail_every_n_scans = 0

-- 매 N번째 attempted scan마다 최근 구간 요약 통계를 남깁니다. 0이면 끕니다.
TRAJECTORY_BUILDER_2D.frozen_submap_scan_matcher.tuning_log_summary_every_n_scans = 0

-- 상세 로그에 상위 몇 개 frozen candidate를 함께 표기할지 정합니다.
TRAJECTORY_BUILDER_2D.frozen_submap_scan_matcher.tuning_log_top_candidates = 3

-- RTC의 선형 탐색 범위입니다. 예측 pose 주변으로 몇 m까지 후보를 살필지 정합니다.
TRAJECTORY_BUILDER_2D.frozen_submap_scan_matcher.real_time_correlative_scan_matcher.linear_search_window = 1.40

-- RTC의 각도 탐색 범위입니다. 예측 pose 주변으로 몇 rad까지 후보를 살필지 정합니다.
TRAJECTORY_BUILDER_2D.frozen_submap_scan_matcher.real_time_correlative_scan_matcher.angular_search_window = math.rad(1.0)

-- RTC에서 예측 pose로부터 멀리 벗어난 translation 후보에 주는 패널티입니다.
TRAJECTORY_BUILDER_2D.frozen_submap_scan_matcher.real_time_correlative_scan_matcher.translation_delta_cost_weight = 1.0

-- RTC에서 예측 pose로부터 멀리 벗어난 rotation 후보에 주는 패널티입니다.
TRAJECTORY_BUILDER_2D.frozen_submap_scan_matcher.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1.0

-- Ceres refine 시 occupancy 일치도를 얼마나 강하게 볼지 정합니다.
TRAJECTORY_BUILDER_2D.frozen_submap_scan_matcher.ceres_scan_matcher.occupied_space_weight = 60.0

-- Ceres refine 시 예측 pose에서 translation이 크게 벗어나는 것을 얼마나 억제할지 정합니다.
TRAJECTORY_BUILDER_2D.frozen_submap_scan_matcher.ceres_scan_matcher.translation_weight = 10.0

-- Ceres refine 시 예측 pose에서 rotation이 크게 벗어나는 것을 얼마나 억제할지 정합니다.
TRAJECTORY_BUILDER_2D.frozen_submap_scan_matcher.ceres_scan_matcher.rotation_weight = 10.0

-- Ceres solver의 non-monotonic step 사용 여부입니다.
TRAJECTORY_BUILDER_2D.frozen_submap_scan_matcher.ceres_scan_matcher.ceres_solver_options.use_nonmonotonic_steps = true

-- Ceres solver 반복 횟수 상한입니다.
TRAJECTORY_BUILDER_2D.frozen_submap_scan_matcher.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 50

-- Ceres solver가 사용할 스레드 수입니다.
TRAJECTORY_BUILDER_2D.frozen_submap_scan_matcher.ceres_scan_matcher.ceres_solver_options.num_threads = 8

--[드리프트 심할 때 키우세요] IMU 설정
  -- 급격한 steering이 있을 경우에는 time_constant와 rotation_weight 증가 고려
TRAJECTORY_BUILDER_2D.imu_gravity_time_constant = 12.0

MAP_BUILDER.num_background_threads = 8
POSE_GRAPH.optimize_every_n_nodes = 1
POSE_GRAPH.constraint_builder.max_constraint_distance = 15.0
POSE_GRAPH.relocalization_trigger_sec = 6.0
POSE_GRAPH.relocalization_recovery_required_successes = 3
POSE_GRAPH.relocalization_recovery_grace_sec = 4.0

POSE_GRAPH.constraint_builder.loop_closure_translation_weight = 2e4
POSE_GRAPH.constraint_builder.loop_closure_rotation_weight = 2e4
POSE_GRAPH.constraint_builder.sampling_ratio = 0.78
return options
