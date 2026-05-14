# setting_good

Last updated: 2026-05-14

이 문서는 현재 `local_SLAM_imu_wheel_ok`에서 정상 동작 확인한 Cartographer 설정값을 따로 보관하기 위한 README입니다.

## Source Files

- Mapping: `src/SLAM/cartographer_ros/configuration_files/Damvi_carto_config.lua`
- Localization: `src/SLAM/cartographer_ros/configuration_files/Damvi_localization_config.lua`
- Wheel metric localization: `src/SLAM/cartographer_ros/configuration_files/Damvi_localization_config_wheel.lua`
- Default 2D options: `src/cartographer/configuration_files/trajectory_builder_2d.lua`
- Pose extrapolator and wheel odom tuning: `config.yaml`

## Sensor Only Metric Tuned Parameters

아래 값은 기존 good setting으로 되돌린 뒤 `Damvi_carto_pure_wheel_metric_launch.py`와 sensor-only bag으로 재검증한 최종값입니다.

```yaml
imu_weight: 0.2
imu_delta_min: 0.3
wheelodom_weight: 0.01
odometry_translation_weight: 3000.0
odometry_rotation_weight: 0.0
```

## Current Outlier Filter

현재 outlier 필터는 입력 파일 없이 scan matching 결과에서 residual을 계산해 판정합니다.
CSV 품질 로그는 성능 부담을 줄이기 위해 꺼둔 상태입니다.

### Mapping

```lua
TRAJECTORY_BUILDER_2D.log_local_quality_metrics_to_csv = false
TRAJECTORY_BUILDER_2D.local_quality_metrics_csv_path = ""
TRAJECTORY_BUILDER_2D.skip_submap_insertion_for_outliers = true
TRAJECTORY_BUILDER_2D.outlier_min_correlative_score = 0.0
TRAJECTORY_BUILDER_2D.outlier_max_translation_residual = 0.10
TRAJECTORY_BUILDER_2D.outlier_max_rotation_residual = 0.02
TRAJECTORY_BUILDER_2D.outlier_min_num_filtered_points = 0
TRAJECTORY_BUILDER_2D.outlier_required_failures = 1
TRAJECTORY_BUILDER_2D.outlier_medium_translation_residual = 0.06
TRAJECTORY_BUILDER_2D.outlier_medium_rotation_residual = 0.012
TRAJECTORY_BUILDER_2D.outlier_medium_required_consecutive = 3
```

### Localization

`Damvi_localization_config.lua`와 `Damvi_localization_config_wheel.lua` 모두 아래 localization 기준 outlier 값을 사용합니다.

```lua
TRAJECTORY_BUILDER_2D.skip_submap_insertion_for_outliers = true
TRAJECTORY_BUILDER_2D.outlier_max_translation_residual = 0.15
TRAJECTORY_BUILDER_2D.outlier_max_rotation_residual = 0.03
TRAJECTORY_BUILDER_2D.outlier_required_failures = 2
TRAJECTORY_BUILDER_2D.outlier_medium_translation_residual = 0.10
TRAJECTORY_BUILDER_2D.outlier_medium_rotation_residual = 0.02
TRAJECTORY_BUILDER_2D.outlier_medium_required_consecutive = 4
TRAJECTORY_BUILDER_2D.outlier_min_correlative_score = 0.0
TRAJECTORY_BUILDER_2D.outlier_min_num_filtered_points = 0
```

## Mapping Parameters

```lua
map_frame = "map"
tracking_frame = "imu"
published_frame = "base_link"
odom_frame = "odom"
provide_odom_frame = true
use_odometry = false
use_nav_sat = false
use_landmarks = false
publish_frame_projected_to_2d = true
use_pose_extrapolator = true
publish_to_tf = true
lookup_transform_timeout_sec = 0.2
submap_publish_period_sec = 0.025
pose_publish_period_sec = 0.025
trajectory_publish_period_sec = 0.025
num_laser_scans = 1
num_multi_echo_laser_scans = 0
num_subdivisions_per_laser_scan = 1
num_point_clouds = 0
rangefinder_sampling_ratio = 1.0
odometry_sampling_ratio = 1.0
imu_sampling_ratio = 1.0
fixed_frame_pose_sampling_ratio = 1.0
landmarks_sampling_ratio = 1.0

MAP_BUILDER.use_trajectory_builder_2d = true
TRAJECTORY_BUILDER_2D.use_imu_data = true
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = false

TRAJECTORY_BUILDER_2D.min_range = 0.1
TRAJECTORY_BUILDER_2D.max_range = 25.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5.0
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_length = 5.0
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.min_num_points = 200
TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.05

TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 1.0
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(10.0)
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 5.0
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 5.0

TRAJECTORY_BUILDER_2D.imu_gravity_time_constant = 30.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 50.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 20.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 20.0

MAP_BUILDER.num_background_threads = 4
POSE_GRAPH.optimize_every_n_nodes = 2
POSE_GRAPH.constraint_builder.min_score = 0.65
POSE_GRAPH.constraint_builder.sampling_ratio = 0.0001
POSE_GRAPH.global_sampling_ratio = 0.005
POSE_GRAPH.constraint_builder.max_constraint_distance = 15.0
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 1.5
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(10.0)
POSE_GRAPH.constraint_builder.loop_closure_translation_weight = 2000.0
POSE_GRAPH.constraint_builder.loop_closure_rotation_weight = 2000.0
```

## Localization Parameters

```lua
map_frame = "map"
tracking_frame = "imu"
published_frame = "base_link"
odom_frame = "odom"
provide_odom_frame = true
use_odometry = false
use_nav_sat = false
use_landmarks = false
publish_frame_projected_to_2d = true
use_pose_extrapolator = true
publish_to_tf = true
lookup_transform_timeout_sec = 0.2
submap_publish_period_sec = 0.1
pose_publish_period_sec = 0.05
trajectory_publish_period_sec = 0.1
num_laser_scans = 1
num_multi_echo_laser_scans = 0
num_subdivisions_per_laser_scan = 1
num_point_clouds = 0
rangefinder_sampling_ratio = 1.0
odometry_sampling_ratio = 1.0
imu_sampling_ratio = 1.0
fixed_frame_pose_sampling_ratio = 1.0
landmarks_sampling_ratio = 1.0

MAP_BUILDER.use_trajectory_builder_2d = true
TRAJECTORY_BUILDER_2D.use_imu_data = true
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.05

POSE_GRAPH.constraint_builder.global_localization_min_score = 0.62
POSE_GRAPH.constraint_builder.min_score = 0.95
POSE_GRAPH.global_constraint_search_after_n_seconds = 0
TRAJECTORY_BUILDER.pure_localization_trimmer = {
  max_submaps_to_keep = 5,
}
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1

POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 0.05
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(1.0)
POSE_GRAPH.global_sampling_ratio = 0.0055

TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.05
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(1.0)
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
TRAJECTORY_BUILDER_2D.imu_gravity_time_constant = 12.0

MAP_BUILDER.num_background_threads = 4
POSE_GRAPH.optimize_every_n_nodes = 1
POSE_GRAPH.constraint_builder.max_constraint_distance = 15.0
POSE_GRAPH.constraint_builder.loop_closure_translation_weight = 100.0
POSE_GRAPH.constraint_builder.loop_closure_rotation_weight = 100.0
POSE_GRAPH.constraint_builder.sampling_ratio = 0.78
```

## Verification

- `cartographer`, `cartographer_ros_msgs`, `cartographer_ros` build passed.
- Targeted Cartographer tests passed.
- `Damvi_carto_config.lua` loaded and trajectory start was confirmed.
- CSV quality metrics logging is disabled.
