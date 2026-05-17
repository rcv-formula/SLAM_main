# Pure Localization Locked Parameters

Last updated: 2026-05-17

`Damvi_carto_pure_launch.py` is the current stable no-wheel localization path.
Do not change these values unless explicitly retuning the no-wheel pure localization
profile.

## Launch

- Launch file: `src/SLAM/cartographer_ros/launch/Damvi_carto_pure_launch.py`
- Lua config: `src/SLAM/cartographer_ros/configuration_files/Damvi_localization_config.lua`
- Default map argument: `pbstream_file:=/home/symoon/Desktop/F1/Local_SLAM_Complete/local_SLAM_imu_wheel_ver5/0125_4.pbstream`
- Default extrapolator config argument: `pose_extrapolator_config:=/home/symoon/Desktop/F1/Local_SLAM_Complete/local_SLAM_imu_wheel_ver5/config.yaml`
- `use_sim_time`: `true`
- `fusion_extrapolator`: `true`
- `use_odometry`: `false`

## Frames

- `map_frame`: `map`
- `tracking_frame`: `imu`
- `published_frame`: `base_link`
- `odom_frame`: `odom`
- `provide_odom_frame`: `true`
- `publish_frame_projected_to_2d`: `true`
- `use_pose_extrapolator`: `true`
- `publish_to_tf`: `true`

## Pure Localization

- `POSE_GRAPH.constraint_builder.global_localization_min_score`: `0.62`
- `POSE_GRAPH.constraint_builder.min_score`: `0.95`
- `POSE_GRAPH.global_constraint_search_after_n_seconds`: `0`
- `TRAJECTORY_BUILDER.pure_localization_trimmer.max_submaps_to_keep`: `5`
- `TRAJECTORY_BUILDER_2D.num_accumulated_range_data`: `1`

## Global Matching

- `fast_correlative_scan_matcher.linear_search_window`: `0.05`
- `fast_correlative_scan_matcher.angular_search_window`: `math.rad(1.0)`
- `POSE_GRAPH.global_sampling_ratio`: `0.0055`
- `POSE_GRAPH.optimize_every_n_nodes`: `1`
- `POSE_GRAPH.constraint_builder.max_constraint_distance`: `15.0`
- `POSE_GRAPH.constraint_builder.loop_closure_translation_weight`: `100.0`
- `POSE_GRAPH.constraint_builder.loop_closure_rotation_weight`: `100.0`
- `POSE_GRAPH.constraint_builder.sampling_ratio`: `0.78`

## Local Scan Matching

- `real_time_correlative_scan_matcher.linear_search_window`: `0.05`
- `real_time_correlative_scan_matcher.angular_search_window`: `math.rad(1.0)`
- `real_time_correlative_scan_matcher.translation_delta_cost_weight`: `25.0`
- `real_time_correlative_scan_matcher.rotation_delta_cost_weight`: `25.0`
- `ceres_scan_matcher.occupied_space_weight`: `15.0`
- `ceres_scan_matcher.translation_weight`: `30.0`
- `ceres_scan_matcher.rotation_weight`: `30.0`

## Sensors

- `TRAJECTORY_BUILDER_2D.use_imu_data`: `true`
- `TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution`: `0.05`
- `TRAJECTORY_BUILDER_2D.min_range`: `0.1`
- `TRAJECTORY_BUILDER_2D.max_range`: `20.0`
- `TRAJECTORY_BUILDER_2D.missing_data_ray_length`: `5.0`
- `TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_length`: `5.0`
- `TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.min_num_points`: `350`
- `TRAJECTORY_BUILDER_2D.voxel_filter_size`: `0.05`
- `TRAJECTORY_BUILDER_2D.imu_gravity_time_constant`: `12.0`

## Outlier Filter

- `skip_submap_insertion_for_outliers`: `true`
- `outlier_max_translation_residual`: `0.15`
- `outlier_max_rotation_residual`: `0.03`
- `outlier_required_failures`: `2`
- `outlier_medium_translation_residual`: `0.10`
- `outlier_medium_rotation_residual`: `0.02`
- `outlier_medium_required_consecutive`: `4`
- `outlier_min_correlative_score`: `0.0`
- `outlier_min_num_filtered_points`: `0`

