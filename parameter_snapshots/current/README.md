# Cartographer Parameter Snapshot

This folder saves the current wheel localization parameter set so the present
tuning point can be recovered after later experiments.

Captured at: `2026-05-27 21:43:17 KST`

## Files

- `cartographer_parameters_saved.yaml`: consolidated snapshot of the active
  launch defaults, Lua localization options, wheel odometry YAML values, and
  relocalization thresholds.
- `README.md`: scope and source-file notes for this snapshot.

## Source Files

The snapshot was taken from the current working tree, including uncommitted
changes in:

- `src/SLAM/cartographer_ros/configuration_files/Damvi_localization_config_wheel.lua`

Other referenced source files:

- `config_mapping_wheel.yaml`
- `src/SLAM/cartographer_ros/launch/Damvi_carto_pure_wheel_launch.py`

## Important Current Values

- `use_sim_time`: `false`
- `fusion_extrapolator`: `true`
- `tracking_frame`: `imu`
- `published_frame`: `base_link`
- `use_odometry`: `true`
- `TRAJECTORY_BUILDER_2D.use_imu_data`: `true`
- `TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution`: `0.05`
- `POSE_GRAPH.constraint_builder.global_localization_min_score`: `0.7`
- `POSE_GRAPH.constraint_builder.min_score`: `0.6`
- `POSE_GRAPH.global_constraint_search_after_n_seconds`: `0.3`
- `POSE_GRAPH.global_sampling_ratio`: `0.02`
- `POSE_GRAPH.relocalization_recovery_grace_sec`: `2.0`
- `wheelodom_weight`: `0.05`
- `wheel_odom_linear_scale`: `2.6`
- `odometry_translation_weight`: `30.0`
- `odometry_rotation_weight`: `0.0`
