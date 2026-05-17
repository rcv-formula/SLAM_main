# Pure Wheel Localization Locked Parameters

Last updated: 2026-05-17

This is the current good `Damvi_carto_pure_wheel_launch.py` profile. Treat it
as locked unless explicitly retuning the wheel localization path.

The intent of this profile is:

- keep the stable `pure_launch` scan matching behavior,
- keep wheel odometry enabled,
- use wheel odometry only as a weak motion prior,
- avoid wide global corrections that make the map look shaky.

## Launch

- Launch file: `src/SLAM/cartographer_ros/launch/Damvi_carto_pure_wheel_launch.py`
- Lua config: `src/SLAM/cartographer_ros/configuration_files/Damvi_localization_config_wheel.lua`
- Wheel config: `config.yaml`
- Default map: `0125_4.pbstream`
- `use_sim_time`: `true`
- `fusion_extrapolator`: `true`
- `use_odometry`: `true`

## Wheel Odom Weights

These are loaded from `config.yaml`.

```yaml
imu_weight: 0.2
imu_delta_min: 0.3
wheelodom_weight: 0.01
odometry_translation_weight: 1000.0
odometry_rotation_weight: 0.0
```

`wheelodom_weight` is intentionally weak. Do not raise it back to `0.05` for
this locked profile; that made `pure_wheel` visually shakier.

## Global Matching

These values mirror the stable no-wheel `pure_launch` behavior.

```yaml
global_localization_min_score: 0.62
constraint_min_score: 0.95
global_constraint_search_after_n_seconds: 0
max_submaps_to_keep: 5
global_linear_search_window: 0.05
global_angular_search_window_deg: 1.0
global_sampling_ratio: 0.0055
constraint_builder_sampling_ratio: 0.78
loop_closure_translation_weight: 100.0
loop_closure_rotation_weight: 100.0
max_constraint_distance: 15.0
optimize_every_n_nodes: 1
```

## Local Scan Matching

```yaml
ceres_occupied_space_weight: 15.0
ceres_translation_weight: 30.0
ceres_rotation_weight: 30.0
```

In `Damvi_localization_config_wheel.lua`:

```lua
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = false
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.05
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(1.0)
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 25.0
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 25.0
```

## LiDAR And IMU

```lua
TRAJECTORY_BUILDER_2D.min_range = 0.1
TRAJECTORY_BUILDER_2D.max_range = 20.0
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_length = 5.0
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.min_num_points = 350
TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.05
TRAJECTORY_BUILDER_2D.imu_gravity_time_constant = 12.0
```

## Outlier Filter

```yaml
outlier_max_translation_residual: 0.15
outlier_max_rotation_residual: 0.03
outlier_required_failures: 2
outlier_medium_translation_residual: 0.10
outlier_medium_rotation_residual: 0.02
outlier_medium_required_consecutive: 4
```

The C++ outlier handling should remain enabled: outliers use prediction pose and
must not be inserted into submaps.

## Runtime Notes

For bag testing, do not use `--loop`; the clock jumps backward and Cartographer
can die on non-monotonic odometry timestamps.

If the bag does not provide an `imu` transform, publish a static transform
between `base_link` and `imu` before testing. The bag checked on 2026-05-17 had
only `base_link -> laser` in `/tf_static`.

