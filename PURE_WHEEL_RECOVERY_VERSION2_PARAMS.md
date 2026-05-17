# Pure Wheel Recovery Version 2 Parameters

Last updated: 2026-05-18

This version2 profile is a recovery-focused variant of the locked pure wheel localization
settings. Keep `PURE_WHEEL_LOCALIZATION_LOCKED_PARAMS.md` as the baseline; use
this profile when the robot usually localizes well but needs to attach back to
the map faster after brief broken sections.

## Launch

- Launch file: `src/SLAM/cartographer_ros/launch/Damvi_carto_pure_wheel_launch.py`
- Lua config: `src/SLAM/cartographer_ros/configuration_files/Damvi_localization_config_wheel.lua`
- Wheel config: `config.yaml`
- Current active test map may be passed with `pbstream_file:=...`
- `use_sim_time`: `true`
- `fusion_extrapolator`: `true`
- `use_odometry`: `true`

## Unchanged From Locked Baseline

```yaml
imu_weight: 0.2
imu_delta_min: 0.3
wheelodom_weight: 0.01
odometry_translation_weight: 1000.0
odometry_rotation_weight: 0.0
global_localization_min_score: 0.62
constraint_min_score: 0.95
global_constraint_search_after_n_seconds: 0
max_submaps_to_keep: 5
loop_closure_translation_weight: 100.0
loop_closure_rotation_weight: 100.0
max_constraint_distance: 15.0
optimize_every_n_nodes: 1
ceres_occupied_space_weight: 15.0
ceres_translation_weight: 30.0
ceres_rotation_weight: 30.0
outlier_max_translation_residual: 0.15
outlier_max_rotation_residual: 0.03
outlier_required_failures: 2
outlier_medium_translation_residual: 0.10
outlier_medium_rotation_residual: 0.02
outlier_medium_required_consecutive: 4
```

## Version 2 Recovery Changes

Compared with the locked baseline, only global recovery search and sampling are
opened up:

```yaml
global_linear_search_window: 0.12
global_angular_search_window_deg: 3.0
global_sampling_ratio: 0.012
constraint_builder_sampling_ratio: 0.90
```

Locked baseline values were:

```yaml
global_linear_search_window: 0.05
global_angular_search_window_deg: 1.0
global_sampling_ratio: 0.0055
constraint_builder_sampling_ratio: 0.78
```

## Intent

- Let global localization search a wider neighborhood after brief localization
  breaks.
- Keep `global_localization_min_score` at `0.62`; raising it to `0.72` made the
  map fail to attach on the 0313 sensor bag.
- Avoid changing wheel odom, Ceres scan matcher, outlier thresholds, or pure
  launch behavior.

## Current Status

This is the currently preferred recovery version2 profile for `pure_wheel` after testing
the 0313 sensor bag visually. It is not a replacement for the locked baseline;
use the locked document when restoring the known stable baseline.
