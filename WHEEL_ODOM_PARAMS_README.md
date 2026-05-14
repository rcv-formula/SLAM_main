# Wheel Odom Localization Parameter Snapshot

Last updated: 2026-05-13

This file records the current wheel odometry localization parameters used for
Cartographer comparison runs.

## Main Files

- Lua config: `src/SLAM/cartographer_ros/configuration_files/Damvi_localization_config_wheel.lua`
- Wheel odom tuning YAML: `config.yaml`
- Metric launch: `src/SLAM/cartographer_ros/launch/Damvi_carto_pure_wheel_metric_launch.py`
- Base pose graph defaults: `src/cartographer/configuration_files/pose_graph.lua`
- Localization map: `src/SLAM/cartographer_ros/pbstream/0312.pbstream`

## Launch Setup

Use this launch for metric comparison with wheel odometry enabled:

```bash
source /opt/ros/humble/setup.bash
source /home/symoon/Desktop/F1/Local_SLAM_Complete/local_SLAM_imu_wheel_ver2/install/setup.bash
ros2 launch cartographer_ros Damvi_carto_pure_wheel_metric_launch.py
```

Important launch values:

```python
--collect_metrics
configuration_basename = Damvi_localization_config_wheel.lua
load_state_filename = pbstream/0312.pbstream
use_sim_time = true
use_odometry = true
provide_odom_frame = true
publish_frame_projected_to_2d = true
pose_extrapolator_config = /home/symoon/Desktop/F1/Local_SLAM_Complete/local_SLAM_imu_wheel_ver2/config.yaml
```

## Input And Frames

```lua
map_frame = "map"
tracking_frame = "imu"
published_frame = "base_link"
odom_frame = "odom"
provide_odom_frame = true
use_odometry = true
use_imu_data = true
```

Wheel odometry topic:

```text
/odom_wheel
```

Sampling:

```lua
rangefinder_sampling_ratio = 1.0
odometry_sampling_ratio = 1.0
imu_sampling_ratio = 1.0
fixed_frame_pose_sampling_ratio = 1.0
landmarks_sampling_ratio = 1.0
```

Publish periods:

```lua
lookup_transform_timeout_sec = 0.2
submap_publish_period_sec = 0.025
pose_publish_period_sec = 0.025
trajectory_publish_period_sec = 0.025
```

## Wheel Odometry Weights

Current values are loaded from workspace root `config.yaml`:

```yaml
odometry_translation_weight: 1.0e3
odometry_rotation_weight: 0.0
```

Current wheel odom weight:

```text
translation weight = 1000
rotation weight    = 0
```

Base default in `pose_graph.lua` before override:

```lua
odometry_translation_weight = 1e5
odometry_rotation_weight = 1e5
```

Meaning:

- Wheel odom translation is kept as a weak pose graph prior.
- Wheel odom yaw/rotation is disabled in pose graph optimization.
- This is intended to avoid map twisting or lateral drift from unstable wheel yaw.
- Wheel odom can still be used by the local extrapolator path because `use_odometry = true`.

## Local Extrapolator Fusion

These values are loaded from workspace root `config.yaml` at
`cartographer_node` startup:

```yaml
imu_weight: 0.2
imu_delta_min: 0.3
wheelodom_weight: 0.01
odometry_translation_weight: 1.0e3
odometry_rotation_weight: 0.0
```

`wheelodom_weight` here is separate from pose graph
`odometry_translation_weight`. Changing this YAML only needs a launch restart.

## Pure Localization And Global Matching

```lua
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.05

POSE_GRAPH.constraint_builder.global_localization_min_score = 0.58
POSE_GRAPH.constraint_builder.min_score = 0.65
POSE_GRAPH.global_constraint_search_after_n_seconds = 0

TRAJECTORY_BUILDER.pure_localization_trimmer = {
  max_submaps_to_keep = 4,
}

TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1
```

Global candidate search:

```lua
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 1.5
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(10.0)
POSE_GRAPH.global_sampling_ratio = 0.005
```

Pose graph constraint settings:

```lua
POSE_GRAPH.optimize_every_n_nodes = 2
POSE_GRAPH.constraint_builder.max_constraint_distance = 15.0
POSE_GRAPH.constraint_builder.loop_closure_translation_weight = 2000.0
POSE_GRAPH.constraint_builder.loop_closure_rotation_weight = 2000.0
POSE_GRAPH.constraint_builder.sampling_ratio = 0.0001
```

## Local Scan Matching

Online correlative scan matching is currently disabled:

```lua
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = false
```

These values are present, but are only active if online correlative scan
matching is turned on:

```lua
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 1.0
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(10.0)
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 5.0
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 5.0
```

Ceres scan matcher:

```lua
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 50.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 20.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 20.0
```

## LiDAR And IMU

```lua
TRAJECTORY_BUILDER_2D.min_range = 0.1
TRAJECTORY_BUILDER_2D.max_range = 25.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5.0
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_length = 5.0
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.min_num_points = 200
TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.05
TRAJECTORY_BUILDER_2D.imu_gravity_time_constant = 30.0
```

## Test Run Notes

For clean metric comparison, run one bag player and one Cartographer launch only.
Avoid `--loop` during parameter comparison because it can cause time jumps:

```bash
ros2 bag play /home/symoon/Desktop/bag/sensor_only/rosbag2_2026_03_13-17_45_23_0.db3 --clock
```

Before a clean run:

```bash
pkill -f "ros2 bag play"
pkill -f "ros2 launch cartographer_ros"
pkill -f "cartographer_node"
pkill -f "cartographer_occupancy_grid_node"
pkill -f "trajectory_to_odom"
```
