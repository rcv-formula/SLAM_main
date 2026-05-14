# Parpameters

Last updated: 2026-05-13

This document summarizes the current wheel odom localization parameters and how
they are applied in Cartographer.

## Active Files

- Main Lua: `src/SLAM/cartographer_ros/configuration_files/Damvi_localization_config_wheel.lua`
- Wheel odom tuning YAML: `config.yaml`
- Metric wheel launch: `src/SLAM/cartographer_ros/launch/Damvi_carto_pure_wheel_metric_launch.py`
- Pose graph defaults: `src/cartographer/configuration_files/pose_graph.lua`
- Local extrapolator logic: `src/cartographer/cartographer/mapping/pose_extrapolator.h`
- Local extrapolator implementation: `src/cartographer/cartographer/mapping/pose_extrapolator.cc`
- Localization map: `src/SLAM/cartographer_ros/pbstream/0312.pbstream`

## Run Command

```bash
source /opt/ros/humble/setup.bash
source /home/symoon/Desktop/F1/Local_SLAM_Complete/local_SLAM_imu_wheel_ver2/install/setup.bash
ros2 launch cartographer_ros Damvi_carto_pure_wheel_metric_launch.py
```

To test another local fusion YAML without editing the default file:

```bash
ros2 launch cartographer_ros Damvi_carto_pure_wheel_metric_launch.py \
  pose_extrapolator_config:=/absolute/path/to/config.yaml
```

For clean parameter comparison, run a single bag player without `--loop`:

```bash
ros2 bag play /home/symoon/Desktop/bag/sensor_only/rosbag2_2026_03_13-17_45_23_0.db3 --clock
```

## Wheel Odom Input

Wheel odom is enabled:

```lua
use_odometry = true
odometry_sampling_ratio = 1.0
```

Cartographer subscribes wheel odom from:

```text
/odom_wheel
```

The wheel odom data is used in two different places:

```text
1. Pose extrapolator
   Local short-term prediction before scan matching.

2. Pose graph optimization
   Relative odom residual between consecutive trajectory nodes.
```

## Pose Graph Wheel Odom Weights

Current values are loaded from workspace root `config.yaml`:

```yaml
odometry_translation_weight: 1.0e3
odometry_rotation_weight: 0.0
```

Current active values:

```text
wheel translation residual weight = 1000
wheel rotation residual weight    = 0
```

Base defaults from `pose_graph.lua`:

```lua
odometry_translation_weight = 1e5
odometry_rotation_weight = 1e5
```

Meaning:

```text
wheel translation is used as a weak pose graph prior
wheel yaw is disabled as a pose graph residual
```

Weight ratio against local SLAM consecutive-node constraints:

```lua
local_slam_pose_translation_weight = 1e5
local_slam_pose_rotation_weight = 1e5
```

So current wheel translation is roughly:

```text
1000 : 100000 = 1 : 100
```

This is not an exact percentage, but it means wheel translation is much weaker
than local SLAM pose constraints.

## Pose Graph Yaw Sources

Wheel yaw residual is off:

```lua
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 0.0
```

Pose graph yaw still comes from:

```text
1. local SLAM relative pose yaw
2. intra-submap scan matching constraints
3. inter-submap/global matching constraints
4. wheel translation residual indirectly, because translation residual depends on node yaw
```

Current important yaw-related weights:

```lua
POSE_GRAPH.optimization_problem.local_slam_pose_rotation_weight = 1e5
POSE_GRAPH.matcher_rotation_weight = 1.6e3
POSE_GRAPH.constraint_builder.loop_closure_rotation_weight = 2000.0
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 0.0
```

## Pose Extrapolator Fusion Parameters

These are runtime parameters loaded from:

```text
config.yaml
```

Current values:

```yaml
imu_weight: 0.2
imu_delta_min: 0.3
wheelodom_weight: 0.01
odometry_translation_weight: 1.0e3
odometry_rotation_weight: 0.0
```

Meaning:

```text
imu_weight = 0.2
  IMU delta velocity affects local extrapolated translation by 20%.

imu_delta_min = 0.3
  If IMU delta velocity norm is greater than this threshold, it is rejected.

wheelodom_weight = 0.01
  Wheel odom velocity difference affects local extrapolated translation by 1%.
```

This is separate from pose graph weight:

```text
wheelodom_weight = 0.01
  Local extrapolator velocity fusion.

odometry_translation_weight = 1000
  Pose graph residual weight.
```

## Pose Extrapolator Wheel Flow

In `pose_extrapolator.cc`, wheel odom is converted into velocities:

```text
AddOdometryData()
  odometry_pose_delta = oldest_odom.inverse() * newest_odom
  angular_velocity_from_odometry_ = odom_delta_rotation / dt
  linear_velocity_from_odometry_ = odom_delta_translation / dt
```

In fusion mode, translation uses scan + IMU + weak wheel correction:

```text
linear_velocity_from_poses_
  + imu_weight * imu_delta_velocity
  + wheelodom_weight * wheel_odom_velocity_difference_along_scan_direction
```

Current effective local translation fusion:

```text
scan/local pose velocity
+ IMU delta velocity at 20%
+ wheel odom correction at 1%
```

## Pose Extrapolator Yaw Flow

With `fusion_extrapolator = true`, extrapolator yaw does not primarily use
wheel odom angular velocity.

Current behavior:

```text
fusion_extrapolator = true
  AdvanceImuTracker() uses angular_velocity_from_poses_
  wheel angular_velocity_from_odometry_ is not used in that branch

fusion_extrapolator = false
  AdvanceImuTracker() can use angular_velocity_from_odometry_
```

So current yaw behavior is:

```text
pose graph wheel yaw residual = off
local extrapolator wheel yaw  = effectively off in fusion mode
local extrapolator translation = wheel correction 1%
pose graph translation         = wheel residual weight 1000
```

## IMU Parameters

Current IMU config:

```lua
TRAJECTORY_BUILDER_2D.use_imu_data = true
imu_sampling_ratio = 1.0
TRAJECTORY_BUILDER_2D.imu_gravity_time_constant = 30.0
```

In 2D pose graph optimization, IMU is not directly used as a residual. It is
used through local extrapolation and local SLAM prediction.

## Map And Matching Parameters

Current mapping/localization settings:

```lua
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
```

LiDAR and IMU:

```lua
MAP_BUILDER.use_trajectory_builder_2d = true
TRAJECTORY_BUILDER_2D.use_imu_data = true

TRAJECTORY_BUILDER_2D.min_range = 0.1
TRAJECTORY_BUILDER_2D.max_range = 25.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5.0

TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_length = 5.0
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.min_num_points = 200
TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.05

TRAJECTORY_BUILDER_2D.imu_gravity_time_constant = 30.0
```

Local scan matcher:

```lua
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = false

TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 1.0
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(10.0)
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 5.0
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 5.0

TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 50.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 20.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 20.0
```

Pose graph and global matching:

```lua
MAP_BUILDER.num_background_threads = 4
POSE_GRAPH.optimize_every_n_nodes = 2

POSE_GRAPH.constraint_builder.global_localization_min_score = 0.58
POSE_GRAPH.constraint_builder.min_score = 0.65
POSE_GRAPH.constraint_builder.sampling_ratio = 0.0001
POSE_GRAPH.global_sampling_ratio = 0.005
POSE_GRAPH.global_constraint_search_after_n_seconds = 0
POSE_GRAPH.constraint_builder.max_constraint_distance = 15.0

POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 1.5
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(10.0)

POSE_GRAPH.constraint_builder.loop_closure_translation_weight = 2000.0
POSE_GRAPH.constraint_builder.loop_closure_rotation_weight = 2000.0
```

Pure localization:

```lua
TRAJECTORY_BUILDER.pure_localization_trimmer = {
  max_submaps_to_keep = 4,
}

TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1
```

## Tuning Notes

Wheel pose graph translation sweep:

```lua
1e3  -- weak, current
5e3  -- visible wheel influence
1e4  -- stronger wheel tracking
1e5  -- same order as local SLAM, risky if wheel odom slips
```

Wheel yaw sweep:

```lua
0.0  -- current, wheel yaw disabled
1e1  -- very weak yaw influence
1e2  -- weak yaw influence
1e3+ -- risky unless wheel yaw is stable
```

Local extrapolator wheel correction sweep:

```yaml
wheelodom_weight: 0.01  # current
wheelodom_weight: 0.05
wheelodom_weight: 0.10
```

Changing Lua or `config.yaml` values only requires restarting the launch.
Changing `pose_extrapolator.h/.cc` still requires rebuilding.

## Clean Run

Before each comparison run:

```bash
pkill -f "ros2 bag play"
pkill -f "ros2 launch cartographer_ros"
pkill -f "cartographer_node"
pkill -f "cartographer_occupancy_grid_node"
pkill -f "trajectory_to_odom"
```

Avoid running multiple bag players. Duplicate `/clock`, `/scan`, `/imu/data`,
or `/odom_wheel` publishers can cause:

```text
Detected jump back in time
TF_OLD_DATA
Non-sorted data added to queue: '(1, imu)'
```
