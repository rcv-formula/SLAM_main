# ver6 scan+IMU mapping tuning base

Base for further tuning when wheel odom must not affect mapping.

Key points:

- Cartographer `use_odometry = false`
- `odometry_sampling_ratio = 0.0`
- `wheelodom_weight = 0.0`
- `featureless_wheelodom_weight = 0.0`
- adaptive straight mode enabled
- scan+IMU only tuning knobs:
  - `featureless_imu_weight = 1.00`
  - `featureless_scan_longitudinal_blend = 0.60`
  - `featureless_scan_lateral_blend = 0.70`
  - `featureless_scan_yaw_blend = 0.75`
  - `featureless_max_step_m = 0.075`

Metric CSV: `/tmp/ver6_cartographer_local_quality.csv`
