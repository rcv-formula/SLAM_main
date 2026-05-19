# ver6 previous tested adaptive parameters

This snapshot stores the earlier ver6 adaptive/step-limiter parameter set before
the later no-wheel Cartographer reset.

Key values:

- `use_odometry = true`
- `wheelodom_weight = 0.01`
- `featureless_wheelodom_weight = 0.01`
- `adaptive_straight_enabled = 1`
- `featureless_imu_weight = 1.00`
- `featureless_scan_longitudinal_blend = 0.65`
- `featureless_scan_lateral_blend = 0.80`
- `featureless_scan_yaw_blend = 0.70`
- `featureless_max_step_m = 0.08`

Metric logging is enabled for `/tmp/ver6_cartographer_local_quality.csv`.
