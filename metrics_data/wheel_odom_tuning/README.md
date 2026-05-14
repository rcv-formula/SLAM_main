# Wheel Odom Metric Tuning - 2026-05-13

## Launch choice

- Use `Damvi_carto_pure_wheel_metric_headless_launch.py` for parameter comparison.
- Use `Damvi_carto_pure_wheel_metric_launch.py` only when occupancy grid / visualization is needed.
- Headless launch keeps only `cartographer_node --collect_metrics`, so the CSV is less noisy.

## Bag and outputs

- Bag: `/home/symoon/Desktop/bag/sensor_only/rosbag2_2026_03_13-17_45_23_0.db3`
- Output directory: `metrics_data/wheel_odom_tuning/headless_runs_v2`
- Duration: 30 seconds per trial

## Compared trials

| Trial | wheelodom_weight | odom translation | dist p95 | global found/search | local found/search | max queue | queue delay |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| current | 0.01 | 1000.0 | 0.0415 | 40/105 = 0.381 | 2/7 = 0.286 | 18 | 0.0078 |
| no_wheel | 0.0 | 0.0 | 0.0408 | 45/105 = 0.429 | 2/7 = 0.286 | 14 | 0.0076 |
| local_wheel_005 | 0.005 | 1000.0 | 0.0423 | 50/105 = 0.476 | 1/7 = 0.143 | 19 | 0.0101 |
| local_wheel_003_reduced | 0.003 | 1000.0 | 0.0377 | 48/105 = 0.457 | 2/7 = 0.286 | 21 | 0.0090 |
| pose_graph_odom_3000 | 0.01 | 3000.0 | 0.0400 | 47/104 = 0.452 | 1/7 = 0.143 | 7 | 0.0044 |
| local005_pg3000 | 0.005 | 3000.0 | 0.0427 | 66/142 = 0.465 | 1/10 = 0.100 | 18 | 0.0083 |

## Selected setting

Keep the local wheel correction unchanged and tune only pose graph odom:

```yaml
wheelodom_weight: 0.01
odometry_translation_weight: 3000.0
odometry_rotation_weight: 0.0
```

Reason:

- `pose_graph_odom_3000` had the most stable pose graph queue: max queue `7`, average queue delay `0.0044`.
- It improved global constraint discovery compared with current: `0.452` vs `0.381`.
- The combined reduced-local trial, `local005_pg3000`, made residuals and local constraint discovery worse.

Rollback snapshot:

- `parameter_snapshots/current/cartographer_parameters_saved.yaml`
- `parameter_snapshots/current/README.md`

## Outlier rerun - 2026-05-14

Outlier 로직을 `Damvi_localization_config_wheel.lua`에도 켠 뒤,
`Damvi_carto_pure_wheel_metric_launch.py`와 sensor-only bag으로 다시 비교했습니다.

- Bag: `/home/symoon/Desktop/bag/sensor_only/rosbag2_2026_03_13-17_45_23_0.db3`
- Output directory: `metrics_data/wheel_odom_tuning/outlier_rerun_20260514_171531`
- Final selected config: `parameter_trials/outlier_imu015_limit025_odom7000.yaml`

60초 검증 결과:

| Trial | dist p95 | angle p95 | global found/search | local found/search | max queue | avg queue | max delay |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| odom6000 | 0.04 | 0.02 | 127/265 = 0.479 | 2/19 = 0.105 | 9 | 1.15 | 0.0550 |
| odom7000 | 0.04 | 0.02 | 127/263 = 0.483 | 6/19 = 0.316 | 11 | 1.15 | 0.0572 |
| odom8000 | 0.04 | 0.02 | 128/263 = 0.487 | 4/19 = 0.211 | 20 | 1.46 | 0.1497 |

Selected values:

```yaml
imu_weight: 0.15
imu_delta_min: 0.25
wheelodom_weight: 0.01
odometry_translation_weight: 7000.0
odometry_rotation_weight: 0.0
```

Reason:

- `odom7000` kept the same residual bucket as the other candidates.
- It found more local constraints than `6000` and had much lower queue spike than `8000`.
- Wheel yaw residual remains disabled because `odom6000_rot50` worsened distance p95 to `0.08`.

## Restored previous good setting - 2026-05-14

사용자가 기존 파라미터가 더 나아 보인다고 판단해서, 기존 good setting을 `config.yaml`에 복원한 뒤
`Damvi_carto_pure_wheel_metric_launch.py` 기본 인자로 다시 60초 검증했습니다.

- Output directory: `metrics_data/wheel_odom_tuning/restored_good_20260514_174111`
- Active config: `config.yaml`

비교 결과:

| Trial | dist p95 | angle p95 | global found/search | local found/search | max queue | avg queue | max delay | avg delay |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| restored_good_3000 | 0.04 | 0.02 | 132/263 = 0.502 | 4/19 = 0.211 | 11 | 1.00 | 0.0855 | 0.0031 |
| previous_7000 | 0.04 | 0.02 | 127/263 = 0.483 | 6/19 = 0.316 | 11 | 1.15 | 0.0572 | 0.0037 |

Restored selected values:

```yaml
imu_weight: 0.2
imu_delta_min: 0.3
wheelodom_weight: 0.01
odometry_translation_weight: 3000.0
odometry_rotation_weight: 0.0
```

Reason:

- 기존 good setting이 global constraint 발견률이 더 높았습니다: `0.502` vs `0.483`.
- 평균 queue와 평균 delay도 더 낮았습니다.
- `7000`은 local constraint 수가 더 많았지만, 전체 안정성 기준으로는 복원값이 더 균형적입니다.

## Wheel/odom-only sweep - 2026-05-14

IMU 관련 값은 고정했습니다.

```yaml
imu_weight: 0.2
imu_delta_min: 0.3
```

테스트한 값:

- `wheelodom_weight`: `0.0`, `0.005`, `0.008`, `0.01`, `0.012`, `0.02`
- `odometry_translation_weight`: `1000`, `2500`, `3000`, `3500`, `5000`, `7000`
- `odometry_rotation_weight`: `0`, `10`

30초 sweep 결과 현재값이 global constraint 발견률이 가장 좋았습니다.

| Trial | dist p95 | angle p95 | global found/search | local found/search | max queue | avg queue | max delay |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| current: local010 odom3000 rot0 | 0.04 | 0.02 | 66/128 = 0.516 | 2/9 = 0.222 | 7 | 0.87 | 0.0482 |
| local000 odom3000 rot0 | 0.08 | 0.02 | 57/128 = 0.445 | 3/9 = 0.333 | 7 | 0.63 | 0.0488 |
| local005 odom3000 rot0 | 0.04 | 0.02 | 64/128 = 0.500 | 2/9 = 0.222 | 9 | 1.20 | 0.0537 |
| local020 odom3000 rot0 | 0.04 | 0.02 | 63/128 = 0.492 | 2/9 = 0.222 | 12 | 1.50 | 0.0968 |
| local010 odom2500 rot0 | 0.04 | 0.02 | 57/128 = 0.445 | 3/9 = 0.333 | 3 | 0.50 | 0.0020 |
| local010 odom3500 rot0 | 0.04 | 0.02 | 64/128 = 0.500 | 2/9 = 0.222 | 19 | 1.57 | 0.1468 |
| local010 odom5000 rot0 | 0.04 | 0.02 | 60/128 = 0.469 | 3/9 = 0.333 | 14 | 1.00 | 0.0989 |
| local010 odom7000 rot0 | 0.08 | 0.02 | 60/127 = 0.472 | 2/9 = 0.222 | 8 | 1.20 | 0.0632 |
| local010 odom3000 rot10 | 0.04 | 0.02 | 60/127 = 0.472 | 1/9 = 0.111 | 18 | 1.33 | 0.1421 |

`odometry_translation_weight=2500`은 queue가 좋아 보여서 60초 검증했지만, 정확도 기준으로 현재값보다 나빴습니다.

| Trial | dist p95 | angle p95 | global found/search | local found/search | max queue | avg queue | max delay | avg delay |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| current odom3000 | 0.04 | 0.02 | 132/263 = 0.502 | 4/19 = 0.211 | 11 | 1.00 | 0.0855 | 0.0031 |
| candidate odom2500 | 0.08 | 0.02 | 127/263 = 0.483 | 4/19 = 0.211 | 9 | 1.19 | 0.0639 | 0.0023 |

Final decision:

```yaml
wheelodom_weight: 0.01
odometry_translation_weight: 3000.0
odometry_rotation_weight: 0.0
```

Reason:

- 다른 wheel/odom 후보보다 global constraint 발견률이 높았습니다.
- `wheelodom_weight=0` 또는 `odometry_translation_weight=7000`은 distance p95가 `0.08`로 나빠졌습니다.
- `odometry_rotation_weight=10`은 queue와 global/local constraint 모두 나빠져서 yaw odom은 계속 끕니다.

## Wheel/odom combo sweep - 2026-05-14

`wheelodom_weight`와 `odometry_translation_weight`를 동시에 바꾸는 경우도 추가로 테스트했습니다.
IMU 값은 계속 고정했습니다.

30초 combo sweep:

| Trial | dist p95 | angle p95 | global found/search | local found/search | max queue | avg queue | max delay |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| current: local010 odom3000 rot0 | 0.04 | 0.02 | 66/128 = 0.516 | 2/9 = 0.222 | 7 | 0.87 | 0.0482 |
| local005 odom2500 rot0 | 0.04 | 0.02 | 63/128 = 0.492 | 1/9 = 0.111 | 15 | 1.73 | 0.1261 |
| local005 odom3500 rot0 | 0.04 | 0.02 | 63/129 = 0.488 | 1/9 = 0.111 | 6 | 0.83 | 0.0199 |
| local005 odom5000 rot0 | 0.04 | 0.02 | 65/128 = 0.508 | 1/9 = 0.111 | 12 | 1.23 | 0.0796 |
| local012 odom2500 rot0 | 0.04 | 0.02 | 57/128 = 0.445 | 2/9 = 0.222 | 17 | 1.90 | 0.1523 |
| local012 odom3500 rot0 | 0.04 | 0.02 | 62/129 = 0.481 | 2/9 = 0.222 | 17 | 1.83 | 0.1274 |
| local012 odom5000 rot0 | 0.04 | 0.02 | 64/128 = 0.500 | 2/9 = 0.222 | 16 | 1.53 | 0.1127 |
| local010 odom3000 rot5 | 0.04 | 0.02 | 67/128 = 0.523 | 1/9 = 0.111 | 15 | 2.17 | 0.1549 |

`rot5`는 30초에서 global 발견률만 약간 높아서 60초 검증을 추가했습니다.

| Trial | dist p95 | angle p95 | global found/search | local found/search | max queue | avg queue | max delay | avg delay |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| current rot0 | 0.04 | 0.02 | 132/263 = 0.502 | 4/19 = 0.211 | 11 | 1.00 | 0.0855 | 0.0031 |
| candidate rot5 | 0.08 | 0.02 | 135/263 = 0.513 | 4/19 = 0.211 | 21 | 2.29 | 0.1510 | 0.0129 |

Final decision remains unchanged:

```yaml
wheelodom_weight: 0.01
odometry_translation_weight: 3000.0
odometry_rotation_weight: 0.0
```

Reason:

- 동시에 바꾼 후보들은 current보다 global/local constraint 또는 queue가 나빠졌습니다.
- `rot5`는 global 발견률이 조금 높았지만 distance p95와 queue가 크게 나빠졌습니다.
