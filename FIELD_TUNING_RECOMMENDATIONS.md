# 필드 튜닝 추천값 정리

대상 workspace: `local_SLAM_imu_wheel_ver5`

이 문서는 wheel/IMU 기반 pure localization에서 필드 테스트를 할 때 어떤 파라미터로 시작하고, 문제가 생기면 어떤 순서로 조정할지 정리한 파일입니다. 현재 `config.yaml`은 아래의 `recommended` 설정과 같은 방향으로 맞춰져 있습니다.

## 실행 방법

wheel metric launch 파일은 이제 `pbstream_file`을 인자로 받을 수 있습니다. 그래서 launch 파일을 직접 수정하지 않고도 0312, 0501, 0512 맵을 바꿔가며 테스트할 수 있습니다.

Cartographer 실행:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

ROS_DOMAIN_ID=73 \
LOCAL_QUALITY_METRICS_CSV=/tmp/local_quality.csv \
ros2 launch cartographer_ros Damvi_carto_pure_wheel_metric_headless_launch.py \
  pose_extrapolator_config:=/home/symoon/Desktop/F1/Local_SLAM_Complete/local_SLAM_imu_wheel_ver5/config.yaml \
  pbstream_file:=/home/symoon/Desktop/bag/0501/0501.pbstream
```

다른 터미널에서 같은 `ROS_DOMAIN_ID`로 bag 재생:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

ROS_DOMAIN_ID=73 \
ros2 bag play /home/symoon/Desktop/bag/0501/0501_DATA.bag --clock
```

## 추천 프로파일

### 1. 기본 추천값

필드에서 가장 먼저 사용할 값입니다. 후반 drift와 갑자기 잘못 끌리는 현상을 줄이는 쪽입니다.

```yaml
outlier_max_translation_residual: 0.08
outlier_max_rotation_residual: 0.03
outlier_required_failures: 1
outlier_medium_translation_residual: 0.07
outlier_medium_rotation_residual: 0.02
outlier_medium_required_consecutive: 2
```

파일:

```text
parameter_trials/field_recommended.yaml
```

### 2. 완화값

깨끗한 코스에서 추천값이 너무 민감하게 outlier를 잡거나, 로컬 pose 보정이 너무 뻣뻣하게 느껴질 때 사용합니다.

```yaml
outlier_max_translation_residual: 0.10
outlier_max_rotation_residual: 0.03
outlier_required_failures: 1
outlier_medium_translation_residual: 0.08
outlier_medium_rotation_residual: 0.02
outlier_medium_required_consecutive: 2
```

파일:

```text
parameter_trials/field_moderate.yaml
```

### 3. 보수값

디버깅용 fallback입니다. valid scan을 덜 버리는 대신, 잘못된 scan match도 더 쉽게 통과시킬 수 있습니다.

```yaml
outlier_max_translation_residual: 0.15
outlier_max_rotation_residual: 0.03
outlier_required_failures: 2
outlier_medium_translation_residual: 0.10
outlier_medium_rotation_residual: 0.02
outlier_medium_required_consecutive: 4
```

파일:

```text
parameter_trials/field_conservative.yaml
```

## 단위

```yaml
outlier_max_translation_residual
outlier_medium_translation_residual
```

단위는 meter입니다.

- `0.08` = 8 cm
- `0.07` = 7 cm
- `0.10` = 10 cm
- `0.15` = 15 cm

```yaml
outlier_max_rotation_residual
outlier_medium_rotation_residual
```

단위는 radian입니다.

- `0.03 rad` = 약 1.72도
- `0.02 rad` = 약 1.15도

```yaml
outlier_required_failures
outlier_medium_required_consecutive
```

단위는 없고 횟수입니다.

## 테스트 결과 요약

테스트 결과는 `local_quality.csv` 기준입니다.

주요 컬럼:

- `translation_residual`: 예측 pose와 scan match 결과의 위치 차이, 단위 meter
- `rotation_residual`: 예측 yaw와 scan match 결과의 yaw 차이, 단위 radian
- `was_outlier`: outlier로 판정되면 1
- `ceres_final_cost`: Ceres scan matcher 최종 cost

테스트 결과 저장 위치:

```text
metrics_data/field_recommendation_20260517_172531
```

| Bag | Profile | Rows | Outliers | tr95 | tr99 | q4 tr95 | last10 tr95 | cost90 |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| 0501 normal | conservative | 3807 | 0 | 0.0215 | 0.0301 | 0.0239 | 0.0211 | 116.0 |
| 0501 normal | moderate | 3806 | 0 | 0.0210 | 0.0298 | 0.0217 | 0.0173 | 110.5 |
| 0501 normal | recommended | 3807 | 0 | 0.0217 | 0.0304 | 0.0234 | 0.0206 | 110.6 |
| 0501 obstacle static+dynamic | conservative | 5662 | 1 | 0.0403 | 0.0607 | 0.0395 | 0.0377 | 171.5 |
| 0501 obstacle static+dynamic | recommended | 5662 | 21 | 0.0392 | 0.0577 | 0.0405 | 0.0380 | 171.1 |
| 0512 test 2 | moderate | 3244 | 3 | 0.0404 | 0.0597 | 0.0433 | 0.0359 | 260.3 |
| 0512 test 2 | recommended | 3244 | 9 | 0.0400 | 0.0620 | 0.0431 | 0.0285 | 269.8 |

0312 full-run 참고 결과:

| Bag | Profile | Rows | Outliers | tr95 | tr99 | last10 tr95 |
|---|---:|---:|---:|---:|---:|---:|
| 0312 sensor_only | 이전 outlier 기준 | 5611 | 0 | 0.0409 | 0.0611 | 0.0394 |
| 0312 sensor_only | recommended | 5624 | 7 | 0.0397 | 0.0574 | 0.0357 |

## 해석

기본 추천값을 먼저 쓰는 이유:

- 0312 bag에서 후반 drift 지표가 좋아졌습니다.
- 0501 obstacle bag에서 `tr95`, `tr99`가 개선됐습니다.
- 0512_test_2에서 마지막 10 percent 구간의 `tr95`가 좋아졌습니다.
- 동적 장애물이 있는 0501 obstacle bag에서도 outlier 비율은 약 0.37 percent라 과도하게 버리는 수준은 아니었습니다.

0501 normal처럼 깨끗한 환경에서는 `field_moderate.yaml`이 아주 근소하게 더 좋았습니다. 그래서 실전에서는 추천값으로 시작하고, 너무 민감하면 moderate로 완화하는 방식이 좋습니다.

## 필드 조정 순서

### 후반에 점점 틀어지거나 순간적으로 잘못 끌릴 때

추천값보다 한 단계 더 민감하게 조정합니다.

```yaml
outlier_max_translation_residual: 0.08 -> 0.07
outlier_medium_translation_residual: 0.07 -> 0.06
```

한 번에 여러 값을 바꾸지 말고, 같은 코스를 다시 한 바퀴 돌려 비교하는 게 좋습니다.

### 로컬 pose가 너무 뻣뻣하거나 정상 scan까지 버리는 느낌일 때

추천값보다 한 단계 완화합니다.

```yaml
outlier_max_translation_residual: 0.08 -> 0.10
outlier_medium_translation_residual: 0.07 -> 0.08
```

### 동적 장애물이 많을 때

먼저 recommended를 유지합니다. 테스트한 obstacle bag에서는 recommended가 p95/p99를 개선했고, outlier 비율도 과하지 않았습니다.

## 먼저 건드리지 말 값

아래 값들은 현재 stable baseline의 일부입니다. field에서 처음부터 바꾸지 않는 것을 권장합니다.

```yaml
imu_weight: 0.2
imu_delta_min: 0.3
wheelodom_weight: 0.01
odometry_translation_weight: 3000.0
odometry_rotation_weight: 0.0
ceres_translation_weight: 40.0
ceres_rotation_weight: 40.0
```

특히 `odometry_rotation_weight: 0.0`은 유지하는 것이 좋습니다. wheel yaw가 안정적이지 않기 때문에 yaw residual을 켜면 후반 drift가 커질 가능성이 있습니다.

## 참고 사항

`0512 normal` bag은 metric row가 50개밖에 나오지 않아서 최종 판단에 사용하지 않았습니다. `/imu/data` 개수가 scan/odom에 비해 너무 적어 wheel+IMU metric 테스트용으로 신뢰도가 낮습니다.

`0507` bag은 scan, IMU, wheel odom 데이터는 충분하지만 `~/Desktop/bag/0507` 아래에서 matching pbstream을 찾지 못해서 최종 추천 테스트에서는 제외했습니다.
