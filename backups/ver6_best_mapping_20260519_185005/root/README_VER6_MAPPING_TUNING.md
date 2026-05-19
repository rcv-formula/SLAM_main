# Ver6 Mapping Tuning Summary

이 문서는 `local_SLAM_imu_wheel_ver6`에서 긴 직선/특징점 부족 구간의 Cartographer mapping 흔들림을 줄이기 위해 반영한 내용을 요약한다.

중요 전제:

- `ver5`는 수정하지 않았다.
- 실행은 `ros2 launch cartographer_ros Damvi_carto_launch.py` 기준이다.
- mapping은 wheel odom 없이 scan + IMU 중심으로 맞춘다.
- wheel odom 관련 weight는 현재 모두 `0.0`이다.

## 현재 추천 상태

현재 `config.yaml` 기준값:

```yaml
wheelodom_weight: 0.0
featureless_wheelodom_weight: 0.0

adaptive_straight_enabled: 1
adaptive_straight_eigen_ratio_threshold: 6.6
adaptive_straight_min_points: 180
adaptive_straight_min_score: 0.20
adaptive_straight_max_score: 0.98
adaptive_straight_enter_streak: 4
adaptive_straight_exit_streak: 12

featureless_imu_weight: 1.00
featureless_scan_longitudinal_blend: 0.68
featureless_scan_lateral_blend: 0.66
featureless_scan_yaw_blend: 0.70
featureless_max_step_m: 0.076

featureless_entry_scan_anchor_enabled: 0
featureless_scan_correction_lpf_enabled: 1
featureless_scan_correction_lpf_alpha: 0.27
featureless_scan_correction_lpf_max_update_m: 0.0
featureless_scan_correction_lpf_max_update_yaw: 0.0
```

현재 의도:

- 긴 직선/featureless 구간을 빨리 감지한다.
- scan matcher의 lateral/yaw 흔들림은 줄인다.
- 진행 방향 보정은 너무 줄이지 않아 맵이 과하게 줄어드는 것을 피한다.
- 최종 pose 전체가 아니라 scan correction residual만 low-pass 한다.

## 실행 설정

수정 파일:

- `src/SLAM/cartographer_ros/launch/Damvi_carto_launch.py`
- `install/cartographer_ros/share/cartographer_ros/launch/Damvi_carto_launch.py`
- `src/SLAM/cartographer_ros/configuration_files/Damvi_carto_config.lua`
- `install/cartographer_ros/share/cartographer_ros/configuration_files/Damvi_carto_config.lua`
- `config.yaml`

launch 쪽:

- `use_sim_time: True`
- `MAPPING_ADAPTIVE_CONFIG`, `POSE_EXTRAPOLATOR_CONFIG`, `WHEEL_ODOM_CONFIG`가 workspace의 `config.yaml`을 읽도록 설정

Cartographer Lua 쪽:

```lua
options.use_odometry = false
options.odometry_sampling_ratio = 0.0
TRAJECTORY_BUILDER_2D.use_imu_data = true
TRAJECTORY_BUILDER_2D.log_local_quality_metrics_to_csv = true
TRAJECTORY_BUILDER_2D.local_quality_metrics_csv_path = "/tmp/ver6_cartographer_local_quality.csv"
```

## C++ 반영 내용

주요 파일:

- `src/cartographer/cartographer/mapping/internal/2d/local_trajectory_builder_2d.cc`
- `src/cartographer/cartographer/mapping/internal/2d/local_trajectory_builder_2d.h`
- `src/cartographer/cartographer/mapping/pose_extrapolator.cc`
- `src/cartographer/cartographer/mapping/pose_extrapolator.h`

추가한 기능:

1. 직선/특징점 부족 감지
   - scan point cloud의 평면 degeneracy ratio를 계산한다.
   - ratio가 threshold 이상이고 point 수/score 조건을 만족하면 `featureless_straight_mode`로 진입한다.

2. featureless 구간 pose blend
   - `pose_prediction`과 `scan pose_estimate`의 차이를 forward/lateral/yaw로 분해한다.
   - 각 축별 blend 값으로 scan correction 반영량을 따로 조절한다.

3. featureless step limiter
   - featureless 구간에서 한 프레임 pose 이동량을 `featureless_max_step_m`으로 제한한다.
   - 현재값은 `0.076`.

4. featureless 진입 첫 프레임 guard
   - featureless가 꺼져 있을 때도 마지막 정상 pose를 저장한다.
   - featureless 진입 첫 프레임부터 step limiter 기준점이 생기도록 했다.

5. scan correction low-pass filter
   - 최종 pose 전체가 아니라 `pose_prediction -> scan pose_estimate` correction만 필터링한다.
   - 현재는 `alpha 0.27`.
   - anchor 방식보다 유연하고 raw scan보다 덜 흔들리게 하는 목적이다.

6. scan correction anchor 옵션
   - 첫 featureless scan correction을 유지하는 옵션을 만들었다.
   - 테스트상 방식이 맞지 않아 현재는 꺼져 있다.

```yaml
featureless_entry_scan_anchor_enabled: 0
```

## 시도했고 제외한 방향

아래 방향은 테스트상 더 나빴거나 불안정했다.

- wheel odom mapping 사용
  - wheel이 불안정해서 mapping에서는 제외.
  - `wheelodom_weight: 0.0`, `featureless_wheelodom_weight: 0.0`.

- featureless 모드를 너무 오래 유지
  - `exit_streak`를 크게 늘리거나 scan yaw를 너무 낮추면 맵이 줄어드는 경향이 커졌다.

- scan 반영을 크게 증가
  - `longitudinal 0.72`, `yaw 0.80`, `step 0.082` 계열은 잘못 붙을 때 맵이 더 늘거나 흔들렸다.

- 첫 LiDAR correction anchor 유지
  - 처음 붙은 correction만 유지하면 이후 상황 변화에 못 따라가서 현재 꺼둠.

- LPF alpha를 너무 낮추고 update limit 적용
  - `alpha 0.20`, `max_update_m 0.015`, `max_update_yaw 0.008`은 이전 correction에 너무 묶여서 개선이 작거나 나빴다.

## 주요 스냅샷

현재 계열:

```bash
parameter_snapshots/BEST_ver6_current_mapping_lpf_alpha030_20260519_184202
parameter_snapshots/ver6_lpf_alpha025_from_stretched_20260519_184538
```

LPF 관련:

```bash
parameter_snapshots/BEST_ver6_scan_correction_lpf_alpha030_lateral66_yaw70_step076_20260519_172951
parameter_snapshots/BEST_ver6_scan_correction_lpf_alpha020_rate_limited_20260519_173901
parameter_snapshots/ver6_reverted_lpf_alpha030_no_rate_limit_20260519_183701
```

anchor/entry guard 관련:

```bash
parameter_snapshots/BEST_ver6_entry_lidar_anchor_lateral66_yaw70_step076_20260519_172210
parameter_snapshots/BEST_ver6_lateral66_yaw70_step076_anchor_off_20260519_172546
parameter_snapshots/BEST_ver6_featureless_entry_guard_lateral66_yaw70_step076_20260519_171006
```

초기 best 계열:

```bash
parameter_snapshots/BEST_ver6_carto_scan_imu_lateral66_yaw70_confirmed_20260519_165947
parameter_snapshots/BEST_ver6_carto_scan_imu_lateral66_yaw70_step076_restored_20260519_170524
```

## 테스트 명령

Cartographer:

```bash
cd /home/symoon/Desktop/F1/Local_SLAM_Complete/local_SLAM_imu_wheel_ver6
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch cartographer_ros Damvi_carto_launch.py
```

Bag:

```bash
ros2 bag play /home/symoon/Desktop/bag/0518_2 --clock
```

C++를 바꾼 뒤에는 빌드 필요:

```bash
cd /home/symoon/Desktop/F1/Local_SLAM_Complete/local_SLAM_imu_wheel_ver6
source /opt/ros/humble/setup.bash
colcon build --packages-select cartographer cartographer_ros --parallel-workers 1 --cmake-args -DCMAKE_BUILD_TYPE=Release -DCARTOGRAPHER_BUILD_TESTS=OFF -DBUILD_GRPC=OFF -DBUILD_PROMETHEUS=OFF
source install/setup.bash
```

`config.yaml`만 바꾼 경우에는 빌드 없이 launch 재시작만 하면 된다.

## 판단 기준

문제 현상:

- 긴 직선/특징점 부족 구간에서 scan matcher가 비슷한 직선 벽 후보 사이를 오간다.
- 잘 붙을 때는 뒤쪽 옅은 scan 흔적과 맞지만, 실패할 때는 다른 곳에 붙어 맵이 늘거나 줄어든다.

현재 접근:

- wheel odom은 쓰지 않는다.
- IMU prediction과 scan correction을 같이 쓰되, featureless 구간에서 scan correction만 완만하게 만든다.
- 너무 강한 anchor나 너무 강한 rate limit은 피한다.

