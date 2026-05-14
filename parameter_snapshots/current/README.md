# Cartographer Parameter Snapshot

이 폴더는 추가 튜닝 전에 현재 로컬라이제이션 실행 파라미터를 따로 보관하기 위한 스냅샷입니다.

## Files

- `cartographer_parameters_saved.yaml`: 현재 실행에 쓰이는 launch, Lua, `config.yaml`, PoseExtrapolator 기본값을 한곳에 정리한 설정 스냅샷입니다.
- `README.md`: 스냅샷 범위와 원본 파일 위치를 설명합니다.

## Source Files

현재 값은 아래 파일 기준으로 정리했습니다.

- `config.yaml`
- `src/SLAM/cartographer_ros/configuration_files/Damvi_localization_config_wheel.lua`
- `src/SLAM/cartographer_ros/launch/Damvi_carto_pure_wheel_launch.py`

## Important Current Values

- `fusion_extrapolator`: `true`
- `use_sim_time`: `true`
- `tracking_frame`: `imu`
- `published_frame`: `base_link`
- `use_odometry`: `true`
- `TRAJECTORY_BUILDER_2D.use_imu_data`: `true`
- `TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution`: `0.05`
- `POSE_GRAPH.global_sampling_ratio`: `0.005`
- `POSE_GRAPH.constraint_builder.min_score`: `0.65`
- `POSE_GRAPH.constraint_builder.global_localization_min_score`: `0.58`
- `wheelodom_weight`: `0.01`
- `odometry_translation_weight`: `1000.0`
- `odometry_rotation_weight`: `0.0`

## Tuning Order

휠 오돔/IMU 관련 튜닝은 보통 아래 순서로 확인하면 됩니다.

1. `config.yaml`의 `wheelodom_weight`, `imu_weight`, `imu_delta_min`
2. `config.yaml`의 `odometry_translation_weight`, `odometry_rotation_weight`
3. `Damvi_localization_config_wheel.lua`의 scan matching score/search window
4. `Damvi_localization_config_wheel.lua`의 pose graph sampling/optimization 주기

`config.yaml` 값만 바꾸는 경우 C++ 재빌드는 필요 없고 launch 재시작으로 반영됩니다. Lua 또는 C++ 코드를 바꾼 경우에는 해당 패키지를 다시 빌드해야 합니다.
