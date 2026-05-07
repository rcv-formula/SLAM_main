# Wheel Odometry 설정 요약

이 문서는 Cartographer에서 **wheel odometry를 반영하는 경우와 반영하지 않는 경우**를 정리하고, 관련된 **launch / lua / pose_extrapolator.h 파라미터**만 간단히 설명한다.

---

## 1. Wheel Odometry 반영 vs 미반영

### 반영하는 경우
- IMU + LiDAR + wheel odom 사용
- 장점: 이동 방향 추정이 안정적이고 yaw 드리프트가 줄어듦
- 단점: wheel encoder가 필요하고, 바퀴 미끄러짐이 있으면 오차가 들어갈 수 있음
- 권장 상황: 바퀴가 있는 로봇, 실내 평탄한 환경, 위치 추정 안정성이 중요한 경우

### 반영하지 않는 경우
- IMU + LiDAR만 사용
- 장점: 구조가 단순하고 wheel encoder가 없어도 동작함
- 단점: IMU 드리프트와 스캔 매칭 의존도가 커짐
- 권장 상황: wheel odom이 없거나 신뢰하기 어려운 경우

---

## 2. Launch 파일 선택 가이드

### 4가지 조합

| 용도 | Wheel 사용 | Launch 파일 |
|---|---:|---|
| 실시간 SLAM | O | Damvi_carto_wheel_launch.py |
| 실시간 SLAM | X | Damvi_carto_launch.py |
| Localization | O | Damvi_carto_pure_wheel_launch.py |
| Localization | X | Damvi_carto_pure_launch.py |

### 실행 의미
- **SLAM**: 새 맵을 만들면서 위치도 추정
- **Localization**: 이미 만든 pbstream 맵에 맞춰 현재 위치만 추정
- **wheel 버전**: `use_odometry = true`
- **non-wheel 버전**: `use_odometry = false`

---

## 3. LUA 설정 파일 설명

보통 아래 4개 파일을 사용한다.

| 파일 | 의미 |
|---|---|
| Damvi_carto_config_wheel.lua | 실시간 SLAM + wheel odom |
| Damvi_carto_config.lua | 실시간 SLAM + no wheel |
| Damvi_localization_config_wheel.lua | Localization + wheel odom |
| Damvi_localization_config.lua | Localization + no wheel |

### 공통으로 확인할 것
- `use_odometry`
  - `true`면 wheel odom 반영
  - `false`면 wheel odom 미반영
- `use_imu_data`
  - IMU를 사용할지 여부
- `tracking_frame`
  - 보통 `imu`를 사용
- `imu_gravity_time_constant`
  - IMU 자세 보정 속도에 영향

---

## 4. `pose_extrapolator.h`에서 추가한 파라미터 설명

현재 선택된 부분은 Cartographer 기본 파라미터가 아니라, wheel odom과 IMU 속도 보정을 위해 추가한 **튜닝용 변수**로 보면 된다.

```cpp
double imu_weight = 0.2;
double imu_delta_min = 0.3;
double wheelodom_weight = 0.01;
Eigen::Vector3d translation_imu_wheel(
    const Eigen::Vector3d* linear_velocity_scan,
    const Eigen::Vector3d* linear_velocity_odom);
```

### `imu_weight = 0.2`
- IMU 기반 속도 변화량을 얼마나 반영할지 정하는 가중치
- 값이 커질수록 IMU 영향을 크게 받음
- 너무 크면 IMU 노이즈가 속도 추정에 많이 들어감

### `imu_delta_min = 0.3`
- IMU로 계산된 속도 보정값이 너무 작을 때 무시하기 위한 임계값
- 값이 작으면 작은 변화도 반영
- 값이 크면 미세한 보정은 제거됨

### `wheelodom_weight = 0.01`
- wheel odom을 최종 속도 보정에 얼마나 섞을지 정하는 값
- 매우 작게 두면 wheel odom 영향이 약함
- 값이 커질수록 wheel odom이 더 강하게 반영됨

### `translation_imu_wheel(...)`
- scan 기반 속도와 odom 기반 속도를 입력받아 최종 translation 값을 계산하는 함수
- 역할은 대략 아래와 같다.
  - scan velocity를 기본 추정값으로 사용
  - IMU 보정량을 추가
  - wheel odom 보정을 추가
  - 최종 translation velocity를 반환

---

---

## 5. 한 줄 정리

- wheel odom이 신뢰 가능하면 `*_wheel.lua` + `Damvi_carto_wheel_launch.py` 계열을 사용
- wheel odom이 없거나 불안정하면 `*.lua` + `Damvi_carto_launch.py` 계열을 사용
- `pose_extrapolator.h`의 `imu_weight`, `imu_delta_min`, `wheelodom_weight`는 속도 보정 강도를 조절하는 튜닝 값이다

