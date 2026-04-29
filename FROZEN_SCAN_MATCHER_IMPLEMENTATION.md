# Frozen Scan Matcher Implementation Guide

## 1. 목적

이 문서는 이 저장소에 추가된 2D `frozen submap scan matcher` 구현을 코드 기준으로 자세히 설명하기 위한 문서입니다.

설계 의도는 다음과 같습니다.

- pure-localization 중 기존 local scan matching 결과가 긴 복도 등에서 순간적으로 미끄러질 수 있는 문제를 완화한다.
- 이미 로드된 `frozen` trajectory의 finished submap을 활용해, 현재 스캔을 한 번 더 정합한다.
- 이 재정합 결과를 조건부로만 반영해서 급격한 튐을 줄인다.
- 내부 local SLAM 파이프라인 전체에 반영할 수도 있고, publish 결과에만 반영할 수도 있게 한다.
- 비교 실험을 위해 raw 결과와 filtered 결과를 동시에 관찰할 수 있는 test mode를 제공한다.

이 구현은 `apt` 바이너리 Cartographer를 감싼 외부 후처리가 아니라, Cartographer 소스 트리 내부의 local trajectory builder와 ROS publish 경로에 직접 통합되어 있습니다.

---

## 2. 한 줄 요약

현재 구현은 `scan-to-scan`이 아니라 `current scan -> frozen finished submap grid` 정합입니다.

동작 순서는 크게 아래와 같습니다.

1. 기존 local matcher가 active submap 기준 local pose를 계산한다.
2. 그 local pose를 `map` frame으로 올린다.
3. `map` frame 근처의 frozen finished submap 후보를 고른다.
4. 각 후보의 frozen-local frame에서 RTC와 Ceres를 다시 수행한다.
5. score, margin, variance, correction guardrail을 통과하면 corrected pose를 채택한다.
6. `FULL_PIPELINE`, `PUBLISH_ONLY`, 또는 `test_mode_publish_filtered_odom`에 따라 반영 범위를 결정한다.

---

## 3. 구현 범위

현재 구현은 다음 범위에 집중합니다.

- 2D Cartographer local trajectory builder
- frozen trajectory finished submap 기반 재정합
- RTC + Ceres 조합
- tuning log
- ROS topic publish 경로
- raw vs filtered 비교용 test mode

### 3.1 빠른 코드 변경 맵

이 문서에서 가장 먼저 봐야 하는 섹션입니다. 실행 흐름 순서대로, 각 파일이 원래 무슨 역할이었고 지금은 어디가 어떻게 바뀌었는지만 짧게 정리합니다.

참고:

- 아래 라인 번호는 현재 브랜치 기준입니다.
- 이후 추가 수정으로 숫자가 조금 바뀔 수 있어도, 함수 이름과 코드 블록 역할은 그대로 추적 가능합니다.

#### 1. Local SLAM 결과 구조

파일: [`local_trajectory_builder_2d.h`](/home/shin/Desktop/gpts/SLAM_main/src/cartographer/cartographer/mapping/internal/2d/local_trajectory_builder_2d.h)

기존 역할:
local SLAM 결과는 사실상 `local_pose` 하나만 밖으로 전달했다.

추가 위치:
[`local_trajectory_builder_2d.h#L54`](/home/shin/Desktop/gpts/SLAM_main/src/cartographer/cartographer/mapping/internal/2d/local_trajectory_builder_2d.h#L54)

현재 역할:
`MatchingResult`가 `local_pose`와 `published_local_pose`를 함께 보관한다.

#### 2. Local SLAM 파이프라인 본체

파일: [`local_trajectory_builder_2d.cc`](/home/shin/Desktop/gpts/SLAM_main/src/cartographer/cartographer/mapping/internal/2d/local_trajectory_builder_2d.cc)

기존 역할:
`ScanMatch()` 결과가 바로 extrapolator, submap insertion, callback으로 흘렀다.

추가 위치:
[`#L476`](/home/shin/Desktop/gpts/SLAM_main/src/cartographer/cartographer/mapping/internal/2d/local_trajectory_builder_2d.cc#L476),
[`#L483`](/home/shin/Desktop/gpts/SLAM_main/src/cartographer/cartographer/mapping/internal/2d/local_trajectory_builder_2d.cc#L483),
[`#L495`](/home/shin/Desktop/gpts/SLAM_main/src/cartographer/cartographer/mapping/internal/2d/local_trajectory_builder_2d.cc#L495),
[`#L504`](/home/shin/Desktop/gpts/SLAM_main/src/cartographer/cartographer/mapping/internal/2d/local_trajectory_builder_2d.cc#L504),
[`#L515`](/home/shin/Desktop/gpts/SLAM_main/src/cartographer/cartographer/mapping/internal/2d/local_trajectory_builder_2d.cc#L515),
[`#L550`](/home/shin/Desktop/gpts/SLAM_main/src/cartographer/cartographer/mapping/internal/2d/local_trajectory_builder_2d.cc#L550)

현재 역할:
active-submap local match 뒤에 frozen matcher를 동기식으로 넣고, raw pose와 publish pose를 분리한다.

#### 3. Frozen matcher 본체

파일: [`frozen_submap_scan_matcher_2d.cc`](/home/shin/Desktop/gpts/SLAM_main/src/cartographer/cartographer/mapping/internal/2d/scan_matching/frozen_submap_scan_matcher_2d.cc)

기존 역할:
이 second-stage matcher 자체가 없었다.

추가 위치:
[`#L125`](/home/shin/Desktop/gpts/SLAM_main/src/cartographer/cartographer/mapping/internal/2d/scan_matching/frozen_submap_scan_matcher_2d.cc#L125),
[`#L253`](/home/shin/Desktop/gpts/SLAM_main/src/cartographer/cartographer/mapping/internal/2d/scan_matching/frozen_submap_scan_matcher_2d.cc#L253),
[`#L281`](/home/shin/Desktop/gpts/SLAM_main/src/cartographer/cartographer/mapping/internal/2d/scan_matching/frozen_submap_scan_matcher_2d.cc#L281),
[`#L317`](/home/shin/Desktop/gpts/SLAM_main/src/cartographer/cartographer/mapping/internal/2d/scan_matching/frozen_submap_scan_matcher_2d.cc#L317),
[`#L383`](/home/shin/Desktop/gpts/SLAM_main/src/cartographer/cartographer/mapping/internal/2d/scan_matching/frozen_submap_scan_matcher_2d.cc#L383),
[`#L412`](/home/shin/Desktop/gpts/SLAM_main/src/cartographer/cartographer/mapping/internal/2d/scan_matching/frozen_submap_scan_matcher_2d.cc#L412)

현재 역할:
frozen 후보 선택, RTC/Ceres 재정합, guardrail 검사, 최종 `filtered_tracking_to_local` 계산을 담당한다.

#### 4. Frozen submap 공급

파일: [`map_builder.cc`](/home/shin/Desktop/gpts/SLAM_main/src/cartographer/cartographer/mapping/map_builder.cc)

기존 역할:
`LocalTrajectoryBuilder2D`로 frozen submap snapshot을 넘기는 경로가 없었다.

추가 위치:
[`map_builder.cc#L133`](/home/shin/Desktop/gpts/SLAM_main/src/cartographer/cartographer/mapping/map_builder.cc#L133)

현재 역할:
pose graph에서 frozen trajectory의 finished submap만 모아 provider callback으로 넘긴다.

#### 5. ROS bridge

파일: [`map_builder_bridge.h`](/home/shin/Desktop/gpts/SLAM_main/src/SLAM/cartographer_ros/include/cartographer_ros/map_builder_bridge.h), [`map_builder_bridge.cpp`](/home/shin/Desktop/gpts/SLAM_main/src/SLAM/cartographer_ros/src/map_builder_bridge.cpp)

기존 역할:
ROS bridge는 local pose 하나만 저장하고 전달했다.

추가 위치:
[`map_builder_bridge.h#L56`](/home/shin/Desktop/gpts/SLAM_main/src/SLAM/cartographer_ros/include/cartographer_ros/map_builder_bridge.h#L56),
[`map_builder_bridge.cpp#L126`](/home/shin/Desktop/gpts/SLAM_main/src/SLAM/cartographer_ros/src/map_builder_bridge.cpp#L126),
[`map_builder_bridge.cpp#L537`](/home/shin/Desktop/gpts/SLAM_main/src/SLAM/cartographer_ros/src/map_builder_bridge.cpp#L537)

현재 역할:
raw/pipeline pose와 published pose를 동시에 ROS 쪽으로 전파한다.

#### 6. ROS publish 분기

파일: [`node_constants.h`](/home/shin/Desktop/gpts/SLAM_main/src/SLAM/cartographer_ros/include/cartographer_ros/node_constants.h), [`node.cpp`](/home/shin/Desktop/gpts/SLAM_main/src/SLAM/cartographer_ros/src/node.cpp)

기존 역할:
tracked pose publish 경로와 extrapolator가 하나뿐이었다.

추가 위치:
[`node_constants.h#L37`](/home/shin/Desktop/gpts/SLAM_main/src/SLAM/cartographer_ros/include/cartographer_ros/node_constants.h#L37),
[`node.cpp#L120`](/home/shin/Desktop/gpts/SLAM_main/src/SLAM/cartographer_ros/src/node.cpp#L120),
[`node.cpp#L224`](/home/shin/Desktop/gpts/SLAM_main/src/SLAM/cartographer_ros/src/node.cpp#L224),
[`node.cpp#L264`](/home/shin/Desktop/gpts/SLAM_main/src/SLAM/cartographer_ros/src/node.cpp#L264),
[`node.cpp#L324`](/home/shin/Desktop/gpts/SLAM_main/src/SLAM/cartographer_ros/src/node.cpp#L324),
[`node.cpp#L339`](/home/shin/Desktop/gpts/SLAM_main/src/SLAM/cartographer_ros/src/node.cpp#L339),
[`node.cpp#L386`](/home/shin/Desktop/gpts/SLAM_main/src/SLAM/cartographer_ros/src/node.cpp#L386)

현재 역할:
test mode에서 raw/published 두 extrapolator를 유지하고, `/tracked_pose`와 `/filtered_tracked_pose`를 분리 publish한다.

#### 7. `/odom` 비교 경로

파일: [`trajectory_to_odom.cpp`](/home/shin/Desktop/gpts/SLAM_main/src/SLAM/cartographer_ros/src/trajectory_to_odom.cpp)

기존 역할:
`/tracked_pose` 기반으로 `/odom` 하나만 만들었다.

추가 위치:
[`trajectory_to_odom.cpp#L29`](/home/shin/Desktop/gpts/SLAM_main/src/SLAM/cartographer_ros/src/trajectory_to_odom.cpp#L29),
[`trajectory_to_odom.cpp#L36`](/home/shin/Desktop/gpts/SLAM_main/src/SLAM/cartographer_ros/src/trajectory_to_odom.cpp#L36),
[`trajectory_to_odom.cpp#L60`](/home/shin/Desktop/gpts/SLAM_main/src/SLAM/cartographer_ros/src/trajectory_to_odom.cpp#L60)

현재 역할:
`/tracked_pose`와 `/filtered_tracked_pose`를 각각 `/odom`, `/filtered_odom`으로 바꿔 비교 가능하게 한다.

#### 8. 옵션과 실제 Damvi 설정

파일: [`frozen_submap_scan_matcher_options_2d.proto`](/home/shin/Desktop/gpts/SLAM_main/src/cartographer/cartographer/mapping/proto/scan_matching/frozen_submap_scan_matcher_options_2d.proto), [`Damvi_localization_config.lua`](/home/shin/Desktop/gpts/SLAM_main/src/SLAM/cartographer_ros/configuration_files/Damvi_localization_config.lua)

기존 역할:
frozen matcher 관련 schema와 Damvi 설정 블록이 없었다.

추가 위치:
[`frozen_submap_scan_matcher_options_2d.proto#L33`](/home/shin/Desktop/gpts/SLAM_main/src/cartographer/cartographer/mapping/proto/scan_matching/frozen_submap_scan_matcher_options_2d.proto#L33),
[`Damvi_localization_config.lua#L89`](/home/shin/Desktop/gpts/SLAM_main/src/SLAM/cartographer_ros/configuration_files/Damvi_localization_config.lua#L89)

현재 역할:
`apply_mode`, guardrail, tuning log, test mode를 포함한 실제 운용 파라미터를 정의한다.

#### 9. frozen matcher 전용 파일

파일:
[`frozen_submap_scan_matcher_2d.h`](/home/shin/Desktop/gpts/SLAM_main/src/cartographer/cartographer/mapping/internal/2d/scan_matching/frozen_submap_scan_matcher_2d.h),
[`frozen_submap_scan_matcher_2d.cc`](/home/shin/Desktop/gpts/SLAM_main/src/cartographer/cartographer/mapping/internal/2d/scan_matching/frozen_submap_scan_matcher_2d.cc)

기존 역할:
frozen submap과 현재 스캔을 직접 비교하는 local matcher 보정 단계가 없었다.

추가 위치:
2D scan matching 폴더

현재 역할:
현재 스캔을 pbstream에서 로드된 frozen finished submap grid와 RTC/Ceres로 동기식 재정합한다.

---

## 4. 좌표계와 용어

이 구현을 이해할 때 가장 중요한 점은 `raw local pose`, `filtered local pose`, `map`, `frozen local` frame이 동시에 등장한다는 점입니다.

### 4.1 핵심 frame

- `tracking`
  센서 추적 기준 frame입니다. Damvi 설정에서는 사실상 `imu`가 tracking frame입니다.
- `current local`
  현재 active trajectory의 local map frame입니다.
- `map`
  pose graph가 제공하는 global frame입니다.
- `frozen local`
  pbstream에서 로드된 frozen trajectory 각각의 local frame입니다.

### 4.2 용어

- `raw_tracking_to_local`
  기존 local scan matcher가 active submap 기준으로 계산한 결과입니다.
- `filtered_tracking_to_local`
  frozen finished submap과 재정합 후 채택된 결과입니다.
- `local_to_map`
  현재 trajectory local frame을 map frame으로 올리는 transform입니다.
- `frozen_local_to_map`
  frozen trajectory local frame을 map frame으로 올리는 transform입니다.

### 4.3 좌표 변환 식

이 구현의 핵심 변환은 아래 네 줄입니다.

```text
raw_tracking_to_map
  = current_local_to_map * raw_tracking_to_local

raw_tracking_to_frozen_local
  = inverse(frozen_local_to_map) * raw_tracking_to_map

corrected_tracking_to_map
  = frozen_local_to_map * corrected_tracking_to_frozen_local

filtered_tracking_to_local
  = inverse(current_local_to_map) * corrected_tracking_to_map
```

즉, 현재 trajectory local frame에서 나온 raw pose를 일단 `map`으로 올리고, 다시 frozen trajectory local frame에 내려서 정합한 뒤, 결과를 다시 `map`으로 올리고 현재 trajectory local frame으로 되돌립니다.

이 구조 덕분에:

- frozen 쪽과 현재 active trajectory 쪽의 local frame 차이를 직접 섞지 않아도 되고
- 결과를 다시 local trajectory builder가 기대하는 `tracking_to_local` 형태로 넣을 수 있습니다.

---

## 5. 상위 아키텍처

상위 흐름은 아래처럼 이해하면 됩니다.

```text
LaserScan / IMU
  -> LocalTrajectoryBuilder2D
  -> 기존 active-submap local matching
  -> frozen finished submap re-matching
  -> raw/published pose 분기
  -> MapBuilderBridge
  -> Node publish
  -> /tracked_pose, /filtered_tracked_pose
  -> trajectory_to_odom
  -> /odom, /filtered_odom
```

---

## 6. frozen submap snapshot 공급

frozen matcher는 스스로 pose graph를 직접 소유하지 않습니다. 필요한 frozen submap snapshot은 `MapBuilder`에서 provider callback 형태로 넣어줍니다.

구현 위치:

- [`src/cartographer/cartographer/mapping/map_builder.cc`](/home/shin/Desktop/gpts/SLAM_main/src/cartographer/cartographer/mapping/map_builder.cc)

이 provider는 아래 정보를 묶어서 `FrozenSubmapQueryResult2D`로 반환합니다.

- 현재 trajectory의 `local_to_map`
- pose graph에서 `FROZEN` 상태인 trajectory만 필터링한 `frozen_local_to_map`
- 각 frozen trajectory의 finished submap만 선택
- 각 submap의 `submap_to_map`

결과적으로 matcher는 다음 정보를 입력으로 받습니다.

- 현재 local pose를 map frame으로 올릴 수 있는 기준
- 주변에 어떤 frozen submap들이 있는지
- 각 frozen submap의 global 위치

중요한 제한은 다음과 같습니다.

- unfinished submap은 후보에서 제외됩니다
- non-frozen trajectory는 후보에서 제외됩니다

즉, runtime active trajectory끼리 비교하는 것이 아니라, 이미 저장된 frozen map과만 비교합니다.

---

## 7. frozen matcher 핵심 데이터 구조

구현 위치:

- [`src/cartographer/cartographer/mapping/internal/2d/scan_matching/frozen_submap_scan_matcher_2d.h`](/home/shin/Desktop/gpts/SLAM_main/src/cartographer/cartographer/mapping/internal/2d/scan_matching/frozen_submap_scan_matcher_2d.h)

### 7.1 `FrozenSubmapSnapshot2D`

후보 submap 하나의 snapshot입니다.

- `id`
- `submap`
- `frozen_local_to_map`
- `submap_to_map`

`submap_to_map`은 주로 후보 거리 계산에 사용됩니다.

### 7.2 `FrozenSubmapQueryResult2D`

한 번의 match 호출에 필요한 전체 입력입니다.

- `local_to_map`
- `submaps`

### 7.3 `FrozenSubmapMatchResult2D`

최종 결과 및 tuning 디버그 정보입니다.

주요 필드:

- `attempted`
- `accepted`
- `status`
- `filtered_tracking_to_local`
- `filtered_tracking_to_map`
- `num_candidates_in_search_radius`
- `num_candidates_evaluated`
- `best_score`
- `second_best_score`
- `top_k_score_variance`
- `translation_correction`
- `rotation_correction`
- `matched_submap_id`
- `candidate_debug_info`

즉, 이 구조 하나에 match 성공 여부, reject 이유, 최종 보정량, 상위 후보 정보가 모두 들어 있습니다.

---

## 8. frozen matcher 알고리즘 상세

구현 위치:

- [`src/cartographer/cartographer/mapping/internal/2d/scan_matching/frozen_submap_scan_matcher_2d.cc`](/home/shin/Desktop/gpts/SLAM_main/src/cartographer/cartographer/mapping/internal/2d/scan_matching/frozen_submap_scan_matcher_2d.cc)

### 8.1 입력

matcher는 다음 3개를 입력으로 받습니다.

- `query`
- `raw_tracking_to_local`
- `filtered_gravity_aligned_point_cloud`

즉, 이미 gravity-aligned + voxel filtered된 point cloud를 그대로 재사용합니다.

### 8.2 early return 조건

아래 경우에는 시도하지 않고 raw 결과를 그대로 유지합니다.

- matcher `enabled == false`
- frozen query에 submap이 없음
- point cloud가 비어 있음

### 8.3 raw pose를 map frame으로 올림

초기값은 아래처럼 잡힙니다.

```text
result.filtered_tracking_to_local = raw_tracking_to_local
result.filtered_tracking_to_map = query.local_to_map * raw_tracking_to_local
```

이 값은 “re-match가 실패했을 때 유지할 기본값”입니다.

### 8.4 후보 submap 수집

모든 frozen finished submap을 도는 것이 아니라, 현재 raw pose의 map 위치를 기준으로 반경 안 후보만 남깁니다.

거리 계산 기준:

- `distance_to_submap = norm(submap_to_map.translation - raw_tracking_to_map.translation)`

필터 순서:

1. `submap` 존재 여부
2. `grid` 존재 여부
3. `search_radius` 이내 여부
4. 거리 오름차순 정렬
5. `max_submaps_to_match` 개수 제한

`use_realtime_correlative_scan_matching == false`면 RTC score 랭킹 대신 사실상 하나의 후보만 보도록 줄입니다.

### 8.5 각 후보에 대해 frozen-local frame으로 변환

각 후보마다 아래 pose를 만듭니다.

```text
raw_tracking_to_frozen_local
  = inverse(snapshot.frozen_local_to_map) * raw_tracking_to_map
```

즉, 현재 raw pose를 그 frozen trajectory local frame 기준 pose로 바꿉니다.

### 8.6 RTC 수행

`use_realtime_correlative_scan_matching`이 켜져 있으면 기본 RTC를 수행합니다.

RTC는 frozen submap grid에서 후보 pose score를 계산하고, 후보가 여러 개일 때 best score, second-best score, score variance를 acceptance guardrail에 사용합니다.

### 8.7 Ceres refinement 수행

`use_ceres_scan_matching`이 켜져 있으면 기본 Ceres matcher로 refinement를 수행합니다.

여기서도 frozen-local frame에서 refinement가 이루어집니다.

### 8.8 후보 점수 정렬

RTC를 쓰는 경우 score 기준으로 재정렬합니다.

정렬 기준:

1. score 내림차순
2. 동점이면 distance 오름차순

이후:

- `best_score`
- `second_best_score`
- `top_k_score_variance`

를 계산합니다.

### 8.9 correction 계산

best candidate를 기준으로:

- translation correction magnitude
- rotation correction magnitude

를 raw map pose 대비 계산합니다.

### 8.10 acceptance / rejection guardrail

reject 순서는 아래와 같습니다.

1. `no candidates`
2. `low score`
3. `low margin`
4. `low variance`
5. `translation correction too large`
6. `rotation correction too large`

모두 통과하면:

- `accepted = true`
- `filtered_tracking_to_map = best_candidate.tracking_to_map`
- `filtered_tracking_to_local = inverse(local_to_map) * filtered_tracking_to_map`

가 됩니다.

### 8.11 중요한 특성

이 matcher는 score가 높다고 무조건 채택하지 않습니다.

특히 긴 복도 같은 경우:

- best와 second-best가 너무 비슷하면 reject
- top-k variance가 너무 작아도 reject

하도록 해서 ambiguous alignment를 최대한 걸러내도록 설계되어 있습니다.

---

## 9. local trajectory builder와의 통합

구현 위치:

- [`src/cartographer/cartographer/mapping/internal/2d/local_trajectory_builder_2d.cc`](/home/shin/Desktop/gpts/SLAM_main/src/cartographer/cartographer/mapping/internal/2d/local_trajectory_builder_2d.cc)
- [`src/cartographer/cartographer/mapping/internal/2d/local_trajectory_builder_2d.h`](/home/shin/Desktop/gpts/SLAM_main/src/cartographer/cartographer/mapping/internal/2d/local_trajectory_builder_2d.h)

### 9.1 기존 local match 이후에 삽입

frozen matcher는 기존 `ScanMatch()`가 raw local pose를 만든 직후에 실행됩니다.

즉 순서는:

1. pose prediction
2. active submap local match
3. frozen submap re-match
4. extrapolator update
5. range_data_in_local 생성
6. submap insertion

### 9.2 raw vs published pose 분리

현재 `MatchingResult`는 두 pose를 같이 들고 갑니다.

- `local_pose`
  pipeline 내부에서 실제로 사용한 pose
- `published_local_pose`
  ROS publish에 사용할 pose

### 9.3 `FULL_PIPELINE`

frozen match가 accept되면:

- `pipeline_pose_estimate_2d = filtered_tracking_to_local`
- `published_pose_estimate_2d = filtered_tracking_to_local`

즉 둘 다 filtered 결과를 씁니다.

결과적으로:

- extrapolator
- active submap insertion
- node insertion
- publish

전부 filtered 기준으로 움직입니다.

### 9.4 `PUBLISH_ONLY`

frozen match가 accept되면:

- `pipeline_pose_estimate_2d`는 raw 유지
- `published_pose_estimate_2d`만 filtered로 바뀜

즉 내부 local SLAM과 submap insert는 raw 기준이고, ROS publish만 filtered 기준입니다.

### 9.5 `test_mode_publish_filtered_odom`

이 플래그가 `true`이면 내부적으로 강제로 `PUBLISH_ONLY` 성격으로 동작합니다.

즉 accept되어도:

- pipeline pose는 raw 유지
- published pose만 filtered 유지

이렇게 해서 raw와 filtered를 동시에 비교 가능한 상태를 만들었습니다.

---

## 10. ROS bridge 전파 구조

구현 위치:

- [`src/SLAM/cartographer_ros/include/cartographer_ros/map_builder_bridge.h`](/home/shin/Desktop/gpts/SLAM_main/src/SLAM/cartographer_ros/include/cartographer_ros/map_builder_bridge.h)
- [`src/SLAM/cartographer_ros/src/map_builder_bridge.cpp`](/home/shin/Desktop/gpts/SLAM_main/src/SLAM/cartographer_ros/src/map_builder_bridge.cpp)

`MapBuilderBridge::LocalSlamData`는 현재 아래 정보를 보관합니다.

- `time`
- `local_pose`
- `published_local_pose`
- `range_data_in_local`

즉 raw/pipeline pose와 publish용 pose가 둘 다 ROS 쪽으로 전달됩니다.

이 구조 덕분에 ROS layer에서:

- TF는 raw를 쓸지 filtered를 쓸지
- `/tracked_pose`는 raw를 쓸지 filtered를 쓸지
- 비교용 topic을 추가로 낼지

를 결정할 수 있게 되었습니다.

---

## 11. ROS publish 경로

구현 위치:

- [`src/SLAM/cartographer_ros/src/node.cpp`](/home/shin/Desktop/gpts/SLAM_main/src/SLAM/cartographer_ros/src/node.cpp)

### 11.1 일반 모드

일반 모드에서는 `published_local_pose`를 기준으로 extrapolator를 갱신하고 publish합니다.

즉 기존 frozen matcher 설계 의도대로:

- `PUBLISH_ONLY`면 publish 쪽만 filtered
- `FULL_PIPELINE`이면 내부/외부 모두 filtered

입니다.

### 11.2 test mode

test mode에서는 `Node`가 extrapolator를 두 개 유지합니다.

- `extrapolators_`
  published/filtered 경로
- `raw_extrapolators_`
  raw 경로

그리고 publish 시 아래처럼 나눕니다.

- TF와 `/tracked_pose`는 raw 기준
- `/filtered_tracked_pose`는 filtered 기준

즉 test mode에서는 비교용 publish 경로가 동시에 살아 있습니다.

### 11.3 topic

기존 topic:

- `/tracked_pose`

추가 topic:

- `/filtered_tracked_pose`

관련 상수는:

- [`src/SLAM/cartographer_ros/include/cartographer_ros/node_constants.h`](/home/shin/Desktop/gpts/SLAM_main/src/SLAM/cartographer_ros/include/cartographer_ros/node_constants.h)

에 있습니다.

---

## 12. `/odom` 과 `/filtered_odom`

구현 위치:

- [`src/SLAM/cartographer_ros/src/trajectory_to_odom.cpp`](/home/shin/Desktop/gpts/SLAM_main/src/SLAM/cartographer_ros/src/trajectory_to_odom.cpp)

### 12.1 변경 전

원래는 `/tracked_pose`를 트리거로 쓰고, TF에서 `map -> base_link`를 읽어 `/odom`을 만들고 있었습니다.

이 방식은 raw와 filtered를 동시에 비교하기에 적합하지 않았습니다.

이유:

- publish source가 하나뿐이었고
- TF 기준 publish라서 raw/filtered를 동시에 분리하기 어려웠기 때문입니다.

### 12.2 변경 후

현재는 아래 두 PoseStamped를 동시에 구독합니다.

- `/tracked_pose`
- `/filtered_tracked_pose`

그리고 둘을 각각:

- `/odom`
- `/filtered_odom`

으로 바꿔 publish합니다.

### 12.3 published frame 복원

`tracked_pose`는 tracking frame pose 기반이므로, 최종 odom은 published frame 기준으로 복원해야 합니다.

그래서 `trajectory_to_odom`는 TF에서:

- `tracking_frame -> published_frame`

transform을 읽고, PoseStamped와 곱해서 최종 published frame pose를 만듭니다.

Damvi launch에서는 기본적으로:

- `tracking_frame = imu`
- `published_frame = base_link`

를 사용합니다.

### 12.4 test mode 의미

test mode에서:

- `/odom` = raw local matcher 기반 결과
- `/filtered_odom` = frozen matcher 적용 결과

입니다.

즉 “코드 변경 전의 odom 느낌”과 “새 filtered 결과”를 side-by-side로 비교할 수 있습니다.

---

## 13. pbstream frozen 후보 제한

현재 frozen matcher의 후보 submap은 로드된 pbstream에서 온 frozen trajectory로 제한합니다.

핵심 의도:

- 실시간으로 새로 만들어진 active 또는 finished submap이 후보에 섞이지 않게 한다.
- localization 기준 맵인 pbstream submap만 기준면으로 사용한다.
- 후보 공급 단계에서 trajectory id를 필터링해 matcher 내부는 순수하게 scan matching 판단만 담당하게 한다.

---

## 14. tuning log

현재 frozen matcher는 전용 tuning log를 제공합니다.

주요 파라미터:

- `tuning_log_enabled`
- `tuning_log_log_rejections`
- `tuning_log_log_acceptances`
- `tuning_log_detail_every_n_scans`
- `tuning_log_summary_every_n_scans`
- `tuning_log_top_candidates`

로그에 포함되는 대표 정보:

- `status`
- `apply_mode`
- candidate 수
- best score
- correction
- matched submap
- top candidate 목록
- 구간 summary acceptance rate

이 로그는 tuning 시 score threshold와 correction threshold를 조정하는 데 사용됩니다.

---

## 15. 파라미터 상세

기준 파일:

- [`src/SLAM/cartographer_ros/configuration_files/Damvi_localization_config.lua`](/home/shin/Desktop/gpts/SLAM_main/src/SLAM/cartographer_ros/configuration_files/Damvi_localization_config.lua)

### 15.1 기본 on/off 및 반영 모드

- `enabled`
  기능 전체 on/off
- `apply_mode`
  `FULL_PIPELINE` 또는 `PUBLISH_ONLY`
- `test_mode_publish_filtered_odom`
  비교 모드용 플래그

### 15.2 후보 수집

- `search_radius`
  map 기준 후보 submap 반경
- `max_submaps_to_match`
  평가할 후보 최대 개수

### 15.3 정합 엔진 on/off

- `use_realtime_correlative_scan_matching`
- `use_ceres_scan_matching`

### 15.4 acceptance guardrail

- `min_realtime_correlative_score`
- `min_score_margin`
- `score_variance_top_k`
- `min_score_variance`
- `max_translation_correction`
- `max_rotation_correction`

### 15.5 RTC 세부 파라미터

- `real_time_correlative_scan_matcher.linear_search_window`
- `real_time_correlative_scan_matcher.angular_search_window`
- `real_time_correlative_scan_matcher.translation_delta_cost_weight`
- `real_time_correlative_scan_matcher.rotation_delta_cost_weight`

### 15.6 Ceres 세부 파라미터

- `ceres_scan_matcher.occupied_space_weight`
- `ceres_scan_matcher.translation_weight`
- `ceres_scan_matcher.rotation_weight`
- `ceres_scan_matcher.ceres_solver_options.use_nonmonotonic_steps`
- `ceres_scan_matcher.ceres_solver_options.max_num_iterations`
- `ceres_scan_matcher.ceres_solver_options.num_threads`

---

## 16. 모드별 실제 의미

### 16.1 `enabled = false`

- frozen matcher 자체 미사용
- pure-localization은 기존 local matcher만 사용

### 16.2 `enabled = true`, `apply_mode = FULL_PIPELINE`

- frozen match accept 시 local SLAM 내부까지 filtered 결과를 사용
- active submap insert와 extrapolator도 filtered 기준

### 16.3 `enabled = true`, `apply_mode = PUBLISH_ONLY`

- local SLAM 내부는 raw
- publish path만 filtered

### 16.4 `test_mode_publish_filtered_odom = true`

- 내부적으로 pipeline은 raw 유지
- `/odom`은 raw
- `/filtered_odom`은 filtered
- 비교 실험용

이 플래그는 사실상 “publish-only 비교 실험 모드”라고 보면 됩니다.

---

## 17. launch / config 구성

현재 Damvi pure-localization 관련 launch는 아래처럼 분리되어 있습니다.

- 기본 원본
  - [`src/SLAM/cartographer_ros/launch/Damvi_carto_pure_launch.py`](/home/shin/Desktop/gpts/SLAM_main/src/SLAM/cartographer_ros/launch/Damvi_carto_pure_launch.py)
  - [`src/SLAM/cartographer_ros/configuration_files/Damvi_localization_config.lua`](/home/shin/Desktop/gpts/SLAM_main/src/SLAM/cartographer_ros/configuration_files/Damvi_localization_config.lua)
- test1
  - [`src/SLAM/cartographer_ros/launch/Damvi_carto_pure_test1_launch.py`](/home/shin/Desktop/gpts/SLAM_main/src/SLAM/cartographer_ros/launch/Damvi_carto_pure_test1_launch.py)
  - [`src/SLAM/cartographer_ros/configuration_files/Damvi_localization_config_test1.lua`](/home/shin/Desktop/gpts/SLAM_main/src/SLAM/cartographer_ros/configuration_files/Damvi_localization_config_test1.lua)
- test2
  - [`src/SLAM/cartographer_ros/launch/Damvi_carto_pure_test2_launch.py`](/home/shin/Desktop/gpts/SLAM_main/src/SLAM/cartographer_ros/launch/Damvi_carto_pure_test2_launch.py)
  - [`src/SLAM/cartographer_ros/configuration_files/Damvi_localization_config_test2.lua`](/home/shin/Desktop/gpts/SLAM_main/src/SLAM/cartographer_ros/configuration_files/Damvi_localization_config_test2.lua)
- test3
  - [`src/SLAM/cartographer_ros/launch/Damvi_carto_pure_test3_launch.py`](/home/shin/Desktop/gpts/SLAM_main/src/SLAM/cartographer_ros/launch/Damvi_carto_pure_test3_launch.py)
  - [`src/SLAM/cartographer_ros/configuration_files/Damvi_localization_config_test3.lua`](/home/shin/Desktop/gpts/SLAM_main/src/SLAM/cartographer_ros/configuration_files/Damvi_localization_config_test3.lua)

권장 용도:

- 기본 launch
  frozen matcher off, baseline 확인
- test1
  PUBLISH_ONLY 기반 실전형 튜닝값
- test2
  FULL_PIPELINE 실험
- test3
  raw `/odom` vs filtered `/filtered_odom` 비교 실험

---

## 18. test mode를 왜 따로 만들었는가

`PUBLISH_ONLY`만으로는 “현재 publish 경로가 filtered로 바뀌었는지”는 볼 수 있지만, raw와 filtered를 동시에 비교하기 어렵습니다.

test mode를 따로 만든 이유는 다음과 같습니다.

- 코드 변경 전과 최대한 비슷한 raw 경로를 유지하고 싶다
- frozen matcher 결과가 실제로 얼마나 다른지 topic 레벨에서 보고 싶다
- `/odom`과 `/filtered_odom`을 동시에 띄워서 RViz나 기록 툴에서 비교하고 싶다

즉, test mode는 성능 향상 기능이라기보다 검증과 관찰을 위한 기능입니다.

---

## 19. 현재 구현의 장점

- source build 상태에서 Cartographer core 내부까지 수정 가능
- frozen map과 local scan match를 동기식으로 통합
- ambiguity를 score, margin, variance로 방어
- correction magnitude limit로 갑작스러운 튐 방어
- PUBLISH_ONLY와 FULL_PIPELINE 분리
- raw vs filtered 비교용 test mode 지원
- tuning log로 반복 튜닝 가능

---

## 20. 현재 구현의 한계와 주의점

### 20.1 score 해석

RTC score는 환경, submap 해상도, 탐색 범위에 따라 절대값 감각이 달라질 수 있습니다.

따라서 다른 맵이나 다른 센서 데이터셋으로 옮길 때는 score tail, margin, variance를 다시 확인하는 편이 안전합니다.

### 20.2 long corridor 한계

variance, margin guardrail을 넣었지만, 긴 복도처럼 구조가 정말 반복적이면:

- no-candidate
- low-score
- low-margin

쪽 reject가 많이 나올 수 있습니다.

이는 일부러 “애매하면 차라리 안 쓰는” 보수적 설계에 가깝습니다.

### 20.3 publish와 pipeline 분리 시 개념 혼동

`PUBLISH_ONLY`나 test mode에서는:

- 내부 map building과
- 외부 publish pose

가 서로 다를 수 있습니다.

이는 의도된 동작이지만, debug 시 혼동되기 쉽습니다.

### 20.4 3D 범위 아님

3D local trajectory builder 쪽은 raw == published로만 호환 유지되어 있으며, frozen re-match 기능 자체는 2D 중심입니다.

---

## 21. 확장 포인트

향후 추가 수정 시 주로 손대게 될 위치는 아래입니다.

### 21.1 acceptance 조건 변경

- [`src/cartographer/cartographer/mapping/internal/2d/scan_matching/frozen_submap_scan_matcher_2d.cc`](/home/shin/Desktop/gpts/SLAM_main/src/cartographer/cartographer/mapping/internal/2d/scan_matching/frozen_submap_scan_matcher_2d.cc)

여기서:

- score rule
- margin rule
- variance rule
- correction rule

을 수정할 수 있습니다.

### 21.2 후보 선택 규칙 변경

동일 파일에서:

- `distance_to_submap`
- 후보 정렬 방식
- `max_submaps_to_match`
- best candidate tie-break

를 바꿀 수 있습니다.

### 21.3 matcher score 계산 변경

- [`src/cartographer/cartographer/mapping/internal/2d/scan_matching/frozen_submap_scan_matcher_2d.cc`](/home/shin/Desktop/gpts/SLAM_main/src/cartographer/cartographer/mapping/internal/2d/scan_matching/frozen_submap_scan_matcher_2d.cc)

에서 RTC/Ceres 호출 방식과 후보별 score 평가 방식을 바꿀 수 있습니다.

### 21.4 publish 정책 변경

- [`src/SLAM/cartographer_ros/src/node.cpp`](/home/shin/Desktop/gpts/SLAM_main/src/SLAM/cartographer_ros/src/node.cpp)
- [`src/SLAM/cartographer_ros/src/trajectory_to_odom.cpp`](/home/shin/Desktop/gpts/SLAM_main/src/SLAM/cartographer_ros/src/trajectory_to_odom.cpp)

에서:

- raw/filtered topic 정책
- TF publish 정책
- odom frame semantics

을 바꿀 수 있습니다.

---

## 22. 빠른 추적 포인트

코드 리딩을 빨리 시작하려면 아래 순서가 가장 좋습니다.

1. [`local_trajectory_builder_2d.cc#L476`](/home/shin/Desktop/gpts/SLAM_main/src/cartographer/cartographer/mapping/internal/2d/local_trajectory_builder_2d.cc#L476)
   active-submap local match 직후 frozen matcher가 실제로 삽입되는 시작점입니다.
2. [`local_trajectory_builder_2d.cc#L504`](/home/shin/Desktop/gpts/SLAM_main/src/cartographer/cartographer/mapping/internal/2d/local_trajectory_builder_2d.cc#L504)
   accept 시 raw pose와 published pose가 어떻게 갈라지는지 보는 핵심 분기입니다.
3. [`frozen_submap_scan_matcher_2d.cc#L253`](/home/shin/Desktop/gpts/SLAM_main/src/cartographer/cartographer/mapping/internal/2d/scan_matching/frozen_submap_scan_matcher_2d.cc#L253)
   frozen matcher 본체의 진입점입니다.
4. [`frozen_submap_scan_matcher_2d.cc#L281`](/home/shin/Desktop/gpts/SLAM_main/src/cartographer/cartographer/mapping/internal/2d/scan_matching/frozen_submap_scan_matcher_2d.cc#L281)
   search radius 기반 후보 수집 구간입니다.
5. [`frozen_submap_scan_matcher_2d.cc#L317`](/home/shin/Desktop/gpts/SLAM_main/src/cartographer/cartographer/mapping/internal/2d/scan_matching/frozen_submap_scan_matcher_2d.cc#L317)
   RTC/Ceres를 실제로 돌리는 구간입니다.
6. [`frozen_submap_scan_matcher_2d.cc#L383`](/home/shin/Desktop/gpts/SLAM_main/src/cartographer/cartographer/mapping/internal/2d/scan_matching/frozen_submap_scan_matcher_2d.cc#L383)
   low score, low margin, low variance, correction limit guardrail이 모여 있는 구간입니다.
7. [`map_builder.cc#L133`](/home/shin/Desktop/gpts/SLAM_main/src/cartographer/cartographer/mapping/map_builder.cc#L133)
   frozen submap snapshot 공급 callback이 생성되는 지점입니다.
8. [`map_builder_bridge.cpp#L126`](/home/shin/Desktop/gpts/SLAM_main/src/SLAM/cartographer_ros/src/map_builder_bridge.cpp#L126)
   local SLAM callback이 raw pose와 published pose 둘 다 받도록 바뀐 지점입니다.
9. [`node.cpp#L264`](/home/shin/Desktop/gpts/SLAM_main/src/SLAM/cartographer_ros/src/node.cpp#L264)
   ROS publish 단계에서 raw/published pose가 어떻게 topic과 TF로 반영되는지 보는 핵심 함수입니다.
10. [`trajectory_to_odom.cpp#L29`](/home/shin/Desktop/gpts/SLAM_main/src/SLAM/cartographer_ros/src/trajectory_to_odom.cpp#L29)
   `/tracked_pose`와 `/filtered_tracked_pose`를 받아 `/odom`, `/filtered_odom`을 만드는 비교용 진입점입니다.

---

## 23. 결론

현재 frozen scan matcher 구현은 단순한 후처리 필터가 아니라, Cartographer 2D local trajectory builder 내부와 ROS publish layer를 모두 연결한 확장입니다.

핵심 특징은 다음 세 가지입니다.

- frozen finished submap을 이용한 동기식 재정합
- raw / filtered pose의 분리 가능한 반영 정책
- tuning과 비교 실험을 위한 로그 및 test mode 지원

실제로 기능을 요약하면 아래처럼 볼 수 있습니다.

- `FULL_PIPELINE`
  frozen 결과를 local SLAM 내부까지 밀어 넣는 모드
- `PUBLISH_ONLY`
  내부는 raw, publish만 filtered
- `test_mode_publish_filtered_odom`
  raw `/odom`과 filtered `/filtered_odom`를 동시에 내보내는 비교 모드

이 문서를 기준으로 보면, 앞으로 내부 코드를 수정할 때 어느 파일을 왜 건드려야 하는지 빠르게 판단할 수 있습니다.
