# Localization Health State Machine

## 현재 동작 요약

상태 머신은 local SLAM scan 처리마다 localization confidence를 계산합니다.
현재 버전에서는 상태를 확인만 하며, SLAM runtime 동작은 바꾸지 않습니다.

출력 topic:

```text
/localization_health         std_msgs/String
/localization_health_marker  RViz TEXT_VIEW_FACING marker
```

RViz marker에는 상태 텍스트만 표시합니다.

```text
GOOD
UNSTABLE
LOST
RECOVERING
```

상세 원인, score, margin, correction, streak 값은 RViz가 아니라 상태 전이
로그로 확인합니다.

```text
Localization health changed: GOOD -> UNSTABLE
frozen_status=rejected_low_margin
accepted=0
reject_streak=3
accept_streak=0
outlier_streak=0
local_residual=...
candidates=...
score=...
margin=...
variance=...
correction=...
```

로그 레벨:

```text
UNSTABLE / LOST      LOG(WARNING)
GOOD / RECOVERING   LOG(INFO)
```

## Runtime 영향

상태 머신은 pose, extrapolator, submap insertion, frozen matcher correction,
pose graph optimization에 개입하지 않습니다.

즉 상태가 `LOST`가 되어도 아래 동작은 기존 Cartographer 흐름 그대로입니다.

```text
scan matching 결과 publish
extrapolator AddPose
active submap insertion
frozen matcher accepted 결과 반영
FULL_PIPELINE 적용
pose graph node 생성
```

이 문서는 상태를 어떻게 계산하고 확인하는지만 설명합니다.

단, frozen map과 새 trajectory 사이의 global constraint는 false attach 방지를
위해 pose graph에 들어가기 전에 별도 검증을 거칩니다. 이 검증은 상태 표시가
아니라 실제 optimization 입력을 제한합니다.

## Global Optimization Jump 검증

global optimization으로 map 기준 pose가 jump하는 것 자체는 정상일 수 있습니다.
문제는 frozen map의 잘못된 위치에 붙는 false attach입니다.

현재 코드는 frozen map inter-trajectory global constraint를 바로 pose graph에
넣지 않습니다. 먼저 pending queue에 보관하고, 서로 비슷한 `map <- local`
보정을 지지하는 constraint가 충분히 모였을 때만 optimization에 넣습니다.

검증 기준:

```text
대상:
  frozen trajectory와 active trajectory 사이의 INTER_SUBMAP constraint

즉시 허용:
  같은 trajectory 내부 constraint
  frozen map과 무관한 constraint

보류:
  frozen map inter-trajectory global constraint
```

pending constraint가 만드는 보정값:

```text
constraint가 암시하는 node global pose
  = frozen_submap_global_pose * constraint_relative_pose

local_to_map_correction
  = implied_node_global_pose * current_node_global_pose^-1
```

commit 조건:

```text
서로 일관된 frozen-map global constraint >= 2개
AND translation 차이 <= 0.70 m
AND rotation 차이 <= 15 deg
```

pending 보관 시간:

```text
12초
```

의미:

```text
단발 global constraint
  -> optimization에 넣지 않음

여러 node/submap에서 같은 위치 보정을 반복 지지
  -> 좋은 global jump로 보고 optimization 허용

서로 다른 위치를 지지하는 constraint
  -> pending 상태로 남거나 만료되어 false attach 억제
```

관련 로그:

```text
Holding N frozen-map global constraints pending consistency confirmation
Accepted N consistent frozen-map global constraints for optimization
```

## 상태 정의

상태는 매 scan 처리 후 `UpdateLocalizationHealthState()`에서 갱신됩니다.
판단은 아래 우선순서로 진행됩니다.

```text
1. LOST 조건 확인
2. 이전 상태가 LOST/RECOVERING인지 확인
3. UNSTABLE 조건 확인
4. 위 조건에 걸리지 않으면 GOOD
```

사용하는 내부 카운터:

```text
local_slam_outlier_streak
  local SLAM outlier가 연속으로 발생한 횟수

local_slam_hard_outlier_streak
  hard local SLAM outlier가 연속으로 발생한 횟수

frozen_match_reject_streak
  frozen matcher attempt가 연속으로 reject된 횟수

frozen_match_accept_streak
  frozen matcher attempt가 연속으로 accept된 횟수
```

현재 threshold는 코드에 고정되어 있습니다.

```text
correction_warning_ratio = 0.7
unstable_reject_count = 12
lost_hard_outlier_count = 80
lost_outlier_count = 240
recovery_required_successes = 3
```

## GOOD

현재 pose를 정상적으로 신뢰할 수 있다고 보는 상태입니다.

조건:

```text
LOST 조건이 아님
AND UNSTABLE 조건이 아님
AND 이전 LOST/RECOVERING 상태를 유지할 조건이 아님
```

`LOST`나 `RECOVERING`에서 `GOOD`으로 복귀하려면 아래 조건이 필요합니다.

```text
frozen matcher accepted
AND correction warning 없음
AND local SLAM outlier 아님
AND frozen_match_accept_streak >= 3
```

## UNSTABLE

pose는 계속 publish하지만 localization confidence가 낮아진 상태입니다.
`LOST`로 보기에는 이르지만, pose shift나 pose loss의 전조로 볼 수 있습니다.

조건:

```text
LOST 조건은 아니지만,

local SLAM outlier 발생
OR frozen matcher reject streak >= 12
OR frozen correction warning streak >= 12
```

frozen correction warning:

```text
translation_correction > 0.7 * max_translation_correction
OR
rotation_correction > 0.7 * max_rotation_correction
```

## LOST

현재 local pose를 신뢰하기 어렵다고 보는 상태입니다.
mapping 당시 map과 현재 환경은 물리적으로 조금씩 다를 수 있으므로,
일반 local outlier가 잠깐 지속되는 것만으로는 `LOST`로 올리지 않습니다.

조건:

```text
hard local SLAM outlier streak >= 80
OR
local_slam_outlier_streak >= 240
```

frozen matcher reject나 frozen constraint 부재만으로는 `LOST`로 올리지 않습니다.
이 경우는 frozen map과 현재 local SLAM 사이의 불일치일 수 있으므로
`UNSTABLE`로만 표시합니다. `LOST`는 local SLAM 자체의 연속 outlier가
충분히 길게 누적되어 현재 local pose를 신뢰하기 어렵다고 볼 때만 사용합니다.

즉 현재 기준에서는 다음처럼 해석합니다.

```text
짧은 residual 증가 / 맵과 실제 환경의 작은 차이
  -> UNSTABLE

큰 residual이 약 2초 이상 지속
  -> LOST

중간 수준 residual이 약 6초 이상 계속 지속
  -> LOST
```

`LOST`는 상태 표시와 로그에만 사용됩니다. `LOST`가 되더라도 pose publish,
extrapolator update, submap insertion, pose graph node 생성은 막지 않습니다.

이전 상태가 `LOST`여도 local SLAM outlier streak가 끊기면 상태가
`RECOVERING` 또는 `GOOD`으로 내려옵니다.

## RECOVERING

`LOST` 이후 frozen matcher accept가 다시 들어오기 시작했지만,
`GOOD`으로 복귀할 만큼 충분한 연속 안정성이 아직 확보되지 않은 상태입니다.

조건:

```text
이전 상태가 LOST 또는 RECOVERING
AND frozen matcher accepted
AND correction warning 없음
AND local SLAM outlier 아님
AND frozen_match_accept_streak < 3
```

`RECOVERING`에서 `GOOD`으로 바뀌는 조건:

```text
이전 상태가 LOST 또는 RECOVERING
AND frozen matcher accepted
AND correction warning 없음
AND local SLAM outlier 아님
AND frozen_match_accept_streak >= 3
```
