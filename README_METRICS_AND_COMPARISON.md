# Cartographer 메트릭 분석 및 비교 가이드

## 📋 목차

1. [Real-Time 메트릭 분석](#1-real-time-메트릭-분석)
2. [pbstream 기반 비교 분석](#2-pbstream-기반-비교-분석)
3. [결과 해석 가이드](#3-결과-해석-가이드)
4. [전체 워크플로우](#4-전체-워크플로우)

---

## 1️⃣ Real-Time 메트릭 분석

SLAM이 **실행 중일 때** Cartographer의 내장 메트릭을 실시간으로 수집합니다.
정합 품질과 함께 `latency`도 같이 확인합니다.

### 실행 방법

#### **0단계: Cartographer를 metric 모드로 실행**

```bash
# 터미널 1: wheel odometry 사용 버전
ros2 launch cartographer_ros Damvi_carto_wheel_metric_launch.py

# 또는 pure lidar/imu 버전
ros2 launch cartographer_ros Damvi_carto_pure_metric_launch.py
```

위 launch 파일에는 `--collect_metrics` 옵션이 들어 있어야 합니다.
현재 `Damvi_carto_wheel_metric_launch.py`와 `Damvi_carto_pure_metric_launch.py`는 metric 수집이 켜진 실행 파일입니다.

서비스가 떠 있는지 확인하려면 다른 터미널에서 확인합니다.

```bash
ros2 service list | grep read_metrics
```

정상이라면 보통 아래처럼 나옵니다.

```text
/read_metrics
```

환경에 따라 `/cartographer_ros/read_metrics`로 보일 수도 있습니다.
`monitor_metrics.py`와 `collect_metrics.py`는 두 이름을 자동으로 찾아서 연결합니다.

#### **방법 A: 실시간 화면 모니터링** (권장)

SLAM이 실행 중인 상태에서 다른 터미널을 열고 실행합니다.

```bash
python3 /home/symoon/SLAM_main/monitor_metrics.py
```

이 방법은 화면에서 현재 상태를 계속 보는 용도입니다.
파일 저장은 하지 않습니다.
종료하려면 `Ctrl+C`를 누릅니다.

실시간 확인만 할 때는 아래 두 터미널만 사용하면 됩니다.

```bash
# 터미널 1: SLAM 실행
ros2 launch cartographer_ros Damvi_carto_wheel_metric_launch.py

# 터미널 2: 메트릭 실시간 확인
python3 /home/symoon/SLAM_main/monitor_metrics.py
```

화면에서 주로 볼 값은 아래 네 가지입니다.

| 항목 | 보는 기준 |
|------|-----------|
| `Latency` | 낮을수록 좋음, 보통 0.05s 이하면 좋음 |
| `Real-Time Ratio` | 1.0 이상이면 실시간 처리 가능 |
| `Score` | 높을수록 scan matching 품질 좋음 |
| `Residuals` | 낮을수록 정합 오차 작음 |

**출력 예:**
```
[반복 #1] 14:23:45
----------------
⏱️  Latency                 : 0.0245s
📊 Real-Time Ratio         : 1.2345
⚙️  CPU Real-Time Ratio    : 2.1234
📐 Residuals (distance)   : 0.001234

📈 상태 분석:
✅ 시스템이 실시간 처리 가능 (ratio: 1.23)
```

#### **방법 B: 실시간 메트릭을 파일로 저장**

SLAM이 실행 중인 상태에서 다른 터미널을 열고 실행합니다.

```bash
# 60초 동안 1초 간격으로 수집 후 CSV/JSON 저장
python3 /home/symoon/SLAM_main/collect_metrics.py \
  --duration 60 \
  --interval 1 \
  --output /home/symoon/SLAM_main/metrics_data
```

결과는 아래 위치에 저장됩니다.

```text
/home/symoon/SLAM_main/metrics_data/metrics_YYYYMMDD_HHMMSS.csv
/home/symoon/SLAM_main/metrics_data/metrics_YYYYMMDD_HHMMSS.json
```

장시간 계속 저장하려면 `--duration`을 크게 잡습니다.

```bash
# 5분 저장
python3 /home/symoon/SLAM_main/collect_metrics.py \
  --duration 300 \
  --interval 1 \
  --output /home/symoon/SLAM_main/metrics_data

# 1시간 저장
python3 /home/symoon/SLAM_main/collect_metrics.py \
  --duration 3600 \
  --interval 1 \
  --output /home/symoon/SLAM_main/metrics_data
```

주의: 수집 도중 `Ctrl+C`로 끊으면 저장 단계까지 가지 않을 수 있습니다.
비교용 데이터는 필요한 시간만큼 `--duration`을 지정해서 끝까지 실행하는 것이 좋습니다.

#### **방법 C: 메트릭 한 번 읽기**

```bash
python3 /home/symoon/SLAM_main/read_metrics_example.py
```

#### **전체 실행 예시**

```bash
# 터미널 1: SLAM 실행
ros2 launch cartographer_ros Damvi_carto_wheel_metric_launch.py

# 터미널 2: 실시간 화면 확인
python3 /home/symoon/SLAM_main/monitor_metrics.py

# 터미널 3: 60초 동안 파일 저장
python3 /home/symoon/SLAM_main/collect_metrics.py \
  --duration 60 \
  --interval 1 \
  --output /home/symoon/SLAM_main/metrics_data

# 터미널 4: 저장된 CSV 분석 및 그래프 생성
python3 /home/symoon/SLAM_main/analyze_metrics.py \
  /home/symoon/SLAM_main/metrics_data/metrics_*.csv \
  --plot
```

### 수집되는 메트릭 정보

| 메트릭명 | 단위 | 설명 |
|---------|------|------|
| **latency** | 초(s) | 첫 포인트 클라우드부터 SLAM 결과까지의 처리 시간 |
| **real_time_ratio** | - | 센서 시간 / 벽시계 시간 (1.0=정상 속도) |
| **cpu_real_time_ratio** | - | 센서 시간 / CPU 시간 (높을수록 효율적) |
| **scores[scan_matcher]** | - | 스캔 매칭 점수 (높을수록 정확) |
| **costs[scan_matcher]** | - | 스캔 매칭 비용 (낮을수록 좋음) |
| **residuals[distance]** | m | 거리 매칭 오차 (낮을수록 정확) |
| **residuals[angle]** | rad | 각도 매칭 오차 (낮을수록 정확) |

### 해석 가이드

#### **Real-Time Ratio (실시간 비율)**

```
ratio > 1.2  ✅ 매우 좋음
             센서보다 빠르게 처리 중
             시스템 여유있음

0.9 ~ 1.2 ⚡ 적당함
           거의 실시간 처리
           정상 작동

0.8 ~ 0.9 ⚠️  주의
           약간 느림
           최적화 필요

< 0.8     ❌ 나쁨
          병목 상태
          시스템 처리 불가
```

#### **Latency (지연시간)**

```
< 50ms   ✅ 매우 빠름
50-100ms ⚡ 정상
100-200ms ⚠️ 느림
> 200ms  ❌ 매우 느림 (최적화 필요)
```

#### **Residuals (매칭 오차)**

```
distance < 0.01m ✅ 매우 정확
         < 0.05m ⚡ 정상
         < 0.1m  ⚠️ 주의
         > 0.1m  ❌ 나쁨 (스캔 매칭 실패 가능)

angle < 0.01 rad ✅ 매우 정확
      < 0.05 rad ⚡ 정상
      < 0.1 rad  ⚠️ 주의
      > 0.1 rad  ❌ 나쁨
```

### 분석 및 저장

```bash
# 수집된 메트릭 분석
python3 /home/symoon/SLAM_main/analyze_metrics.py \
  /home/symoon/SLAM_main/metrics_data/metrics_*.csv \
  --plot

# 출력: 통계, 그래프(CSV), 그래프 이미지(PNG)
```

---

## 2️⃣ pbstream 기반 비교 분석

SLAM이 **완료된 후** 저장된 pbstream 파일들을 **정량적으로 비교**합니다.

### 단계 1: Ground Truth (GT) 생성

기준이 될 **최고 품질 pbstream**에서 GT를 생성합니다.

```bash
cd /home/symoon/SLAM_main

./build/cartographer/cartographer_autogenerate_ground_truth \
  --pose_graph_filename=0119_1.pbstream \
  --output_filename=relations_0119_1.pbstream \
  --min_covered_distance=100 \
  --outlier_threshold_meters=0.15 \
  --outlier_threshold_radians=0.02
```

**옵션 설명:**

| 옵션 | 값 | 설명 |
|------|-----|------|
| `--pose_graph_filename` | pbstream 파일 | **기준**: 최고 품질의 참고 데이터 |
| `--output_filename` | relations 파일명 | **출력**: 생성될 GT 파일 |
| `--min_covered_distance` | 100 | 최소 100m 이상의 거리 커버 (단거리 오류 제외) |
| `--outlier_threshold_meters` | 0.15 | 15cm 이상의 오차는 노이즈로 제외 |
| `--outlier_threshold_radians` | 0.02 | 0.02 rad(≈1.15°) 이상의 회전 오차 제외 |

**실행 결과:**
```
✅ relations_0119_1.pbstream 생성
```

### 단계 2: 다른 pbstream과 비교

생성된 GT와 test.pbstream을 비교합니다.

```bash
# 0120.pbstream과 비교
./build/cartographer/cartographer_compute_relations_metrics \
  --relations_filename=relations_0119_1.pbstream \
  --pose_graph_filename=0120.pbstream

# 0125_1.pbstream과 비교
./build/cartographer/cartographer_compute_relations_metrics \
  --relations_filename=relations_0119_1.pbstream \
  --pose_graph_filename=0125_1.pbstream

# 0125_2.pbstream과 비교
./build/cartographer/cartographer_compute_relations_metrics \
  --relations_filename=relations_0119_1.pbstream \
  --pose_graph_filename=0125_2.pbstream
```

### 단계 3: 결과 해석

**출력 예:**

```
Abs translational error: 0.234 +/- 0.089 m
Sqr translational error: 0.115 +/- 0.042 m^2
Abs rotational error: 1.23 +/- 0.45 deg
Sqr rotational error: 2.34 +/- 0.89 deg^2
```

#### **결과 항목 설명**

| 항목 | 단위 | 의미 |
|------|------|------|
| **Abs translational error** | m | 위치 오차 평균 |
| **± (표준편차)** | m | 위치 오차의 신뢰도 |
| **Abs rotational error** | deg | 회전 오차 평균 |
| **± (표준편차)** | deg | 회전 오차의 신뢰도 |

#### **해석 가이드**

```
위치 오차 (Translational)
0.05m 이하    ✅ 매우 정확 (실내 네비게이션 가능)
0.05 ~ 0.1m  ⚡ 정상 (특수 목적 가능)
0.1 ~ 0.5m   ⚠️  주의 (센서/셋팅 확인 필요)
> 0.5m       ❌ 나쁨 (실패 상태)

회전 오차 (Rotational)
0.5° 이하    ✅ 매우 정확
0.5 ~ 2°     ⚡ 정상
2 ~ 5°       ⚠️  주의
> 5°         ❌ 나쁨 (실패 상태)

표준편차 (±)
작을수록 ✅ 일관성 있음 (안정적)
클수록   ❌ 불일관 (불안정)
```

### 예시: 여러 비교 결과 정리

```
기준: 0119_1.pbstream
────────────────────────────────────────────────────────
0119_1 vs 0120:
  위치 오차: 0.145 ± 0.062 m  ⚡ 정상
  회전 오차: 0.89 ± 0.34°    ✅ 정확

0119_1 vs 0125_1:
  위치 오차: 0.312 ± 0.145 m  ⚠️  주의 (약간 높음)
  회전 오차: 1.23 ± 0.56°    ⚡ 정상

0119_1 vs 0125_2:
  위치 오차: 0.089 ± 0.031 m  ✅ 매우 정확
  회전 오차: 0.54 ± 0.21°    ✅ 정확

결론: 0125_2 > 0020 > 0125_1
→ 0125_2 실행 조건이 가장 좋음
```

---

## 3️⃣ 결과 해석 가이드

### Real-Time vs pbstream 비교

| 상황 | Real-Time | pbstream | 의미 |
|------|-----------|---------|------|
| 둘 다 좋음 | ratio>1.2 | error<0.1m | ✅ 최적의 SLAM 시스템 |
| RT 좋음, PB 나쁨 | ratio>1.2 | error>0.5m | ⚠️  실시간은 좋지만 정확도 낮음 (센서/환경 문제) |
| RT 나쁨, PB 좋음 | ratio<0.8 | error<0.1m | ⚠️  느리지만 정확함 (CPU 부하) |
| 둘 다 나쁨 | ratio<0.8 | error>0.5m | ❌ 심각한 문제 (센서/하드웨어 교정 필요) |

### 최적화 가이드

```
문제: real_time_ratio < 0.8
원인 분석:
  1. latency가 높으면 → 로컬 SLAM 최적화
  2. cpu_ratio가 낮으면 → CPU 부하 줄이기
     (센서 프레임 레이트 감소, 해상도 조정)

문제: pbstream 오차 > 0.5m
원인 분석:
  1. residuals가 크면 → 스캔 매칭 파라미터 조정
  2. 특정 구간에만 높으면 → 센서 노이즈 또는 환경 문제
```

---

## 4️⃣ 전체 워크플로우

### 워크플로우 A: 시스템 성능 평가

```bash
# 1️⃣ SLAM 실행
ros2 launch cartographer_ros Damvi_carto_wheel_metric_launch.py

# 2️⃣ 실시간 화면 확인
python3 /home/symoon/SLAM_main/monitor_metrics.py

# 3️⃣ 장기간 저장
python3 /home/symoon/SLAM_main/collect_metrics.py \
  --duration 300 \
  --interval 1 \
  --output /home/symoon/SLAM_main/metrics_data

# 4️⃣ 저장 데이터 분석
python3 /home/symoon/SLAM_main/analyze_metrics.py \
  /home/symoon/SLAM_main/metrics_data/metrics_*.csv \
  --plot
```

### 워크플로우 B: 정확도 평가

```
1️⃣ SLAM 완료 후
   ├─ pbstream 파일 확보
   └─ 최고 품질 선택 (0119_1.pbstream)

2️⃣ GT 생성
   ├─ cartographer_autogenerate_ground_truth 실행
   └─ relations_0119_1.pbstream 생성

3️⃣ 비교 분석
   ├─ 각 pbstream과 비교
   ├─ cartographer_compute_relations_metrics 실행
   └─ 오차 계산

4️⃣ 결과 정리
   ├─ 테이블로 정렬
   ├─ 최고/최저 확인
   └─ 원인 분석

결론: 정확도 평가 완료
```

### 워크플로우 C: 최적화 순환

```
1️⃣ 성능 평가 (Workflow A + B)

2️⃣ 병목 지점 식별
   ├─ Real-Time: ratio < 0.8?
   ├─ Accuracy: error > 0.1m?
   └─ Residuals: > 0.05m?

3️⃣ 최적화 수행
   ├─ 파라미터 조정
   ├─ 센서 설정 변경
   └─ 하드웨어 업그레이드

4️⃣ 다시 평가 (1️⃣로 돌아가기)

개선됨? → 완료
안 됨? → 다른 파라미터 조정
```

---

## 🔧 실용 예시: 당신의 데이터로 분석하기

### 예시 1: 0119_1 기준으로 모든 파일 비교

```bash
cd /home/symoon/SLAM_main

# 1️⃣ GT 생성
./build/cartographer/cartographer_autogenerate_ground_truth \
  --pose_graph_filename=0119_1.pbstream \
  --output_filename=relations_0119_1.pbstream \
  --min_covered_distance=100 \
  --outlier_threshold_meters=0.15 \
  --outlier_threshold_radians=0.02

# 2️⃣ 모든 파일과 비교
for file in 0119.pbstream 0120.pbstream 0122.pbstream 0123.pbstream \
            0125_1.pbstream 0125_2.pbstream 0125_3.pbstream \
            0125_4.pbstream 0125_5.pbstream 0203.pbstream 0312.pbstream 0320.pbstream; do
  echo "=== Comparing $file ==="
  ./build/cartographer/cartographer_compute_relations_metrics \
    --relations_filename=relations_0119_1.pbstream \
    --pose_graph_filename=$file
  echo ""
done
```

**결과 정렬:**
```
파일명                위치오차          회전오차          평가
────────────────────────────────────────────────────────
0119_1    (기준)    0.000 ± 0.000    0.00 ± 0.00    ✅✅✅
0120      compare   0.145 ± 0.062    0.89 ± 0.34    ✅ 좋음
0125_2    compare   0.089 ± 0.031    0.54 ± 0.21    ✅ 매우좋음
0125_1    compare   0.312 ± 0.145    1.23 ± 0.56    ⚠️ 주의
0312      compare   0.523 ± 0.234    3.45 ± 1.23    ❌ 나쁨
```

### 예시 2: Real-Time 모니터링 + 이후 비교

```bash
# 터미널 1: SLAM 실행
ros2 launch cartographer_ros Damvi_carto_wheel_metric_launch.py

# 터미널 2: 실시간 모니터링
python3 /home/symoon/SLAM_main/monitor_metrics.py

# 터미널 3: 실행 중 메트릭을 CSV/JSON으로 저장
python3 /home/symoon/SLAM_main/collect_metrics.py \
  --duration 300 \
  --interval 1 \
  --output /home/symoon/SLAM_main/metrics_data

# SLAM 완료 후
# 터미널 4: pbstream 비교
./build/cartographer/cartographer_autogenerate_ground_truth \
  --pose_graph_filename=result.pbstream \
  --output_filename=relations.pbstream

./build/cartographer/cartographer_compute_relations_metrics \
  --relations_filename=relations.pbstream \
  --pose_graph_filename=old_result.pbstream

# 결과: real_time_ratio와 정확도를 함께 비교
```

---

## 📊 결과 저장 및 보고

### 자동 리포트 생성

```bash
# 1️⃣ 실시간 메트릭 (CSV + JSON)
python3 /home/symoon/SLAM_main/collect_metrics.py \
  --duration 60 \
  --interval 1 \
  --output /home/symoon/SLAM_main/metrics_data

# 2️⃣ pbstream 분석 (JSON)
python3 /home/symoon/SLAM_main/analyze_pbstream.py

# 3️⃣ PGM 맵 분석 (CSV + JSON)
python3 /home/symoon/SLAM_main/analyze_pgm.py

# → metrics_data/ 디렉토리에 모든 결과 저장
```

### 결과 파일 확인

```bash
ls -lh /home/symoon/SLAM_main/metrics_data/

metrics_20260429_120000.csv       ← 실시간 메트릭
metrics_20260429_120000.json      ← 메트릭 JSON
metrics_20260429_120000_plot.png  ← 그래프
pbstream_analysis_*.json          ← pbstream 분석
pgm_analysis_*.json               ← 맵 분석
pgm_analysis_*.csv                ← 맵 분석 CSV
```

---

## ✅ 빠른 체크리스트

### Real-Time 성능 평가 체크리스트

- [ ] `real_time_ratio > 1.0` 확인
- [ ] `latency < 100ms` 확인
- [ ] `residuals < 0.05m` 확인
- [ ] `cpu_real_time_ratio > 1.5` 확인

### pbstream 정확도 평가 체크리스트

- [ ] GT 파일 성공적으로 생성
- [ ] 모든 비교 완료
- [ ] 위치 오차 < 0.1m 확인
- [ ] 회전 오차 < 2° 확인
- [ ] 표준편차 작은 파일 확인 (안정성)

### 최적화 후 재검증 체크리스트

- [ ] Real-Time 메트릭 개선 확인
- [ ] pbstream 정확도 개선 확인
- [ ] 두 지표 모두 개선되었는지 확인

---

## 🎯 최종 해석 예시

**상황:**
- Real-Time Ratio: 1.15 (좋음)
- Latency: 0.045s (정상)
- Residuals: 0.023m (정상)
- pbstream 오차: 0.089m ± 0.031m (정상)

**결론:**
✅ **SLAM 시스템 정상 작동**
- 실시간 처리 가능
- 정확도도 우수함
- 추가 최적화 불필요

---

## 📞 문제 해결

| 문제 | 원인 | 해결책 |
|------|------|--------|
| real_time_ratio < 0.8 | CPU 병목 | 센서 FPS 감소, 해상도 조정 |
| latency > 200ms | 처리 최적화 필요 | 파라미터 튜닝 |
| pbstream 오차 > 0.5m | 센서 오정렬 또는 환경 | 센서 캘리브레이션, 환경 개선 |
| residuals > 0.1m | 스캔 매칭 실패 | 매턴 또는 센서 노이즈 |

---

## 📚 참고 파일

- `/home/symoon/SLAM_main/monitor_metrics.py` - 실시간 모니터링
- `/home/symoon/SLAM_main/collect_metrics.py` - 데이터 수집
- `/home/symoon/SLAM_main/analyze_metrics.py` - 데이터 분석
- `/home/symoon/SLAM_main/analyze_pbstream.py` - pbstream 분석
- `/home/symoon/SLAM_main/analyze_pgm.py` - 맵 분석

**모든 도구는 `/home/symoon/SLAM_main/` 디렉토리에 있습니다.**

---

**작성일:** 2026년 4월 29일
**마지막 수정:** 2026년 4월 29일
