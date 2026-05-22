# Cartographer Real-Time 분석 가이드

## 📊 Cartographer의 두 가지 분석 방법

### 1️⃣ **Real-Time 메트릭 분석** (실행 중)
- **언제**: SLAM이 실행되는 동안
- **뭘 측정**: Latency, Real-time ratio, CPU usage, 스캔 매칭 성능 등
- **도구**: ROS 서비스 `/cartographer_ros/read_metrics`

### 2️⃣ **저장된 결과 분석** (사후)
- **언제**: SLAM 실행 완료 후
- **뭘 측정**: pbstream 구조, pgm 맵 품질, 포즈 그래프 정보
- **도구**: `cartographer_pbstream info`, 맵 이미지 분석

---

## 🚀 실행 방법

### 시나리오 1: 실행 중 성능 모니터링

```bash
# 1. Cartographer 노드 시작
ros2 run cartographer_ros cartographer_node ...

# 2. 다른 터미널에서 실시간 메트릭 모니터링
python3 /home/symoon/SLAM_main/monitor_metrics.py

# 또는 한 번에 읽기
python3 /home/symoon/SLAM_main/read_metrics_example.py
```

**측정 지표:**
- `real_time_ratio > 1.0` ✅ : 시스템이 센서보다 빠름 (실시간 처리 가능)
- `real_time_ratio < 0.8` ⚠️ : 병목 상태 (센서보다 느림)
- `latency < 100ms` ✅ : 빠른 응답

---

### 시나리오 2: 완료된 SLAM 결과 분석

#### 2-1. pbstream 정보 분석

```bash
# 모든 SLAM 결과 분석
python3 /home/symoon/SLAM_main/analyze_pbstream.py

# 또는 특정 파일 분석
python3 /home/symoon/SLAM_main/analyze_pbstream.py --file 0119_1.pbstream
```

**출력 정보:**
- 서브맵 개수
- 트래젝토리 노드 개수
- 제약 조건 개수
- 포즈 그래프 구조

#### 2-2. PGM 맵 이미지 분석

```bash
# 모든 맵 분석
python3 /home/symoon/SLAM_main/analyze_pgm.py

# 또는 특정 맵 분석
python3 /home/symoon/SLAM_main/analyze_pgm.py --file 0119_1.pgm
```

**출력 정보:**
- 맵 해상도
- 점유 공간 비율
- 자유 공간 비율
- 이미지 품질 (평균값, 표준편차)

---

## 📈 분석 결과 해석

### Real-Time 메트릭 해석

| 메트릭 | 좋음 | 주의 | 나쁨 |
|--------|------|------|------|
| Real-Time Ratio | > 1.2 | 0.8~1.2 | < 0.8 |
| Latency | < 50ms | 50~200ms | > 200ms |
| CPU Real-Time Ratio | > 2.0 | 1.0~2.0 | < 1.0 |

### pbstream 분석 해석

| 지표 | 의미 |
|------|------|
| 서브맵 개수 ↑ | 더 많은 맵 조각 = 더 정밀한 매핑 |
| 노드 개수 ↑ | 더 많은 포즈 = 더 상세한 궤적 |
| 제약 개수 ↑ | 더 많은 루프 폐쇄 = 더 관성있는 매핑 |

### PGM 맵 분석 해석

| 지표 | 좋음 | 나쁨 |
|------|------|------|
| 점유 픽셀 (%) | 낮음 10-20% | 높음 > 40% |
| 자유 픽셀 (%) | 높음 60-80% | 낮음 < 30% |
| 평균값 | 균형잡힘 | 너무 어둡거나 밝음 |

---

## 💡 분석 워크플로우

### 최적화 프로세스

```
1️⃣ Real-Time 모니터링
   ↓
   real_time_ratio 확인
   ↓ (< 0.8이면)
   
2️⃣ 병목 지점 찾기
   - latency 확인 (높으면 로컬 SLAM)
   - cpu_ratio 확인 (높으면 CPU 부족)
   
3️⃣ 최적화 실시
   - 센서 설정 변경
   - 처리 파라미터 조정
   - 하드웨어 업그레이드
   
4️⃣ 완료 후 분석
   - pbstream 및 pgm 분석
   - 구성미 개선 여부 확인
```

---

## 🔧 사용 예시

### 예시 1: 여러 실행 결과 비교

```bash
# 실행 1: 기본 설정
# → 0119_1.pbstream, 0119_1.pgm 생성

# 실행 2: 최적화된 설정
# → 0119_2.pbstream, 0119_2.pgm 생성

# 분석
python3 analyze_pbstream.py
python3 analyze_pgm.py

# 결과 비교
cat pbstream_analysis_*.json  # 구조 비교
cat pgm_analysis_*.json       # 맵 품질 비교
```

### 예시 2: 성능 문제 진단

```bash
# 1. 실시간 모니터링으로 늦음 확인
python3 monitor_metrics.py
# → real_time_ratio: 0.6 (너무 느림)

# 2. SLAM 결과 분석
python3 analyze_pbstream.py
python3 analyze_pgm.py

# 3. 원인 분석
# - 많은 제약 조건? → 루프 클로저 비활성화
# - 맵 품질 나쁨? → 해상도 조정
# - CPU 부하 높음? → 센서 프레임 레이트 낮춤
```

---

## 📁 생성되는 파일

| 파일 | 설명 |
|------|------|
| `metrics_data/*.csv` | 실시간 메트릭 데이터 |
| `pbstream_analysis_*.json` | pbstream 분석 결과 |
| `pgm_analysis_*.json` | PGM 맵 분석 결과 |
| `pgm_analysis_*.csv` | PGM 맵 분석 CSV |

---

## ⚡ 빠른 참조

```bash
# 📊 실시간 성능 확인
python3 monitor_metrics.py

# 💾 장시간 데이터 수집
python3 collect_metrics.py --duration 60

# 📈 SLAM 구조 분석
python3 analyze_pbstream.py

# 🗺️  맵 품질 분석
python3 analyze_pgm.py

# 📉 수집된 데이터 분석
python3 analyze_metrics.py metrics_data/*.csv --plot
```

---

## 🎯 최종 체크리스트

SLAM 시스템 최적화를 위해:

- [ ] 실시간 메트릭에서 real_time_ratio > 1.0 확인
- [ ] latency < 100ms 확인
- [ ] pbstream에서 제약 조건이 충분한지 확인
- [ ] pgm 맵에서 이상한 부분(큰 노이즈) 확인
- [ ] 여러 실행 결과를 비교해 일관성 확인

모두 완료되면 SLAM 시스템이 최적화된 것!

---

## 📚 참고

- Real-Time Metrics: `/home/symoon/SLAM_main/monitor_metrics.py`
- Pbstream Analysis: `/home/symoon/SLAM_main/analyze_pbstream.py`
- PGM Analysis: `/home/symoon/SLAM_main/analyze_pgm.py`
- 메트릭 수집: `/home/symoon/SLAM_main/collect_metrics.py`
- 메트릭 분석: `/home/symoon/SLAM_main/analyze_metrics.py`
