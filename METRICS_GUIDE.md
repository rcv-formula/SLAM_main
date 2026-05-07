# Cartographer 메트릭 사용 가이드

## 📊 사용 가능한 Metrics

### 2D Local SLAM
- **latency**: Local SLAM 결과까지의 지연시간 (초)
- **real_time_ratio**: 센서 시간 / 벽시계 시간 (1.0 = 실시간)
- **cpu_real_time_ratio**: 센서 시간 / CPU 시간
- **scores[scan_matcher]**: 스캔 매칭 점수
- **costs[scan_matcher]**: 스캔 매칭 비용
- **residuals[component]**: 스캔 매칭 잔차

### 3D Local SLAM
- **latency**: 3D Local SLAM 지연시간
- **voxel_filter_fraction**: 복셀 필터 처리 시간 비율
- **scan_matcher_fraction**: 스캔 매칭 처리 시간 비율

### Constraint Builder
- **scores[search_region]**: 제약 조건 점수 (local/global)
- **num_submap_scan_matchers**: 구성된 submap 스캔 매칭 개수

### Pose Graph
- **constraints**: 포즈 그래프의 제약 조건 개수
- **work_queue_size**: 작업 큐 크기
- **work_queue_delay**: 작업 큐의 가장 오래된 항목의 나이

---

## 🚀 사용 방법

### 1️⃣ ROS 2 서비스 직접 호출

```bash
# 메트릭 읽기
ros2 service call /cartographer_ros/read_metrics cartographer_ros_msgs/srv/ReadMetrics
```

### 2️⃣ Python 스크립트로 메트릭 읽기

```bash
python3 /home/symoon/SLAM_main/read_metrics_example.py
```

**기능:**
- 모든 메트릭 확인
- 2D Local SLAM 메트릭만 추출
- JSON 형식 출력

### 3️⃣ 실시간 모니터링

```bash
python3 /home/symoon/SLAM_main/monitor_metrics.py
```

**기능:**
- 1초 간격으로 주요 메트릭 수집
- 실시간 성능 분석
- 병목 지점 자동 감지

### 4️⃣ 통합 실행 스크립트

```bash
bash /home/symoon/SLAM_main/run_with_metrics.sh
```

---

## 📈 해석 가이드

### Real-Time Ratio 해석
```
ratio > 1.1  : ✅ 실시간 처리 가능 (센서보다 빠름)
0.8 < ratio < 1.1 : ⚡ 적당한 실시간 성능
ratio < 0.8  : ⚠️  병목 상태 (센서보다 느림)
```

### Latency 해석
```
latency < 100ms  : ✅ 매우 빠름
100ms < latency < 500ms : ⚡ 정상
latency > 500ms  : ⚠️  느림 (최적화 필요)
```

### Residuals 해석
- **distance residuals**: 위치 오차
- **angle residuals**: 회전 오차
- 작을수록 좋음 (스캔 매칭이 더 정확)

---

## 🛠️ 메트릭 활성화

Cartographer ROS 노드 실행 시 메트릭 수집 활성화:

```bash
ros2 run cartographer_ros cartographer_node \
  --ros-args \
  -p use_metrics:=true
```

또는 launch 파일에서:
```xml
<arg name="use_metrics" default="true"/>
```

---

## 💾 메트릭 데이터 저장

Python에서 JSON으로 저장:

```python
import json

reader = MetricsReader()
metrics = reader.get_specific_metrics()

with open('metrics.json', 'w') as f:
    json.dump(metrics, f, indent=2)

# CSV로 저장
import csv

with open('metrics.csv', 'w', newline='') as f:
    writer = csv.writer(f)
    writer.writerow(['timestamp', 'metric', 'value'])
    for name, value in metrics.items():
        writer.writerow([time.time(), name, value])
```

---

## 🔗 관련 파일

- `/home/symoon/SLAM_main/read_metrics_example.py` - 메트릭 읽기 예제
- `/home/symoon/SLAM_main/monitor_metrics.py` - 실시간 모니터링
- `/home/symoon/SLAM_main/run_with_metrics.sh` - 통합 스크립트
- ROS Service: `/cartographer_ros/read_metrics`
  - Type: `cartographer_ros_msgs/srv/ReadMetrics`
  - Response: `status`, `metric_families`, `timestamp`

---

## ⚠️ 주의사항

1. **메트릭 수집 오버헤드**: CPU/메모리 약 2~5% 추가 사용
2. **실시간 성능**: 메트릭 수집이 활성화되면 약간의 지연 가능
3. **서비스 가용성**: Cartographer 노드가 실행 중이어야 서비스 호출 가능

---

## 📚 더 알아보기

- Cartographer 소스: `/home/symoon/SLAM_main/src/cartographer/`
- 메트릭 정의: `cartographer/metric/register.cc`
- ROS 래퍼: `cartographer_ros/src/metrics/`
