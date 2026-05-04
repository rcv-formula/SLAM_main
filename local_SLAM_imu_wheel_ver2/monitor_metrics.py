#!/usr/bin/env python3
"""
Cartographer 메트릭 실시간 모니터링 스크립트
주기적으로 메트릭을 수집하고 분석
"""

import rclpy
from rclpy.node import Node
from cartographer_ros_msgs.srv import ReadMetrics
import time
from collections import defaultdict


class MetricsMonitor(Node):
    def __init__(self, interval=1.0):
        super().__init__('metrics_monitor')
        self.interval = interval
        
        # ReadMetrics 서비스 클라이언트
        # 환경에 따라 /read_metrics 또는 /cartographer_ros/read_metrics 로 노출될 수 있음
        self.service_name = None
        self.client = None
        for service_name in ('/read_metrics', '/cartographer_ros/read_metrics'):
            client = self.create_client(ReadMetrics, service_name)
            if client.wait_for_service(timeout_sec=1.0):
                self.client = client
                self.service_name = service_name
                break
        if self.client is None:
            self.client = self.create_client(ReadMetrics, '/read_metrics')
        
        # 메트릭 히스토리 저장
        self.history = defaultdict(list)
        self.max_history = 100
        
        while self.service_name is None:
            self.get_logger().info('Waiting for ReadMetrics service...')
            for service_name in ('/read_metrics', '/cartographer_ros/read_metrics'):
                if self.create_client(ReadMetrics, service_name).wait_for_service(timeout_sec=0.1):
                    self.service_name = service_name
                    self.client = self.create_client(ReadMetrics, service_name)
                    break
    
    def read_metrics(self):
        """메트릭 읽기"""
        request = ReadMetrics.Request()
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        try:
            response = future.result()
            if response is not None and hasattr(response, 'status'):
                if response.status.code == 14:
                    self.get_logger().warn(
                        'Runtime metrics are disabled. Launch cartographer_node with collect_metrics enabled.')
            return response
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')
            return None
    
    def extract_value(self, metric):
        """메트릭에서 값 추출"""
        if metric.type in (metric.TYPE_GAUGE, metric.TYPE_COUNTER):
            return metric.value
        return None

    def labels_to_dict(self, metric):
        labels = {}
        for label in metric.labels:
            # cartographer_ros_msgs/MetricLabel uses `key`, but keep fallback
            # for environments that expose the older `name` field.
            key = getattr(label, 'key', None)
            if key is None:
                key = getattr(label, 'name', None)
            if key is None:
                continue
            labels[key] = label.value
        return labels

    def format_metric_value(self, metric):
        """메트릭 값 문자열 생성"""
        value = self.extract_value(metric)
        if value is not None:
            return str(value), value

        if metric.type == metric.TYPE_HISTOGRAM:
            buckets = list(metric.counts_by_bucket)
            total = sum(bucket.count for bucket in buckets)
            if total <= 0:
                return "histogram{n=0}", None

            peak_bucket = max(buckets, key=lambda b: b.count)
            cumulative = 0.0
            rep_sum = 0.0
            prev_boundary = 0.0
            p50 = None
            p90 = None
            p95 = None
            for bucket in buckets:
                bucket_count = bucket.count
                upper = bucket.bucket_boundary
                if upper != float('inf'):
                    midpoint = (prev_boundary + upper) / 2.0
                    rep_sum += midpoint * bucket_count
                cumulative += bucket.count
                ratio = cumulative / total
                if p50 is None and ratio >= 0.50:
                    p50 = bucket.bucket_boundary
                if p90 is None and ratio >= 0.90:
                    p90 = bucket.bucket_boundary
                if p95 is None and ratio >= 0.95:
                    p95 = bucket.bucket_boundary
                prev_boundary = bucket.bucket_boundary

            def fmt(v):
                return "inf" if v == float('inf') else f"{v:g}"

            rep = rep_sum / total

            return (
                f"histogram{{n={int(total)}, "
                f"rep~={rep:.4g}, "
                f"p50<={fmt(p50)}, p90<={fmt(p90)}, p95<={fmt(p95)}, "
                f"peak<={fmt(peak_bucket.bucket_boundary)}({int(peak_bucket.count)})}}",
                rep,
            )

        return None, None

    def classify_metric(self, metric_name, value):
        if value is None:
            return "unknown"

        if metric_name.startswith('⏱️  Latency'):
            if value <= 0.05:
                return "good"
            if value <= 0.1:
                return "ok"
            return "high"

        if metric_name.startswith('📊 Real-Time Ratio') or metric_name.startswith('⚙️  CPU Real-Time Ratio'):
            if value >= 1.2:
                return "good"
            if value >= 0.9:
                return "ok"
            return "low"

        if metric_name.startswith('⭐ Score'):
            if value >= 0.9:
                return "good"
            if value >= 0.7:
                return "ok"
            return "low"

        if metric_name.startswith('💰 Cost'):
            if value <= 0.2:
                return "good"
            if value <= 0.5:
                return "ok"
            return "high"

        if metric_name.startswith('📐 Residuals'):
            if value <= 0.02:
                return "good"
            if value <= 0.05:
                return "ok"
            return "high"

        return "unknown"

    def assess_quality(self, score, cost, dist_residual, angle_residual, latency=None):
        notes = []

        if score is not None:
            if score >= 0.9:
                notes.append(f"score good ({score:.3f})")
            elif score >= 0.7:
                notes.append(f"score ok ({score:.3f})")
            else:
                notes.append(f"score low ({score:.3f})")

        if cost is not None:
            if cost <= 0.2:
                notes.append(f"cost good ({cost:.3f})")
            elif cost <= 0.5:
                notes.append(f"cost ok ({cost:.3f})")
            else:
                notes.append(f"cost high ({cost:.3f})")

        if dist_residual is not None:
            if dist_residual <= 0.02:
                notes.append(f"distance good ({dist_residual:.4f})")
            elif dist_residual <= 0.05:
                notes.append(f"distance ok ({dist_residual:.4f})")
            else:
                notes.append(f"distance high ({dist_residual:.4f})")

        if angle_residual is not None:
            if angle_residual <= 0.02:
                notes.append(f"angle good ({angle_residual:.4f})")
            elif angle_residual <= 0.05:
                notes.append(f"angle ok ({angle_residual:.4f})")
            else:
                notes.append(f"angle high ({angle_residual:.4f})")

        if latency is not None:
            if latency <= 0.05:
                notes.append(f"latency good ({latency:.4f}s)")
            elif latency <= 0.1:
                notes.append(f"latency ok ({latency:.4f}s)")
            else:
                notes.append(f"latency high ({latency:.4f}s)")

        return notes
    
    def monitor(self):
        """실시간 모니터링"""
        print("\n" + "="*80)
        print("🔍 Cartographer Metrics Monitor")
        print("="*80)
        print("정합 품질 메트릭을 raw 형태로 그대로 출력합니다.")
        print("기준: latency<=0.05s good, ratio>=1.2 good, score>=0.9 good, cost<=0.2 good, residual<=0.02 good")
        print("="*80 + "\n")
        
        try:
            iteration = 0
            while True:
                response = self.read_metrics()
                
                if response is None:
                    time.sleep(self.interval)
                    continue
                
                iteration += 1
                print(f"\n[반복 #{iteration}] {time.strftime('%H:%M:%S')}")
                print("-" * 80)
                
                # 중요 메트릭 정리
                key_metrics = {}
                
                for metric_family in response.metric_families:
                    fn = metric_family.name

                    # 핵심 메트릭만 수집
                    if fn.startswith('mapping_2d_local_trajectory_builder_'):
                        for metric in metric_family.metrics:
                            formatted_value, numeric_value = self.format_metric_value(metric)
                            if formatted_value is None:
                                continue

                            labels = self.labels_to_dict(metric)

                            if fn.endswith('_latency'):
                                key = '⏱️  Latency'
                                status = self.classify_metric(key, numeric_value)
                                key_metrics[key] = f"{formatted_value if formatted_value.endswith('s') else formatted_value + 's'} [{status}]"

                            elif fn.endswith('_real_time_ratio'):
                                key = '⚙️  CPU Real-Time Ratio' if 'cpu' in fn else '📊 Real-Time Ratio'
                                status = self.classify_metric(key, numeric_value)
                                key_metrics[key] = f"{formatted_value} [{status}]"

                            elif '_scores' in fn:
                                matcher = labels.get('scan_matcher', 'unknown')
                                key = f'⭐ Score ({matcher})'
                                status = self.classify_metric(key, numeric_value)
                                key_metrics[key] = f"{formatted_value} [{status}]"

                            elif '_costs' in fn:
                                matcher = labels.get('scan_matcher', 'unknown')
                                key = f'💰 Cost ({matcher})'
                                status = self.classify_metric(key, numeric_value)
                                key_metrics[key] = f"{formatted_value} [{status}]"

                            elif '_residuals' in fn:
                                component = labels.get('component', 'unknown')
                                key = f'📐 Residuals ({component})'
                                status = self.classify_metric(key, numeric_value)
                                key_metrics[key] = f"{formatted_value} [{status}]"
                
                # 출력
                for key, value in sorted(key_metrics.items()):
                    print(f"  {key:30} : {value}")
                
                time.sleep(self.interval)
        
        except KeyboardInterrupt:
            print("\n\n모니터링 중단됨")


def main():
    rclpy.init()
    
    # 간격 설정 (초)
    interval = 1.0
    
    monitor = MetricsMonitor(interval=interval)
    monitor.monitor()
    
    monitor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
