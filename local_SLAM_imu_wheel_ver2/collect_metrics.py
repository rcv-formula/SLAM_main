#!/usr/bin/env python3
"""
Cartographer 메트릭 수집 및 분석 도구
시간 기반으로 메트릭을 수집하고 CSV/JSON으로 저장
"""

import rclpy
from rclpy.node import Node
from cartographer_ros_msgs.srv import ReadMetrics
import csv
import json
import time
from datetime import datetime
from pathlib import Path
import argparse


class MetricsCollector(Node):
    def __init__(self, output_dir='/home/symoon/Desktop/metric_data', duration=60, interval=1.0):
        super().__init__('metrics_collector')
        
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(exist_ok=True)
        
        self.duration = duration
        self.interval = interval
        
        # ReadMetrics 서비스 클라이언트
        # launch namespace에 따라 /read_metrics 또는 /cartographer_ros/read_metrics 로 노출될 수 있음
        self.service_name = None
        self.client = None
        self.service_candidates = ('/read_metrics', '/cartographer_ros/read_metrics')

        while self.client is None:
            for service_name in self.service_candidates:
                client = self.create_client(ReadMetrics, service_name)
                if client.wait_for_service(timeout_sec=0.2):
                    self.client = client
                    self.service_name = service_name
                    break
            if self.client is None:
                self.get_logger().info(
                    'Waiting for ReadMetrics service '
                    f'({", ".join(self.service_candidates)})...')
                time.sleep(0.6)

        self.get_logger().info(f'Using ReadMetrics service: {self.service_name}')
        
        # 데이터 저장소
        self.data = []
        self.start_time = None
    
    def read_metrics(self):
        """메트릭 읽기"""
        request = ReadMetrics.Request()
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        try:
            return future.result()
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
            key = getattr(label, 'key', None)
            if key is None:
                key = getattr(label, 'name', None)
            if key is None:
                continue
            labels[key] = label.value
        return labels

    def metric_name(self, family_name, metric):
        labels = self.labels_to_dict(metric)
        if not labels:
            return family_name
        labels_str = "_".join([f"{key}={value}" for key, value in labels.items()])
        return f"{family_name}[{labels_str}]"

    def histogram_summary(self, metric):
        counts = [(bucket.bucket_boundary, bucket.count) for bucket in metric.counts_by_bucket]
        if not counts:
            return {}

        total = sum(count for _, count in counts)
        if total <= 0:
            return {'n': 0}

        peak_boundary, peak_count = max(counts, key=lambda item: item[1])
        summary = {
            'n': total,
            'peak_boundary': peak_boundary,
            'peak_count': peak_count,
        }

        for percentile, threshold in (('p50', 0.50), ('p90', 0.90), ('p95', 0.95)):
            target = total * threshold
            cumulative = 0
            for boundary, count in counts:
                cumulative += count
                if cumulative >= target:
                    summary[percentile] = boundary
                    break

        return summary
    
    def collect(self):
        """메트릭 수집"""
        self.start_time = time.time()
        elapsed = 0
        iteration = 0
        
        print(f"\n📊 메트릭 수집 시작")
        print(f"   기간: {self.duration}초")
        print(f"   간격: {self.interval}초")
        print(f"   출력: {self.output_dir}")
        print("-" * 60)
        
        while elapsed < self.duration:
            response = self.read_metrics()
            
            if response is None:
                time.sleep(self.interval)
                elapsed = time.time() - self.start_time
                continue
            
            iteration += 1
            current_time = time.time()
            elapsed = current_time - self.start_time
            
            # 메트릭 데이터 추출
            row = {
                'timestamp': datetime.now().isoformat(),
                'elapsed_time': elapsed,
                'iteration': iteration
            }
            
            for metric_family in response.metric_families:
                fn = metric_family.name
                
                # 주요 메트릭만 수집
                if '2d_local_trajectory_builder' in fn or 'constraint_builder' in fn or 'pose_graph' in fn:
                    for metric in metric_family.metrics:
                        value = self.extract_value(metric)
                        
                        if value is not None:
                            row[self.metric_name(fn, metric)] = value
                        elif metric.type == metric.TYPE_HISTOGRAM:
                            metric_name = self.metric_name(fn, metric)
                            for key, summary_value in self.histogram_summary(metric).items():
                                row[f"{metric_name}.{key}"] = summary_value
            
            self.data.append(row)
            
            progress = (elapsed / self.duration) * 100
            print(f"[{progress:5.1f}%] 반복 #{iteration:3d} - 수집된 항목: {len(row)-3}", end='\r')
            
            time.sleep(self.interval)
        
        print(f"\n✅ 수집 완료: {iteration}회, {len(self.data[-1])-3}개 메트릭")
    
    def save_csv(self):
        """CSV 파일로 저장"""
        if not self.data:
            print("⚠️  수집된 데이터가 없습니다")
            return
        
        csv_file = self.output_dir / f"metrics_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        
        # 모든 컬럼 수집
        all_columns = set()
        for row in self.data:
            all_columns.update(row.keys())
        
        columns = ['timestamp', 'elapsed_time', 'iteration'] + sorted([c for c in all_columns if c not in ['timestamp', 'elapsed_time', 'iteration']])
        
        with open(csv_file, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=columns)
            writer.writeheader()
            writer.writerows(self.data)
        
        print(f"💾 CSV 저장: {csv_file}")
        print(f"   행: {len(self.data)}")
        print(f"   열: {len(columns)}")
    
    def save_json(self):
        """JSON 파일로 저장"""
        if not self.data:
            print("⚠️  수집된 데이터가 없습니다")
            return
        
        json_file = self.output_dir / f"metrics_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
        
        with open(json_file, 'w') as f:
            json.dump(self.data, f, indent=2)
        
        print(f"💾 JSON 저장: {json_file}")
    
    def print_statistics(self):
        """통계 출력"""
        if not self.data:
            return
        
        print("\n" + "="*60)
        print("📈 통계")
        print("="*60)
        
        # 텍스트 기반 메트릭 찾기
        text_metrics = {}
        for row in self.data:
            for key, value in row.items():
                if key not in ['timestamp', 'elapsed_time', 'iteration']:
                    if isinstance(value, (int, float)):
                        if key not in text_metrics:
                            text_metrics[key] = []
                        text_metrics[key].append(value)
        
        # 통계 계산
        for metric_name, values in sorted(text_metrics.items()):
            if len(values) > 0:
                avg = sum(values) / len(values)
                min_val = min(values)
                max_val = max(values)
                
                print(f"\n📊 {metric_name}")
                print(f"   평균: {avg:.6f}")
                print(f"   최소: {min_val:.6f}")
                print(f"   최대: {max_val:.6f}")
                print(f"   데이터 포인트: {len(values)}")


def main():
    parser = argparse.ArgumentParser(description='Cartographer 메트릭 수집')
    parser.add_argument('--duration', type=int, default=60, help='수집 기간 (초)')
    parser.add_argument('--interval', type=float, default=1.0, help='수집 간격 (초)')
    parser.add_argument('--output', type=str, default='/home/symoon/Desktop/metric_data', help='출력 디렉토리')
    parser.add_argument('--csv-only', action='store_true', help='CSV만 저장')
    parser.add_argument('--json-only', action='store_true', help='JSON만 저장')
    
    args = parser.parse_args()
    
    rclpy.init()
    
    try:
        collector = MetricsCollector(
            output_dir=args.output,
            duration=args.duration,
            interval=args.interval
        )
        
        collector.collect()
        
        # 저장
        if not args.json_only:
            collector.save_csv()
        if not args.csv_only:
            collector.save_json()
        
        collector.print_statistics()
    
    except KeyboardInterrupt:
        print("\n\n수집 중단됨")
    except RuntimeError as e:
        print(f"\n❌ 에러: {e}")
        print("\n💡 해결방법:")
        print("   1. SLAM 프로세스가 실행 중인지 확인")
        print("   2. 또는 테스트 모드로 실행: python3 collect_metrics.py --test")
    finally:
        try:
            collector.destroy_node()
        except:
            pass
        rclpy.shutdown()


if __name__ == '__main__':
    main()
