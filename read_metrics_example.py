#!/usr/bin/env python3
"""
Cartographer 메트릭 읽기 예제
ROS 2 서비스를 통해 실시간 메트릭 수집
"""

import rclpy
from rclpy.node import Node
from cartographer_ros_msgs.srv import ReadMetrics
import json
from datetime import datetime


class MetricsReader(Node):
    def __init__(self):
        super().__init__('metrics_reader')
        
        # ReadMetrics 서비스 클라이언트 생성
        self.client = self.create_client(ReadMetrics, '/cartographer_ros/read_metrics')
        
        # 서비스가 준비될 때까지 대기
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        
        self.get_logger().info('ReadMetrics service ready!')
    
    def read_metrics(self):
        """메트릭 읽기"""
        request = ReadMetrics.Request()
        
        # 동기 호출
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        try:
            response = future.result()
            return response
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')
            return None
    
    def print_metrics(self):
        """메트릭 출력"""
        response = self.read_metrics()
        
        if response is None:
            return
        
        print(f"\n{'='*60}")
        print(f"Timestamp: {datetime.now()}")
        print(f"Status: {response.status.code} - {response.status.message}")
        print(f"{'='*60}\n")
        
        # 메트릭 종류별로 출력
        for metric_family in response.metric_families:
            print(f"📊 {metric_family.name}")
            print(f"   Description: {metric_family.description}")
            print(f"   Type: {metric_family.type}")
            
            for metric in metric_family.metrics:
                if metric.labels:
                    labels_str = ", ".join([f"{l.name}='{l.value}'" for l in metric.labels])
                    print(f"   [{labels_str}]")
                
                # 메트릭 타입에 따라 값 출력
                if metric.type == metric.TYPE_COUNTER:
                    print(f"      Counter: {metric.value}")
                elif metric.type == metric.TYPE_GAUGE:
                    print(f"      Gauge: {metric.value}")
                elif metric.type == metric.TYPE_HISTOGRAM:
                    hist = metric.counts_by_bucket
                    print(f"      Histogram:")
                    print(f"         Buckets: {len(hist)}")
                    for bucket in hist:
                        print(f"            <= {bucket.upper_bound}: {bucket.cumulative_count}")
            print()
    
    def get_specific_metrics(self):
        """특정 메트릭만 추출"""
        response = self.read_metrics()
        if response is None:
            return
        
        metrics_dict = {}
        
        for metric_family in response.metric_families:
            # 2D Local SLAM 메트릭만 추출
            if '2d_local_trajectory_builder' in metric_family.name:
                for metric in metric_family.metrics:
                    metric_name = metric_family.name
                    
                    # 라벨이 있으면 라벨도 포함
                    if metric.labels:
                        labels_str = "_".join([f"{l.name}_{l.value}" for l in metric.labels])
                        metric_name = f"{metric_name}[{labels_str}]"
                    
                    # 값 추출
                    if metric.type in (metric.TYPE_GAUGE, metric.TYPE_COUNTER):
                        metrics_dict[metric_name] = metric.value
        
        # JSON 형식으로 출력
        print("\n📈 2D Local SLAM Metrics (JSON):")
        print(json.dumps(metrics_dict, indent=2))
        
        return metrics_dict


def main():
    rclpy.init()
    
    reader = MetricsReader()
    
    print("\n✅ MetricsReader 초기화 완료")
    print("옵션:")
    print("  1. 모든 메트릭 출력")
    print("  2. 2D Local SLAM 메트릭만 추출")
    
    try:
        choice = input("\n선택 (1 또는 2): ").strip()
        
        if choice == '1':
            reader.print_metrics()
        elif choice == '2':
            reader.get_specific_metrics()
        else:
            print("잘못된 선택")
        
        # 주기적으로 읽기 (optional)
        # while True:
        #     reader.print_metrics()
        #     time.sleep(1)
    
    except KeyboardInterrupt:
        print("\n\n중단됨")
    finally:
        reader.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
