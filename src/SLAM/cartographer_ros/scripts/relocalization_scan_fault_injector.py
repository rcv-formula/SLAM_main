#!/usr/bin/env python3
"""Relay LaserScan messages and corrupt them during a time window.

This is meant for relocalization testing with rosbag playback:
play the bag's /scan as /scan_fault_in, then relay it back to /scan.
"""

import math
import random

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


def stamp_to_sec(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


class RelocalizationScanFaultInjector(Node):
    def __init__(self):
        super().__init__("relocalization_scan_fault_injector")
        self.declare_parameter("input_topic", "/scan_fault_in")
        self.declare_parameter("output_topic", "/scan")
        self.declare_parameter("fault_start_sec", 20.0)
        self.declare_parameter("fault_duration_sec", 8.0)
        self.declare_parameter("mode", "shift")
        self.declare_parameter("shift_fraction", 0.5)
        self.declare_parameter("occlusion_fraction", 0.35)
        self.declare_parameter("occlusion_distance", 0.35)
        self.declare_parameter("noise_seed", 17)

        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value
        self.fault_start_sec = float(self.get_parameter("fault_start_sec").value)
        self.fault_duration_sec = float(
            self.get_parameter("fault_duration_sec").value)
        self.mode = str(self.get_parameter("mode").value)
        self.shift_fraction = float(self.get_parameter("shift_fraction").value)
        self.occlusion_fraction = float(
            self.get_parameter("occlusion_fraction").value)
        self.occlusion_distance = float(
            self.get_parameter("occlusion_distance").value)
        self.noise_rng = random.Random(int(self.get_parameter("noise_seed").value))
        self.first_stamp_sec = None
        self.was_faulting = False
        self.last_clean_scan = None

        self.publisher = self.create_publisher(LaserScan, output_topic, 10)
        self.subscription = self.create_subscription(
            LaserScan, input_topic, self.handle_scan, 10)
        self.get_logger().info(
            f"Relaying {input_topic} -> {output_topic}; mode={self.mode}, "
            f"fault=[{self.fault_start_sec}, "
            f"{self.fault_start_sec + self.fault_duration_sec}]s")

    def handle_scan(self, msg):
        stamp_sec = stamp_to_sec(msg.header.stamp)
        if self.first_stamp_sec is None:
            self.first_stamp_sec = stamp_sec
        elapsed = stamp_sec - self.first_stamp_sec
        faulting = (
            self.fault_start_sec <= elapsed <
            self.fault_start_sec + self.fault_duration_sec)

        if faulting != self.was_faulting:
            self.get_logger().warn(
                f"scan fault {'ON' if faulting else 'OFF'} at bag t={elapsed:.2f}s")
            self.was_faulting = faulting

        if not faulting:
            self.last_clean_scan = msg
            self.publisher.publish(msg)
            return

        if self.mode == "drop":
            return

        if self.mode == "freeze":
            if self.last_clean_scan is None:
                self.publisher.publish(msg)
                return
            out = LaserScan()
            out.header = msg.header
            out.angle_min = self.last_clean_scan.angle_min
            out.angle_max = self.last_clean_scan.angle_max
            out.angle_increment = self.last_clean_scan.angle_increment
            out.time_increment = self.last_clean_scan.time_increment
            out.scan_time = self.last_clean_scan.scan_time
            out.range_min = self.last_clean_scan.range_min
            out.range_max = self.last_clean_scan.range_max
            out.ranges = list(self.last_clean_scan.ranges)
            out.intensities = list(self.last_clean_scan.intensities)
            self.publisher.publish(out)
            return

        out = LaserScan()
        out.header = msg.header
        out.angle_min = msg.angle_min
        out.angle_max = msg.angle_max
        out.angle_increment = msg.angle_increment
        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time
        out.range_min = msg.range_min
        out.range_max = msg.range_max
        out.ranges = list(msg.ranges)
        out.intensities = list(msg.intensities)

        if self.mode == "reverse":
            out.ranges.reverse()
            out.intensities.reverse()
        elif self.mode == "invalid":
            out.ranges = [math.inf] * len(out.ranges)
        elif self.mode == "noise":
            low = max(float(out.range_min), 0.2)
            high = min(float(out.range_max), 8.0)
            if not math.isfinite(high) or high <= low:
                high = low + 5.0
            out.ranges = [
                self.noise_rng.uniform(low, high) for _ in out.ranges]
            if out.intensities:
                out.intensities = [0.0] * len(out.intensities)
        elif self.mode == "sparse_noise":
            low = max(float(out.range_min), 0.2)
            high = min(float(out.range_max), 8.0)
            if not math.isfinite(high) or high <= low:
                high = low + 5.0
            sparse_ranges = [math.inf] * len(out.ranges)
            sample_count = min(24, len(sparse_ranges))
            for index in self.noise_rng.sample(range(len(sparse_ranges)), sample_count):
                sparse_ranges[index] = self.noise_rng.uniform(low, high)
            out.ranges = sparse_ranges
            if out.intensities:
                out.intensities = [0.0] * len(out.intensities)
        elif self.mode == "front_occlusion":
            if out.ranges:
                fraction = min(max(self.occlusion_fraction, 0.0), 1.0)
                count = max(1, int(len(out.ranges) * fraction))
                center = len(out.ranges) // 2
                start = max(0, center - count // 2)
                end = min(len(out.ranges), start + count)
                distance = min(
                    max(self.occlusion_distance, float(out.range_min)),
                    float(out.range_max))
                for index in range(start, end):
                    out.ranges[index] = distance
        else:
            shift = int(len(out.ranges) * self.shift_fraction)
            if out.ranges:
                shift %= len(out.ranges)
                out.ranges = out.ranges[shift:] + out.ranges[:shift]
            if out.intensities:
                shift_i = shift % len(out.intensities)
                out.intensities = out.intensities[shift_i:] + out.intensities[:shift_i]

        self.publisher.publish(out)


def main():
    rclpy.init()
    node = RelocalizationScanFaultInjector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
