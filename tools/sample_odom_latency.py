#!/usr/bin/env python3
"""Sample /odom timestamp latency and related input ages."""

import argparse
import csv
import json
import math
import statistics
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, LaserScan


def stamp_to_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def percentile(values, ratio):
    if not values:
        return None
    sorted_values = sorted(values)
    return sorted_values[int(ratio * (len(sorted_values) - 1))]


def summarize(rows):
    numeric_keys = [
        "odom_latency_sec",
        "tracked_pose_latency_sec",
        "scan_age_sec",
        "imu_age_sec",
        "odom_wheel_age_sec",
        "tracked_to_odom_stamp_delta_sec",
    ]
    summary = {"samples": len(rows)}
    for key in numeric_keys:
        values = [float(row[key]) for row in rows if row.get(key) != ""]
        if not values:
            continue
        summary[key] = {
            "mean": statistics.fmean(values),
            "min": min(values),
            "max": max(values),
            "p50": statistics.median(values),
            "p90": percentile(values, 0.90),
            "p95": percentile(values, 0.95),
        }
    monotonic_errors = 0
    previous_stamp = None
    for row in rows:
        stamp = float(row["odom_stamp"])
        if previous_stamp is not None and stamp < previous_stamp:
            monotonic_errors += 1
        previous_stamp = stamp
    summary["odom_stamp_monotonic_errors"] = monotonic_errors
    if rows:
        summary["first_odom_stamp"] = float(rows[0]["odom_stamp"])
        summary["last_odom_stamp"] = float(rows[-1]["odom_stamp"])
        summary["duration_stamp"] = (
            summary["last_odom_stamp"] - summary["first_odom_stamp"])
        summary["final_pose"] = {
            "x": float(rows[-1]["x"]),
            "y": float(rows[-1]["y"]),
            "yaw": float(rows[-1]["yaw"]),
        }
    return summary


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, required=True)
    parser.add_argument("--output-csv", required=True)
    parser.add_argument("--summary-json", required=True)
    args = parser.parse_args()

    rclpy.init()
    node = rclpy.create_node("odom_latency_sampler")
    best_effort_qos = QoSProfile(depth=100)
    best_effort_qos.reliability = ReliabilityPolicy.BEST_EFFORT
    best_effort_qos.durability = DurabilityPolicy.VOLATILE
    rows = []
    latest_clock = None
    latest_stamps = {
        "tracked_pose": None,
        "scan": None,
        "imu": None,
        "odom_wheel": None,
    }

    def now_sec():
        if latest_clock is not None:
            return latest_clock
        return node.get_clock().now().nanoseconds * 1e-9

    def age(current_time, key):
        stamp = latest_stamps[key]
        if stamp is None:
            return ""
        return current_time - stamp

    def clock_callback(message):
        nonlocal latest_clock
        latest_clock = stamp_to_sec(message.clock)

    def tracked_pose_callback(message):
        latest_stamps["tracked_pose"] = stamp_to_sec(message.header.stamp)

    def scan_callback(message):
        latest_stamps["scan"] = stamp_to_sec(message.header.stamp)

    def imu_callback(message):
        latest_stamps["imu"] = stamp_to_sec(message.header.stamp)

    def odom_wheel_callback(message):
        latest_stamps["odom_wheel"] = stamp_to_sec(message.header.stamp)

    def odom_callback(message):
        current_time = now_sec()
        odom_stamp = stamp_to_sec(message.header.stamp)
        tracked_stamp = latest_stamps["tracked_pose"]
        pose = message.pose.pose
        row = {
            "elapsed_wall": time.monotonic() - start,
            "clock": current_time,
            "odom_stamp": odom_stamp,
            "odom_latency_sec": current_time - odom_stamp,
            "tracked_pose_stamp": tracked_stamp if tracked_stamp is not None else "",
            "tracked_pose_latency_sec": age(current_time, "tracked_pose"),
            "scan_stamp": latest_stamps["scan"] if latest_stamps["scan"] is not None else "",
            "scan_age_sec": age(current_time, "scan"),
            "imu_stamp": latest_stamps["imu"] if latest_stamps["imu"] is not None else "",
            "imu_age_sec": age(current_time, "imu"),
            "odom_wheel_stamp": (latest_stamps["odom_wheel"]
                                  if latest_stamps["odom_wheel"] is not None
                                  else ""),
            "odom_wheel_age_sec": age(current_time, "odom_wheel"),
            "tracked_to_odom_stamp_delta_sec": (
                odom_stamp - tracked_stamp if tracked_stamp is not None else ""),
            "frame_id": message.header.frame_id,
            "child_frame_id": message.child_frame_id,
            "x": pose.position.x,
            "y": pose.position.y,
            "z": pose.position.z,
            "yaw": yaw_from_quaternion(pose.orientation),
        }
        rows.append(row)

    node.create_subscription(Clock, "/clock", clock_callback, best_effort_qos)
    node.create_subscription(PoseStamped, "/tracked_pose",
                             tracked_pose_callback, best_effort_qos)
    node.create_subscription(LaserScan, "/scan", scan_callback,
                             best_effort_qos)
    node.create_subscription(Imu, "/imu/data", imu_callback,
                             best_effort_qos)
    node.create_subscription(Odometry, "/odom_wheel",
                             odom_wheel_callback, best_effort_qos)
    node.create_subscription(Odometry, "/odom", odom_callback,
                             best_effort_qos)

    start = time.monotonic()
    while rclpy.ok() and time.monotonic() - start < args.duration:
        rclpy.spin_once(node, timeout_sec=0.05)

    fieldnames = [
        "elapsed_wall", "clock", "odom_stamp", "odom_latency_sec",
        "tracked_pose_stamp", "tracked_pose_latency_sec", "scan_stamp",
        "scan_age_sec", "imu_stamp", "imu_age_sec", "odom_wheel_stamp",
        "odom_wheel_age_sec", "tracked_to_odom_stamp_delta_sec",
        "frame_id", "child_frame_id", "x", "y", "z", "yaw",
    ]
    with open(args.output_csv, "w", newline="") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)

    summary = summarize(rows)
    with open(args.summary_json, "w") as json_file:
        json.dump(summary, json_file, indent=2, sort_keys=True)
    print(json.dumps(summary, indent=2, sort_keys=True))

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
