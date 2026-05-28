#!/usr/bin/env python3
"""Record /tracked_pose samples for short output-similarity checks."""

import argparse
import csv
import json
import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, required=True)
    parser.add_argument("--topic", default="/tracked_pose")
    parser.add_argument("--output-csv", required=True)
    parser.add_argument("--summary-json", required=True)
    args = parser.parse_args()

    rclpy.init()
    node = Node("tracked_pose_recorder")
    rows = []
    start = time.monotonic()

    def callback(message):
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        pose = message.pose
        rows.append({
            "elapsed_wall": time.monotonic() - start,
            "stamp": stamp,
            "frame_id": message.header.frame_id,
            "x": pose.position.x,
            "y": pose.position.y,
            "z": pose.position.z,
            "qx": pose.orientation.x,
            "qy": pose.orientation.y,
            "qz": pose.orientation.z,
            "qw": pose.orientation.w,
            "yaw": yaw_from_quaternion(pose.orientation),
        })

    subscription = node.create_subscription(PoseStamped, args.topic, callback, 100)
    while rclpy.ok() and time.monotonic() - start < args.duration:
        rclpy.spin_once(node, timeout_sec=0.1)

    fieldnames = (
        "elapsed_wall", "stamp", "frame_id", "x", "y", "z",
        "qx", "qy", "qz", "qw", "yaw",
    )
    with open(args.output_csv, "w", newline="") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)

    path_length = 0.0
    for previous, current in zip(rows, rows[1:]):
        path_length += math.hypot(current["x"] - previous["x"],
                                  current["y"] - previous["y"])

    summary = {
        "samples": len(rows),
        "topic": args.topic,
        "path_length": path_length,
    }
    if rows:
        summary.update({
            "first_stamp": rows[0]["stamp"],
            "last_stamp": rows[-1]["stamp"],
            "duration_stamp": rows[-1]["stamp"] - rows[0]["stamp"],
            "first_pose": rows[0],
            "final_pose": rows[-1],
        })
    with open(args.summary_json, "w") as json_file:
        json.dump(summary, json_file, indent=2, sort_keys=True)
    print(json.dumps(summary, indent=2, sort_keys=True))

    node.destroy_subscription(subscription)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
