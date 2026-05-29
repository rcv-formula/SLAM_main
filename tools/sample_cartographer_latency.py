#!/usr/bin/env python3
"""Sample Cartographer runtime latency metrics with bounded service calls."""

import argparse
import csv
import json
import statistics
import time

import rclpy
from rclpy.node import Node

from cartographer_ros_msgs.srv import ReadMetrics


METRIC_KEYS = (
    "mapping_2d_local_trajectory_builder_latency",
    "mapping_2d_local_trajectory_builder_real_time_ratio",
    "mapping_2d_local_trajectory_builder_cpu_real_time_ratio",
    "mapping_global_trajectory_builder_local_slam_results[type=MatchingResult]",
    "mapping_global_trajectory_builder_local_slam_results[type=InsertionResult]",
    "mapping_2d_pose_graph_work_queue_delay",
    "mapping_2d_pose_graph_work_queue_size",
)


def metric_name(family_name, metric):
    if not metric.labels:
        return family_name
    labels = "_".join(f"{label.key}={label.value}" for label in metric.labels)
    return f"{family_name}[{labels}]"


def read_once(node, client, timeout_sec):
    future = client.call_async(ReadMetrics.Request())
    deadline = time.monotonic() + timeout_sec
    while rclpy.ok() and time.monotonic() < deadline and not future.done():
        rclpy.spin_once(node, timeout_sec=0.03)
    if not future.done() or future.result() is None:
        return None

    metrics = {}
    for family in future.result().metric_families:
        for metric in family.metrics:
            if metric.type in (metric.TYPE_GAUGE, metric.TYPE_COUNTER):
                metrics[metric_name(family.name, metric)] = metric.value
    return metrics


def summarize(rows):
    numeric = {}
    for key in METRIC_KEYS:
        values = []
        for row in rows:
            value = row.get(key)
            if value in ("", None):
                continue
            value = float(value)
            if (key.endswith("_latency") or key.endswith("_ratio")) and value <= 0.0:
                continue
            values.append(value)
        if not values:
            continue
        sorted_values = sorted(values)
        numeric[key] = {
            "samples": len(values),
            "mean": statistics.fmean(values),
            "min": min(values),
            "max": max(values),
            "p50": statistics.median(values),
            "p90": sorted_values[int(0.90 * (len(sorted_values) - 1))],
            "p95": sorted_values[int(0.95 * (len(sorted_values) - 1))],
        }
        if "local_slam_results" in key:
            numeric[key]["final"] = values[-1]
    return numeric


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, required=True)
    parser.add_argument("--interval", type=float, default=0.5)
    parser.add_argument("--call-timeout", type=float, default=0.5)
    parser.add_argument("--output-csv", required=True)
    parser.add_argument("--summary-json", required=True)
    parser.add_argument(
        "--service",
        action="append",
        default=["/read_metrics", "/cartographer_ros/read_metrics"],
    )
    args = parser.parse_args()

    rclpy.init()
    node = Node("cartographer_latency_sampler")

    client = None
    service_name = None
    while rclpy.ok() and client is None:
        for candidate in args.service:
            candidate_client = node.create_client(ReadMetrics, candidate)
            if candidate_client.wait_for_service(timeout_sec=0.2):
                client = candidate_client
                service_name = candidate
                break
        if client is None:
            time.sleep(0.2)

    rows = []
    start = time.monotonic()
    while time.monotonic() - start < args.duration:
        loop_start = time.monotonic()
        row = {
            "elapsed_time": loop_start - start,
            "service": service_name,
            "ok": 0,
        }
        metrics = read_once(node, client, args.call_timeout)
        if metrics is not None:
            row["ok"] = 1
            for key in METRIC_KEYS:
                row[key] = metrics.get(key, "")
        rows.append(row)
        time.sleep(max(0.0, args.interval - (time.monotonic() - loop_start)))

    fieldnames = ("elapsed_time", "service", "ok") + METRIC_KEYS
    with open(args.output_csv, "w", newline="") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)

    summary = {
        "rows": len(rows),
        "successful_reads": sum(int(row["ok"]) for row in rows),
        "metrics": summarize(rows),
    }
    with open(args.summary_json, "w") as json_file:
        json.dump(summary, json_file, indent=2, sort_keys=True)

    print(json.dumps(summary, indent=2, sort_keys=True))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
