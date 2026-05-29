#!/usr/bin/env python3
"""Compare two tracked_pose CSV files by relative ROS timestamp."""

import argparse
import csv
import json
import math


def load_rows(path):
    with open(path, newline="") as csv_file:
        rows = list(csv.DictReader(csv_file))
    result = []
    for row in rows:
        stamp = row.get("stamp") or row.get("odom_stamp") or row.get(
            "tracked_pose_stamp")
        result.append({
            "stamp": float(stamp),
            "x": float(row["x"]),
            "y": float(row["y"]),
            "yaw": float(row["yaw"]),
        })
    return result


def wrap_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def relative_rows(rows):
    if not rows:
        return []
    first_stamp = rows[0]["stamp"]
    return [{**row, "relative_stamp": row["stamp"] - first_stamp}
            for row in rows]


def nearest_matches(reference, candidate, max_dt):
    matches = []
    candidate_index = 0
    for ref in reference:
        while (candidate_index + 1 < len(candidate) and
               candidate[candidate_index + 1]["relative_stamp"] <=
               ref["relative_stamp"]):
            candidate_index += 1
        choices = [candidate[candidate_index]]
        if candidate_index + 1 < len(candidate):
            choices.append(candidate[candidate_index + 1])
        best = min(
            choices,
            key=lambda row: abs(row["relative_stamp"] - ref["relative_stamp"]),
        )
        dt = best["relative_stamp"] - ref["relative_stamp"]
        if abs(dt) <= max_dt:
            matches.append((ref, best, dt))
    return matches


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--reference-csv", required=True)
    parser.add_argument("--candidate-csv", required=True)
    parser.add_argument("--max-dt", type=float, default=0.08)
    parser.add_argument("--max-relative-time", type=float)
    parser.add_argument("--summary-json", required=True)
    args = parser.parse_args()

    reference = relative_rows(load_rows(args.reference_csv))
    candidate = relative_rows(load_rows(args.candidate_csv))
    if args.max_relative_time is not None:
        reference = [
            row for row in reference
            if row["relative_stamp"] <= args.max_relative_time
        ]
        candidate = [
            row for row in candidate
            if row["relative_stamp"] <= args.max_relative_time
        ]
    matches = nearest_matches(reference, candidate, args.max_dt)

    xy_errors = []
    yaw_errors = []
    dts = []
    for ref, cand, dt in matches:
        xy_errors.append(math.hypot(cand["x"] - ref["x"],
                                    cand["y"] - ref["y"]))
        yaw_errors.append(abs(wrap_angle(cand["yaw"] - ref["yaw"])))
        dts.append(abs(dt))

    summary = {
        "reference_samples": len(reference),
        "candidate_samples": len(candidate),
        "matched_samples": len(matches),
        "max_match_dt": max(dts) if dts else None,
        "mean_match_dt": sum(dts) / len(dts) if dts else None,
    }
    if xy_errors:
        summary.update({
            "xy_rmse": math.sqrt(sum(error * error for error in xy_errors) /
                                 len(xy_errors)),
            "xy_mean": sum(xy_errors) / len(xy_errors),
            "xy_max": max(xy_errors),
            "yaw_mean": sum(yaw_errors) / len(yaw_errors),
            "yaw_max": max(yaw_errors),
        })
    if reference and candidate:
        final_xy = math.hypot(candidate[-1]["x"] - reference[-1]["x"],
                              candidate[-1]["y"] - reference[-1]["y"])
        summary.update({
            "final_xy_delta": final_xy,
            "final_yaw_delta": abs(wrap_angle(candidate[-1]["yaw"] -
                                              reference[-1]["yaw"])),
            "reference_duration": reference[-1]["relative_stamp"],
            "candidate_duration": candidate[-1]["relative_stamp"],
        })

    with open(args.summary_json, "w") as json_file:
        json.dump(summary, json_file, indent=2, sort_keys=True)
    print(json.dumps(summary, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
