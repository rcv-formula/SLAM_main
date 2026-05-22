#!/usr/bin/env python3
"""Summarize Cartographer pose-graph constraint yaw metrics."""

import argparse
import csv
import math
from pathlib import Path


def percentile(values, pct):
    if not values:
        return 0.0
    ordered = sorted(values)
    index = (len(ordered) - 1) * pct
    lower = int(math.floor(index))
    upper = int(math.ceil(index))
    if lower == upper:
        return ordered[lower]
    ratio = index - lower
    return ordered[lower] * (1.0 - ratio) + ordered[upper] * ratio


def as_float(row, name, default=0.0):
    try:
        return float(row.get(name, default) or default)
    except ValueError:
        return default


def summarize(path):
    rows = []
    with path.open(newline="") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            if row.get("match_full_submap") != "1":
                continue
            rows.append(row)

    accepted = [row for row in rows if row.get("status") == "accepted"]
    rejected = [row for row in rows if row.get("status") == "fast_rejected"]
    yaw_deg = [
        math.degrees(abs(as_float(row, "initial_to_final_yaw")))
        for row in accepted
    ]
    fast_yaw_deg = [
        math.degrees(
            abs(as_float(row, "fast_pose_yaw") - as_float(row, "initial_pose_yaw"))
        )
        for row in accepted
    ]
    margins = [as_float(row, "fcsm_top1_top2_margin") for row in accepted]
    near_top = [as_float(row, "fcsm_near_top_0p02") for row in accepted]
    scores = [as_float(row, "score") for row in accepted]

    print(f"file: {path}")
    print(f"global attempts: {len(rows)}")
    print(f"accepted: {len(accepted)}")
    print(f"rejected: {len(rejected)}")
    if not accepted:
        return

    print("accepted initial_to_final_yaw_deg:")
    print(f"  mean: {sum(yaw_deg) / len(yaw_deg):.3f}")
    print(f"  p50 : {percentile(yaw_deg, 0.50):.3f}")
    print(f"  p90 : {percentile(yaw_deg, 0.90):.3f}")
    print(f"  p95 : {percentile(yaw_deg, 0.95):.3f}")
    print(f"  max : {max(yaw_deg):.3f}")
    print("fast_match_yaw_delta_deg:")
    print(f"  p90 : {percentile(fast_yaw_deg, 0.90):.3f}")
    print(f"  max : {max(fast_yaw_deg):.3f}")
    print("score:")
    print(f"  mean: {sum(scores) / len(scores):.4f}")
    print(f"  min : {min(scores):.4f}")
    print("ambiguity:")
    print(f"  top1_top2_margin_p50: {percentile(margins, 0.50):.5f}")
    print(f"  near_top_0p02_p50   : {percentile(near_top, 0.50):.1f}")


def main():
    parser = argparse.ArgumentParser(
        description="Summarize yaw-only global constraint metrics."
    )
    parser.add_argument("csv", type=Path)
    args = parser.parse_args()
    summarize(args.csv)


if __name__ == "__main__":
    main()
