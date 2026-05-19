#!/usr/bin/env python3
"""Analyze straight-section mapping stability from local quality CSV.

This is intentionally ground-truth free.  It measures whether the local SLAM
estimate stays smooth and self-consistent in feature-poor straight sections.
Lower lateral/yaw jitter and lower scan-vs-prediction disagreement are better.
"""

import argparse
import csv
import json
import math
import statistics
from pathlib import Path


DEFAULT_CSV = "/tmp/ver6_cartographer_local_quality.csv"


def finite_float(value, default=float("nan")):
    try:
        result = float(value)
    except (TypeError, ValueError):
        return default
    return result if math.isfinite(result) else default


def percentile(values, ratio):
    values = sorted(v for v in values if math.isfinite(v))
    if not values:
        return float("nan")
    index = int(round((len(values) - 1) * ratio))
    return values[max(0, min(index, len(values) - 1))]


def rms(values):
    values = [v for v in values if math.isfinite(v)]
    if not values:
        return float("nan")
    return math.sqrt(sum(v * v for v in values) / len(values))


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def read_rows(path):
    with open(path, newline="") as handle:
        reader = csv.DictReader(handle)
        rows = []
        for index, row in enumerate(reader):
            parsed = {"index": index}
            for key, value in row.items():
                parsed[key] = finite_float(value)
            rows.append(parsed)
    return rows


def is_straight_row(row, args):
    by_geometry = (
        row.get("geometry_degeneracy_ratio", float("nan"))
        >= args.ratio_threshold
        and row.get("num_filtered_points", 0.0) >= args.min_points
    )
    by_mode = row.get("featureless_straight_mode", 0.0) >= 0.5
    if args.mode_source == "geometry":
        return by_geometry
    if args.mode_source == "mode":
        return by_mode
    return by_geometry or by_mode


def valid_pose(row, prefix):
    keys = [f"{prefix}_x", f"{prefix}_y", f"{prefix}_yaw"]
    return all(math.isfinite(row.get(key, float("nan"))) for key in keys)


def consecutive_segments(rows, mask, min_rows):
    segments = []
    current = []
    for row, selected in zip(rows, mask):
        if selected and valid_pose(row, "pose_estimate"):
            current.append(row)
        else:
            if len(current) >= min_rows:
                segments.append(current)
            current = []
    if len(current) >= min_rows:
        segments.append(current)
    return segments


def pose_steps(segments, backward_epsilon):
    step_len = []
    lateral_step = []
    longitudinal_step = []
    yaw_step = []
    backward_count = 0
    total_count = 0

    for segment in segments:
        for prev, cur in zip(segment, segment[1:]):
            dx = cur["pose_estimate_x"] - prev["pose_estimate_x"]
            dy = cur["pose_estimate_y"] - prev["pose_estimate_y"]
            yaw = prev["pose_estimate_yaw"]
            forward_x = math.cos(yaw)
            forward_y = math.sin(yaw)
            lateral_x = -math.sin(yaw)
            lateral_y = math.cos(yaw)
            long = dx * forward_x + dy * forward_y
            lat = dx * lateral_x + dy * lateral_y

            step_len.append(math.hypot(dx, dy))
            longitudinal_step.append(long)
            lateral_step.append(lat)
            yaw_step.append(normalize_angle(cur["pose_estimate_yaw"] - yaw))
            if long < -backward_epsilon:
                backward_count += 1
            total_count += 1

    backward_ratio = backward_count / total_count if total_count else float("nan")
    return {
        "step_len": step_len,
        "lateral_step": lateral_step,
        "longitudinal_step": longitudinal_step,
        "yaw_step": yaw_step,
        "backward_ratio": backward_ratio,
    }


def correction_metrics(rows, mask):
    trans = []
    lateral = []
    longitudinal = []
    yaw = []
    for row, selected in zip(rows, mask):
        if not selected:
            continue
        if not (valid_pose(row, "pose_prediction") and valid_pose(row, "pose_estimate")):
            continue
        dx = row["pose_estimate_x"] - row["pose_prediction_x"]
        dy = row["pose_estimate_y"] - row["pose_prediction_y"]
        pred_yaw = row["pose_prediction_yaw"]
        forward_x = math.cos(pred_yaw)
        forward_y = math.sin(pred_yaw)
        lateral_x = -math.sin(pred_yaw)
        lateral_y = math.cos(pred_yaw)
        longitudinal.append(dx * forward_x + dy * forward_y)
        lateral.append(dx * lateral_x + dy * lateral_y)
        trans.append(math.hypot(dx, dy))
        yaw.append(normalize_angle(row["pose_estimate_yaw"] - pred_yaw))
    return {
        "prediction_correction_trans": trans,
        "prediction_correction_lateral": lateral,
        "prediction_correction_longitudinal": longitudinal,
        "prediction_correction_yaw": yaw,
    }


def line_fit_residuals(segments):
    residuals = []
    segment_summaries = []
    for segment in segments:
        points = [(r["pose_estimate_x"], r["pose_estimate_y"]) for r in segment]
        if len(points) < 2:
            continue
        mean_x = statistics.mean(x for x, _ in points)
        mean_y = statistics.mean(y for _, y in points)
        cxx = statistics.mean((x - mean_x) ** 2 for x, _ in points)
        cyy = statistics.mean((y - mean_y) ** 2 for _, y in points)
        cxy = statistics.mean((x - mean_x) * (y - mean_y) for x, y in points)

        principal_angle = 0.5 * math.atan2(2.0 * cxy, cxx - cyy)
        normal_x = -math.sin(principal_angle)
        normal_y = math.cos(principal_angle)
        segment_residuals = [
            abs((x - mean_x) * normal_x + (y - mean_y) * normal_y)
            for x, y in points
        ]
        residuals.extend(segment_residuals)
        segment_summaries.append(
            {
                "rows": len(segment),
                "start_index": segment[0]["index"],
                "end_index": segment[-1]["index"],
                "line_rms_m": rms(segment_residuals),
                "line_p95_m": percentile(segment_residuals, 0.95),
            }
        )
    return residuals, segment_summaries


def summarize_series(values):
    abs_values = [abs(v) for v in values if math.isfinite(v)]
    return {
        "mean_abs": statistics.mean(abs_values) if abs_values else float("nan"),
        "rms": rms(abs_values),
        "p50_abs": percentile(abs_values, 0.50),
        "p95_abs": percentile(abs_values, 0.95),
        "p99_abs": percentile(abs_values, 0.99),
        "max_abs": max(abs_values) if abs_values else float("nan"),
    }


def score_from_metrics(metrics, args):
    penalties = {
        "lateral_step": 25.0
        * min(2.0, metrics["lateral_step_p95_m"] / args.target_lateral_step_p95),
        "yaw_step": 20.0
        * min(2.0, metrics["yaw_step_p95_rad"] / args.target_yaw_step_p95),
        "prediction_correction": 25.0
        * min(
            2.0,
            metrics["prediction_correction_p95_m"]
            / args.target_prediction_correction_p95,
        ),
        "line_residual": 20.0
        * min(2.0, metrics["line_residual_p95_m"] / args.target_line_residual_p95),
        "jump": 10.0 * min(2.0, metrics["step_len_p95_m"] / args.target_step_len_p95),
        "backward": 20.0 * min(1.0, metrics["backward_ratio"] / 0.02),
    }
    score = max(0.0, 100.0 - sum(penalties.values()))
    return score, penalties


def analyze(path, args):
    rows = read_rows(path)
    if not rows:
        raise SystemExit(f"No rows in CSV: {path}")

    mask = [is_straight_row(row, args) for row in rows]
    segments = consecutive_segments(rows, mask, args.min_segment_rows)
    steps = pose_steps(segments, args.backward_epsilon)
    corrections = correction_metrics(rows, mask)
    line_residuals, segment_summaries = line_fit_residuals(segments)

    lateral_summary = summarize_series(steps["lateral_step"])
    yaw_summary = summarize_series(steps["yaw_step"])
    step_summary = summarize_series(steps["step_len"])
    correction_summary = summarize_series(corrections["prediction_correction_trans"])
    correction_yaw_summary = summarize_series(corrections["prediction_correction_yaw"])
    line_summary = summarize_series(line_residuals)

    metrics = {
        "rows_total": len(rows),
        "straight_rows": sum(mask),
        "straight_row_ratio": sum(mask) / len(rows),
        "straight_segments": len(segments),
        "lateral_step_rms_m": lateral_summary["rms"],
        "lateral_step_p95_m": lateral_summary["p95_abs"],
        "lateral_step_max_m": lateral_summary["max_abs"],
        "yaw_step_rms_rad": yaw_summary["rms"],
        "yaw_step_p95_rad": yaw_summary["p95_abs"],
        "yaw_step_max_rad": yaw_summary["max_abs"],
        "step_len_p95_m": step_summary["p95_abs"],
        "step_len_max_m": step_summary["max_abs"],
        "backward_ratio": steps["backward_ratio"],
        "prediction_correction_rms_m": correction_summary["rms"],
        "prediction_correction_p95_m": correction_summary["p95_abs"],
        "prediction_correction_max_m": correction_summary["max_abs"],
        "prediction_correction_yaw_p95_rad": correction_yaw_summary["p95_abs"],
        "line_residual_rms_m": line_summary["rms"],
        "line_residual_p95_m": line_summary["p95_abs"],
        "line_residual_max_m": line_summary["max_abs"],
    }
    score, penalties = score_from_metrics(metrics, args)
    metrics["straight_stability_score"] = score
    metrics["penalties"] = penalties
    metrics["segments"] = segment_summaries
    return metrics


def print_report(path, metrics):
    print(f"Straight stability report: {path}")
    print("=" * 72)
    print(f"score: {metrics['straight_stability_score']:.1f} / 100")
    print(
        "straight rows: "
        f"{metrics['straight_rows']} / {metrics['rows_total']} "
        f"({metrics['straight_row_ratio'] * 100:.1f}%), "
        f"segments: {metrics['straight_segments']}"
    )
    print()
    print("Stability metrics, lower is better")
    print(
        f"  lateral step p95/rms/max: "
        f"{metrics['lateral_step_p95_m']:.4f} / "
        f"{metrics['lateral_step_rms_m']:.4f} / "
        f"{metrics['lateral_step_max_m']:.4f} m"
    )
    print(
        f"  yaw step p95/rms/max: "
        f"{metrics['yaw_step_p95_rad']:.5f} / "
        f"{metrics['yaw_step_rms_rad']:.5f} / "
        f"{metrics['yaw_step_max_rad']:.5f} rad"
    )
    print(
        f"  step length p95/max: "
        f"{metrics['step_len_p95_m']:.4f} / {metrics['step_len_max_m']:.4f} m"
    )
    print(f"  backward step ratio: {metrics['backward_ratio'] * 100:.2f}%")
    print(
        f"  prediction correction p95/rms/max: "
        f"{metrics['prediction_correction_p95_m']:.4f} / "
        f"{metrics['prediction_correction_rms_m']:.4f} / "
        f"{metrics['prediction_correction_max_m']:.4f} m"
    )
    print(
        f"  prediction yaw correction p95: "
        f"{metrics['prediction_correction_yaw_p95_rad']:.5f} rad"
    )
    print(
        f"  fitted-line residual p95/rms/max: "
        f"{metrics['line_residual_p95_m']:.4f} / "
        f"{metrics['line_residual_rms_m']:.4f} / "
        f"{metrics['line_residual_max_m']:.4f} m"
    )
    print()
    print("Score penalties")
    for name, value in sorted(metrics["penalties"].items()):
        print(f"  {name}: -{value:.1f}")


def parse_args():
    parser = argparse.ArgumentParser(
        description="Analyze straight-section mapping stability."
    )
    parser.add_argument("csv", nargs="?", default=DEFAULT_CSV)
    parser.add_argument("--mode-source", choices=("geometry", "mode", "either"),
                        default="geometry")
    parser.add_argument("--ratio-threshold", type=float, default=6.0)
    parser.add_argument("--min-points", type=int, default=180)
    parser.add_argument("--min-segment-rows", type=int, default=20)
    parser.add_argument("--backward-epsilon", type=float, default=0.005)
    parser.add_argument("--target-lateral-step-p95", type=float, default=0.03)
    parser.add_argument("--target-yaw-step-p95", type=float, default=0.01)
    parser.add_argument("--target-prediction-correction-p95", type=float,
                        default=0.08)
    parser.add_argument("--target-line-residual-p95", type=float, default=0.12)
    parser.add_argument("--target-step-len-p95", type=float, default=0.08)
    parser.add_argument("--json-out")
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    csv_path = Path(args.csv)
    result = analyze(csv_path, args)
    print_report(csv_path, result)
    if args.json_out:
        with open(args.json_out, "w") as handle:
            json.dump(result, handle, indent=2, sort_keys=True)
        print(f"\nWrote JSON: {args.json_out}")
