#!/usr/bin/env python3
"""Analyze straight-section mapping stability from Cartographer local CSV.

The metric targets feature-poor straight corridors where scan matching can
slide along the corridor. It does not need ground truth. It measures whether
the estimated pose is internally stable:

* lateral wobble around the best-fit straight line
* yaw jitter after removing slow yaw drift
* per-frame pose jumps
* scan estimate disagreement with the IMU/extrapolator prediction
* reverse/backtracking along the straight direction

Raw metrics are the important part. The composite score is only for ranking
parameter trials against each other.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple


NumberRow = Dict[str, float]


def finite(value: float) -> bool:
    return math.isfinite(value)


def parse_float(value: object, default: float = math.nan) -> float:
    if value is None:
        return default
    if isinstance(value, (int, float)):
        return float(value)
    text = str(value).strip()
    if not text:
        return default
    try:
        return float(text)
    except ValueError:
        return default


def percentile(values: Sequence[float], q: float) -> float:
    clean = sorted(v for v in values if finite(v))
    if not clean:
        return math.nan
    if len(clean) == 1:
        return clean[0]
    position = q * (len(clean) - 1)
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return clean[lower]
    ratio = position - lower
    return clean[lower] * (1.0 - ratio) + clean[upper] * ratio


def mean(values: Sequence[float]) -> float:
    clean = [v for v in values if finite(v)]
    return sum(clean) / len(clean) if clean else math.nan


def rms(values: Sequence[float]) -> float:
    clean = [v for v in values if finite(v)]
    if not clean:
        return math.nan
    return math.sqrt(sum(v * v for v in clean) / len(clean))


def unwrap_angles(values: Sequence[float]) -> List[float]:
    if not values:
        return []
    unwrapped = [values[0]]
    previous = values[0]
    offset = 0.0
    for value in values[1:]:
        delta = value - previous
        while delta > math.pi:
            offset -= 2.0 * math.pi
            delta -= 2.0 * math.pi
        while delta < -math.pi:
            offset += 2.0 * math.pi
            delta += 2.0 * math.pi
        unwrapped.append(value + offset)
        previous = value
    return unwrapped


def angle_diff(a: float, b: float) -> float:
    diff = a - b
    while diff > math.pi:
        diff -= 2.0 * math.pi
    while diff < -math.pi:
        diff += 2.0 * math.pi
    return diff


def linear_detrend_residuals(times: Sequence[float],
                             values: Sequence[float]) -> List[float]:
    if len(times) != len(values) or len(values) < 2:
        baseline = mean(values)
        return [v - baseline for v in values]
    t0 = times[0]
    ts = [t - t0 for t in times]
    t_mean = mean(ts)
    v_mean = mean(values)
    denom = sum((t - t_mean) ** 2 for t in ts)
    if denom <= 1e-12:
        return [v - v_mean for v in values]
    slope = sum((t - t_mean) * (v - v_mean) for t, v in zip(ts, values)) / denom
    intercept = v_mean - slope * t_mean
    return [v - (intercept + slope * t) for t, v in zip(ts, values)]


def best_fit_line(points: Sequence[Tuple[float, float]]) -> Tuple[Tuple[float, float],
                                                                  Tuple[float, float]]:
    mx = mean([p[0] for p in points])
    my = mean([p[1] for p in points])
    centered = [(x - mx, y - my) for x, y in points]
    cxx = mean([x * x for x, _ in centered])
    cyy = mean([y * y for _, y in centered])
    cxy = mean([x * y for x, y in centered])
    angle = 0.5 * math.atan2(2.0 * cxy, cxx - cyy)
    forward = (math.cos(angle), math.sin(angle))
    if len(points) >= 2:
        net = (points[-1][0] - points[0][0], points[-1][1] - points[0][1])
        if net[0] * forward[0] + net[1] * forward[1] < 0.0:
            forward = (-forward[0], -forward[1])
    return (mx, my), forward


def read_csv(path: Path) -> List[NumberRow]:
    rows: List[NumberRow] = []
    with path.open(newline="") as file:
        reader = csv.DictReader(file)
        for raw in reader:
            row = {key: parse_float(value) for key, value in raw.items()}
            rows.append(row)

    if not rows:
        return rows

    if "elapsed_time" in rows[0] and finite(rows[0]["elapsed_time"]):
        for row in rows:
            row["_time_sec"] = row["elapsed_time"]
            row["_run_id"] = 0.0
        return rows

    raw_stamps = [row.get("stamp", math.nan) for row in rows]
    finite_stamps = [v for v in raw_stamps if finite(v)]
    if not finite_stamps:
        for index, row in enumerate(rows):
            row["_time_sec"] = float(index)
            row["_run_id"] = 0.0
        return rows

    diffs = [
        abs(b - a)
        for a, b in zip(finite_stamps, finite_stamps[1:])
        if finite(a) and finite(b) and b != a
    ]
    typical_diff = percentile(diffs, 0.5) if diffs else 1.0
    # Cartographer common::ToUniversal is in 100 ns ticks.
    scale = 1e7 if typical_diff > 1000.0 else 1.0
    run_id = 0
    run_start: Optional[float] = None
    previous_stamp: Optional[float] = None
    reset_threshold = scale * 1.0
    for row in rows:
        stamp = row.get("stamp", math.nan)
        if finite(stamp):
            if previous_stamp is not None and stamp + reset_threshold < previous_stamp:
                run_id += 1
                run_start = stamp
            if run_start is None:
                run_start = stamp
            row["_time_sec"] = (stamp - run_start) / scale
            previous_stamp = stamp
        else:
            row["_time_sec"] = math.nan
        row["_run_id"] = float(run_id)
    return rows


def is_straight_candidate(row: NumberRow,
                          degeneracy_threshold: float,
                          min_points: int,
                          use_mode_only: bool) -> bool:
    mode = row.get("featureless_straight_mode", 0.0) > 0.5
    if use_mode_only:
        return mode
    ratio = row.get("geometry_degeneracy_ratio", math.nan)
    points = row.get("num_filtered_points", math.nan)
    geometry = (
        finite(ratio)
        and finite(points)
        and ratio >= degeneracy_threshold
        and points >= min_points
    )
    return mode or geometry


def split_segments(rows: Sequence[NumberRow],
                   degeneracy_threshold: float,
                   min_points: int,
                   min_segment_sec: float,
                   min_segment_samples: int,
                   max_gap_sec: float,
                   use_mode_only: bool) -> List[List[NumberRow]]:
    segments: List[List[NumberRow]] = []
    current: List[NumberRow] = []
    previous_time: Optional[float] = None
    previous_run_id: Optional[float] = None

    for row in rows:
        pose_ok = all(
            finite(row.get(key, math.nan))
            for key in ("pose_estimate_x", "pose_estimate_y", "pose_estimate_yaw")
        )
        straight = pose_ok and is_straight_candidate(
            row, degeneracy_threshold, min_points, use_mode_only)
        time_sec = row.get("_time_sec", math.nan)
        gap = (
            finite(time_sec)
            and previous_time is not None
            and time_sec - previous_time > max_gap_sec
        )
        run_changed = previous_run_id is not None and row.get("_run_id") != previous_run_id
        if not straight or gap or run_changed:
            if current:
                segments.append(current)
            current = []
        if straight:
            current.append(row)
            if finite(time_sec):
                previous_time = time_sec
        elif finite(time_sec):
            previous_time = time_sec
        previous_run_id = row.get("_run_id")

    if current:
        segments.append(current)

    filtered: List[List[NumberRow]] = []
    for segment in segments:
        duration = segment[-1]["_time_sec"] - segment[0]["_time_sec"]
        if len(segment) >= min_segment_samples and duration >= min_segment_sec:
            filtered.append(segment)
    return filtered


def value_stats(rows: Sequence[NumberRow], key: str,
                absolute: bool = False) -> Dict[str, float]:
    values = [row.get(key, math.nan) for row in rows]
    if absolute:
        values = [abs(v) for v in values]
    return {
        "mean": mean(values),
        "p50": percentile(values, 0.50),
        "p95": percentile(values, 0.95),
        "max": max((v for v in values if finite(v)), default=math.nan),
    }


def analyze_segment(segment: Sequence[NumberRow], index: int) -> Dict[str, float]:
    times = [row["_time_sec"] for row in segment]
    xs = [row["pose_estimate_x"] for row in segment]
    ys = [row["pose_estimate_y"] for row in segment]
    yaws = [row["pose_estimate_yaw"] for row in segment]
    points = list(zip(xs, ys))
    center, forward = best_fit_line(points)
    lateral_axis = (-forward[1], forward[0])

    centered = [(x - center[0], y - center[1]) for x, y in points]
    longitudinal = [x * forward[0] + y * forward[1] for x, y in centered]
    lateral = [x * lateral_axis[0] + y * lateral_axis[1] for x, y in centered]
    dlong = [b - a for a, b in zip(longitudinal, longitudinal[1:])]
    steps = [
        math.hypot(bx - ax, by - ay)
        for (ax, ay), (bx, by) in zip(points, points[1:])
    ]
    reverse_distance = sum(abs(v) for v in dlong if v < 0.0)
    total_distance = sum(abs(v) for v in dlong)
    reverse_ratio = reverse_distance / total_distance if total_distance > 1e-9 else 0.0

    unwrapped_yaws = unwrap_angles(yaws)
    yaw_residuals = linear_detrend_residuals(times, unwrapped_yaws)
    yaw_steps = [abs(angle_diff(b, a)) for a, b in zip(yaws, yaws[1:])]

    pred_long_error: List[float] = []
    pred_lat_error: List[float] = []
    pred_yaw_error: List[float] = []
    for row in segment:
        required = (
            "pose_prediction_x",
            "pose_prediction_y",
            "pose_prediction_yaw",
            "pose_estimate_x",
            "pose_estimate_y",
            "pose_estimate_yaw",
        )
        if not all(finite(row.get(key, math.nan)) for key in required):
            continue
        yaw = row["pose_prediction_yaw"]
        fx, fy = math.cos(yaw), math.sin(yaw)
        lx, ly = -fy, fx
        dx = row["pose_estimate_x"] - row["pose_prediction_x"]
        dy = row["pose_estimate_y"] - row["pose_prediction_y"]
        pred_long_error.append(dx * fx + dy * fy)
        pred_lat_error.append(dx * lx + dy * ly)
        pred_yaw_error.append(
            angle_diff(row["pose_estimate_yaw"], row["pose_prediction_yaw"]))

    duration = times[-1] - times[0] if len(times) >= 2 else 0.0
    distance = sum(steps)
    lateral_p95 = percentile([abs(v) for v in lateral], 0.95)
    yaw_jitter_p95 = percentile([abs(v) for v in yaw_residuals], 0.95)
    step_p95 = percentile(steps, 0.95)
    pred_lat_p95 = percentile([abs(v) for v in pred_lat_error], 0.95)
    pred_long_p95 = percentile([abs(v) for v in pred_long_error], 0.95)

    # Tuned for ranking trials, not for pass/fail. Raw metrics above remain
    # the authoritative values.
    instability_index = 0.0
    for value, scale, weight in (
        (lateral_p95, 0.15, 1.2),
        (yaw_jitter_p95, 0.05, 1.0),
        (step_p95, 0.08, 0.8),
        (pred_lat_p95, 0.10, 0.8),
        (pred_long_p95, 0.20, 0.5),
        (reverse_ratio, 0.05, 1.0),
    ):
        if finite(value):
            instability_index += weight * min(value / scale, 10.0)
    stability_score = 100.0 / (1.0 + instability_index)

    result: Dict[str, float] = {
        "segment_index": float(index),
        "run_id": segment[0].get("_run_id", 0.0),
        "samples": float(len(segment)),
        "start_sec": times[0],
        "end_sec": times[-1],
        "duration_sec": duration,
        "path_len_m": distance,
        "net_progress_m": longitudinal[-1] - longitudinal[0],
        "reverse_ratio": reverse_ratio,
        "lateral_rms_m": rms(lateral),
        "lateral_p95_m": lateral_p95,
        "lateral_max_m": percentile([abs(v) for v in lateral], 1.0),
        "yaw_jitter_rms_rad": rms(yaw_residuals),
        "yaw_jitter_p95_rad": yaw_jitter_p95,
        "yaw_jitter_p95_deg": math.degrees(yaw_jitter_p95)
        if finite(yaw_jitter_p95) else math.nan,
        "yaw_step_p95_rad": percentile(yaw_steps, 0.95),
        "yaw_step_max_rad": percentile(yaw_steps, 1.0),
        "step_p95_m": step_p95,
        "step_max_m": percentile(steps, 1.0),
        "prediction_long_error_rms_m": rms(pred_long_error),
        "prediction_long_error_p95_m": pred_long_p95,
        "prediction_lat_error_rms_m": rms(pred_lat_error),
        "prediction_lat_error_p95_m": pred_lat_p95,
        "prediction_yaw_error_rms_rad": rms(pred_yaw_error),
        "prediction_yaw_error_p95_rad": percentile(
            [abs(v) for v in pred_yaw_error], 0.95),
        "translation_residual_p95_m": value_stats(
            segment, "translation_residual", absolute=True)["p95"],
        "rotation_residual_p95_rad": value_stats(
            segment, "rotation_residual", absolute=True)["p95"],
        "ceres_cost_p50": value_stats(segment, "ceres_final_cost")["p50"],
        "ceres_cost_p95": value_stats(segment, "ceres_final_cost")["p95"],
        "rt_score_p50": value_stats(segment, "rt_correlative_score")["p50"],
        "degeneracy_ratio_mean": value_stats(
            segment, "geometry_degeneracy_ratio")["mean"],
        "insertion_rate": mean([
            row.get("inserted_to_submap", math.nan) for row in segment
        ]),
        "straight_instability_index": instability_index,
        "straight_stability_score": stability_score,
    }
    return result


def summarize(rows: Sequence[NumberRow],
              segments: Sequence[Sequence[NumberRow]],
              segment_metrics: Sequence[Dict[str, float]]) -> Dict[str, object]:
    straight_samples = sum(len(segment) for segment in segments)
    total_samples = len(rows)
    runs = len(set(row.get("_run_id", 0.0) for row in rows))
    if not segment_metrics:
        return {
            "total_samples": total_samples,
            "runs": runs,
            "straight_samples": straight_samples,
            "straight_sample_ratio": straight_samples / total_samples
            if total_samples else 0.0,
            "segments": 0,
        }

    weights = [m["duration_sec"] for m in segment_metrics]
    if sum(weights) <= 0.0:
        weights = [m["samples"] for m in segment_metrics]

    def weighted_metric(key: str) -> float:
        pairs = [
            (m[key], w)
            for m, w in zip(segment_metrics, weights)
            if finite(m.get(key, math.nan)) and w > 0.0
        ]
        total = sum(w for _, w in pairs)
        return sum(v * w for v, w in pairs) / total if total else math.nan

    worst = max(segment_metrics,
                key=lambda item: item.get("straight_instability_index", 0.0))
    return {
        "total_samples": total_samples,
        "runs": runs,
        "straight_samples": straight_samples,
        "straight_sample_ratio": straight_samples / total_samples
        if total_samples else 0.0,
        "segments": len(segment_metrics),
        "straight_duration_sec": sum(m["duration_sec"] for m in segment_metrics),
        "weighted_stability_score": weighted_metric("straight_stability_score"),
        "weighted_instability_index": weighted_metric(
            "straight_instability_index"),
        "weighted_lateral_p95_m": weighted_metric("lateral_p95_m"),
        "weighted_yaw_jitter_p95_deg": weighted_metric(
            "yaw_jitter_p95_deg"),
        "weighted_step_p95_m": weighted_metric("step_p95_m"),
        "weighted_prediction_lat_error_p95_m": weighted_metric(
            "prediction_lat_error_p95_m"),
        "weighted_prediction_long_error_p95_m": weighted_metric(
            "prediction_long_error_p95_m"),
        "weighted_reverse_ratio": weighted_metric("reverse_ratio"),
        "worst_segment_index": worst["segment_index"],
        "worst_segment_score": worst["straight_stability_score"],
        "worst_segment_start_sec": worst["start_sec"],
        "worst_segment_end_sec": worst["end_sec"],
    }


def analyze_file(path: Path, args: argparse.Namespace) -> Dict[str, object]:
    rows = read_csv(path)
    segments = split_segments(
        rows,
        degeneracy_threshold=args.degeneracy_threshold,
        min_points=args.min_points,
        min_segment_sec=args.min_segment_sec,
        min_segment_samples=args.min_segment_samples,
        max_gap_sec=args.max_gap_sec,
        use_mode_only=args.use_mode_only,
    )
    metrics = [analyze_segment(segment, i) for i, segment in enumerate(segments)]
    return {
        "file": str(path),
        "config": {
            "degeneracy_threshold": args.degeneracy_threshold,
            "min_points": args.min_points,
            "min_segment_sec": args.min_segment_sec,
            "min_segment_samples": args.min_segment_samples,
            "max_gap_sec": args.max_gap_sec,
            "use_mode_only": args.use_mode_only,
        },
        "summary": summarize(rows, segments, metrics),
        "segments": metrics,
    }


def fmt(value: object, precision: int = 3) -> str:
    if isinstance(value, float):
        if not finite(value):
            return "nan"
        return f"{value:.{precision}f}"
    return str(value)


def print_report(result: Dict[str, object], top_segments: int) -> None:
    summary = result["summary"]
    assert isinstance(summary, dict)
    print(f"\nStraight mapping stability: {result['file']}")
    print("=" * 78)
    print(f"total_samples={summary.get('total_samples')} "
          f"runs={summary.get('runs')} "
          f"straight_samples={summary.get('straight_samples')} "
          f"straight_ratio={fmt(summary.get('straight_sample_ratio'))} "
          f"segments={summary.get('segments')}")
    if not result["segments"]:
        print("No straight segments matched the thresholds.")
        return
    print(f"straight_duration={fmt(summary.get('straight_duration_sec'))}s "
          f"stability_score={fmt(summary.get('weighted_stability_score'))}/100 "
          f"instability_index={fmt(summary.get('weighted_instability_index'))}")
    print(f"lateral_p95={fmt(summary.get('weighted_lateral_p95_m'))}m "
          f"yaw_jitter_p95={fmt(summary.get('weighted_yaw_jitter_p95_deg'))}deg "
          f"step_p95={fmt(summary.get('weighted_step_p95_m'))}m")
    print(f"prediction_error_p95: "
          f"long={fmt(summary.get('weighted_prediction_long_error_p95_m'))}m "
          f"lat={fmt(summary.get('weighted_prediction_lat_error_p95_m'))}m "
          f"reverse_ratio={fmt(summary.get('weighted_reverse_ratio'))}")
    print(f"worst_segment={fmt(summary.get('worst_segment_index'), 0)} "
          f"time={fmt(summary.get('worst_segment_start_sec'))}-"
          f"{fmt(summary.get('worst_segment_end_sec'))}s "
          f"score={fmt(summary.get('worst_segment_score'))}/100")

    segments = sorted(
        result["segments"],
        key=lambda item: item.get("straight_instability_index", 0.0),
        reverse=True,
    )
    print("\nWorst straight segments")
    print("idx run  time[s]        score  lat_p95  yaw_p95deg  step_p95  pred_lat  pred_long  reverse")
    for segment in segments[:top_segments]:
        print(
            f"{int(segment['segment_index']):>3}  "
            f"{int(segment['run_id']):>3}  "
            f"{fmt(segment['start_sec']):>6}-{fmt(segment['end_sec']):<6}  "
            f"{fmt(segment['straight_stability_score']):>6}  "
            f"{fmt(segment['lateral_p95_m']):>7}  "
            f"{fmt(segment['yaw_jitter_p95_deg']):>10}  "
            f"{fmt(segment['step_p95_m']):>8}  "
            f"{fmt(segment['prediction_lat_error_p95_m']):>8}  "
            f"{fmt(segment['prediction_long_error_p95_m']):>9}  "
            f"{fmt(segment['reverse_ratio']):>7}"
        )


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Analyze straight-section mapping stability from CSV.")
    parser.add_argument(
        "csv_files",
        nargs="*",
        default=["/tmp/ver6_cartographer_local_quality.csv"],
        help="Cartographer local quality CSV files.")
    parser.add_argument("--degeneracy-threshold", type=float, default=6.0)
    parser.add_argument("--min-points", type=int, default=180)
    parser.add_argument("--min-segment-sec", type=float, default=1.0)
    parser.add_argument("--min-segment-samples", type=int, default=20)
    parser.add_argument("--max-gap-sec", type=float, default=0.25)
    parser.add_argument(
        "--use-mode-only",
        action="store_true",
        help="Use only featureless_straight_mode instead of geometry threshold.")
    parser.add_argument("--top-segments", type=int, default=5)
    parser.add_argument("--json", action="store_true", help="Print JSON only.")
    parser.add_argument("--output-json", help="Write JSON summary to this file.")
    args = parser.parse_args()

    results = [analyze_file(Path(path), args) for path in args.csv_files]
    if args.output_json:
        output = Path(args.output_json)
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(results, indent=2), encoding="utf-8")

    if args.json:
        print(json.dumps(results, indent=2))
        return 0

    for result in results:
        print_report(result, args.top_segments)

    if len(results) > 1:
        print("\nComparison")
        print("file  score  lat_p95  yaw_p95deg  step_p95  pred_lat  reverse")
        for result in results:
            summary = result["summary"]
            assert isinstance(summary, dict)
            print(
                f"{Path(str(result['file'])).name}  "
                f"{fmt(summary.get('weighted_stability_score'))}  "
                f"{fmt(summary.get('weighted_lateral_p95_m'))}  "
                f"{fmt(summary.get('weighted_yaw_jitter_p95_deg'))}  "
                f"{fmt(summary.get('weighted_step_p95_m'))}  "
                f"{fmt(summary.get('weighted_prediction_lat_error_p95_m'))}  "
                f"{fmt(summary.get('weighted_reverse_ratio'))}"
            )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
