#!/usr/bin/env python3

import argparse
import json
import math
import os
from pathlib import Path
import re
import shutil
import signal
import statistics
import subprocess
import sys
import time


REPO_ROOT = Path("/home/shin/Desktop/gpts/SLAM_main")
CONFIG_PATH = REPO_ROOT / "src/SLAM/cartographer_ros/configuration_files/Damvi_localization_config.lua"
PBSTREAM_PATH = Path("/home/shin/Downloads/0125_1.pbstream")
BAG_PATH = Path("/home/shin/Downloads/only_damvi")
RESULTS_ROOT = REPO_ROOT / "frozen_tuning_results"

SCAN_RE = re.compile(
    r"\[FrozenMatcherTune\]\[scan (?P<scan>\d+)\] "
    r"status=(?P<status>\S+) "
    r"apply_mode=(?P<apply_mode>\S+) "
    r"candidates=(?P<candidates_in>\d+)/(?P<candidates_eval>\d+) "
    r"matched_submap=(?P<matched_submap>.+?) "
    r"submap_dist=(?P<submap_dist>[-+0-9.]+)m "
    r"corr=(?P<corr_translation>[-+0-9.]+)m/(?P<corr_rotation_deg>[-+0-9.]+)deg"
    r"(?: score=(?P<score>[-+0-9.]+))?"
    r"(?: second=(?P<second_score>[-+0-9.]+) margin=(?P<margin>[-+0-9.]+))?"
    r"(?: variance=(?P<variance>[-+0-9.]+))?"
)


def percentile(values, fraction):
    if not values:
        return None
    if len(values) == 1:
        return values[0]
    position = (len(values) - 1) * fraction
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return values[lower]
    weight = position - lower
    return values[lower] * (1.0 - weight) + values[upper] * weight


def safe_mean(values):
    return statistics.fmean(values) if values else None


def parse_scan_log(log_path):
    records = []
    with open(log_path, "r", encoding="utf-8", errors="replace") as f:
      for line in f:
        match = SCAN_RE.search(line)
        if not match:
          continue
        groups = match.groupdict()
        records.append(
            {
                "scan": int(groups["scan"]),
                "status": groups["status"],
                "apply_mode": groups["apply_mode"],
                "candidates_in_search_radius": int(groups["candidates_in"]),
                "candidates_evaluated": int(groups["candidates_eval"]),
                "matched_submap": groups["matched_submap"],
                "submap_distance_m": float(groups["submap_dist"]),
                "correction_translation_m": float(groups["corr_translation"]),
                "correction_rotation_deg": float(groups["corr_rotation_deg"]),
                "score": float(groups["score"]) if groups["score"] is not None else None,
                "second_score": float(groups["second_score"]) if groups["second_score"] is not None else None,
                "margin": float(groups["margin"]) if groups["margin"] is not None else None,
                "variance": float(groups["variance"]) if groups["variance"] is not None else None,
            }
        )
    return records


def summarize(records, warmup_scans):
    if not records:
        return {
            "num_scans": 0,
            "status_counts": {},
            "accepted_count": 0,
            "accepted_ratio": None,
            "candidate_ratio": None,
            "no_candidate_ratio": None,
            "longest_no_candidate_streak": 0,
            "min_best_score": None,
            "p05_best_score": None,
            "p50_best_score": None,
            "mean_best_score": None,
            "max_translation_correction_m": None,
            "p95_translation_correction_m": None,
            "max_rotation_correction_deg": None,
            "p95_rotation_correction_deg": None,
        }

    if warmup_scans > 0:
        cutoff_scan = records[0]["scan"] + warmup_scans
        filtered = [r for r in records if r["scan"] >= cutoff_scan]
        if not filtered:
            filtered = records
    else:
        filtered = records

    status_counts = {}
    score_values = []
    translation_values = []
    rotation_values = []
    no_candidate_streak = 0
    longest_no_candidate_streak = 0
    accepted_count = 0
    candidate_count = 0
    no_candidate_count = 0

    for record in filtered:
        status_counts[record["status"]] = status_counts.get(record["status"], 0) + 1
        if record["status"] == "accepted":
            accepted_count += 1
        if record["candidates_in_search_radius"] > 0:
            candidate_count += 1
            no_candidate_streak = 0
        else:
            no_candidate_count += 1
            no_candidate_streak += 1
            longest_no_candidate_streak = max(longest_no_candidate_streak, no_candidate_streak)
        if record["score"] is not None and record["candidates_evaluated"] > 0:
            score_values.append(record["score"])
        if record["matched_submap"] != "none":
            translation_values.append(record["correction_translation_m"])
            rotation_values.append(record["correction_rotation_deg"])

    score_values.sort()
    translation_values.sort()
    rotation_values.sort()
    num_scans = len(filtered)
    return {
        "num_scans": num_scans,
        "status_counts": status_counts,
        "accepted_count": accepted_count,
        "accepted_ratio": (accepted_count / num_scans) if num_scans else None,
        "candidate_ratio": (candidate_count / num_scans) if num_scans else None,
        "no_candidate_ratio": (no_candidate_count / num_scans) if num_scans else None,
        "longest_no_candidate_streak": longest_no_candidate_streak,
        "min_best_score": score_values[0] if score_values else None,
        "p05_best_score": percentile(score_values, 0.05),
        "p50_best_score": percentile(score_values, 0.50),
        "mean_best_score": safe_mean(score_values),
        "max_translation_correction_m": translation_values[-1] if translation_values else None,
        "p95_translation_correction_m": percentile(translation_values, 0.95),
        "max_rotation_correction_deg": rotation_values[-1] if rotation_values else None,
        "p95_rotation_correction_deg": percentile(rotation_values, 0.95),
    }


def wait_for_phrase(log_path, phrase, timeout_seconds):
    deadline = time.time() + timeout_seconds
    while time.time() < deadline:
        if log_path.exists():
            text = log_path.read_text(encoding="utf-8", errors="replace")
            if phrase in text:
                return True
        time.sleep(0.5)
    return False


def stop_process_tree(process):
    if process.poll() is not None:
        return
    try:
        os.killpg(os.getpgid(process.pid), signal.SIGINT)
    except ProcessLookupError:
        return
    try:
        process.wait(timeout=10)
        return
    except subprocess.TimeoutExpired:
        pass
    try:
        os.killpg(os.getpgid(process.pid), signal.SIGTERM)
    except ProcessLookupError:
        return
    try:
        process.wait(timeout=10)
    except subprocess.TimeoutExpired:
        os.killpg(os.getpgid(process.pid), signal.SIGKILL)
        process.wait(timeout=5)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--run-name", required=True)
    parser.add_argument("--warmup-scans", type=int, default=50)
    parser.add_argument("--startup-timeout", type=int, default=40)
    parser.add_argument("--post-bag-wait", type=float, default=3.0)
    args = parser.parse_args()

    run_dir = RESULTS_ROOT / args.run_name
    run_dir.mkdir(parents=True, exist_ok=True)
    launch_log_path = run_dir / "launch.log"
    bag_log_path = run_dir / "bag.log"
    metrics_path = run_dir / "metrics.json"
    shutil.copy2(CONFIG_PATH, run_dir / "Damvi_localization_config.lua")

    launch_cmd = (
        "source /opt/ros/jazzy/setup.bash && "
        f"source {REPO_ROOT / 'install/setup.bash'} && "
        "ros2 launch cartographer_ros Damvi_carto_pure_launch.py "
        f"pbstream_file:={PBSTREAM_PATH}"
    )
    bag_cmd = (
        "source /opt/ros/jazzy/setup.bash && "
        f"source {REPO_ROOT / 'install/setup.bash'} && "
        f"ros2 bag play {BAG_PATH} --clock"
    )

    with open(launch_log_path, "w", encoding="utf-8") as launch_log:
        launch_process = subprocess.Popen(
            ["bash", "-lc", launch_cmd],
            cwd=REPO_ROOT,
            stdout=launch_log,
            stderr=subprocess.STDOUT,
            preexec_fn=os.setsid,
        )
    try:
        if not wait_for_phrase(launch_log_path, "Added trajectory with ID", args.startup_timeout):
            raise RuntimeError(
                f"Cartographer launch did not become ready within {args.startup_timeout}s. "
                f"Check {launch_log_path}."
            )

        with open(bag_log_path, "w", encoding="utf-8") as bag_log:
            bag_process = subprocess.Popen(
                ["bash", "-lc", bag_cmd],
                cwd=REPO_ROOT,
                stdout=bag_log,
                stderr=subprocess.STDOUT,
                preexec_fn=os.setsid,
            )
            bag_return_code = bag_process.wait(timeout=180)
            if bag_return_code != 0:
                raise RuntimeError(f"Bag playback exited with code {bag_return_code}. Check {bag_log_path}.")
        time.sleep(args.post_bag_wait)
    finally:
        stop_process_tree(launch_process)

    records = parse_scan_log(launch_log_path)
    if not records:
        raise RuntimeError(f"No FrozenMatcherTune scan logs found in {launch_log_path}.")

    metrics = {
        "run_name": args.run_name,
        "log_path": str(launch_log_path),
        "bag_log_path": str(bag_log_path),
        "config_path": str(CONFIG_PATH),
        "warmup_scans": args.warmup_scans,
        "all_scans": summarize(records, warmup_scans=0),
        "steady_state": summarize(records, warmup_scans=args.warmup_scans),
        "last_apply_mode": records[-1]["apply_mode"],
        "num_raw_scan_records": len(records),
    }
    with open(metrics_path, "w", encoding="utf-8") as f:
        json.dump(metrics, f, indent=2, sort_keys=True)

    print(json.dumps(metrics, indent=2, sort_keys=True))
    print(f"\nSaved metrics to {metrics_path}")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(130)
