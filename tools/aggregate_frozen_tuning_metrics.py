#!/usr/bin/env python3

import argparse
import json
from pathlib import Path
import statistics


def safe_mean(values):
    values = [v for v in values if v is not None]
    return statistics.fmean(values) if values else None


def safe_median(values):
    values = [v for v in values if v is not None]
    return statistics.median(values) if values else None


def safe_min(values):
    values = [v for v in values if v is not None]
    return min(values) if values else None


def safe_max(values):
    values = [v for v in values if v is not None]
    return max(values) if values else None


def load_metric(path, section):
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)
    return data["run_name"], data[section]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("metrics", nargs="+", help="metrics.json files to aggregate")
    parser.add_argument("--section", default="steady_state", choices=["steady_state", "all_scans"])
    args = parser.parse_args()

    rows = []
    for metric_path in args.metrics:
        run_name, section = load_metric(metric_path, args.section)
        rows.append((run_name, section))

    def collect(key):
        return [section.get(key) for _, section in rows]

    aggregate = {
        "section": args.section,
        "runs": [run_name for run_name, _ in rows],
        "num_runs": len(rows),
        "worst_longest_no_candidate_streak": safe_max(collect("longest_no_candidate_streak")),
        "worst_no_candidate_ratio": safe_max(collect("no_candidate_ratio")),
        "worst_min_best_score": safe_min(collect("min_best_score")),
        "worst_p05_best_score": safe_min(collect("p05_best_score")),
        "best_accepted_ratio": safe_max(collect("accepted_ratio")),
        "mean_accepted_ratio": safe_mean(collect("accepted_ratio")),
        "mean_candidate_ratio": safe_mean(collect("candidate_ratio")),
        "mean_no_candidate_ratio": safe_mean(collect("no_candidate_ratio")),
        "median_p05_best_score": safe_median(collect("p05_best_score")),
        "mean_p05_best_score": safe_mean(collect("p05_best_score")),
        "median_p50_best_score": safe_median(collect("p50_best_score")),
        "mean_p50_best_score": safe_mean(collect("p50_best_score")),
        "worst_max_translation_correction_m": safe_max(collect("max_translation_correction_m")),
        "worst_max_rotation_correction_deg": safe_max(collect("max_rotation_correction_deg")),
    }

    print(json.dumps(aggregate, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
