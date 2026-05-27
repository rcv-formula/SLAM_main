#!/usr/bin/env python3
import csv
import math
import sys
from collections import Counter


def as_float(row, key, default=0.0):
    try:
        value = float(row.get(key, "") or default)
        return value if math.isfinite(value) else default
    except ValueError:
        return default


def percentile(values, p):
    if not values:
        return float("nan")
    values = sorted(values)
    index = min(len(values) - 1, max(0, int(round((len(values) - 1) * p))))
    return values[index]


def summarize(rows, title):
    accepted = [row for row in rows if row.get("status") == "accepted"]
    rejected = [row for row in rows if row.get("status") != "accepted"]
    print(f"\n[{title}]")
    print(f"rows={len(rows)} accepted={len(accepted)} rejected={len(rejected)}")
    print("status:", dict(Counter(row.get("status", "") for row in rows)))
    print(
        "match_full_submap:",
        dict(Counter(row.get("match_full_submap", "") for row in rows)),
    )
    if rows and "ambiguous_downweighted" in rows[0]:
        print(
            "ambiguous_downweighted:",
            dict(Counter(row.get("ambiguous_downweighted", "") for row in accepted)),
        )
    if accepted:
        scores = [as_float(row, "score") for row in accepted]
        trans = [as_float(row, "initial_to_final_translation") for row in accepted]
        yaw = [as_float(row, "initial_to_final_yaw") for row in accepted]
        print(
            "accepted score min/p50/p90/max:",
            f"{min(scores):.4f}",
            f"{percentile(scores, 0.50):.4f}",
            f"{percentile(scores, 0.90):.4f}",
            f"{max(scores):.4f}",
        )
        print(
            "initial_to_final_translation min/p50/p90/max:",
            f"{min(trans):.4f}",
            f"{percentile(trans, 0.50):.4f}",
            f"{percentile(trans, 0.90):.4f}",
            f"{max(trans):.4f}",
        )
        print(
            "initial_to_final_yaw min/p50/p90/max:",
            f"{min(yaw):.4f}",
            f"{percentile(yaw, 0.50):.4f}",
            f"{percentile(yaw, 0.90):.4f}",
            f"{max(yaw):.4f}",
        )
        suspicious = sorted(
            accepted,
            key=lambda row: (
                as_float(row, "initial_to_final_translation"),
                as_float(row, "initial_to_final_yaw"),
            ),
            reverse=True,
        )[:20]
        print("\nTop accepted constraints by initial_to_final_translation:")
        for row in suspicious:
            print(
                "submap={}/{} node={}/{} full={} score={:.4f} "
                "dtrans={:.3f} dyaw={:.3f} outlier={} weight={:.1f}/{:.1f}".format(
                    row.get("submap_trajectory_id"),
                    row.get("submap_index"),
                    row.get("node_trajectory_id"),
                    row.get("node_index"),
                    row.get("match_full_submap"),
                    as_float(row, "score"),
                    as_float(row, "initial_to_final_translation"),
                    as_float(row, "initial_to_final_yaw"),
                    row.get("is_outlier"),
                    as_float(row, "translation_weight"),
                    as_float(row, "rotation_weight"),
                )
            )


def main():
    if len(sys.argv) != 2:
        print("usage: analyze_pose_graph_constraints.py POSE_GRAPH_CONSTRAINTS.csv")
        return 2
    path = sys.argv[1]
    with open(path, newline="") as stream:
        rows = list(csv.DictReader(stream))
    summarize(rows, path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
