#!/usr/bin/env python3
"""Run a bounded Cartographer rosbag benchmark and collect metrics."""

import argparse
import csv
import json
import os
import signal
import shlex
import subprocess
import time
from pathlib import Path


SCRIPT_DIR = Path(__file__).resolve().parent


def shell_source_command(workspace, command):
    return (
        "source /opt/ros/jazzy/setup.bash && "
        f"source {shlex.quote(str(Path(workspace) / 'install/setup.bash'))} && "
        f"exec {command}"
    )


def read_process_table():
    output = subprocess.check_output(
        ["ps", "-eo", "pid=,ppid=,pcpu=,pmem=,rss=,comm="],
        text=True,
    )
    table = {}
    children = {}
    for line in output.splitlines():
        parts = line.split(None, 5)
        if len(parts) != 6:
            continue
        pid = int(parts[0])
        ppid = int(parts[1])
        row = {
            "pid": pid,
            "ppid": ppid,
            "cpu_percent": float(parts[2]),
            "mem_percent": float(parts[3]),
            "rss_kb": int(parts[4]),
            "comm": parts[5],
        }
        table[pid] = row
        children.setdefault(ppid, []).append(pid)
    return table, children


def descendants(root_pid):
    table, children = read_process_table()
    result = []
    stack = [root_pid]
    while stack:
        pid = stack.pop()
        if pid in table:
            result.append(pid)
        stack.extend(children.get(pid, []))
    return [table[pid] for pid in result if pid in table]


def parse_drm_fdinfo(pid):
    engines = {}
    memory = {}
    fdinfo_dir = Path("/proc") / str(pid) / "fdinfo"
    if not fdinfo_dir.exists():
        return engines, memory
    for fdinfo in fdinfo_dir.iterdir():
        try:
            lines = fdinfo.read_text(errors="ignore").splitlines()
        except OSError:
            continue
        for line in lines:
            if ":" not in line:
                continue
            key, value = line.split(":", 1)
            key = key.strip()
            tokens = value.strip().split()
            if not tokens:
                continue
            try:
                amount = int(tokens[0])
            except ValueError:
                continue
            if key.startswith("drm-engine-"):
                engines[key] = max(engines.get(key, 0), amount)
            elif key.startswith("drm-memory-") or key.startswith("drm-total-"):
                memory[key] = max(memory.get(key, 0), amount)
    return engines, memory


def sample_resources(root_pid, duration, interval, csv_path, summary_path):
    rows = []
    start = time.monotonic()
    while time.monotonic() - start < duration:
        elapsed = time.monotonic() - start
        processes = descendants(root_pid)
        total_cpu = sum(process["cpu_percent"] for process in processes)
        total_rss = sum(process["rss_kb"] for process in processes)
        carto_cpu = sum(process["cpu_percent"] for process in processes
                        if "cartographer" in process["comm"])
        carto_rss = sum(process["rss_kb"] for process in processes
                        if "cartographer" in process["comm"])
        engine_totals = {}
        memory_totals = {}
        for process in processes:
            engines, memory = parse_drm_fdinfo(process["pid"])
            for key, value in engines.items():
                engine_totals[key] = engine_totals.get(key, 0) + value
            for key, value in memory.items():
                memory_totals[key] = memory_totals.get(key, 0) + value
        row = {
            "elapsed": elapsed,
            "process_count": len(processes),
            "total_cpu_percent": total_cpu,
            "total_rss_kb": total_rss,
            "cartographer_cpu_percent": carto_cpu,
            "cartographer_rss_kb": carto_rss,
        }
        row.update(engine_totals)
        row.update(memory_totals)
        rows.append(row)
        time.sleep(interval)

    fieldnames = [
        "elapsed", "process_count", "total_cpu_percent", "total_rss_kb",
        "cartographer_cpu_percent", "cartographer_rss_kb",
    ]
    extra_fields = sorted({key for row in rows for key in row
                           if key not in fieldnames})
    with open(csv_path, "w", newline="") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames + extra_fields)
        writer.writeheader()
        writer.writerows(rows)

    def mean(key):
        values = [float(row.get(key, 0.0)) for row in rows]
        return sum(values) / len(values) if values else 0.0

    def max_value(key):
        values = [float(row.get(key, 0.0)) for row in rows]
        return max(values) if values else 0.0

    summary = {
        "samples": len(rows),
        "total_cpu_percent_mean": mean("total_cpu_percent"),
        "total_cpu_percent_max": max_value("total_cpu_percent"),
        "total_rss_mb_mean": mean("total_rss_kb") / 1024.0,
        "total_rss_mb_max": max_value("total_rss_kb") / 1024.0,
        "cartographer_cpu_percent_mean": mean("cartographer_cpu_percent"),
        "cartographer_cpu_percent_max": max_value("cartographer_cpu_percent"),
        "cartographer_rss_mb_mean": mean("cartographer_rss_kb") / 1024.0,
        "cartographer_rss_mb_max": max_value("cartographer_rss_kb") / 1024.0,
        "drm_counter_deltas": {},
    }
    for key in extra_fields:
        values = [int(row.get(key, 0) or 0) for row in rows]
        if values:
            summary["drm_counter_deltas"][key] = max(values) - min(values)
    with open(summary_path, "w") as json_file:
        json.dump(summary, json_file, indent=2, sort_keys=True)


def terminate_process_group(process, timeout_sec=8.0):
    if process.poll() is not None:
        return
    try:
        os.killpg(process.pid, signal.SIGINT)
    except ProcessLookupError:
        return
    deadline = time.monotonic() + timeout_sec
    while process.poll() is None and time.monotonic() < deadline:
        time.sleep(0.1)
    if process.poll() is None:
        os.killpg(process.pid, signal.SIGTERM)
        try:
            process.wait(timeout=timeout_sec)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait()


def start_bash(command, env, log_path):
    log_file = open(log_path, "w")
    process = subprocess.Popen(
        ["bash", "-lc", command],
        env=env,
        stdout=log_file,
        stderr=subprocess.STDOUT,
        preexec_fn=os.setsid,
        text=True,
    )
    return process, log_file


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--workspace", required=True)
    parser.add_argument("--bag")
    parser.add_argument("--label", required=True)
    parser.add_argument("--duration", type=float, required=True)
    parser.add_argument("--interval", type=float, default=0.5)
    parser.add_argument("--output-root", default="latency_results")
    parser.add_argument("--ros-domain-id", type=int, default=91)
    parser.add_argument("--online-correlative", action="store_true")
    parser.add_argument("--vulkan", action="store_true")
    args = parser.parse_args()

    workspace = Path(args.workspace).resolve()
    bag = Path(args.bag).resolve() if args.bag else workspace / "0518_2_humble"
    stamp = time.strftime("%Y%m%d_%H%M%S")
    run_dir = Path(args.output_root).resolve() / f"{args.label}_{stamp}"
    run_dir.mkdir(parents=True, exist_ok=True)

    env = os.environ.copy()
    env["ROS_DOMAIN_ID"] = str(args.ros_domain_id)
    env["ROS_LOCALHOST_ONLY"] = "1"
    env["SLAM_MAIN_DIR"] = str(workspace)
    env["CARTOGRAPHER_USE_ONLINE_CORRELATIVE_SCAN_MATCHING"] = (
        "true" if args.online_correlative else "false"
    )
    if args.vulkan:
        env["CARTOGRAPHER_VULKAN_CORRELATIVE_SCAN_MATCHER"] = "true"
    else:
        env.pop("CARTOGRAPHER_VULKAN_CORRELATIVE_SCAN_MATCHER", None)

    launch_args = [
        "ros2", "launch", "cartographer_ros", "Damvi_rosbag_wheel_launch.py",
        f"bagfiles:={bag}",
        "use_sim_time:=true",
        "fusion_extrapolator:=true",
    ]
    launch_command = shell_source_command(
        workspace, " ".join(shlex.quote(str(part)) for part in launch_args)
    )
    launch_process, launch_log = start_bash(
        launch_command, env, run_dir / "launch.log"
    )

    sampler_command = shell_source_command(
        workspace,
        " ".join([
            shlex.quote(str(SCRIPT_DIR / "sample_cartographer_latency.py")),
            "--duration", shlex.quote(str(args.duration)),
            "--interval", shlex.quote(str(args.interval)),
            "--output-csv", shlex.quote(str(run_dir / "latency.csv")),
            "--summary-json", shlex.quote(str(run_dir / "latency_summary.json")),
        ]),
    )
    sampler_process, sampler_log = start_bash(
        sampler_command, env, run_dir / "latency_sampler.log"
    )

    pose_command = shell_source_command(
        workspace,
        " ".join([
            shlex.quote(str(SCRIPT_DIR / "record_tracked_pose.py")),
            "--duration", shlex.quote(str(args.duration)),
            "--output-csv", shlex.quote(str(run_dir / "tracked_pose.csv")),
            "--summary-json", shlex.quote(str(run_dir / "tracked_pose_summary.json")),
        ]),
    )
    pose_process, pose_log = start_bash(
        pose_command, env, run_dir / "tracked_pose_recorder.log"
    )

    try:
        sample_resources(
            launch_process.pid,
            args.duration,
            args.interval,
            run_dir / "resource_samples.csv",
            run_dir / "resource_summary.json",
        )
    finally:
        for process in (sampler_process, pose_process):
            try:
                process.wait(timeout=4.0)
            except subprocess.TimeoutExpired:
                terminate_process_group(process)
        terminate_process_group(launch_process)
        for log_file in (launch_log, sampler_log, pose_log):
            log_file.close()

    run_summary = {
        "label": args.label,
        "workspace": str(workspace),
        "bag": str(bag),
        "config": str(config),
        "duration": args.duration,
        "ros_domain_id": args.ros_domain_id,
        "online_correlative": args.online_correlative,
        "vulkan": args.vulkan,
        "launch_returncode": launch_process.returncode,
        "sampler_returncode": sampler_process.returncode,
        "pose_returncode": pose_process.returncode,
        "run_dir": str(run_dir),
    }
    with open(run_dir / "run_summary.json", "w") as json_file:
        json.dump(run_summary, json_file, indent=2, sort_keys=True)
    print(json.dumps(run_summary, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
