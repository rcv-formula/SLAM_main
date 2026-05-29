#!/usr/bin/env python3
"""Run carto_pure_wheel_launch with rosbag clock and sample /odom latency."""

import argparse
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


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--workspace", required=True)
    parser.add_argument("--bag")
    parser.add_argument("--label", required=True)
    parser.add_argument("--duration", type=float, default=60.0)
    parser.add_argument("--pbstream")
    parser.add_argument("--output-root", default="latency_results/odom_latency")
    parser.add_argument("--ros-domain-id", type=int, default=131)
    parser.add_argument("--startup-delay", type=float, default=2.0)
    parser.add_argument("--trace-provenance", action="store_true")
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
    if args.trace_provenance:
        env["CARTOGRAPHER_ODOM_PROVENANCE_CSV_PATH"] = str(
            run_dir / "odom_provenance.csv")
        env["CARTOGRAPHER_ODOM_OUTPUT_TRACE_CSV_PATH"] = str(
            run_dir / "odom_output_trace.csv")

    config_dir = workspace / "src/SLAM/cartographer_ros/configuration_files"
    qos_path = config_dir / "rosbag_play_qos_overrides.yaml"
    pbstream = Path(args.pbstream).resolve() if args.pbstream else (
        workspace / "src/SLAM/cartographer_ros/pbstream/latest.pbstream"
    )
    if not pbstream.exists():
        fallback_pbstream = Path(
            "/home/rcv/SLAM_main-SLAM_IMU_WHEEL_tun/src/SLAM/"
            "cartographer_ros/pbstream/latest.pbstream"
        )
        if fallback_pbstream.exists():
            pbstream = fallback_pbstream
    launch_parts = [
        "ros2", "launch", "cartographer_ros",
        "Damvi_carto_pure_wheel_launch.py",
        "use_sim_time:=true",
        f"pbstream_file:={pbstream}",
    ]
    launch_command = shell_source_command(
        workspace, " ".join(shlex.quote(str(part)) for part in launch_parts)
    )
    launch_process, launch_log = start_bash(
        launch_command, env, run_dir / "launch.log")

    sampler_parts = [
        str(SCRIPT_DIR / "sample_odom_latency.py"),
        "--duration", str(args.duration),
        "--output-csv", str(run_dir / "odom_latency.csv"),
        "--summary-json", str(run_dir / "odom_latency_summary.json"),
    ]
    sampler_command = shell_source_command(
        workspace, " ".join(shlex.quote(part) for part in sampler_parts)
    )
    sampler_process, sampler_log = start_bash(
        sampler_command, env, run_dir / "sampler.log")

    time.sleep(args.startup_delay)
    bag_parts = [
        "ros2", "bag", "play", str(bag), "--clock",
        "--qos-profile-overrides-path", str(qos_path),
        "--topics", "/scan", "/imu/data", "/odom_wheel", "/tf",
        "/tf_static", "/ackermann_cmd",
    ]
    bag_command = shell_source_command(
        workspace, " ".join(shlex.quote(part) for part in bag_parts)
    )
    bag_process, bag_log = start_bash(bag_command, env, run_dir / "bag.log")

    try:
        sampler_process.wait(timeout=args.duration + 20.0)
    except subprocess.TimeoutExpired:
        terminate_process_group(sampler_process)
    finally:
        terminate_process_group(bag_process)
        terminate_process_group(launch_process)
        for log_file in (launch_log, sampler_log, bag_log):
            log_file.close()

    summary = {
        "label": args.label,
        "workspace": str(workspace),
        "bag": str(bag),
        "duration": args.duration,
        "run_dir": str(run_dir),
        "ros_domain_id": args.ros_domain_id,
        "trace_provenance": args.trace_provenance,
        "launch_returncode": launch_process.returncode,
        "bag_returncode": bag_process.returncode,
        "sampler_returncode": sampler_process.returncode,
    }
    with open(run_dir / "run_summary.json", "w") as json_file:
        json.dump(summary, json_file, indent=2, sort_keys=True)
    print(json.dumps(summary, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
