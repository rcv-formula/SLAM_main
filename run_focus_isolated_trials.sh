#!/usr/bin/env bash

source /opt/ros/humble/setup.bash
source install/setup.bash

set -u

BAG=/home/symoon/Desktop/bag/sensor_only/rosbag2_2026_03_13-17_45_23_0.db3
OUTROOT=${1:-metrics_data/wheel_odom_tuning/focus_detached_$(date +%Y%m%d_%H%M%S)}
mkdir -p "$OUTROOT"

echo "OUTROOT=$OUTROOT"

TRIALS=(
  parameter_trials/focus_active_current.yaml
  parameter_trials/focus_outlier_medium006.yaml
  parameter_trials/focus_outlier_req1_009.yaml
  parameter_trials/focus_outlier_req1_008.yaml
  parameter_trials/focus_ceres_loose1010.yaml
  parameter_trials/focus_ceres_occ70_tr15.yaml
)

domain=80
for yaml in "${TRIALS[@]}"; do
  name=$(basename "$yaml" .yaml)
  out="$OUTROOT/$name"
  mkdir -p "$out"
  echo "RUN $name domain=$domain $(date +%H:%M:%S)"

  ROS_DOMAIN_ID=$domain LOCAL_QUALITY_METRICS_CSV="$PWD/$out/local_quality.csv" \
    setsid ros2 launch cartographer_ros Damvi_carto_pure_wheel_metric_headless_launch.py \
    pose_extrapolator_config:="$PWD/$yaml" > "$out/launch.log" 2>&1 &
  launch_pid=$!

  sleep 7

  ROS_DOMAIN_ID=$domain setsid timeout 100s ros2 bag play "$BAG" --clock \
    --disable-keyboard-controls > "$out/bag.log" 2>&1 &
  bag_pid=$!

  while kill -0 "$bag_pid" 2>/dev/null; do
    rows=$(wc -l < "$out/local_quality.csv" 2>/dev/null || echo 0)
    echo "TICK $name rows=$rows $(date +%H:%M:%S)"
    sleep 10
  done
  wait "$bag_pid" 2>/dev/null || true

  kill -INT -"$launch_pid" 2>/dev/null || true
  sleep 3
  kill -TERM -"$launch_pid" 2>/dev/null || true
  wait "$launch_pid" 2>/dev/null || true

  rows=$(wc -l < "$out/local_quality.csv" 2>/dev/null || echo 0)
  echo "DONE $name domain=$domain quality_rows=$rows $(date +%H:%M:%S)"
  domain=$((domain + 1))
  sleep 8
done

echo "ALL_DONE $OUTROOT"
