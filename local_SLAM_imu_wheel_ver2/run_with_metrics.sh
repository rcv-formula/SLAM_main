#!/usr/bin/env bash
# Cartographer Node 실행 + 메트릭 모니터링

# 1. Cartographer ROS 노드 시작 (백그라운드)
echo "🚀 Cartographer ROS Node 시작..."
source /opt/ros/humble/setup.bash
source /home/symoon/SLAM_main/install/setup.bash

# 여기에 실제 cartographer ros 런 명령 추가
# ros2 run cartographer_ros cartographer_node \
#   --ros-args -p use_sim_time:=false \
#   -r use_metrics:=true &

# 잠시 대기 (노드 시작 완료 대기)
sleep 3

# 2. 메트릭 모니터링 시작
echo "📊 메트릭 모니터링 시작..."
python3 /home/symoon/SLAM_main/monitor_metrics.py
