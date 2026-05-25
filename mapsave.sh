#!/usr/bin/env bash
set -euo pipefail

PRIMARY_WS="/home/rcv/SLAM_main"
MIRROR_WS="/home/rcv/SLAM_main-SLAM_IMU_WHEEL_tun"

PRIMARY_CARTO="$PRIMARY_WS/src/SLAM/cartographer_ros"
MIRROR_CARTO="$MIRROR_WS/src/SLAM/cartographer_ros"
PRIMARY_INSTALL_CARTO="$PRIMARY_WS/install/cartographer_ros/share/cartographer_ros"
MIRROR_INSTALL_CARTO="$MIRROR_WS/install/cartographer_ros/share/cartographer_ros"

# Optional first argument: archive name for the previous latest files.
RAW="${1:-$(date +%m%d_%H%M%S)}"
BASE="${RAW#\'}"
BASE="${BASE%\'}"
if [[ -z "$BASE" ]]; then
  BASE="$(date +%m%d_%H%M%S)"
fi

mkdir -p \
  "$PRIMARY_CARTO/pbstream" "$PRIMARY_CARTO/maps" \
  "$MIRROR_CARTO/pbstream" "$MIRROR_CARTO/maps" \
  "$PRIMARY_INSTALL_CARTO/pbstream" "$PRIMARY_INSTALL_CARTO/maps" \
  "$MIRROR_INSTALL_CARTO/pbstream" "$MIRROR_INSTALL_CARTO/maps"

sync_latest_outputs() {
  mkdir -p \
    "$MIRROR_CARTO/pbstream" "$MIRROR_CARTO/maps" \
    "$PRIMARY_INSTALL_CARTO/pbstream" "$PRIMARY_INSTALL_CARTO/maps" \
    "$MIRROR_INSTALL_CARTO/pbstream" "$MIRROR_INSTALL_CARTO/maps"

  if [[ -e "$PRIMARY_CARTO/pbstream/latest.pbstream" ]]; then
    cp -f "$PRIMARY_CARTO/pbstream/latest.pbstream" "$MIRROR_CARTO/pbstream/latest.pbstream"
    cp -f "$PRIMARY_CARTO/pbstream/latest.pbstream" "$PRIMARY_INSTALL_CARTO/pbstream/latest.pbstream"
    cp -f "$PRIMARY_CARTO/pbstream/latest.pbstream" "$MIRROR_INSTALL_CARTO/pbstream/latest.pbstream"
  fi

  if [[ -e "$PRIMARY_CARTO/maps/latest.pgm" ]]; then
    cp -f "$PRIMARY_CARTO/maps/latest.pgm" "$MIRROR_CARTO/maps/latest.pgm"
    cp -f "$PRIMARY_CARTO/maps/latest.pgm" "$PRIMARY_INSTALL_CARTO/maps/latest.pgm"
    cp -f "$PRIMARY_CARTO/maps/latest.pgm" "$MIRROR_INSTALL_CARTO/maps/latest.pgm"
  fi

  if [[ -e "$PRIMARY_CARTO/maps/latest.yaml" ]]; then
    cp -f "$PRIMARY_CARTO/maps/latest.yaml" "$MIRROR_CARTO/maps/latest.yaml"
    cp -f "$PRIMARY_CARTO/maps/latest.yaml" "$PRIMARY_INSTALL_CARTO/maps/latest.yaml"
    cp -f "$PRIMARY_CARTO/maps/latest.yaml" "$MIRROR_INSTALL_CARTO/maps/latest.yaml"
  fi
}

trap sync_latest_outputs EXIT

name_exists() {
  local name="$1"
  [[ -e "$PRIMARY_CARTO/pbstream/$name.pbstream" ||
     -e "$PRIMARY_CARTO/maps/$name.pgm" ||
     -e "$PRIMARY_CARTO/maps/$name.yaml" ||
     -e "$MIRROR_CARTO/pbstream/$name.pbstream" ||
     -e "$MIRROR_CARTO/maps/$name.pgm" ||
     -e "$MIRROR_CARTO/maps/$name.yaml" ||
     -e "$PRIMARY_INSTALL_CARTO/pbstream/$name.pbstream" ||
     -e "$PRIMARY_INSTALL_CARTO/maps/$name.pgm" ||
     -e "$PRIMARY_INSTALL_CARTO/maps/$name.yaml" ||
     -e "$MIRROR_INSTALL_CARTO/pbstream/$name.pbstream" ||
     -e "$MIRROR_INSTALL_CARTO/maps/$name.pgm" ||
     -e "$MIRROR_INSTALL_CARTO/maps/$name.yaml" ]]
}

NAME="$BASE"
NUM=1
while name_exists "$NAME"; do
  NAME="${BASE}_${NUM}"
  NUM=$((NUM + 1))
done

if [[ -e "$PRIMARY_CARTO/pbstream/latest.pbstream" ]]; then
  mv "$PRIMARY_CARTO/pbstream/latest.pbstream" "$PRIMARY_CARTO/pbstream/$NAME.pbstream"
  cp "$PRIMARY_CARTO/pbstream/$NAME.pbstream" "$MIRROR_CARTO/pbstream/$NAME.pbstream"
  cp "$PRIMARY_CARTO/pbstream/$NAME.pbstream" "$PRIMARY_INSTALL_CARTO/pbstream/$NAME.pbstream"
  cp "$PRIMARY_CARTO/pbstream/$NAME.pbstream" "$MIRROR_INSTALL_CARTO/pbstream/$NAME.pbstream"
fi

if [[ -e "$PRIMARY_CARTO/maps/latest.pgm" ]]; then
  mv "$PRIMARY_CARTO/maps/latest.pgm" "$PRIMARY_CARTO/maps/$NAME.pgm"
  cp "$PRIMARY_CARTO/maps/$NAME.pgm" "$MIRROR_CARTO/maps/$NAME.pgm"
  cp "$PRIMARY_CARTO/maps/$NAME.pgm" "$PRIMARY_INSTALL_CARTO/maps/$NAME.pgm"
  cp "$PRIMARY_CARTO/maps/$NAME.pgm" "$MIRROR_INSTALL_CARTO/maps/$NAME.pgm"
fi

if [[ -e "$PRIMARY_CARTO/maps/latest.yaml" ]]; then
  mv "$PRIMARY_CARTO/maps/latest.yaml" "$PRIMARY_CARTO/maps/$NAME.yaml"
  sed -i -E "s|^image:.*|image: $NAME.pgm|g" "$PRIMARY_CARTO/maps/$NAME.yaml"
  cp "$PRIMARY_CARTO/maps/$NAME.yaml" "$MIRROR_CARTO/maps/$NAME.yaml"
  cp "$PRIMARY_CARTO/maps/$NAME.yaml" "$PRIMARY_INSTALL_CARTO/maps/$NAME.yaml"
  cp "$PRIMARY_CARTO/maps/$NAME.yaml" "$MIRROR_INSTALL_CARTO/maps/$NAME.yaml"
fi

source "$PRIMARY_WS/install/setup.bash"

ros2 service call /write_state cartographer_ros_msgs/srv/WriteState \
  "{filename: '$PRIMARY_CARTO/pbstream/latest.pbstream'}"

ros2 run nav2_map_server map_saver_cli \
  -f "$PRIMARY_CARTO/maps/latest" \
  --ros-args -p map_subscribe_transient_local:=true

sed -i -E "s|^image:.*|image: latest.pgm|g" "$PRIMARY_CARTO/maps/latest.yaml"

sync_latest_outputs

cd "$PRIMARY_CARTO/maps"
printf 'Saved latest map and pbstream. Archived previous latest as: %s\n' "$NAME"
