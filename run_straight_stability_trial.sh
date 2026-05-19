#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BAG_PATH="${1:-/home/symoon/Desktop/bag/0518_2}"
DURATION_SEC="${2:-125}"
STAMP="$(date +%Y%m%d_%H%M%S)"
OUT_DIR="${3:-${ROOT_DIR}/metrics_data/straight_stability_${STAMP}}"
CSV_PATH="/tmp/ver6_cartographer_local_quality.csv"

mkdir -p "${OUT_DIR}"
rm -f "${CSV_PATH}"

LAUNCH_PID=""
BAG_PID=""

cleanup() {
  if [[ -n "${BAG_PID}" ]]; then
    kill -INT "-${BAG_PID}" 2>/dev/null || true
  fi
  if [[ -n "${LAUNCH_PID}" ]]; then
    kill -INT "-${LAUNCH_PID}" 2>/dev/null || true
  fi
}
trap cleanup EXIT

cd "${ROOT_DIR}"
set +u
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u

setsid bash -lc \
  "source /opt/ros/humble/setup.bash; source '${ROOT_DIR}/install/setup.bash'; exec ros2 launch cartographer_ros Damvi_carto_wheel_launch.py" \
  > "${OUT_DIR}/launch.log" 2>&1 &
LAUNCH_PID=$!

for _ in $(seq 1 80); do
  if rg -q "Loaded pose extrapolator config" "${OUT_DIR}/launch.log"; then
    break
  fi
  sleep 0.25
done

setsid bash -lc \
  "source /opt/ros/humble/setup.bash; source '${ROOT_DIR}/install/setup.bash'; exec ros2 bag play '${BAG_PATH}' --clock" \
  > "${OUT_DIR}/bag.log" 2>&1 &
BAG_PID=$!

SECONDS_WAITED=0
while kill -0 "${BAG_PID}" 2>/dev/null && [[ "${SECONDS_WAITED}" -lt "${DURATION_SEC}" ]]; do
  sleep 1
  SECONDS_WAITED=$((SECONDS_WAITED + 1))
done

if kill -0 "${BAG_PID}" 2>/dev/null; then
  kill -INT "-${BAG_PID}" 2>/dev/null || true
  wait "${BAG_PID}" 2>/dev/null || true
fi
BAG_PID=""

sleep 2

if [[ ! -s "${CSV_PATH}" ]]; then
  echo "No local quality CSV was produced: ${CSV_PATH}" >&2
  exit 1
fi

cp "${CSV_PATH}" "${OUT_DIR}/local_quality.csv"
python3 analyze_straight_stability.py "${OUT_DIR}/local_quality.csv" \
  --json-out "${OUT_DIR}/straight_stability.json" \
  > "${OUT_DIR}/straight_stability.txt"

cat "${OUT_DIR}/straight_stability.txt"
echo
echo "Output directory: ${OUT_DIR}"
