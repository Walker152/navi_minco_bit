#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

START_LIO=true
WITH_COMMUNICATION=false
USE_MIXED_LIDAR=false
OUTPUT_ROOT="${CALIBRATION_BAG_ROOT:-${HOME}/rosbag}"
ODOM_TOPIC="/aft_mapped_to_init"

LIO_PID=""
COMM_PID=""
RECORD_PID=""
FINISHED=false

usage() {
  cat <<EOF
Usage:
  bash scripts/collect_lidar_calibration.bash [options]

Options:
  --existing              Do not start Livox + Point-LIO; use an already running stack
  --mixed                 Start the mixed dual-MID360 launch instead of the current single launch
  --with-communication    Also start communication and record /sentry/offline_info
  --output-root PATH      Bag parent directory (default: \$HOME/rosbag)
  -h, --help              Show this help

The script never commands the gimbal or chassis. Start slow gimbal rotation manually
only after checking that the robot is safe and the bag recorder is running.
EOF
}

cleanup() {
  if [[ -n "${RECORD_PID}" ]] && kill -0 "${RECORD_PID}" 2>/dev/null; then
    kill -INT "${RECORD_PID}" 2>/dev/null || true
    wait "${RECORD_PID}" 2>/dev/null || true
  fi
  if [[ -n "${COMM_PID}" ]] && kill -0 "${COMM_PID}" 2>/dev/null; then
    kill -INT "${COMM_PID}" 2>/dev/null || true
    wait "${COMM_PID}" 2>/dev/null || true
  fi
  if [[ -n "${LIO_PID}" ]] && kill -0 "${LIO_PID}" 2>/dev/null; then
    kill -INT "${LIO_PID}" 2>/dev/null || true
    wait "${LIO_PID}" 2>/dev/null || true
  fi
}

trap cleanup EXIT
trap 'exit 130' INT TERM

while [[ $# -gt 0 ]]; do
  case "$1" in
    --existing)
      START_LIO=false
      shift
      ;;
    --mixed)
      USE_MIXED_LIDAR=true
      shift
      ;;
    --with-communication)
      WITH_COMMUNICATION=true
      shift
      ;;
    --output-root)
      if [[ $# -lt 2 ]]; then
        echo "Error: --output-root requires a path." >&2
        exit 2
      fi
      OUTPUT_ROOT="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Error: unknown option '$1'." >&2
      usage >&2
      exit 2
      ;;
  esac
done

if [[ -f /opt/ros/humble/setup.bash ]]; then
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
fi
if [[ -f "${WORKSPACE_DIR}/install/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "${WORKSPACE_DIR}/install/setup.bash"
fi

if ! command -v ros2 >/dev/null 2>&1; then
  echo "Error: ros2 command not found. Source ROS 2 Humble first." >&2
  exit 1
fi

TIMESTAMP="$(date +%Y%m%d_%H%M%S)"
RUN_LOG_DIR="/tmp/lidar_calibration_collect_${TIMESTAMP}"
mkdir -p "${RUN_LOG_DIR}" "${OUTPUT_ROOT}"
export ROS_LOG_DIR="${RUN_LOG_DIR}/ros"
mkdir -p "${ROS_LOG_DIR}"

if [[ "${USE_MIXED_LIDAR}" == true ]]; then
  LIO_LAUNCH="mixed_livox_pointlio_intra_process.launch.py"
  IMU_TOPIC="/livox/imu_192_168_1_135"
else
  LIO_LAUNCH="single_livox_pointlio_intra_process.launch.py"
  IMU_TOPIC="/livox/imu"
fi

if [[ "${START_LIO}" == true ]]; then
  echo "Starting Livox + Point-LIO:"
  echo "  ros2 launch point_lio ${LIO_LAUNCH}"
  ros2 launch point_lio "${LIO_LAUNCH}" >"${RUN_LOG_DIR}/point_lio.log" 2>&1 &
  LIO_PID=$!
else
  echo "Using the already running Livox + Point-LIO stack."
fi

if [[ "${WITH_COMMUNICATION}" == true ]]; then
  echo "Starting communication for optional encoder recording."
  ros2 launch communication com.launch.py >"${RUN_LOG_DIR}/communication.log" 2>&1 &
  COMM_PID=$!
fi

echo "Waiting for ${ODOM_TOPIC} ..."
ODOM_TYPE=""
for _ in $(seq 1 45); do
  ODOM_TYPE="$(timeout 2 ros2 topic type "${ODOM_TOPIC}" 2>/dev/null || true)"
  if [[ -n "${ODOM_TYPE}" ]]; then
    break
  fi
  sleep 1
done

if [[ "${ODOM_TYPE}" != "nav_msgs/msg/Odometry" ]]; then
  echo "Error: expected ${ODOM_TOPIC} type nav_msgs/msg/Odometry, got '${ODOM_TYPE:-none}'." >&2
  echo "Point-LIO log: ${RUN_LOG_DIR}/point_lio.log" >&2
  exit 1
fi

echo ""
echo "Odometry is ready."
echo "Before recording:"
echo "  1. Keep the chassis and rotation axis fixed."
echo "  2. Keep the LiDAR x axis facing forward at startup."
echo "  3. Wait at least 5 seconds for Point-LIO IMU initialization."
echo "  4. Do not start high-speed small-gyro motion."
echo ""
read -r -p "Press Enter to start recording..."

BAG_PATH="${OUTPUT_ROOT}/lidar_calibration_${TIMESTAMP}"
RECORD_TOPICS=(
  "${ODOM_TOPIC}"
  "/tf"
  "/tf_static"
  "${IMU_TOPIC}"
)
if [[ "${WITH_COMMUNICATION}" == true ]]; then
  RECORD_TOPICS+=("/sentry/offline_info")
fi

echo "Recording to: ${BAG_PATH}"
echo "Topics:"
printf '  %s\n' "${RECORD_TOPICS[@]}"
ros2 bag record -o "${BAG_PATH}" "${RECORD_TOPICS[@]}" >"${RUN_LOG_DIR}/rosbag.log" 2>&1 &
RECORD_PID=$!

sleep 1
if ! kill -0 "${RECORD_PID}" 2>/dev/null; then
  echo "Error: ros2 bag record exited unexpectedly." >&2
  echo "Recorder log: ${RUN_LOG_DIR}/rosbag.log" >&2
  exit 1
fi

echo ""
echo "Recorder is running."
echo "Now rotate the LiDAR/gimbal slowly for at least one complete turn (1.5-2 turns recommended)."
echo "Keep the chassis still. Stop the rotation and wait 2-3 seconds before finishing."
echo ""
read -r -p "Press Enter to stop recording..."

kill -INT "${RECORD_PID}" 2>/dev/null || true
wait "${RECORD_PID}" 2>/dev/null || true
RECORD_PID=""
FINISHED=true

echo ""
echo "Bag saved: ${BAG_PATH}"
echo "Run calibration:"
echo "  python3 tools/lidar_calibration/cli.py \\"
echo "    '${BAG_PATH}' \\"
if [[ "${WITH_COMMUNICATION}" == true ]]; then
  echo "    --odom-topic '${ODOM_TOPIC}' \\"
  echo "    --gimbal-topic /sentry/offline_info"
else
  echo "    --odom-topic '${ODOM_TOPIC}'"
fi
echo ""
echo "Runtime logs: ${RUN_LOG_DIR}"

if [[ "${FINISHED}" != true ]]; then
  exit 1
fi
