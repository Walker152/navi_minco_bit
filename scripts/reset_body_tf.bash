#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

usage() {
  cat <<EOF
Usage:
  bash scripts/reset_body_tf.bash [parent_frame] [child_frame]

Default:
  bash scripts/reset_body_tf.bash camera_init body

This publishes a zero static TF:
  parent_frame -> child_frame = x y z roll pitch yaw = 0 0 0 0 0 0
EOF
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi

if [[ -f "${WORKSPACE_DIR}/install/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "${WORKSPACE_DIR}/install/setup.bash"
elif [[ -f /opt/ros/humble/setup.bash ]]; then
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
fi

if ! command -v ros2 >/dev/null 2>&1; then
  echo "Error: ros2 command not found. Please source your ROS environment first." >&2
  exit 1
fi

PARENT_FRAME="${1:-camera_init}"
CHILD_FRAME="${2:-body}"

export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp}"

echo "Publishing zero TF reset:"
echo "  parent: ${PARENT_FRAME}"
echo "  child : ${CHILD_FRAME}"
echo "  pose  : 0 0 0 0 0 0"
echo "Press Ctrl+C to stop."

exec ros2 run tf2_ros static_transform_publisher \
  0 0 0 \
  0 0 0 \
  "${PARENT_FRAME}" "${CHILD_FRAME}"
