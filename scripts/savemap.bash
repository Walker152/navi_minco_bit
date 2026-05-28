#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

usage() {
  cat <<EOF
Usage:
  bash scripts/savemap.bash [map_topic] [map_name]

Examples:
  bash scripts/savemap.bash
  bash scripts/savemap.bash /slam_map
  bash scripts/savemap.bash /slam_map src/navigation/navi2_bringup/maps/2026/my_slam

Environment overrides:
  FORMAT=pgm|png|bmp       default: pgm
  MODE=trinary|scale|raw   default: trinary
  OCC=0.65                 default: 0.65
  FREE=0.25                default: 0.25
EOF
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi

if ! command -v ros2 >/dev/null 2>&1; then
  if [[ -f /opt/ros/humble/setup.bash ]]; then
    # shellcheck disable=SC1091
    source /opt/ros/humble/setup.bash
  fi
fi

if ! command -v ros2 >/dev/null 2>&1; then
  echo "Error: ros2 command not found. Please source your ROS environment first." >&2
  exit 1
fi

MAP_TOPIC="${1:-${MAP_TOPIC:-/map}}"
TIMESTAMP="$(date +%Y%m%d_%H%M%S)"
DEFAULT_MAP_NAME="${WORKSPACE_DIR}/src/navigation/navi2_bringup/maps/2026/slam_${TIMESTAMP}"
MAP_NAME="${2:-${MAP_NAME:-$DEFAULT_MAP_NAME}}"

FORMAT="${FORMAT:-pgm}"
MODE="${MODE:-trinary}"
OCC="${OCC:-0.65}"
FREE="${FREE:-0.25}"

# map_saver_cli expects a path prefix, not a .yaml/.pgm/.png filename.
MAP_NAME="${MAP_NAME%.yaml}"
MAP_NAME="${MAP_NAME%.pgm}"
MAP_NAME="${MAP_NAME%.png}"
MAP_NAME="${MAP_NAME%.bmp}"

if [[ "$MAP_NAME" != /* ]]; then
  MAP_NAME="${WORKSPACE_DIR}/${MAP_NAME}"
fi

mkdir -p "$(dirname "$MAP_NAME")"

export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp}"

echo "Saving map..."
echo "  topic : ${MAP_TOPIC}"
echo "  output: ${MAP_NAME}.yaml / ${MAP_NAME}.${FORMAT}"
echo "  mode  : ${MODE}"

ros2 run nav2_map_server map_saver_cli \
  -t "$MAP_TOPIC" \
  -f "$MAP_NAME" \
  --fmt "$FORMAT" \
  --mode "$MODE" \
  --occ "$OCC" \
  --free "$FREE"

echo "Done."
