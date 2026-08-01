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
  AUTO_SAVE_INTERVAL=60    save repeatedly after this many seconds; unset for one save
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
REQUESTED_MAP_NAME="${2:-${MAP_NAME:-}}"

FORMAT="${FORMAT:-pgm}"
MODE="${MODE:-trinary}"
OCC="${OCC:-0.65}"
FREE="${FREE:-0.25}"
AUTO_SAVE_INTERVAL="${AUTO_SAVE_INTERVAL:-}"

export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp}"

save_map() {
  local map_name

  if [[ -n "$REQUESTED_MAP_NAME" ]]; then
    map_name="$REQUESTED_MAP_NAME"
  else
    # map_name="${WORKSPACE_DIR}/src/navigation/navi2_bringup/maps/2026/slam_$(date +%Y%m%d_%H%M%S)"
    map_name="pgm/slam"
  fi

  # map_saver_cli expects a path prefix, not a .yaml/.pgm/.png filename.
  map_name="${map_name%.yaml}"
  map_name="${map_name%.pgm}"
  map_name="${map_name%.png}"
  map_name="${map_name%.bmp}"

  if [[ "$map_name" != /* ]]; then
    map_name="${WORKSPACE_DIR}/${map_name}"
  fi

  mkdir -p "$(dirname "$map_name")"

  echo "Saving map..."
  echo "  topic : ${MAP_TOPIC}"
  echo "  output: ${map_name}.yaml / ${map_name}.${FORMAT}"
  echo "  mode  : ${MODE}"

  if ! ros2 run nav2_map_server map_saver_cli \
    -t "$MAP_TOPIC" \
    -f "$map_name" \
    --fmt "$FORMAT" \
    --mode "$MODE" \
    --occ "$OCC" \
    --free "$FREE"; then
    echo "Error: failed to save map from ${MAP_TOPIC}." >&2
    return 1
  fi

  echo "Done."
}

if [[ -n "$AUTO_SAVE_INTERVAL" ]]; then
  if [[ ! "$AUTO_SAVE_INTERVAL" =~ ^[0-9]+([.][0-9]+)?$ ]] || \
     [[ -z "${AUTO_SAVE_INTERVAL//[0.]/}" ]]; then
    echo "Error: AUTO_SAVE_INTERVAL must be a positive number of seconds." >&2
    exit 1
  fi

  trap 'exit 0' INT TERM
  echo "Automatic map saving enabled: every ${AUTO_SAVE_INTERVAL} seconds."

  while true; do
    sleep "$AUTO_SAVE_INTERVAL"
    if ! save_map; then
      echo "Warning: automatic map save failed; retrying after ${AUTO_SAVE_INTERVAL} seconds." >&2
    fi
  done
else
  save_map
fi
