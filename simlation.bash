#!/usr/bin/env bash
set -euo pipefail

usage() {
  echo "Usage: ./simlation.bash [omni|diff] [rmuc_2024|rmul_2024|rmuc_2025|rmul_2025] [--headless] [--no-rviz] [--check]"
}

chassis_type="omni"
world="rmuc_2025"
headless="false"
rviz="true"
preflight_only="false"
positional_index=0

for argument in "$@"; do
  case "${argument}" in
    --headless) headless="true" ;;
    --no-rviz) rviz="false" ;;
    --check) preflight_only="true" ;;
    -h|--help) usage; exit 0 ;;
    --*) echo "Unknown option: ${argument}" >&2; usage >&2; exit 2 ;;
    *)
      case "${positional_index}" in
        0) chassis_type="${argument}" ;;
        1) world="${argument}" ;;
        *) echo "Unexpected positional argument: ${argument}" >&2; usage >&2; exit 2 ;;
      esac
      positional_index=$((positional_index + 1))
      ;;
  esac
done

case "${chassis_type}" in
  omni|diff) ;;
  *) echo "Unsupported chassis: ${chassis_type}" >&2; usage >&2; exit 2 ;;
esac

case "${world}" in
  rmuc_2024|rmul_2024|rmuc_2025|rmul_2025) ;;
  *) echo "Unsupported world: ${world}" >&2; usage >&2; exit 2 ;;
esac

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "ROS 2 Humble was not found at /opt/ros/humble." >&2
  exit 1
fi

set +u
source /opt/ros/humble/setup.bash
if [[ ! -f "${script_dir}/install/setup.bash" ]]; then
  echo "Workspace install/setup.bash is missing. Build the workspace after reviewing src/simulation/README.md." >&2
  exit 1
fi
source "${script_dir}/install/setup.bash"
set -u

required_ros_packages=(
  sentry_simulation
  ros_gz_sim
  ros_gz_bridge
  navi2
  point_lio
  minco_planner
  minco_controller
  rog_map
)
for required_ros_package in "${required_ros_packages[@]}"; do
  if ! ros2 pkg prefix "${required_ros_package}" >/dev/null 2>&1; then
    echo "Required ROS package is missing from the sourced overlay: ${required_ros_package}" >&2
    exit 1
  fi
done

if [[ "${chassis_type}" == "omni" ]]; then
  simulation_prefix="$(ros2 pkg prefix sentry_simulation)"
  if [[ ! -f "${simulation_prefix}/plugins/libMecanumDrive2.so" ]]; then
    echo "MecanumDrive2 was not installed under sentry_simulation/plugins." >&2
    exit 1
  fi
fi

if [[ "${preflight_only}" == "true" ]]; then
  echo "Simulation preflight passed for ${chassis_type} in ${world}."
  exit 0
fi

exec ros2 launch sentry_simulation simulation.launch.py \
  chassis_type:="${chassis_type}" \
  world:="${world}" \
  headless:="${headless}" \
  rviz:="${rviz}"
