#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/humble/setup.bash
# source ../ws_livox/install/setup.bash

set -u

# Hard memory budget for build process (GB).
MEM_LIMIT_GB=${MEM_LIMIT_GB:-14}
# Estimated memory usage per concurrently building package (GB).
MEM_PER_WORKER_GB=${MEM_PER_WORKER_GB:-1}

if (( MEM_PER_WORKER_GB <= 0 )); then
	echo "MEM_PER_WORKER_GB must be > 0"
	exit 1
fi

cpu_workers=$(nproc)
max_workers_by_mem=$(( MEM_LIMIT_GB / MEM_PER_WORKER_GB ))
if (( max_workers_by_mem < 1 )); then
	max_workers_by_mem=1
fi

parallel_workers=$cpu_workers
if (( parallel_workers > max_workers_by_mem )); then
	parallel_workers=$max_workers_by_mem
fi

mem_limit_mb=$(( MEM_LIMIT_GB * 1024 ))

echo "[build] MEM_LIMIT_GB=${MEM_LIMIT_GB}, MEM_PER_WORKER_GB=${MEM_PER_WORKER_GB}, CPU_WORKERS=${cpu_workers}, PARALLEL_WORKERS=${parallel_workers}"

monitor_memory() {
	local limit_mb="$1"
	while true; do
		local mem_total_kb mem_available_kb mem_used_mb mem_available_mb
		mem_total_kb=$(awk '/MemTotal:/ {print $2}' /proc/meminfo)
		mem_available_kb=$(awk '/MemAvailable:/ {print $2}' /proc/meminfo)
		mem_used_mb=$(( (mem_total_kb - mem_available_kb) / 1024 ))
		mem_available_mb=$(( mem_available_kb / 1024 ))

		if (( mem_used_mb > limit_mb )); then
			echo "[mem] WARNING used=${mem_used_mb}MB > limit=${limit_mb}MB, available=${mem_available_mb}MB"
		else
			echo "[mem] used=${mem_used_mb}MB, limit=${limit_mb}MB, available=${mem_available_mb}MB"
		fi
		sleep 2
	done
}

monitor_memory "$mem_limit_mb" &
monitor_pid=$!

cleanup() {
	kill "$monitor_pid" >/dev/null 2>&1 || true
}
trap cleanup EXIT INT TERM

cmake_args=(
	-DCMAKE_BUILD_TYPE=Release
	-DCMAKE_EXPORT_COMPILE_COMMANDS=1
	-DPython3_EXECUTABLE=/usr/bin/python3
	-DPYTHON_EXECUTABLE=/usr/bin/python3
)

serial_packages=(
	livox_ros_driver2
	rog_map
	icp_relocalization
	point_lio
	dbscan_cluster
	communication
	bt_manager
)

echo "[build] Stage 1/2: build critical packages one-by-one"
for pkg in "${serial_packages[@]}"; do
	echo "[build] Serial build package: ${pkg}"
	colcon build \
		--symlink-install \
		--parallel-workers 1 \
		--packages-select "${pkg}" \
		--cmake-args "${cmake_args[@]}" \
		--event-handlers console_direct+ status+
done

echo "[build] Stage 2/2: build remaining packages in parallel"
colcon build \
	--symlink-install \
	--parallel-workers "$parallel_workers" \
	--packages-skip "${serial_packages[@]}" \
	--cmake-args "${cmake_args[@]}" \
	--event-handlers console_direct+ status+