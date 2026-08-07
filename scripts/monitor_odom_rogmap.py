#!/usr/bin/env python3
"""Record low-overhead diagnostics for Point-LIO and ROGMap.

The ``match`` mode avoids subscriptions to large point-cloud messages.  The
``test`` mode additionally observes the merged Livox input and Point-LIO full
cloud, with one isolated worker process per large topic so those copies cannot
distort the primary odom/map timing executor.  It also records bond/lifecycle
events, node presence, monitor scheduling lag, map CRC samples and Linux PSI.
An explicit ``--with-bag`` option is available only in test mode.

Run after sourcing the workspace:

  source install/setup.bash
  python3 scripts/monitor_odom_rogmap.py --mode match
  python3 scripts/monitor_odom_rogmap.py --mode test --duration 180
"""

from __future__ import annotations

import argparse
import csv
import datetime as dt
import json
import math
import os
import platform
import shutil
import signal
import subprocess
import sys
import threading
import time
import zlib
from pathlib import Path
from typing import Dict, Iterable, List, NamedTuple, Optional, Sequence, TextIO


NSEC_PER_SEC = 1_000_000_000
MSEC_PER_SEC = 1_000.0
ROSOUT_WARN_LEVEL = 30
DEFAULT_OUTPUT_ROOT = Path.home() / "odom_rogmap_monitor"
DEFAULT_INTERFACE = "enp86s0"


class TopicSpec(NamedTuple):
    topic: str
    kind: str
    large: bool = False


MATCH_TOPICS = (
    TopicSpec("/aft_mapped_to_init", "odom"),
    TopicSpec("/livox/imu_192_168_1_135", "imu"),
    TopicSpec("/rog_map/layer_value", "occupancy_grid"),
)

TEST_ONLY_TOPICS = (
    TopicSpec("/livox/lidar", "livox_custom", True),
    TopicSpec("/cloud_registered_full", "pointcloud2", True),
)

LIFECYCLE_NODES = (
    "/controller_server",
    "/smoother_server",
    "/planner_server",
    "/behavior_server",
    "/bt_navigator",
    "/waypoint_follower",
    "/velocity_smoother",
)

CRITICAL_NODES = (
    "/lifecycle_manager_navigation",
    "/livox_driver_node",
    "/laserMapping",
    *LIFECYCLE_NODES,
)

ROSOUT_KEYWORDS = (
    "point-lio",
    "point_lio",
    "livox",
    "merge",
    "lidar",
    "imu",
    "odom",
    "rogmap",
    "rog_map",
    "point cloud",
    "timeout",
    "loop back",
    "unfinished frame",
    "drop",
    "queue",
    "ptp",
    "dds",
    "component_container",
    "bond",
    "heartbeat",
    "lifecycle",
    "transition",
    "shutting down related nodes",
)


def mode_topics(mode: str) -> Sequence[TopicSpec]:
    if mode == "match":
        return MATCH_TOPICS
    if mode == "test":
        return MATCH_TOPICS + TEST_ONLY_TOPICS
    raise ValueError(f"unsupported mode: {mode}")


def primary_topics(mode: str) -> Sequence[TopicSpec]:
    """Topics handled by the latency-sensitive primary executor."""
    if mode in ("match", "test"):
        return MATCH_TOPICS
    raise ValueError(f"unsupported mode: {mode}")


def large_worker_topics(mode: str) -> Sequence[TopicSpec]:
    """Large topics isolated from primary timing observations."""
    if mode == "match":
        return ()
    if mode == "test":
        return TEST_ONLY_TOPICS
    raise ValueError(f"unsupported mode: {mode}")


def large_worker_output_stem(topic: str) -> str:
    stem = topic.strip("/").replace("/", "_")
    return stem or "root"


def large_worker_command(
    session: Path,
    spec: TopicSpec,
    duration: float,
    summary_interval: float,
) -> List[str]:
    return [
        sys.executable,
        str(Path(__file__).resolve()),
        "--mode",
        "test",
        "--duration",
        str(duration),
        "--summary-interval",
        str(summary_interval),
        "--_large-worker-topic",
        spec.topic,
        "--_session-dir",
        str(session),
    ]


def fully_qualified_node_names(
    names_and_namespaces: Iterable[tuple[str, str]],
) -> set[str]:
    result = set()
    for name, namespace in names_and_namespaces:
        prefix = namespace.rstrip("/")
        result.add(f"{prefix}/{name}" if prefix else f"/{name}")
    return result


def topology_topics() -> Sequence[TopicSpec]:
    """Topics whose endpoint state is cheap enough to inspect in every mode."""
    return MATCH_TOPICS + TEST_ONLY_TOPICS


def bag_topics() -> Sequence[str]:
    return (
        "/livox/lidar",
        "/livox/imu_192_168_1_135",
        "/aft_mapped_to_init",
        "/cloud_registered_full",
        "/rog_map/layer_value",
        "/tf",
        "/tf_static",
        "/bond",
        *(f"{node}/transition_event" for node in LIFECYCLE_NODES),
    )


def safe_counter_delta(current: int, previous: int) -> int:
    """Return a monotonic-counter delta while tolerating reset or wrap."""
    return current - previous if current >= previous else current


def should_capture_rosout(level: int, message: str) -> bool:
    if level >= ROSOUT_WARN_LEVEL:
        return True
    lowered = message.lower()
    return any(keyword in lowered for keyword in ROSOUT_KEYWORDS)


def _new_stats_bucket() -> Dict[str, float]:
    return {
        "count": 0,
        "first_arrival_ns": 0,
        "last_arrival_ns": 0,
        "max_gap_ms": math.nan,
        "age_sum_ms": 0.0,
        "age_count": 0,
        "max_age_ms": math.nan,
        "sensor_time_regressions": 0,
        "payload_bytes": 0,
        "points": 0,
    }


class TopicStats:
    """Incremental arrival/stamp statistics with bounded memory use."""

    def __init__(self, topic: str):
        self.topic = topic
        self._last_arrival_ns: Optional[int] = None
        self._last_stamp_ns: Optional[int] = None
        self._total = _new_stats_bucket()
        self._window = _new_stats_bucket()

    @staticmethod
    def _update_bucket(
        bucket: Dict[str, float],
        arrival_ns: int,
        age_ms: float,
        interarrival_ms: float,
        sensor_time_regression: bool,
        payload_size: int,
        points: int,
    ) -> None:
        if bucket["count"] == 0:
            bucket["first_arrival_ns"] = arrival_ns
        bucket["count"] += 1
        bucket["last_arrival_ns"] = arrival_ns
        if math.isfinite(interarrival_ms):
            if not math.isfinite(bucket["max_gap_ms"]):
                bucket["max_gap_ms"] = interarrival_ms
            else:
                bucket["max_gap_ms"] = max(bucket["max_gap_ms"], interarrival_ms)
        if math.isfinite(age_ms):
            bucket["age_sum_ms"] += age_ms
            bucket["age_count"] += 1
            if not math.isfinite(bucket["max_age_ms"]):
                bucket["max_age_ms"] = age_ms
            else:
                bucket["max_age_ms"] = max(bucket["max_age_ms"], age_ms)
        if sensor_time_regression:
            bucket["sensor_time_regressions"] += 1
        bucket["payload_bytes"] += max(0, int(payload_size))
        bucket["points"] += max(0, int(points))

    def observe(
        self,
        arrival_ns: int,
        stamp_ns: int,
        payload_size: int = 0,
        points: int = 0,
    ) -> Dict[str, object]:
        interarrival_ms = math.nan
        if self._last_arrival_ns is not None:
            interarrival_ms = (arrival_ns - self._last_arrival_ns) / 1.0e6

        age_ms = math.nan if stamp_ns <= 0 else (arrival_ns - stamp_ns) / 1.0e6
        sensor_dt_ms = math.nan
        sensor_time_regression = False
        if stamp_ns > 0 and self._last_stamp_ns is not None:
            sensor_dt_ms = (stamp_ns - self._last_stamp_ns) / 1.0e6
            sensor_time_regression = stamp_ns < self._last_stamp_ns

        self._update_bucket(
            self._total,
            arrival_ns,
            age_ms,
            interarrival_ms,
            sensor_time_regression,
            payload_size,
            points,
        )
        self._update_bucket(
            self._window,
            arrival_ns,
            age_ms,
            interarrival_ms,
            sensor_time_regression,
            payload_size,
            points,
        )
        self._last_arrival_ns = arrival_ns
        if stamp_ns > 0:
            self._last_stamp_ns = stamp_ns

        return {
            "interarrival_ms": interarrival_ms,
            "age_ms": age_ms,
            "sensor_dt_ms": sensor_dt_ms,
            "sensor_time_regression": sensor_time_regression,
        }

    @staticmethod
    def _summarize(bucket: Dict[str, float]) -> Dict[str, float]:
        count = int(bucket["count"])
        duration_ns = bucket["last_arrival_ns"] - bucket["first_arrival_ns"]
        hz = (
            (count - 1) * NSEC_PER_SEC / duration_ns
            if count >= 2 and duration_ns > 0
            else math.nan
        )
        mean_age_ms = (
            bucket["age_sum_ms"] / bucket["age_count"]
            if bucket["age_count"] > 0
            else math.nan
        )
        return {
            "count": count,
            "hz": hz,
            "max_gap_ms": bucket["max_gap_ms"],
            "mean_age_ms": mean_age_ms,
            "max_age_ms": bucket["max_age_ms"],
            "sensor_time_regressions": int(bucket["sensor_time_regressions"]),
            "payload_bytes": int(bucket["payload_bytes"]),
            "points": int(bucket["points"]),
        }

    def summary(self) -> Dict[str, float]:
        return self._summarize(self._total)

    def window_summary(self, reset: bool = True) -> Dict[str, float]:
        result = self._summarize(self._window)
        if reset:
            self._window = _new_stats_bucket()
        return result


class CsvRecorder:
    def __init__(self, path: Path, fields: Sequence[str]):
        self.path = path
        self._file = path.open("w", newline="", encoding="utf-8")
        self._writer = csv.DictWriter(self._file, fieldnames=fields, extrasaction="ignore")
        self._writer.writeheader()
        self._lock = threading.Lock()

    def write(self, row: Dict[str, object]) -> None:
        with self._lock:
            self._writer.writerow(row)

    def flush(self) -> None:
        with self._lock:
            self._file.flush()

    def close(self) -> None:
        with self._lock:
            self._file.flush()
            self._file.close()


def iso_now() -> str:
    return dt.datetime.now().astimezone().isoformat(timespec="milliseconds")


def stamp_to_ns(stamp: object) -> int:
    sec = int(getattr(stamp, "sec", 0))
    nanosec = int(getattr(stamp, "nanosec", 0))
    return sec * NSEC_PER_SEC + nanosec


def bond_status_to_row(msg: object, arrival_ns: int, wall_time: str) -> Dict[str, object]:
    return {
        "wall_time": wall_time,
        "arrival_unix_ns": arrival_ns,
        "bond_id": getattr(msg, "id", ""),
        "instance_id": getattr(msg, "instance_id", ""),
        "active": bool(getattr(msg, "active", False)),
        "heartbeat_period_sec": float(getattr(msg, "heartbeat_period", math.nan)),
        "heartbeat_timeout_sec": float(getattr(msg, "heartbeat_timeout", math.nan)),
    }


def transition_event_to_row(
    topic: str,
    msg: object,
    arrival_ns: int,
    wall_time: str,
) -> Dict[str, object]:
    transition = getattr(msg, "transition", object())
    start_state = getattr(msg, "start_state", object())
    goal_state = getattr(msg, "goal_state", object())
    timestamp = getattr(msg, "timestamp", 0)
    event_stamp_ns = (
        int(timestamp) if isinstance(timestamp, (int, float)) else stamp_to_ns(timestamp)
    )
    return {
        "wall_time": wall_time,
        "arrival_unix_ns": arrival_ns,
        "event_stamp_ns": event_stamp_ns,
        "topic": topic,
        "node": topic.removesuffix("/transition_event"),
        "transition_id": int(getattr(transition, "id", 0)),
        "transition_label": getattr(transition, "label", ""),
        "start_state_id": int(getattr(start_state, "id", 0)),
        "start_state_label": getattr(start_state, "label", ""),
        "goal_state_id": int(getattr(goal_state, "id", 0)),
        "goal_state_label": getattr(goal_state, "label", ""),
    }


class MapContentSampler:
    """Rate-limited CRC for distinguishing publication from map progress."""

    def __init__(self, interval_sec: float):
        self._interval_ns = max(1, int(interval_sec * NSEC_PER_SEC))
        self._last_sample_ns: Optional[int] = None
        self._last_crc: Optional[int] = None

    @staticmethod
    def _crc32(data: object) -> int:
        try:
            view = memoryview(data).cast("B")
            return zlib.crc32(view) & 0xFFFFFFFF
        except (TypeError, ValueError):
            packed = bytes(int(value) & 0xFF for value in data)
            return zlib.crc32(packed) & 0xFFFFFFFF

    def observe(self, data: object, monotonic_ns: int) -> Dict[str, object]:
        if (
            self._last_sample_ns is not None
            and monotonic_ns - self._last_sample_ns < self._interval_ns
        ):
            return {"map_hash_sampled": False, "map_crc32": "", "map_changed": ""}

        crc = self._crc32(data)
        changed: object = "" if self._last_crc is None else crc != self._last_crc
        self._last_sample_ns = monotonic_ns
        self._last_crc = crc
        return {
            "map_hash_sampled": True,
            "map_crc32": f"{crc:08x}",
            "map_changed": changed,
        }


def create_session_dir(root: Path, mode: str) -> Path:
    root.mkdir(parents=True, exist_ok=True)
    base = dt.datetime.now().strftime(f"%Y%m%d_%H%M%S_{mode}")
    candidate = root / base
    suffix = 0
    while candidate.exists():
        suffix += 1
        candidate = root / f"{base}_{suffix}"
    candidate.mkdir()
    return candidate


def run_command(command: Sequence[str], timeout_sec: float = 8.0) -> str:
    try:
        result = subprocess.run(
            list(command),
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            timeout=timeout_sec,
            check=False,
        )
        return f"$ {' '.join(command)}\nexit={result.returncode}\n{result.stdout.rstrip()}\n"
    except (OSError, subprocess.TimeoutExpired) as exc:
        return f"$ {' '.join(command)}\nERROR: {exc}\n"


def file_metadata(path: Path) -> Dict[str, object]:
    result: Dict[str, object] = {"path": str(path), "exists": path.exists()}
    try:
        stat_result = path.stat()
        result.update(
            size=stat_result.st_size,
            mtime=dt.datetime.fromtimestamp(stat_result.st_mtime).astimezone().isoformat(),
            resolved_path=str(path.resolve()),
            is_symlink=path.is_symlink(),
        )
    except OSError as exc:
        result["error"] = str(exc)
    return result


def write_manifest(path: Path, args: argparse.Namespace, topics: Sequence[TopicSpec]) -> None:
    repo_root = Path(__file__).resolve().parents[1]
    artifact_paths = (
        repo_root / "src/perception/Point-LIO/src/laserMapping.cpp",
        repo_root / "src/perception/Point-LIO/config/mid360.yaml",
        repo_root / "build/point_lio/libpoint_lio_component.so",
        repo_root / "src/perception/rog_map/include/rog_map_ros/rog_map_ros2.hpp",
        repo_root / "build/minco_planner/libminco_planner.so",
        repo_root / "src/navigation/navi2_bringup/params/sentry1.yaml",
    )
    manifest = {
        "start_time": iso_now(),
        "mode": args.mode,
        "duration_sec": args.duration,
        "interface": args.interface,
        "with_bag": args.with_bag,
        "topics": [item._asdict() for item in topics],
        "monitor_architecture": {
            "primary_topics": [item.topic for item in primary_topics(args.mode)],
            "isolated_large_topic_workers": [
                item.topic for item in large_worker_topics(args.mode)
            ],
            "map_hash_interval_sec": args.map_hash_interval,
            "monitor_health_interval_sec": args.monitor_health_interval,
            "node_liveness_interval_sec": args.node_liveness_interval,
        },
        "hostname": platform.node(),
        "platform": platform.platform(),
        "python": sys.version,
        "cwd": os.getcwd(),
        "repo_root": str(repo_root),
        "environment": {
            "ROS_DOMAIN_ID": os.environ.get("ROS_DOMAIN_ID", ""),
            "RMW_IMPLEMENTATION": os.environ.get("RMW_IMPLEMENTATION", ""),
            "CYCLONEDDS_URI": os.environ.get("CYCLONEDDS_URI", ""),
        },
        "artifacts": [file_metadata(item) for item in artifact_paths],
        "git_head": run_command(["git", "rev-parse", "HEAD"], 3.0).strip(),
        "git_status": run_command(["git", "status", "--short"], 3.0),
    }
    path.write_text(json.dumps(manifest, ensure_ascii=False, indent=2), encoding="utf-8")


def update_manifest_end(
    path: Path,
    reason: str,
    diagnostic_sources: Optional[Sequence[Dict[str, object]]] = None,
    large_workers: Optional[Sequence[Dict[str, object]]] = None,
) -> None:
    try:
        manifest = json.loads(path.read_text(encoding="utf-8"))
        manifest["end_time"] = iso_now()
        manifest["stop_reason"] = reason
        if diagnostic_sources is not None:
            manifest["diagnostic_sources"] = list(diagnostic_sources)
        if large_workers is not None:
            manifest["large_topic_workers"] = list(large_workers)
        path.write_text(json.dumps(manifest, ensure_ascii=False, indent=2), encoding="utf-8")
    except (OSError, json.JSONDecodeError):
        pass


def read_key_value_file(path: Path) -> Dict[str, str]:
    values: Dict[str, str] = {}
    try:
        for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
            if ":" in line:
                key, value = line.split(":", 1)
                values[key.strip()] = value.strip()
    except OSError:
        pass
    return values


def read_int(path: Path, default: int = 0) -> int:
    try:
        return int(path.read_text(encoding="utf-8").strip())
    except (OSError, ValueError):
        return default


def read_cpu_snapshot() -> tuple[int, int]:
    try:
        fields = Path("/proc/stat").read_text(encoding="utf-8").splitlines()[0].split()[1:]
        values = [int(value) for value in fields]
        idle = values[3] + (values[4] if len(values) > 4 else 0)
        return sum(values), idle
    except (OSError, ValueError, IndexError):
        return 0, 0


def read_meminfo() -> Dict[str, int]:
    result: Dict[str, int] = {}
    for key, value in read_key_value_file(Path("/proc/meminfo")).items():
        try:
            result[key] = int(value.split()[0])
        except (ValueError, IndexError):
            continue
    return result


def parse_pressure_text(raw: str) -> Dict[str, object]:
    """Parse Linux PSI ``some``/``full`` lines without external dependencies."""
    result: Dict[str, object] = {}
    for line in raw.splitlines():
        fields = line.split()
        if not fields or fields[0] not in ("some", "full"):
            continue
        prefix = fields[0]
        for field in fields[1:]:
            if "=" not in field:
                continue
            key, value = field.split("=", 1)
            output_key = f"{prefix}_{'total_us' if key == 'total' else key}"
            try:
                result[output_key] = int(value) if key == "total" else float(value)
            except ValueError:
                continue
    return result


def read_pressure(resource: str) -> Dict[str, object]:
    try:
        raw = (Path("/proc/pressure") / resource).read_text(encoding="utf-8")
    except OSError:
        return {}
    return parse_pressure_text(raw)


def summarize_cpu_frequencies_khz(values: Iterable[int]) -> Dict[str, float]:
    valid = [float(value) / 1000.0 for value in values if value > 0]
    if not valid:
        return {"min_mhz": math.nan, "mean_mhz": math.nan, "max_mhz": math.nan}
    return {
        "min_mhz": min(valid),
        "mean_mhz": sum(valid) / len(valid),
        "max_mhz": max(valid),
    }


def read_cpu_frequency_summary() -> Dict[str, float]:
    paths = Path("/sys/devices/system/cpu").glob("cpu[0-9]*/cpufreq/scaling_cur_freq")
    return summarize_cpu_frequencies_khz(read_int(path, -1) for path in paths)


def read_thermal_throttle_count() -> int:
    total = 0
    found = False
    root = Path("/sys/devices/system/cpu")
    for name in ("core_throttle_count", "package_throttle_count"):
        for path in root.glob(f"cpu[0-9]*/thermal_throttle/{name}"):
            value = read_int(path, -1)
            if value >= 0:
                total += value
                found = True
    return total if found else -1


def read_net_stats(interface: str) -> Dict[str, int]:
    base = Path("/sys/class/net") / interface / "statistics"
    names = (
        "rx_bytes",
        "tx_bytes",
        "rx_packets",
        "tx_packets",
        "rx_dropped",
        "tx_dropped",
        "rx_errors",
        "tx_errors",
        "rx_missed_errors",
    )
    return {name: read_int(base / name) for name in names}


def max_temperature_c() -> float:
    values: List[float] = []
    for path in Path("/sys/class/thermal").glob("thermal_zone*/temp"):
        value = read_int(path, -1)
        if value >= 0:
            values.append(value / 1000.0 if value > 1000 else float(value))
    return max(values) if values else math.nan


def parse_stat_line(raw: str) -> Dict[str, object]:
    close = raw.rfind(")")
    if close < 0:
        raise ValueError("missing comm terminator in proc stat")
    comm = raw[raw.find("(") + 1 : close]
    fields = raw[close + 2 :].split()
    return {
        "comm": comm,
        "state": fields[0],
        "ticks": int(fields[11]) + int(fields[12]),
        "priority": int(fields[15]),
        "nice": int(fields[16]),
        "threads": int(fields[17]),
        "processor": int(fields[36]),
        "policy": int(fields[38]),
    }


def parse_stat_path(path: Path) -> Optional[Dict[str, object]]:
    try:
        return parse_stat_line(path.read_text(encoding="utf-8"))
    except (OSError, ValueError, IndexError):
        return None


def parse_proc_stat(pid: int) -> Optional[Dict[str, object]]:
    return parse_stat_path(Path("/proc") / str(pid) / "stat")


def parse_thread_stat(pid: int, tid: int) -> Optional[Dict[str, object]]:
    return parse_stat_path(Path("/proc") / str(pid) / "task" / str(tid) / "stat")


PROCESS_PATTERNS = (
    "component_container_mt",
    "livox_pointlio_container",
    "planner_server",
    "ros2 bag record",
    "rviz2",
    "ptp4l",
    "phc2sys",
    "monitor_odom_rogmap.py",
)


def iter_monitored_processes() -> Iterable[tuple[int, str]]:
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit():
            continue
        try:
            cmdline = (entry / "cmdline").read_bytes().replace(b"\0", b" ").decode(
                "utf-8", errors="replace"
            ).strip()
        except OSError:
            continue
        if cmdline and any(pattern in cmdline for pattern in PROCESS_PATTERNS):
            yield int(entry.name), cmdline


def read_process_io(pid: int) -> Dict[str, int]:
    result: Dict[str, int] = {}
    for key, value in read_key_value_file(Path("/proc") / str(pid) / "io").items():
        try:
            result[key] = int(value)
        except ValueError:
            continue
    return result


class SystemRecorder(threading.Thread):
    SYSTEM_FIELDS = (
        "wall_time",
        "monotonic_ns",
        "sample_interval_ms",
        "sample_lag_ms",
        "cpu_percent",
        "cpu_freq_min_mhz",
        "cpu_freq_mean_mhz",
        "cpu_freq_max_mhz",
        "thermal_throttle_count",
        "thermal_throttle_delta",
        "cpu_pressure_some_avg10",
        "cpu_pressure_full_avg10",
        "memory_pressure_some_avg10",
        "memory_pressure_full_avg10",
        "io_pressure_some_avg10",
        "io_pressure_full_avg10",
        "load_1m",
        "load_5m",
        "load_15m",
        "mem_total_mb",
        "mem_available_mb",
        "swap_free_mb",
        "max_temp_c",
        "disk_free_gb",
        "interface",
        "rx_mbps",
        "tx_mbps",
        "rx_dropped_delta",
        "tx_dropped_delta",
        "rx_errors_delta",
        "tx_errors_delta",
        "rx_missed_errors_delta",
    )
    PROCESS_FIELDS = (
        "wall_time",
        "monotonic_ns",
        "pid",
        "comm",
        "state",
        "cpu_percent",
        "rss_mb",
        "threads",
        "processor",
        "policy",
        "priority",
        "nice",
        "read_bytes",
        "write_bytes",
        "voluntary_ctxt_switches",
        "nonvoluntary_ctxt_switches",
        "cmdline",
    )
    THREAD_FIELDS = (
        "wall_time",
        "monotonic_ns",
        "pid",
        "tid",
        "process_comm",
        "thread_comm",
        "state",
        "cpu_percent",
        "processor",
        "policy",
        "priority",
        "nice",
    )

    def __init__(self, session: Path, interface: str, interval_sec: float):
        super().__init__(name="system-recorder", daemon=True)
        self._session = session
        self._interface = interface
        self._interval_sec = interval_sec
        self._stop_event = threading.Event()
        self._system = CsvRecorder(session / "system.csv", self.SYSTEM_FIELDS)
        self._process = CsvRecorder(session / "process.csv", self.PROCESS_FIELDS)
        self._thread = CsvRecorder(session / "thread.csv", self.THREAD_FIELDS)
        self._last_cpu = read_cpu_snapshot()
        self._last_net = read_net_stats(interface)
        self._last_time = time.monotonic()
        self._expected_sample_time = self._last_time
        self._last_throttle_count = read_thermal_throttle_count()
        self._last_process: Dict[int, tuple[int, float]] = {}
        self._last_thread: Dict[tuple[int, int], tuple[int, float]] = {}

    def stop(self) -> None:
        self._stop_event.set()

    def _record_system(self, now_mono: float, now_ns: int) -> None:
        cpu = read_cpu_snapshot()
        total_delta = safe_counter_delta(cpu[0], self._last_cpu[0])
        idle_delta = safe_counter_delta(cpu[1], self._last_cpu[1])
        cpu_percent = (
            100.0 * (total_delta - idle_delta) / total_delta if total_delta > 0 else math.nan
        )
        elapsed = max(1.0e-6, now_mono - self._last_time)
        net = read_net_stats(self._interface)
        net_delta = {
            key: safe_counter_delta(value, self._last_net.get(key, 0))
            for key, value in net.items()
        }
        mem = read_meminfo()
        frequencies = read_cpu_frequency_summary()
        throttle_count = read_thermal_throttle_count()
        throttle_delta: object = ""
        if throttle_count >= 0 and self._last_throttle_count >= 0:
            throttle_delta = safe_counter_delta(throttle_count, self._last_throttle_count)
        pressures = {name: read_pressure(name) for name in ("cpu", "memory", "io")}
        try:
            loads = os.getloadavg()
        except OSError:
            loads = (math.nan, math.nan, math.nan)
        try:
            disk_free_gb = shutil.disk_usage(self._session).free / (1024.0**3)
        except OSError:
            disk_free_gb = math.nan
        self._system.write(
            {
                "wall_time": iso_now(),
                "monotonic_ns": now_ns,
                "sample_interval_ms": elapsed * MSEC_PER_SEC,
                "sample_lag_ms": max(
                    0.0, (now_mono - self._expected_sample_time) * MSEC_PER_SEC
                ),
                "cpu_percent": cpu_percent,
                "cpu_freq_min_mhz": frequencies["min_mhz"],
                "cpu_freq_mean_mhz": frequencies["mean_mhz"],
                "cpu_freq_max_mhz": frequencies["max_mhz"],
                "thermal_throttle_count": throttle_count if throttle_count >= 0 else "",
                "thermal_throttle_delta": throttle_delta,
                "cpu_pressure_some_avg10": pressures["cpu"].get("some_avg10", ""),
                "cpu_pressure_full_avg10": pressures["cpu"].get("full_avg10", ""),
                "memory_pressure_some_avg10": pressures["memory"].get("some_avg10", ""),
                "memory_pressure_full_avg10": pressures["memory"].get("full_avg10", ""),
                "io_pressure_some_avg10": pressures["io"].get("some_avg10", ""),
                "io_pressure_full_avg10": pressures["io"].get("full_avg10", ""),
                "load_1m": loads[0],
                "load_5m": loads[1],
                "load_15m": loads[2],
                "mem_total_mb": mem.get("MemTotal", 0) / 1024.0,
                "mem_available_mb": mem.get("MemAvailable", 0) / 1024.0,
                "swap_free_mb": mem.get("SwapFree", 0) / 1024.0,
                "max_temp_c": max_temperature_c(),
                "disk_free_gb": disk_free_gb,
                "interface": self._interface,
                "rx_mbps": net_delta.get("rx_bytes", 0) * 8.0 / elapsed / 1.0e6,
                "tx_mbps": net_delta.get("tx_bytes", 0) * 8.0 / elapsed / 1.0e6,
                "rx_dropped_delta": net_delta.get("rx_dropped", 0),
                "tx_dropped_delta": net_delta.get("tx_dropped", 0),
                "rx_errors_delta": net_delta.get("rx_errors", 0),
                "tx_errors_delta": net_delta.get("tx_errors", 0),
                "rx_missed_errors_delta": net_delta.get("rx_missed_errors", 0),
            }
        )
        self._last_cpu = cpu
        self._last_net = net
        self._last_time = now_mono
        self._expected_sample_time = now_mono + self._interval_sec
        self._last_throttle_count = throttle_count

    def _record_processes(self, now_mono: float, now_ns: int) -> None:
        clock_ticks = float(os.sysconf(os.sysconf_names["SC_CLK_TCK"]))
        seen = set()
        for pid, cmdline in iter_monitored_processes():
            stat = parse_proc_stat(pid)
            if stat is None:
                continue
            seen.add(pid)
            previous = self._last_process.get(pid)
            cpu_percent = math.nan
            if previous is not None and now_mono > previous[1]:
                tick_delta = safe_counter_delta(int(stat["ticks"]), previous[0])
                cpu_percent = 100.0 * tick_delta / clock_ticks / (now_mono - previous[1])
            self._last_process[pid] = (int(stat["ticks"]), now_mono)
            status = read_key_value_file(Path("/proc") / str(pid) / "status")
            io = read_process_io(pid)
            try:
                rss_kb = int(status.get("VmRSS", "0 kB").split()[0])
            except (ValueError, IndexError):
                rss_kb = 0
            self._process.write(
                {
                    "wall_time": iso_now(),
                    "monotonic_ns": now_ns,
                    "pid": pid,
                    "comm": stat["comm"],
                    "state": stat["state"],
                    "cpu_percent": cpu_percent,
                    "rss_mb": rss_kb / 1024.0,
                    "threads": stat["threads"],
                    "processor": stat["processor"],
                    "policy": stat["policy"],
                    "priority": stat["priority"],
                    "nice": stat["nice"],
                    "read_bytes": io.get("read_bytes", 0),
                    "write_bytes": io.get("write_bytes", 0),
                    "voluntary_ctxt_switches": status.get("voluntary_ctxt_switches", ""),
                    "nonvoluntary_ctxt_switches": status.get("nonvoluntary_ctxt_switches", ""),
                    "cmdline": cmdline[:1000],
                }
            )
            task_dir = Path("/proc") / str(pid) / "task"
            try:
                task_entries = tuple(task_dir.iterdir())
            except OSError:
                task_entries = ()
            for task_entry in task_entries:
                if not task_entry.name.isdigit():
                    continue
                tid = int(task_entry.name)
                thread_stat = parse_thread_stat(pid, tid)
                if thread_stat is None:
                    continue
                thread_key = (pid, tid)
                previous_thread = self._last_thread.get(thread_key)
                thread_cpu_percent = math.nan
                if previous_thread is not None and now_mono > previous_thread[1]:
                    tick_delta = safe_counter_delta(
                        int(thread_stat["ticks"]), previous_thread[0]
                    )
                    thread_cpu_percent = (
                        100.0 * tick_delta / clock_ticks / (now_mono - previous_thread[1])
                    )
                self._last_thread[thread_key] = (int(thread_stat["ticks"]), now_mono)
                self._thread.write(
                    {
                        "wall_time": iso_now(),
                        "monotonic_ns": now_ns,
                        "pid": pid,
                        "tid": tid,
                        "process_comm": stat["comm"],
                        "thread_comm": thread_stat["comm"],
                        "state": thread_stat["state"],
                        "cpu_percent": thread_cpu_percent,
                        "processor": thread_stat["processor"],
                        "policy": thread_stat["policy"],
                        "priority": thread_stat["priority"],
                        "nice": thread_stat["nice"],
                    }
                )
        self._last_process = {
            pid: value for pid, value in self._last_process.items() if pid in seen
        }
        self._last_thread = {
            key: value for key, value in self._last_thread.items() if key[0] in seen
        }

    def run(self) -> None:
        while not self._stop_event.is_set():
            start = time.monotonic()
            mono_ns = time.monotonic_ns()
            self._record_system(start, mono_ns)
            self._record_processes(start, mono_ns)
            self._system.flush()
            self._process.flush()
            self._thread.flush()
            remaining = self._interval_sec - (time.monotonic() - start)
            self._stop_event.wait(max(0.05, remaining))
        self._system.close()
        self._process.close()
        self._thread.close()


class FileFollower(threading.Thread):
    """Copy lines appended to an optional source log after monitoring starts."""

    def __init__(self, source: Path, destination: Path):
        super().__init__(name=f"follow-{source.name}", daemon=True)
        self._source = source
        self._destination = destination
        self._stop_event = threading.Event()
        self._source_existed_at_start = source.exists()
        self._new_bytes = 0
        self._new_lines = 0

    def stop(self) -> None:
        self._stop_event.set()

    def status(self) -> Dict[str, object]:
        return {
            "source": str(self._source),
            "destination": str(self._destination),
            "source_existed_at_start": self._source_existed_at_start,
            "source_exists_at_end": self._source.exists(),
            "new_bytes": self._new_bytes,
            "new_lines": self._new_lines,
        }

    def run(self) -> None:
        offset = 0
        initialized = False
        with self._destination.open("w", encoding="utf-8") as output:
            while not self._stop_event.is_set():
                try:
                    size = self._source.stat().st_size
                    if not initialized:
                        if self._source_existed_at_start:
                            existing_tail = read_tail_lines(self._source, 200)
                            if existing_tail:
                                output.write("===== pre-existing tail at monitor start =====\n")
                                output.write(existing_tail)
                                output.flush()
                            offset = size
                        else:
                            offset = 0
                        initialized = True
                    if size < offset:
                        offset = 0
                    if size > offset:
                        with self._source.open("r", encoding="utf-8", errors="replace") as source:
                            source.seek(offset)
                            content = source.read()
                            offset = source.tell()
                        if content:
                            output.write(content)
                            output.flush()
                            self._new_bytes += len(content.encode("utf-8"))
                            self._new_lines += len(content.splitlines())
                except OSError:
                    pass
                self._stop_event.wait(0.5)


def read_tail_lines(path: Path, line_count: int) -> str:
    if line_count <= 0:
        return ""
    try:
        with path.open("rb") as stream:
            stream.seek(0, os.SEEK_END)
            position = stream.tell()
            data = b""
            while position > 0 and data.count(b"\n") <= line_count:
                read_size = min(64 * 1024, position)
                position -= read_size
                stream.seek(position)
                data = stream.read(read_size) + data
    except OSError:
        return ""
    lines = data.decode("utf-8", errors="replace").splitlines(keepends=True)
    return "".join(lines[-line_count:])


class TopologyRecorder(threading.Thread):
    def __init__(
        self,
        session: Path,
        topics: Sequence[TopicSpec],
        interface: str,
        interval_sec: float,
    ):
        super().__init__(name="topology-recorder", daemon=True)
        self._path = session / "ros_topology.log"
        self._topics = topics
        self._interface = interface
        self._interval_sec = interval_sec
        self._stop_event = threading.Event()
        self._first = True

    def stop(self) -> None:
        self._stop_event.set()

    def _commands(self) -> List[Sequence[str]]:
        commands: List[Sequence[str]] = [
            ("ros2", "node", "list"),
            ("ros2", "component", "list"),
        ]
        for item in self._topics:
            commands.append(("ros2", "topic", "info", item.topic, "--verbose"))
        commands.append(("ros2", "topic", "info", "/bond", "--verbose"))
        commands.extend(
            [
                ("ip", "-s", "link", "show", "dev", self._interface),
                ("timedatectl", "status"),
            ]
        )
        if shutil.which("ethtool"):
            commands.append(("ethtool", "-S", self._interface))
        if self._first:
            commands.extend(
                [
                    ("ros2", "param", "dump", "/laserMapping"),
                    ("ros2", "param", "dump", "/planner_server"),
                    ("ros2", "param", "dump", "/lifecycle_manager_navigation"),
                ]
            )
            commands.extend(("ros2", "lifecycle", "get", node) for node in LIFECYCLE_NODES)
        return commands

    def run(self) -> None:
        with self._path.open("w", encoding="utf-8") as output:
            while not self._stop_event.is_set():
                output.write(f"\n===== snapshot {iso_now()} =====\n")
                for command in self._commands():
                    if self._stop_event.is_set():
                        break
                    output.write(run_command(command))
                output.flush()
                self._first = False
                self._stop_event.wait(self._interval_sec)


class BagRecorder:
    def __init__(self, session: Path):
        self._log: Optional[TextIO] = None
        self._process: Optional[subprocess.Popen] = None
        command = [
            "ros2",
            "bag",
            "record",
            "-o",
            str(session / "diagnostic_bag"),
            *bag_topics(),
        ]
        self._log = (session / "rosbag.log").open("w", encoding="utf-8")
        self._process = subprocess.Popen(
            command,
            stdout=self._log,
            stderr=subprocess.STDOUT,
            text=True,
            start_new_session=True,
        )

    def stop(self) -> None:
        if self._process is not None and self._process.poll() is None:
            try:
                os.killpg(self._process.pid, signal.SIGINT)
                self._process.wait(timeout=15)
            except (OSError, subprocess.TimeoutExpired):
                try:
                    os.killpg(self._process.pid, signal.SIGTERM)
                    self._process.wait(timeout=5)
                except (OSError, subprocess.TimeoutExpired):
                    pass
        if self._log is not None:
            self._log.close()


class LargeTopicWorkers:
    """Run each large-message subscription in its own Python process."""

    def __init__(
        self,
        session: Path,
        specs: Sequence[TopicSpec],
        duration: float,
        summary_interval: float,
    ):
        self._workers: List[Dict[str, object]] = []
        for spec in specs:
            stem = large_worker_output_stem(spec.topic)
            log = (session / f"{stem}_worker.log").open("w", encoding="utf-8")
            try:
                process = subprocess.Popen(
                    large_worker_command(session, spec, duration, summary_interval),
                    stdout=log,
                    stderr=subprocess.STDOUT,
                    text=True,
                    start_new_session=True,
                )
            except OSError:
                log.close()
                raise
            self._workers.append({"topic": spec.topic, "process": process, "log": log})

    def stop(self) -> None:
        for worker in self._workers:
            process = worker["process"]
            if isinstance(process, subprocess.Popen) and process.poll() is None:
                try:
                    os.killpg(process.pid, signal.SIGINT)
                except OSError:
                    pass
        for worker in self._workers:
            process = worker["process"]
            if isinstance(process, subprocess.Popen) and process.poll() is None:
                try:
                    process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    try:
                        os.killpg(process.pid, signal.SIGTERM)
                        process.wait(timeout=5)
                    except (OSError, subprocess.TimeoutExpired):
                        pass
            log = worker["log"]
            if hasattr(log, "close"):
                log.close()

    def status(self) -> List[Dict[str, object]]:
        result = []
        for worker in self._workers:
            process = worker["process"]
            result.append(
                {
                    "topic": worker["topic"],
                    "pid": process.pid if isinstance(process, subprocess.Popen) else "",
                    "returncode": (
                        process.poll() if isinstance(process, subprocess.Popen) else ""
                    ),
                }
            )
        return result


EVENT_FIELDS = (
    "wall_time",
    "arrival_unix_ns",
    "arrival_monotonic_ns",
    "topic",
    "header_stamp_ns",
    "age_ms",
    "interarrival_ms",
    "sensor_dt_ms",
    "sensor_time_regression",
    "payload_bytes",
    "points",
    "frame_id",
    "position_x",
    "position_y",
    "position_z",
    "origin_x",
    "origin_y",
    "origin_z",
    "map_width",
    "map_height",
    "map_resolution",
    "map_hash_sampled",
    "map_crc32",
    "map_changed",
)

SUMMARY_FIELDS = (
    "wall_time",
    "topic",
    "scope",
    "count",
    "hz",
    "max_gap_ms",
    "mean_age_ms",
    "max_age_ms",
    "sensor_time_regressions",
    "payload_bytes",
    "points",
)

ROSOUT_FIELDS = (
    "wall_time",
    "stamp_ns",
    "level",
    "name",
    "file",
    "function",
    "line",
    "message",
)

BOND_FIELDS = (
    "wall_time",
    "arrival_unix_ns",
    "bond_id",
    "instance_id",
    "active",
    "heartbeat_period_sec",
    "heartbeat_timeout_sec",
)

LIFECYCLE_FIELDS = (
    "wall_time",
    "arrival_unix_ns",
    "event_stamp_ns",
    "topic",
    "node",
    "transition_id",
    "transition_label",
    "start_state_id",
    "start_state_label",
    "goal_state_id",
    "goal_state_label",
)

NODE_LIVENESS_FIELDS = (
    "wall_time",
    "monotonic_ns",
    "node",
    "present",
    "observed_node_count",
)

MONITOR_HEALTH_FIELDS = (
    "wall_time",
    "monotonic_ns",
    "actual_interval_ms",
    "timer_lag_ms",
    "primary_event_total",
    "primary_event_delta",
)


def _message_details(kind: str, msg: object) -> Dict[str, object]:
    header = getattr(msg, "header", None)
    stamp_ns = stamp_to_ns(getattr(header, "stamp", object())) if header is not None else 0
    result: Dict[str, object] = {
        "header_stamp_ns": stamp_ns,
        "payload_bytes": 0,
        "points": 0,
        "frame_id": getattr(header, "frame_id", "") if header is not None else "",
        "position_x": "",
        "position_y": "",
        "position_z": "",
        "origin_x": "",
        "origin_y": "",
        "origin_z": "",
        "map_width": "",
        "map_height": "",
        "map_resolution": "",
        "map_hash_sampled": "",
        "map_crc32": "",
        "map_changed": "",
    }
    if kind == "odom":
        position = msg.pose.pose.position
        result.update(
            position_x=position.x,
            position_y=position.y,
            position_z=position.z,
        )
    elif kind == "livox_custom":
        result["points"] = int(getattr(msg, "point_num", len(getattr(msg, "points", []))))
        result["payload_bytes"] = int(result["points"]) * 19
    elif kind == "pointcloud2":
        result["points"] = int(getattr(msg, "width", 0)) * int(getattr(msg, "height", 0))
        result["payload_bytes"] = len(getattr(msg, "data", b""))
    elif kind == "occupancy_grid":
        origin = msg.info.origin.position
        result.update(origin_x=origin.x, origin_y=origin.y, origin_z=origin.z)
        result.update(
            map_width=int(msg.info.width),
            map_height=int(msg.info.height),
            map_resolution=float(msg.info.resolution),
        )
        result["payload_bytes"] = len(getattr(msg, "data", []))
        result["points"] = int(msg.info.width) * int(msg.info.height)
    return result


def run_large_topic_worker(args: argparse.Namespace) -> int:
    spec = next(
        (item for item in TEST_ONLY_TOPICS if item.topic == args._large_worker_topic),
        None,
    )
    if spec is None or not args._session_dir:
        print("[ERROR] invalid isolated large-topic worker arguments", file=sys.stderr)
        return 2
    try:
        import rclpy
        from rclpy.executors import ExternalShutdownException
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
        if spec.kind == "livox_custom":
            from livox_ros_driver2.msg import CustomMsg

            message_type = CustomMsg
        else:
            from sensor_msgs.msg import PointCloud2

            message_type = PointCloud2
    except ImportError as exc:
        print(f"[ERROR] large-topic worker dependency unavailable: {exc}", file=sys.stderr)
        return 2

    session = Path(args._session_dir).resolve()
    stem = large_worker_output_stem(spec.topic)
    event_csv = CsvRecorder(session / f"{stem}_events.csv", EVENT_FIELDS)
    summary_csv = CsvRecorder(session / f"{stem}_summary.csv", SUMMARY_FIELDS)
    tracker = TopicStats(spec.topic)
    stop_requested = threading.Event()
    stop_reason = "normal"

    def request_stop(signum: int, _frame: object) -> None:
        nonlocal stop_reason
        stop_reason = f"signal_{signum}"
        stop_requested.set()

    previous_sigint = signal.getsignal(signal.SIGINT)
    previous_sigterm = signal.getsignal(signal.SIGTERM)
    rclpy.init(args=None)
    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    node = Node(f"odom_rogmap_large_{stem}_{os.getpid()}")
    qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )

    def callback(msg: object) -> None:
        arrival_ns = time.time_ns()
        monotonic_ns = time.monotonic_ns()
        details = _message_details(spec.kind, msg)
        event = tracker.observe(
            arrival_ns,
            int(details["header_stamp_ns"]),
            int(details["payload_bytes"]),
            int(details["points"]),
        )
        event_csv.write(
            {
                "wall_time": iso_now(),
                "arrival_unix_ns": arrival_ns,
                "arrival_monotonic_ns": monotonic_ns,
                "topic": spec.topic,
                **details,
                **event,
            }
        )

    def write_window_summary() -> None:
        summary_csv.write(
            {
                "wall_time": iso_now(),
                "topic": spec.topic,
                "scope": "window",
                **tracker.window_summary(),
            }
        )
        event_csv.flush()
        summary_csv.flush()

    subscription = node.create_subscription(message_type, spec.topic, callback, qos)
    timer = node.create_timer(args.summary_interval, write_window_summary)
    _ = subscription, timer
    start_mono = time.monotonic()
    try:
        while rclpy.ok() and not stop_requested.is_set():
            rclpy.spin_once(node, timeout_sec=0.2)
            if args.duration > 0 and time.monotonic() - start_mono >= args.duration:
                stop_reason = "duration_elapsed"
                break
    except ExternalShutdownException:
        stop_reason = "rclpy_shutdown"
    finally:
        write_window_summary()
        summary_csv.write(
            {
                "wall_time": iso_now(),
                "topic": spec.topic,
                "scope": "total",
                **tracker.summary(),
            }
        )
        event_csv.close()
        summary_csv.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        signal.signal(signal.SIGINT, previous_sigint)
        signal.signal(signal.SIGTERM, previous_sigterm)
        print(f"[INFO] worker topic={spec.topic} stopped reason={stop_reason}")
    return 0


def run_monitor(args: argparse.Namespace) -> int:
    try:
        import rclpy
        from bond.msg import Status
        from lifecycle_msgs.msg import TransitionEvent
        from nav_msgs.msg import OccupancyGrid, Odometry
        from rcl_interfaces.msg import Log
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
        from sensor_msgs.msg import Imu
    except ImportError as exc:
        print(
            f"[ERROR] ROS 2 Python dependency unavailable: {exc}\n"
            "Run: source /opt/ros/humble/setup.bash && source install/setup.bash",
            file=sys.stderr,
        )
        return 2

    topics = primary_topics(args.mode)
    message_types = {
        "odom": Odometry,
        "imu": Imu,
        "occupancy_grid": OccupancyGrid,
    }

    session = create_session_dir(Path(args.output_dir).expanduser().resolve(), args.mode)
    manifest_path = session / "manifest.json"
    write_manifest(manifest_path, args, mode_topics(args.mode))
    print(f"[INFO] monitor mode={args.mode} output={session}")
    if args.mode == "test":
        print("[INFO] test-only large topics use isolated worker processes.")

    rclpy.init(args=None)
    node = Node(f"odom_rogmap_monitor_{os.getpid()}")
    event_csv = CsvRecorder(session / "topic_events.csv", EVENT_FIELDS)
    summary_csv = CsvRecorder(session / "topic_summary.csv", SUMMARY_FIELDS)
    rosout_csv = CsvRecorder(session / "rosout.csv", ROSOUT_FIELDS)
    bond_csv = CsvRecorder(session / "bond.csv", BOND_FIELDS)
    lifecycle_csv = CsvRecorder(session / "lifecycle.csv", LIFECYCLE_FIELDS)
    liveness_csv = CsvRecorder(session / "node_liveness.csv", NODE_LIVENESS_FIELDS)
    health_csv = CsvRecorder(session / "monitor_health.csv", MONITOR_HEALTH_FIELDS)
    trackers = {item.topic: TopicStats(item.topic) for item in topics}
    map_sampler = MapContentSampler(args.map_hash_interval)

    sensor_qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )
    rosout_qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=200,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )
    control_qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=100,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )

    subscriptions = []
    for spec in topics:
        tracker = trackers[spec.topic]

        def callback(msg: object, topic_spec: TopicSpec = spec, topic_tracker: TopicStats = tracker) -> None:
            arrival_ns = time.time_ns()
            monotonic_ns = time.monotonic_ns()
            details = _message_details(topic_spec.kind, msg)
            if topic_spec.kind == "occupancy_grid":
                details.update(map_sampler.observe(msg.data, monotonic_ns))
            event = topic_tracker.observe(
                arrival_ns,
                int(details["header_stamp_ns"]),
                int(details["payload_bytes"]),
                int(details["points"]),
            )
            event_csv.write(
                {
                    "wall_time": iso_now(),
                    "arrival_unix_ns": arrival_ns,
                    "arrival_monotonic_ns": monotonic_ns,
                    "topic": topic_spec.topic,
                    **details,
                    **event,
                }
            )

        subscriptions.append(
            node.create_subscription(message_types[spec.kind], spec.topic, callback, sensor_qos)
        )

    def rosout_callback(msg: object) -> None:
        if not should_capture_rosout(int(msg.level), str(msg.msg)):
            return
        rosout_csv.write(
            {
                "wall_time": iso_now(),
                "stamp_ns": stamp_to_ns(msg.stamp),
                "level": int(msg.level),
                "name": msg.name,
                "file": msg.file,
                "function": msg.function,
                "line": int(msg.line),
                "message": msg.msg,
            }
        )

    subscriptions.append(node.create_subscription(Log, "/rosout", rosout_callback, rosout_qos))

    def bond_callback(msg: object) -> None:
        bond_csv.write(bond_status_to_row(msg, time.time_ns(), iso_now()))

    subscriptions.append(node.create_subscription(Status, "/bond", bond_callback, control_qos))

    for lifecycle_node in LIFECYCLE_NODES:
        transition_topic = f"{lifecycle_node}/transition_event"

        def transition_callback(
            msg: object,
            topic: str = transition_topic,
        ) -> None:
            lifecycle_csv.write(
                transition_event_to_row(topic, msg, time.time_ns(), iso_now())
            )

        subscriptions.append(
            node.create_subscription(
                TransitionEvent, transition_topic, transition_callback, control_qos
            )
        )

    def write_summaries() -> None:
        wall_time = iso_now()
        for topic, tracker in trackers.items():
            summary_csv.write(
                {"wall_time": wall_time, "topic": topic, "scope": "window", **tracker.window_summary()}
            )
        event_csv.flush()
        summary_csv.flush()
        rosout_csv.flush()
        bond_csv.flush()
        lifecycle_csv.flush()

    summary_timer = node.create_timer(args.summary_interval, write_summaries)

    def write_node_liveness() -> None:
        monotonic_ns = time.monotonic_ns()
        discovered = fully_qualified_node_names(node.get_node_names_and_namespaces())
        wall_time = iso_now()
        for critical_node in CRITICAL_NODES:
            liveness_csv.write(
                {
                    "wall_time": wall_time,
                    "monotonic_ns": monotonic_ns,
                    "node": critical_node,
                    "present": critical_node in discovered,
                    "observed_node_count": len(discovered),
                }
            )
        liveness_csv.flush()

    health_last_ns = time.monotonic_ns()
    health_expected_ns = health_last_ns + int(args.monitor_health_interval * NSEC_PER_SEC)
    health_last_event_total = 0

    def write_monitor_health() -> None:
        nonlocal health_last_ns, health_expected_ns, health_last_event_total
        now_ns = time.monotonic_ns()
        event_total = sum(int(tracker.summary()["count"]) for tracker in trackers.values())
        health_csv.write(
            {
                "wall_time": iso_now(),
                "monotonic_ns": now_ns,
                "actual_interval_ms": (now_ns - health_last_ns) / 1.0e6,
                "timer_lag_ms": max(0.0, (now_ns - health_expected_ns) / 1.0e6),
                "primary_event_total": event_total,
                "primary_event_delta": safe_counter_delta(
                    event_total, health_last_event_total
                ),
            }
        )
        health_csv.flush()
        health_last_ns = now_ns
        health_expected_ns += int(args.monitor_health_interval * NSEC_PER_SEC)
        if health_expected_ns < now_ns:
            health_expected_ns = now_ns + int(
                args.monitor_health_interval * NSEC_PER_SEC
            )
        health_last_event_total = event_total

    liveness_timer = node.create_timer(args.node_liveness_interval, write_node_liveness)
    health_timer = node.create_timer(args.monitor_health_interval, write_monitor_health)
    _ = summary_timer, liveness_timer, health_timer, subscriptions

    system_recorder = SystemRecorder(session, args.interface, args.system_interval)
    topology_recorder = TopologyRecorder(
        session, topology_topics(), args.interface, args.snapshot_interval
    )
    followers = [
        FileFollower(
            Path(f"/tmp/ptp4l_{args.interface}.log"),
            session / "ptp4l.log",
        ),
        FileFollower(
            Path(f"/tmp/phc2sys_{args.interface}.log"),
            session / "phc2sys.log",
        ),
        FileFollower(
            Path("/tmp/point_lio_log/performance.csv"),
            session / "point_lio_performance.csv",
        ),
        FileFollower(
            Path("/tmp/point_lio_log/imu_pbp.txt"),
            session / "point_lio_imu_pbp.txt",
        ),
        FileFollower(
            Path("/tmp/rog_map_perf_detailed.csv"),
            session / "rog_map_perf_detailed.csv",
        ),
        FileFollower(
            Path("/tmp/rog_map_perf_summary.csv"),
            session / "rog_map_perf_summary.csv",
        ),
        FileFollower(
            Path("/tmp/minco_perf_detailed.csv"),
            session / "minco_perf_detailed.csv",
        ),
    ]
    system_recorder.start()
    topology_recorder.start()
    for follower in followers:
        follower.start()

    large_workers: Optional[LargeTopicWorkers] = None
    if large_worker_topics(args.mode):
        try:
            large_workers = LargeTopicWorkers(
                session,
                large_worker_topics(args.mode),
                args.duration,
                args.summary_interval,
            )
        except OSError as exc:
            print(f"[ERROR] failed to start large-topic workers: {exc}", file=sys.stderr)

    bag: Optional[BagRecorder] = None
    if args.with_bag:
        try:
            bag = BagRecorder(session)
        except OSError as exc:
            print(f"[ERROR] failed to start rosbag: {exc}", file=sys.stderr)

    stop_requested = threading.Event()
    stop_reason = "normal"

    def request_stop(signum: int, _frame: object) -> None:
        nonlocal stop_reason
        stop_reason = f"signal_{signum}"
        stop_requested.set()

    previous_sigint = signal.signal(signal.SIGINT, request_stop)
    previous_sigterm = signal.signal(signal.SIGTERM, request_stop)
    start_mono = time.monotonic()
    try:
        while rclpy.ok() and not stop_requested.is_set():
            rclpy.spin_once(node, timeout_sec=0.2)
            if args.duration > 0 and time.monotonic() - start_mono >= args.duration:
                stop_reason = "duration_elapsed"
                break
    finally:
        signal.signal(signal.SIGINT, previous_sigint)
        signal.signal(signal.SIGTERM, previous_sigterm)
        write_summaries()
        wall_time = iso_now()
        for topic, tracker in trackers.items():
            summary_csv.write(
                {"wall_time": wall_time, "topic": topic, "scope": "total", **tracker.summary()}
            )
        if bag is not None:
            bag.stop()
        if large_workers is not None:
            large_workers.stop()
        topology_recorder.stop()
        system_recorder.stop()
        for follower in followers:
            follower.stop()
        topology_recorder.join(timeout=10)
        system_recorder.join(timeout=5)
        for follower in followers:
            follower.join(timeout=2)
        event_csv.close()
        summary_csv.close()
        rosout_csv.close()
        bond_csv.close()
        lifecycle_csv.close()
        liveness_csv.close()
        health_csv.close()
        node.destroy_node()
        rclpy.shutdown()
        diagnostic_sources = [follower.status() for follower in followers]
        for status in diagnostic_sources:
            source = str(status["source"])
            if ("performance" in source or "perf_" in source) and not status["new_bytes"]:
                print(f"[WARN] no new internal performance evidence: {source}")
        worker_status = large_workers.status() if large_workers is not None else []
        for status in worker_status:
            if status["returncode"] != 0:
                print(
                    f"[WARN] large-topic worker failed: "
                    f"topic={status['topic']} returncode={status['returncode']}"
                )
        update_manifest_end(
            manifest_path,
            stop_reason,
            diagnostic_sources=diagnostic_sources,
            large_workers=worker_status,
        )
        print(f"[INFO] monitor stopped reason={stop_reason} output={session}")
    return 0


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Record Point-LIO/ROGMap timing, ROS topology and host diagnostics."
    )
    parser.add_argument(
        "--mode",
        choices=("match", "test"),
        default="match",
        help="match avoids large topics; test also observes merged/full point clouds",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=0.0,
        help="recording duration in seconds; 0 means until SIGINT/SIGTERM",
    )
    parser.add_argument(
        "--output-dir",
        default=str(DEFAULT_OUTPUT_ROOT),
        help=f"session directory root (default: {DEFAULT_OUTPUT_ROOT})",
    )
    parser.add_argument("--interface", default=DEFAULT_INTERFACE)
    parser.add_argument("--system-interval", type=float, default=1.0)
    parser.add_argument("--summary-interval", type=float, default=1.0)
    parser.add_argument("--snapshot-interval", type=float, default=30.0)
    parser.add_argument("--map-hash-interval", type=float, default=1.0)
    parser.add_argument("--monitor-health-interval", type=float, default=0.1)
    parser.add_argument("--node-liveness-interval", type=float, default=1.0)
    parser.add_argument(
        "--with-bag",
        action="store_true",
        help="test mode only: record raw boundary topics into a diagnostic rosbag",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="print selected topics and exit without creating subscriptions",
    )
    parser.add_argument("--_large-worker-topic", default="", help=argparse.SUPPRESS)
    parser.add_argument("--_session-dir", default="", help=argparse.SUPPRESS)
    return parser


def validate_args(parser: argparse.ArgumentParser, args: argparse.Namespace) -> None:
    if args.duration < 0:
        parser.error("--duration must be non-negative")
    for name in (
        "system_interval",
        "summary_interval",
        "snapshot_interval",
        "map_hash_interval",
        "monitor_health_interval",
        "node_liveness_interval",
    ):
        if getattr(args, name) <= 0:
            parser.error(f"--{name.replace('_', '-')} must be positive")
    if args.with_bag and args.mode != "test":
        parser.error("--with-bag is available only with --mode test")
    if args._large_worker_topic:
        if args._large_worker_topic not in {item.topic for item in TEST_ONLY_TOPICS}:
            parser.error("invalid isolated large-topic worker")
        if not args._session_dir:
            parser.error("--_session-dir is required for isolated large-topic worker")


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = build_argument_parser()
    args = parser.parse_args(argv)
    validate_args(parser, args)
    if args._large_worker_topic:
        return run_large_topic_worker(args)
    if args.dry_run:
        print(f"mode={args.mode}")
        for item in primary_topics(args.mode):
            print(f"{item.topic}\t{item.kind}\tlane=primary")
        for item in large_worker_topics(args.mode):
            print(f"{item.topic}\t{item.kind}\tlane=isolated-worker")
        if args.with_bag:
            print("bag_topics=" + ",".join(bag_topics()))
        return 0
    return run_monitor(args)


if __name__ == "__main__":
    raise SystemExit(main())
