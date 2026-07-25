#!/usr/bin/env python3
"""ROS 2 bag reader for the standalone calibration tool."""

from __future__ import annotations

import re
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np


ODOMETRY_TYPE = "nav_msgs/msg/Odometry"


class BagReadError(RuntimeError):
    """Raised when a bag cannot provide the requested calibration data."""


@dataclass
class OdomBagData:
    timestamps: np.ndarray
    bag_timestamps: np.ndarray
    positions: np.ndarray
    quaternions: np.ndarray
    frame_ids: List[str]
    child_frame_ids: List[str]


@dataclass
class ScalarBagData:
    timestamps: np.ndarray
    values: np.ndarray


def _import_rosbag_modules():
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except ImportError as exc:
        raise BagReadError(
            "ROS 2 Python bag modules are unavailable. Source /opt/ros/humble/setup.bash "
            "and this workspace's install/setup.bash before running the tool."
        ) from exc
    return rosbag2_py, deserialize_message, get_message


def _storage_identifier(path: Path) -> str:
    if path.is_file():
        if path.suffix == ".mcap":
            return "mcap"
        if path.suffix == ".db3":
            return "sqlite3"

    metadata = path / "metadata.yaml"
    if metadata.is_file():
        match = re.search(
            r"^\s*storage_identifier:\s*['\"]?([^'\"\s]+)",
            metadata.read_text(encoding="utf-8", errors="replace"),
            flags=re.MULTILINE,
        )
        if match:
            return match.group(1)
    return "sqlite3"


def _open_reader(path: Path):
    rosbag2_py, deserialize_message, get_message = _import_rosbag_modules()
    if not path.exists():
        raise BagReadError(f"Bag path does not exist: {path}")

    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(
        uri=str(path), storage_id=_storage_identifier(path)
    )
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr", output_serialization_format="cdr"
    )
    try:
        reader.open(storage_options, converter_options)
    except Exception as exc:
        raise BagReadError(f"Failed to open ROS 2 bag '{path}': {exc}") from exc
    return reader, deserialize_message, get_message


def list_topics(path: Path) -> List[Tuple[str, str]]:
    reader, _, _ = _open_reader(Path(path))
    return sorted((item.name, item.type) for item in reader.get_all_topics_and_types())


def _message_stamp_seconds(msg, fallback_nanoseconds: int) -> float:
    header = getattr(msg, "header", None)
    stamp = getattr(header, "stamp", None)
    if stamp is not None:
        sec = int(getattr(stamp, "sec", 0))
        nanosec = int(getattr(stamp, "nanosec", 0))
        if sec != 0 or nanosec != 0:
            return float(sec) + float(nanosec) * 1.0e-9
    return float(fallback_nanoseconds) * 1.0e-9


def read_calibration_topics(
    path: Path,
    odom_topic: str,
    gimbal_topic: Optional[str] = None,
) -> Tuple[OdomBagData, Optional[ScalarBagData], List[Tuple[str, str]]]:
    reader, deserialize_message, get_message = _open_reader(Path(path))
    topics = sorted((item.name, item.type) for item in reader.get_all_topics_and_types())
    type_by_topic: Dict[str, str] = dict(topics)

    if odom_topic not in type_by_topic:
        available = "\n".join(f"  {name} ({msg_type})" for name, msg_type in topics)
        raise BagReadError(
            f"Odometry topic '{odom_topic}' was not found. Available topics:\n{available}"
        )
    if type_by_topic[odom_topic] != ODOMETRY_TYPE:
        raise BagReadError(
            f"Topic '{odom_topic}' has type '{type_by_topic[odom_topic]}', "
            f"expected '{ODOMETRY_TYPE}'."
        )
    if gimbal_topic and gimbal_topic not in type_by_topic:
        raise BagReadError(f"Optional gimbal topic '{gimbal_topic}' was not found in the bag.")

    odom_class = get_message(type_by_topic[odom_topic])
    gimbal_class = get_message(type_by_topic[gimbal_topic]) if gimbal_topic else None

    timestamps: List[float] = []
    bag_timestamps: List[float] = []
    positions: List[Tuple[float, float, float]] = []
    quaternions: List[Tuple[float, float, float, float]] = []
    frame_ids: List[str] = []
    child_frame_ids: List[str] = []
    gimbal_timestamps: List[float] = []
    gimbal_values: List[float] = []

    while reader.has_next():
        topic, raw_data, bag_stamp = reader.read_next()
        if topic == odom_topic:
            msg = deserialize_message(raw_data, odom_class)
            pose = msg.pose.pose
            timestamps.append(_message_stamp_seconds(msg, bag_stamp))
            bag_timestamps.append(float(bag_stamp) * 1.0e-9)
            positions.append(
                (float(pose.position.x), float(pose.position.y), float(pose.position.z))
            )
            quaternions.append(
                (
                    float(pose.orientation.x),
                    float(pose.orientation.y),
                    float(pose.orientation.z),
                    float(pose.orientation.w),
                )
            )
            frame_ids.append(str(msg.header.frame_id))
            child_frame_ids.append(str(msg.child_frame_id))
        elif gimbal_topic and topic == gimbal_topic:
            msg = deserialize_message(raw_data, gimbal_class)
            if not hasattr(msg, "yaw_camerainit_to_gimbal"):
                raise BagReadError(
                    f"Gimbal topic '{gimbal_topic}' does not contain "
                    "'yaw_camerainit_to_gimbal'."
                )
            gimbal_timestamps.append(_message_stamp_seconds(msg, bag_stamp))
            gimbal_values.append(float(msg.yaw_camerainit_to_gimbal))

    if not timestamps:
        raise BagReadError(f"Topic '{odom_topic}' contains no messages.")

    odom = OdomBagData(
        timestamps=np.asarray(timestamps, dtype=float),
        bag_timestamps=np.asarray(bag_timestamps, dtype=float),
        positions=np.asarray(positions, dtype=float),
        quaternions=np.asarray(quaternions, dtype=float),
        frame_ids=frame_ids,
        child_frame_ids=child_frame_ids,
    )
    gimbal = None
    if gimbal_topic and gimbal_timestamps:
        gimbal = ScalarBagData(
            timestamps=np.asarray(gimbal_timestamps, dtype=float),
            values=np.asarray(gimbal_values, dtype=float),
        )
    return odom, gimbal, topics

