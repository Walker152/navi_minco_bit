#!/usr/bin/env python3
"""Command-line entry point for current-project LiDAR x/y and roll calibration."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import List

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
    from lidar_calibration.bag_reader import BagReadError, list_topics, read_calibration_topics
    from lidar_calibration.calibrator_core import (
        CalibrationError,
        CalibrationThresholds,
        calibrate,
    )
    from lidar_calibration.report_writer import write_reports
else:
    from .bag_reader import BagReadError, list_topics, read_calibration_topics
    from .calibrator_core import CalibrationError, CalibrationThresholds, calibrate
    from .report_writer import write_reports


DEFAULT_ODOM_TOPIC = "/aft_mapped_to_init"


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Calibrate center→body/lidar x/y lever arm and the level-ground Roll baseline "
            "from a ROS 2 bag."
        )
    )
    parser.add_argument("bag", type=Path, help="ROS 2 bag directory, .db3 file or .mcap file")
    parser.add_argument(
        "--odom-topic",
        default=DEFAULT_ODOM_TOPIC,
        help=f"Odometry topic (default: {DEFAULT_ODOM_TOPIC})",
    )
    parser.add_argument(
        "--gimbal-topic",
        default=None,
        help=(
            "Optional ros_interfaces/SentryInfoOffline topic. Its encoder yaw is used only "
            "for quality comparison, not for x/y."
        ),
    )
    parser.add_argument(
        "--discard-seconds",
        type=float,
        default=5.0,
        help="Discard this many seconds after the first valid odom sample (default: 5)",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=None,
        help="Report directory (default: <bag>/lidar_calibration_report)",
    )
    parser.add_argument(
        "--config",
        type=Path,
        default=None,
        help="Current sentry1.yaml for old/new comparison",
    )
    parser.add_argument(
        "--min-samples",
        type=int,
        default=100,
        help="Minimum usable odometry samples after filtering (default: 100)",
    )
    parser.add_argument(
        "--list-topics",
        action="store_true",
        help="List bag topics and exit without calibration",
    )
    parser.add_argument(
        "--no-svg",
        action="store_true",
        help="Do not generate the dependency-free SVG visualization",
    )
    return parser


def _default_config() -> Path:
    repository_root = Path(__file__).resolve().parents[2]
    return repository_root / "src/navigation/navi2_bringup/params/sentry1.yaml"


def _default_output(bag: Path) -> Path:
    if bag.is_dir():
        return bag / "lidar_calibration_report"
    return bag.parent / f"{bag.stem}_lidar_calibration_report"


def _print_topics(topics: List[tuple]) -> None:
    print("Available topics:")
    for name, msg_type in topics:
        print(f"  {name:<45} {msg_type}")


def _single_or_mixed(values: List[str]) -> str:
    unique = sorted(set(value for value in values if value))
    return unique[0] if len(unique) == 1 else ", ".join(unique) if unique else "(empty)"


def main() -> int:
    args = _parser().parse_args()
    try:
        if args.list_topics:
            _print_topics(list_topics(args.bag))
            return 0

        odom, gimbal, topics = read_calibration_topics(
            args.bag, args.odom_topic, args.gimbal_topic
        )
        print(f"Read {len(odom.timestamps)} odometry samples from {args.odom_topic}")
        if args.gimbal_topic:
            count = 0 if gimbal is None else len(gimbal.timestamps)
            print(f"Read {count} optional encoder samples from {args.gimbal_topic}")

        thresholds = CalibrationThresholds(min_samples=max(10, args.min_samples))
        result, debug = calibrate(
            timestamps=odom.timestamps,
            positions=odom.positions,
            quaternions=odom.quaternions,
            discard_seconds=max(0.0, args.discard_seconds),
            thresholds=thresholds,
            encoder_timestamps=None if gimbal is None else gimbal.timestamps,
            encoder_degrees=None if gimbal is None else gimbal.values,
        )
        if args.gimbal_topic and gimbal is None:
            result.warnings.append(
                f"可选编码器话题 {args.gimbal_topic} 没有消息；x/y 和 Roll 仍只使用 odom。"
            )

        parent_frame = _single_or_mixed(odom.frame_ids)
        child_frame = _single_or_mixed(odom.child_frame_ids)
        if parent_frame != "camera_init":
            result.warnings.append(
                f"odom parent frame is '{parent_frame}', expected 'camera_init'."
            )
        if child_frame != "body":
            result.warnings.append(
                f"odom child frame is '{child_frame}', expected 'body'."
            )
        if result.status == "PASS" and result.warnings:
            result.status = "WARNING"

        output_dir = args.output_dir or _default_output(args.bag)
        config_path = args.config or _default_config()
        report_paths = write_reports(
            output_dir=output_dir,
            result=result,
            debug=debug,
            odom_topic=args.odom_topic,
            frames={"parent": parent_frame, "child": child_frame},
            current_config=config_path,
            write_svg=not args.no_svg,
        )

        print("")
        print(f"Result: {result.status}")
        print(
            "  center -> body/lidar: "
            f"x={result.lidar_offset_x_m:.6f} m, y={result.lidar_offset_y_m:.6f} m"
        )
        print(
            f"  radius={result.radius_m:.6f} m, "
            f"circle RMSE={result.circle_rmse_m * 1000.0:.2f} mm"
        )
        print(
            f"  Roll={result.roll_rad:.6f} rad ({result.roll_deg:.3f} deg), "
            f"std={result.roll_std_deg:.3f} deg"
        )
        print(
            f"  Pitch diagnostic={result.pitch_diagnostic_deg:.3f} deg, "
            f"yaw coverage={result.yaw_coverage_deg:.1f} deg"
        )
        if result.warnings:
            print("Warnings:")
            for warning in result.warnings:
                print(f"  - {warning}")
        if result.errors:
            print("Errors:")
            for error in result.errors:
                print(f"  - {error}")
        print("Reports:")
        for name, path in report_paths.items():
            print(f"  {name:<15} {path}")
        print("")
        print("No project configuration was modified.")
        return 0 if result.status in ("PASS", "WARNING") else 2
    except (BagReadError, CalibrationError, ValueError) as exc:
        print(f"Calibration failed: {exc}", file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        print("Interrupted.", file=sys.stderr)
        return 130


if __name__ == "__main__":
    raise SystemExit(main())
