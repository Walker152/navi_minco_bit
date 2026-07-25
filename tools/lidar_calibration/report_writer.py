#!/usr/bin/env python3
"""Write human-readable calibration reports without requiring Matplotlib."""

from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Dict, Optional

import numpy as np

from .calibrator_core import CalibrationResult


def _finite_or_none(value):
    if isinstance(value, dict):
        return {key: _finite_or_none(item) for key, item in value.items()}
    if isinstance(value, list):
        return [_finite_or_none(item) for item in value]
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


def read_current_navigation_values(path: Optional[Path]) -> Dict[str, Optional[float]]:
    values: Dict[str, Optional[float]] = {
        "controller_lidar_offset_x": None,
        "controller_lidar_offset_y": None,
        "controller_lidar_roll_offset": None,
        "planner_lidar_offset_x": None,
        "planner_lidar_offset_y": None,
    }
    if path is None or not path.is_file():
        return values

    try:
        import yaml

        document = yaml.safe_load(path.read_text(encoding="utf-8"))
        controller = document["controller_server"]["ros__parameters"]["FollowPath"]
        planner = document["planner_server"]["ros__parameters"]["MincoPlanner"]
        values.update(
            {
                "controller_lidar_offset_x": float(controller["lidar_offset_x"]),
                "controller_lidar_offset_y": float(controller["lidar_offset_y"]),
                "controller_lidar_roll_offset": float(controller["lidar_roll_offset"]),
                "planner_lidar_offset_x": float(planner["lidar_offset_x"]),
                "planner_lidar_offset_y": float(planner["lidar_offset_y"]),
            }
        )
    except (ImportError, KeyError, TypeError, ValueError):
        pass
    return values


def _fmt_optional(value: Optional[float], scale: float = 1.0, suffix: str = "") -> str:
    if value is None or not math.isfinite(value):
        return "unavailable"
    return f"{value * scale:.6f}{suffix}"


def _write_candidate_yaml(
    path: Path,
    result: CalibrationResult,
    current_values: Dict[str, Optional[float]],
) -> None:
    text = f"""# Candidate only. Review the report and validate on the robot before editing sentry1.yaml.
# Direction: rotation center -> body/lidar origin, expressed in the odom yaw frame.
calibration_result:
  status: {result.status}
  frame_definition: center_to_body_lidar
  lidar_offset_x: {result.lidar_offset_x_m:.9f}
  lidar_offset_y: {result.lidar_offset_y_m:.9f}
  lidar_roll_offset: {result.roll_rad:.9f}

quality:
  radius: {result.radius_m:.9f}
  circle_rmse: {result.circle_rmse_m:.9f}
  yaw_coverage_deg: {result.yaw_coverage_deg:.3f}
  local_x_std: {result.lidar_offset_x_std_m:.9f}
  local_y_std: {result.lidar_offset_y_std_m:.9f}
  roll_std_deg: {result.roll_std_deg:.3f}
  pitch_diagnostic_deg: {result.pitch_diagnostic_deg:.3f}
  pitch_std_deg: {result.pitch_std_deg:.3f}

current_values:
  controller_lidar_offset_x: {_fmt_optional(current_values["controller_lidar_offset_x"])}
  controller_lidar_offset_y: {_fmt_optional(current_values["controller_lidar_offset_y"])}
  controller_lidar_roll_offset: {_fmt_optional(current_values["controller_lidar_roll_offset"])}
  planner_lidar_offset_x: {_fmt_optional(current_values["planner_lidar_offset_x"])}
  planner_lidar_offset_y: {_fmt_optional(current_values["planner_lidar_offset_y"])}

copy_candidate_manually:
  controller_server:
    ros__parameters:
      FollowPath:
        lidar_offset_x: {result.lidar_offset_x_m:.9f}
        lidar_offset_y: {result.lidar_offset_y_m:.9f}
        lidar_roll_offset: {result.roll_rad:.9f}
  planner_server:
    ros__parameters:
      MincoPlanner:
        lidar_offset_x: {result.lidar_offset_x_m:.9f}
        lidar_offset_y: {result.lidar_offset_y_m:.9f}
"""
    path.write_text(text, encoding="utf-8")


def _write_markdown(
    path: Path,
    result: CalibrationResult,
    odom_topic: str,
    frames: Dict[str, str],
    current_values: Dict[str, Optional[float]],
) -> None:
    warning_lines = "\n".join(f"- {item}" for item in result.warnings) or "- 无"
    error_lines = "\n".join(f"- {item}" for item in result.errors) or "- 无"
    encoder_lines = "- 未启用"
    if result.encoder_check:
        encoder_lines = (
            f"- 覆盖范围：{result.encoder_check['coverage_deg']:.2f}°\n"
            f"- 与 LIO yaw 变化差标准差："
            f"{result.encoder_check['odom_encoder_delta_std_deg']:.2f}°\n"
            "- 编码器只参与质量诊断，没有参与 x/y 计算。"
        )

    text = f"""# 雷达 x/y 与 Roll 标定报告

## 结果

**{result.status}**

结果方向为“旋转中心 → body/lidar 原点”，x/y 表达在 `/aft_mapped_to_init` 的 yaw 局部坐标中。

| 参数 | 结果 |
|---|---:|
| lidar_offset_x | {result.lidar_offset_x_m:.6f} m ({result.lidar_offset_x_m * 1000.0:.2f} mm) |
| lidar_offset_y | {result.lidar_offset_y_m:.6f} m ({result.lidar_offset_y_m * 1000.0:.2f} mm) |
| lidar_roll_offset | {result.roll_rad:.6f} rad ({result.roll_deg:.3f}°) |
| 拟合半径 | {result.radius_m:.6f} m |

## 输入语义

- odom topic：`{odom_topic}`
- parent frame：`{frames.get("parent", "unknown")}`
- child frame：`{frames.get("child", "unknown")}`
- 前置丢弃时间：已丢弃 {result.discarded_warmup} 个初始化样本
- 原始/采用样本：{result.sample_count_raw} / {result.sample_count_used}

## 圆拟合与 x/y 质量

| 指标 | 数值 |
|---|---:|
| 圆心 | ({result.circle_center_x_m:.6f}, {result.circle_center_y_m:.6f}) m |
| 半径 | {result.radius_m * 1000.0:.3f} mm |
| 圆 RMSE | {result.circle_rmse_m * 1000.0:.3f} mm |
| 圆最大内点残差 | {result.circle_max_residual_m * 1000.0:.3f} mm |
| x/y 模长 | {result.local_radius_m * 1000.0:.3f} mm |
| 模长与半径差 | {result.radius_consistency_error_m * 1000.0:.3f} mm |
| x 标准差 | {result.lidar_offset_x_std_m * 1000.0:.3f} mm |
| y 标准差 | {result.lidar_offset_y_std_m * 1000.0:.3f} mm |
| 分段圆心最大漂移 | {_fmt_optional(result.segment_center_drift_m, 1000.0, " mm")} |
| Yaw 覆盖 | {result.yaw_coverage_deg:.2f}° |
| Yaw 累计行程 | {result.yaw_total_travel_deg:.2f}° |
| 圆拟合剔除点 | {result.outlier_count} |

## Roll 与 Pitch 诊断

| 指标 | 数值 |
|---|---:|
| 重力方向法 Roll | {result.roll_deg:.4f}° |
| Euler Roll 均值 | {result.euler_roll_deg:.4f}° |
| 两种 Roll 差值 | {result.roll_method_difference_deg:.4f}° |
| Roll 标准差 | {result.roll_std_deg:.4f}° |
| Pitch 诊断均值 | {result.pitch_diagnostic_deg:.4f}° |
| Pitch 标准差 | {result.pitch_std_deg:.4f}° |

Pitch 不作为标定输出，只用于检查地面、旋转轴和初始化是否稳定。

## 时间质量

- 数据时长：{result.duration_s:.3f} s
- 消息中位间隔：{result.median_dt_s:.6f} s
- 最大消息间隔：{result.max_dt_s:.6f} s
- 非递增时间戳：{result.timestamp_non_increasing}
- 非法四元数：{result.invalid_quaternions}

## 编码器可选检查

{encoder_lines}

## 当前配置对比

| 字段 | 当前值 | 候选值 |
|---|---:|---:|
| controller lidar_offset_x | {_fmt_optional(current_values["controller_lidar_offset_x"])} | {result.lidar_offset_x_m:.6f} |
| controller lidar_offset_y | {_fmt_optional(current_values["controller_lidar_offset_y"])} | {result.lidar_offset_y_m:.6f} |
| controller lidar_roll_offset | {_fmt_optional(current_values["controller_lidar_roll_offset"])} | {result.roll_rad:.6f} |
| planner lidar_offset_x | {_fmt_optional(current_values["planner_lidar_offset_x"])} | {result.lidar_offset_x_m:.6f} |
| planner lidar_offset_y | {_fmt_optional(current_values["planner_lidar_offset_y"])} | {result.lidar_offset_y_m:.6f} |

报告不会修改正式配置。只有 `PASS` 且低速原地旋转验证通过后，才应手工同步修改 planner 和 controller。

## Warnings

{warning_lines}

## Errors

{error_lines}
"""
    path.write_text(text, encoding="utf-8")


def _svg_polyline(
    x: np.ndarray,
    y: np.ndarray,
    map_x,
    map_y,
    color: str,
    width: float = 1.5,
    opacity: float = 1.0,
) -> str:
    points = " ".join(f"{map_x(px):.2f},{map_y(py):.2f}" for px, py in zip(x, y))
    return (
        f'<polyline points="{points}" fill="none" stroke="{color}" '
        f'stroke-width="{width}" opacity="{opacity}"/>'
    )


def _write_svg(path: Path, result: CalibrationResult, debug: Dict[str, np.ndarray]) -> None:
    positions = debug["positions"]
    timestamps = debug["timestamps"]
    inliers = debug["inliers"]
    roll_deg = np.degrees(debug["roll"])
    pitch_deg = np.degrees(debug["pitch"])

    max_points = 1600
    stride = max(1, len(positions) // max_points)
    plot_indices = np.arange(0, len(positions), stride)

    width, height = 1200, 620
    left = (70.0, 70.0, 500.0, 480.0)
    right = (650.0, 70.0, 500.0, 480.0)

    x_values = np.append(positions[:, 0], result.circle_center_x_m)
    y_values = np.append(positions[:, 1], result.circle_center_y_m)
    x_margin = max(np.ptp(x_values) * 0.08, result.radius_m * 0.15, 0.01)
    y_margin = max(np.ptp(y_values) * 0.08, result.radius_m * 0.15, 0.01)
    x_min, x_max = float(np.min(x_values) - x_margin), float(np.max(x_values) + x_margin)
    y_min, y_max = float(np.min(y_values) - y_margin), float(np.max(y_values) + y_margin)
    span = max(x_max - x_min, y_max - y_min)
    x_mid, y_mid = 0.5 * (x_min + x_max), 0.5 * (y_min + y_max)
    x_min, x_max = x_mid - span / 2.0, x_mid + span / 2.0
    y_min, y_max = y_mid - span / 2.0, y_mid + span / 2.0

    def lx(value):
        return left[0] + (value - x_min) / (x_max - x_min) * left[2]

    def ly(value):
        return left[1] + left[3] - (value - y_min) / (y_max - y_min) * left[3]

    relative_time = timestamps - timestamps[0]
    angle_min = float(min(np.min(roll_deg), np.min(pitch_deg), 0.0))
    angle_max = float(max(np.max(roll_deg), np.max(pitch_deg), 0.0))
    angle_margin = max((angle_max - angle_min) * 0.12, 1.0)
    angle_min -= angle_margin
    angle_max += angle_margin
    time_max = max(float(relative_time[-1]), 1.0)

    def rx(value):
        return right[0] + value / time_max * right[2]

    def ry(value):
        return right[1] + right[3] - (value - angle_min) / (angle_max - angle_min) * right[3]

    theta = np.linspace(0.0, 2.0 * np.pi, 240)
    circle_x = result.circle_center_x_m + result.radius_m * np.cos(theta)
    circle_y = result.circle_center_y_m + result.radius_m * np.sin(theta)

    svg = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" '
        f'viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="#ffffff"/>',
        '<style>text{font-family:DejaVu Sans,Arial,sans-serif;fill:#202124}'
        '.title{font-size:20px;font-weight:bold}.label{font-size:13px}'
        '.small{font-size:11px}</style>',
        f'<text x="40" y="34" class="title">LiDAR x/y and Roll Calibration — {result.status}</text>',
        f'<rect x="{left[0]}" y="{left[1]}" width="{left[2]}" height="{left[3]}" '
        'fill="#fafafa" stroke="#9aa0a6"/>',
        f'<rect x="{right[0]}" y="{right[1]}" width="{right[2]}" height="{right[3]}" '
        'fill="#fafafa" stroke="#9aa0a6"/>',
        _svg_polyline(circle_x, circle_y, lx, ly, "#d93025", 2.0),
    ]
    for index in plot_indices:
        color = "#1a73e8" if inliers[index] else "#d93025"
        svg.append(
            f'<circle cx="{lx(positions[index, 0]):.2f}" '
            f'cy="{ly(positions[index, 1]):.2f}" r="1.8" fill="{color}" opacity="0.72"/>'
        )
    svg.extend(
        [
            f'<line x1="{lx(result.circle_center_x_m) - 7:.2f}" '
            f'y1="{ly(result.circle_center_y_m):.2f}" '
            f'x2="{lx(result.circle_center_x_m) + 7:.2f}" '
            f'y2="{ly(result.circle_center_y_m):.2f}" stroke="#d93025" stroke-width="2"/>',
            f'<line x1="{lx(result.circle_center_x_m):.2f}" '
            f'y1="{ly(result.circle_center_y_m) - 7:.2f}" '
            f'x2="{lx(result.circle_center_x_m):.2f}" '
            f'y2="{ly(result.circle_center_y_m) + 7:.2f}" stroke="#d93025" stroke-width="2"/>',
            f'<text x="{left[0]}" y="{left[1] - 14}" class="label">'
            f'XY circle: R={result.radius_m * 1000.0:.2f} mm, '
            f'RMSE={result.circle_rmse_m * 1000.0:.2f} mm</text>',
            f'<text x="{left[0]}" y="{left[1] + left[3] + 28}" class="label">'
            f'center→body: x={result.lidar_offset_x_m * 1000.0:.2f} mm, '
            f'y={result.lidar_offset_y_m * 1000.0:.2f} mm</text>',
            _svg_polyline(
                relative_time[plot_indices],
                roll_deg[plot_indices],
                rx,
                ry,
                "#1a73e8",
                1.4,
            ),
            _svg_polyline(
                relative_time[plot_indices],
                pitch_deg[plot_indices],
                rx,
                ry,
                "#188038",
                1.4,
            ),
            f'<line x1="{right[0]}" y1="{ry(result.roll_deg):.2f}" '
            f'x2="{right[0] + right[2]}" y2="{ry(result.roll_deg):.2f}" '
            'stroke="#d93025" stroke-width="1.2" stroke-dasharray="6,4"/>',
            f'<text x="{right[0]}" y="{right[1] - 14}" class="label">'
            f'Attitude: Roll={result.roll_deg:.3f}°, Pitch diagnostic='
            f'{result.pitch_diagnostic_deg:.3f}°</text>',
            f'<text x="{right[0] + 10}" y="{right[1] + 22}" class="small" '
            'fill="#1a73e8">Blue: Roll</text>',
            f'<text x="{right[0] + 95}" y="{right[1] + 22}" class="small" '
            'fill="#188038">Green: Pitch</text>',
            f'<text x="{right[0]}" y="{right[1] + right[3] + 28}" class="label">'
            f'Time [s], yaw coverage={result.yaw_coverage_deg:.1f}°, '
            f'roll std={result.roll_std_deg:.3f}°</text>',
            "</svg>",
        ]
    )
    path.write_text("\n".join(svg), encoding="utf-8")


def write_reports(
    output_dir: Path,
    result: CalibrationResult,
    debug: Dict[str, np.ndarray],
    odom_topic: str,
    frames: Dict[str, str],
    current_config: Optional[Path],
    write_svg: bool = True,
) -> Dict[str, Path]:
    output_dir.mkdir(parents=True, exist_ok=True)
    current_values = read_current_navigation_values(current_config)

    json_path = output_dir / "report.json"
    markdown_path = output_dir / "report.md"
    yaml_path = output_dir / "candidate_parameters.yaml"
    json_path.write_text(
        json.dumps(_finite_or_none(result.to_dict()), ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    _write_markdown(markdown_path, result, odom_topic, frames, current_values)
    _write_candidate_yaml(yaml_path, result, current_values)

    paths = {
        "json": json_path,
        "markdown": markdown_path,
        "candidate_yaml": yaml_path,
    }
    if write_svg:
        svg_path = output_dir / "calibration.svg"
        _write_svg(svg_path, result, debug)
        paths["svg"] = svg_path
    return paths
