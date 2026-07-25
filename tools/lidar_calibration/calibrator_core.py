#!/usr/bin/env python3
"""Core x/y lever-arm and roll calibration independent of ROS interfaces."""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from typing import Dict, List, Optional, Tuple

import numpy as np

from .frame_math import (
    angular_difference,
    circular_mean,
    normalize_quaternions,
    quaternion_to_rotation_matrices,
    quaternion_to_rpy,
    roll_pitch_from_mean_up,
)


class CalibrationError(RuntimeError):
    """Raised when the input data cannot support calibration."""


@dataclass
class CalibrationThresholds:
    min_samples: int = 100
    recommended_samples: int = 500
    min_yaw_coverage_deg: float = 270.0
    recommended_yaw_coverage_deg: float = 330.0
    max_circle_rmse_m: float = 0.010
    max_local_std_m: float = 0.010
    max_radius_consistency_m: float = 0.005
    max_segment_center_drift_m: float = 0.010
    max_roll_std_deg: float = 1.0
    max_roll_method_difference_deg: float = 0.5
    max_pitch_std_deg: float = 1.0
    max_gap_factor: float = 10.0


@dataclass
class CalibrationResult:
    status: str
    sample_count_raw: int
    sample_count_used: int
    discarded_warmup: int
    invalid_quaternions: int
    outlier_count: int
    duration_s: float
    median_dt_s: float
    max_dt_s: float
    timestamp_non_increasing: int
    circle_center_x_m: float
    circle_center_y_m: float
    radius_m: float
    circle_rmse_m: float
    circle_max_residual_m: float
    segment_center_drift_m: float
    yaw_coverage_deg: float
    yaw_total_travel_deg: float
    lidar_offset_x_m: float
    lidar_offset_y_m: float
    lidar_offset_x_std_m: float
    lidar_offset_y_std_m: float
    local_radius_m: float
    radius_consistency_error_m: float
    roll_rad: float
    roll_deg: float
    roll_std_deg: float
    euler_roll_rad: float
    euler_roll_deg: float
    roll_method_difference_deg: float
    pitch_diagnostic_rad: float
    pitch_diagnostic_deg: float
    pitch_std_deg: float
    mean_up_body: List[float]
    warnings: List[str] = field(default_factory=list)
    errors: List[str] = field(default_factory=list)
    encoder_check: Optional[Dict[str, float]] = None

    def to_dict(self) -> Dict[str, object]:
        return asdict(self)


def _fit_circle_once(x: np.ndarray, y: np.ndarray, weights: Optional[np.ndarray] = None):
    design = np.column_stack((2.0 * x, 2.0 * y, np.ones_like(x)))
    target = x * x + y * y
    if weights is not None:
        root_w = np.sqrt(np.clip(weights, 1.0e-12, None))
        design = design * root_w[:, None]
        target = target * root_w
    solution, _, rank, _ = np.linalg.lstsq(design, target, rcond=None)
    if rank < 3:
        raise CalibrationError("XY trajectory is degenerate and cannot define a circle.")
    center_x, center_y, constant = solution
    radius_sq = constant + center_x * center_x + center_y * center_y
    if not np.isfinite(radius_sq) or radius_sq <= 0.0:
        raise CalibrationError("Circle fit produced a non-positive radius.")
    return float(center_x), float(center_y), float(np.sqrt(radius_sq))


def robust_circle_fit(
    x: np.ndarray, y: np.ndarray
) -> Tuple[float, float, float, np.ndarray, np.ndarray]:
    """Huber-weighted algebraic fit followed by a geometric inlier refit."""
    x = np.asarray(x, dtype=float)
    y = np.asarray(y, dtype=float)
    if len(x) < 10:
        raise CalibrationError("At least 10 points are required for circle fitting.")
    if np.ptp(x) < 1.0e-4 and np.ptp(y) < 1.0e-4:
        raise CalibrationError("XY trajectory has almost no motion.")

    weights = np.ones(len(x), dtype=float)
    center_x, center_y, radius = _fit_circle_once(x, y)
    for _ in range(12):
        center_x, center_y, radius = _fit_circle_once(x, y, weights)
        residuals = np.hypot(x - center_x, y - center_y) - radius
        median = float(np.median(residuals))
        mad_sigma = 1.4826 * float(np.median(np.abs(residuals - median)))
        scale = max(mad_sigma, 1.0e-4)
        huber_limit = 1.5 * scale
        abs_centered = np.abs(residuals - median)
        new_weights = np.ones_like(weights)
        outside = abs_centered > huber_limit
        new_weights[outside] = huber_limit / abs_centered[outside]
        if np.max(np.abs(new_weights - weights)) < 1.0e-3:
            weights = new_weights
            break
        weights = new_weights

    residuals = np.hypot(x - center_x, y - center_y) - radius
    median = float(np.median(residuals))
    mad_sigma = 1.4826 * float(np.median(np.abs(residuals - median)))
    inlier_limit = max(3.0 * mad_sigma, 0.001)
    inliers = np.abs(residuals - median) <= inlier_limit
    if int(np.count_nonzero(inliers)) < max(10, int(0.6 * len(x))):
        raise CalibrationError("Too many XY samples were rejected as circle-fit outliers.")

    center_x, center_y, radius = _fit_circle_once(x[inliers], y[inliers])
    residuals = np.hypot(x - center_x, y - center_y) - radius
    return center_x, center_y, radius, residuals, inliers


def _segment_center_drift(x: np.ndarray, y: np.ndarray, reference: Tuple[float, float]) -> float:
    drifts: List[float] = []
    for indices in np.array_split(np.arange(len(x)), 4):
        if len(indices) < 10:
            continue
        try:
            segment_x, segment_y, _, _, _ = robust_circle_fit(x[indices], y[indices])
        except CalibrationError:
            continue
        drifts.append(float(np.hypot(segment_x - reference[0], segment_y - reference[1])))
    return max(drifts) if drifts else float("nan")


def _encoder_quality_check(
    odom_timestamps: np.ndarray,
    odom_yaw_unwrapped: np.ndarray,
    encoder_timestamps: Optional[np.ndarray],
    encoder_degrees: Optional[np.ndarray],
) -> Optional[Dict[str, float]]:
    if encoder_timestamps is None or encoder_degrees is None or len(encoder_timestamps) < 2:
        return None

    order = np.argsort(encoder_timestamps)
    encoder_t = np.asarray(encoder_timestamps, dtype=float)[order]
    encoder_yaw = np.unwrap(np.deg2rad(np.asarray(encoder_degrees, dtype=float)[order]))
    overlap = (odom_timestamps >= encoder_t[0]) & (odom_timestamps <= encoder_t[-1])
    if int(np.count_nonzero(overlap)) < 10:
        return {
            "sample_count": float(np.count_nonzero(overlap)),
            "coverage_deg": float(np.degrees(np.ptp(encoder_yaw))),
            "odom_encoder_delta_std_deg": float("nan"),
        }

    interpolated = np.interp(odom_timestamps[overlap], encoder_t, encoder_yaw)
    odom_change = odom_yaw_unwrapped[overlap] - odom_yaw_unwrapped[overlap][0]
    encoder_change = interpolated - interpolated[0]
    difference = odom_change - encoder_change
    difference -= np.median(difference)
    return {
        "sample_count": float(np.count_nonzero(overlap)),
        "coverage_deg": float(np.degrees(np.ptp(encoder_yaw))),
        "odom_encoder_delta_std_deg": float(np.degrees(np.std(difference))),
    }


def calibrate(
    timestamps: np.ndarray,
    positions: np.ndarray,
    quaternions: np.ndarray,
    discard_seconds: float = 5.0,
    thresholds: Optional[CalibrationThresholds] = None,
    encoder_timestamps: Optional[np.ndarray] = None,
    encoder_degrees: Optional[np.ndarray] = None,
) -> Tuple[CalibrationResult, Dict[str, np.ndarray]]:
    thresholds = thresholds or CalibrationThresholds()
    timestamps = np.asarray(timestamps, dtype=float)
    positions = np.asarray(positions, dtype=float)
    quaternions = np.asarray(quaternions, dtype=float)

    if timestamps.ndim != 1:
        raise CalibrationError("timestamps must be a one-dimensional array")
    if positions.shape != (len(timestamps), 3):
        raise CalibrationError("positions must have shape (N, 3)")
    if quaternions.shape != (len(timestamps), 4):
        raise CalibrationError("quaternions must have shape (N, 4)")
    if len(timestamps) == 0:
        raise CalibrationError("No odometry samples were provided.")

    sample_count_raw = len(timestamps)
    finite = (
        np.isfinite(timestamps)
        & np.isfinite(positions).all(axis=1)
        & np.isfinite(quaternions).all(axis=1)
    )
    normalized_q, quaternion_valid = normalize_quaternions(quaternions)
    invalid_quaternions = int(np.count_nonzero(~quaternion_valid))
    valid = finite & quaternion_valid
    if not np.any(valid):
        raise CalibrationError("No finite samples with valid quaternions remain.")

    first_valid_time = float(timestamps[np.flatnonzero(valid)[0]])
    warmup = timestamps < first_valid_time + max(0.0, discard_seconds)
    discarded_warmup = int(np.count_nonzero(valid & warmup))
    valid &= ~warmup

    timestamps = timestamps[valid]
    positions = positions[valid]
    normalized_q = normalized_q[valid]
    if len(timestamps) < thresholds.min_samples:
        raise CalibrationError(
            f"Only {len(timestamps)} usable samples remain; at least "
            f"{thresholds.min_samples} are required."
        )

    deltas = np.diff(timestamps)
    timestamp_non_increasing = int(np.count_nonzero(deltas <= 0.0))
    positive_deltas = deltas[deltas > 0.0]
    median_dt = float(np.median(positive_deltas)) if len(positive_deltas) else float("nan")
    max_dt = float(np.max(positive_deltas)) if len(positive_deltas) else float("nan")
    duration = float(timestamps[-1] - timestamps[0])

    roll, pitch, yaw = quaternion_to_rpy(normalized_q)
    yaw_unwrapped = np.unwrap(yaw)
    yaw_coverage_deg = float(np.degrees(np.ptp(yaw_unwrapped)))
    yaw_total_travel_deg = float(np.degrees(np.sum(np.abs(np.diff(yaw_unwrapped)))))

    center_x, center_y, radius, residuals, inliers = robust_circle_fit(
        positions[:, 0], positions[:, 1]
    )
    inlier_residuals = residuals[inliers]
    circle_rmse = float(np.sqrt(np.mean(inlier_residuals * inlier_residuals)))
    circle_max_residual = float(np.max(np.abs(inlier_residuals)))
    segment_drift = _segment_center_drift(
        positions[inliers, 0], positions[inliers, 1], (center_x, center_y)
    )

    radial_x = positions[:, 0] - center_x
    radial_y = positions[:, 1] - center_y
    local_x = radial_x * np.cos(yaw) + radial_y * np.sin(yaw)
    local_y = -radial_x * np.sin(yaw) + radial_y * np.cos(yaw)
    offset_x = float(np.mean(local_x[inliers]))
    offset_y = float(np.mean(local_y[inliers]))
    offset_x_std = float(np.std(local_x[inliers]))
    offset_y_std = float(np.std(local_y[inliers]))
    local_radius = float(np.hypot(offset_x, offset_y))
    radius_consistency = abs(local_radius - radius)

    matrices = quaternion_to_rotation_matrices(normalized_q[inliers])
    gravity_roll, gravity_pitch, mean_up = roll_pitch_from_mean_up(matrices)
    euler_roll = circular_mean(roll[inliers])
    roll_std_deg = float(np.degrees(np.std(angular_difference(roll[inliers], euler_roll))))
    pitch_std_deg = float(np.degrees(np.std(pitch[inliers])))
    roll_method_difference_deg = abs(
        float(np.degrees(angular_difference(np.array([euler_roll]), gravity_roll)[0]))
    )

    warnings: List[str] = []
    errors: List[str] = []
    if len(timestamps) < thresholds.recommended_samples:
        warnings.append(
            f"有效样本 {len(timestamps)} 少于推荐值 {thresholds.recommended_samples}。"
        )
    if yaw_coverage_deg < thresholds.min_yaw_coverage_deg:
        errors.append(
            f"Yaw 覆盖 {yaw_coverage_deg:.1f}° 小于最低建议 "
            f"{thresholds.min_yaw_coverage_deg:.1f}°。"
        )
    elif yaw_coverage_deg < thresholds.recommended_yaw_coverage_deg:
        warnings.append(
            f"Yaw 覆盖 {yaw_coverage_deg:.1f}°，建议达到 "
            f"{thresholds.recommended_yaw_coverage_deg:.1f}°。"
        )
    if circle_rmse > thresholds.max_circle_rmse_m:
        errors.append(
            f"圆拟合 RMSE {circle_rmse * 1000.0:.2f} mm 超过 "
            f"{thresholds.max_circle_rmse_m * 1000.0:.2f} mm。"
        )
    if max(offset_x_std, offset_y_std) > thresholds.max_local_std_m:
        errors.append(
            f"局部偏置标准差达到 {max(offset_x_std, offset_y_std) * 1000.0:.2f} mm。"
        )
    if radius_consistency > thresholds.max_radius_consistency_m:
        errors.append(
            f"x/y 模长与拟合半径相差 {radius_consistency * 1000.0:.2f} mm。"
        )
    if np.isfinite(segment_drift) and segment_drift > thresholds.max_segment_center_drift_m:
        warnings.append(
            f"分段圆心最大漂移 {segment_drift * 1000.0:.2f} mm，检查底盘移动或 LIO 漂移。"
        )
    if roll_std_deg > thresholds.max_roll_std_deg:
        warnings.append(f"Roll 标准差 {roll_std_deg:.2f}°，安装角结果可能不稳定。")
    if roll_method_difference_deg > thresholds.max_roll_method_difference_deg:
        warnings.append(
            f"重力法与 Euler Roll 相差 {roll_method_difference_deg:.2f}°。"
        )
    if pitch_std_deg > thresholds.max_pitch_std_deg:
        warnings.append(
            f"Pitch 标准差 {pitch_std_deg:.2f}°，检查旋转轴、地面或初始化稳定性。"
        )
    if timestamp_non_increasing:
        warnings.append(f"发现 {timestamp_non_increasing} 个非递增 odom 时间戳。")
    if (
        np.isfinite(median_dt)
        and np.isfinite(max_dt)
        and max_dt > thresholds.max_gap_factor * median_dt
    ):
        warnings.append(
            f"最大消息间隔 {max_dt:.3f}s，超过中位间隔 {median_dt:.3f}s 的 "
            f"{thresholds.max_gap_factor:.0f} 倍。"
        )

    status = "FAIL" if errors else ("WARNING" if warnings else "PASS")
    encoder_check = _encoder_quality_check(
        timestamps,
        yaw_unwrapped,
        encoder_timestamps,
        encoder_degrees,
    )
    if encoder_check and np.isfinite(encoder_check["odom_encoder_delta_std_deg"]):
        if encoder_check["coverage_deg"] < thresholds.min_yaw_coverage_deg:
            warnings.append(
                f"可选编码器覆盖只有 {encoder_check['coverage_deg']:.1f}°。"
            )
        if encoder_check["odom_encoder_delta_std_deg"] > 3.0:
            warnings.append(
                "编码器与 LIO yaw 变化差的标准差超过 3°；编码器仅作诊断，未参与 x/y 计算。"
            )
            if status == "PASS":
                status = "WARNING"

    result = CalibrationResult(
        status=status,
        sample_count_raw=sample_count_raw,
        sample_count_used=int(np.count_nonzero(inliers)),
        discarded_warmup=discarded_warmup,
        invalid_quaternions=invalid_quaternions,
        outlier_count=int(np.count_nonzero(~inliers)),
        duration_s=duration,
        median_dt_s=median_dt,
        max_dt_s=max_dt,
        timestamp_non_increasing=timestamp_non_increasing,
        circle_center_x_m=center_x,
        circle_center_y_m=center_y,
        radius_m=radius,
        circle_rmse_m=circle_rmse,
        circle_max_residual_m=circle_max_residual,
        segment_center_drift_m=segment_drift,
        yaw_coverage_deg=yaw_coverage_deg,
        yaw_total_travel_deg=yaw_total_travel_deg,
        lidar_offset_x_m=offset_x,
        lidar_offset_y_m=offset_y,
        lidar_offset_x_std_m=offset_x_std,
        lidar_offset_y_std_m=offset_y_std,
        local_radius_m=local_radius,
        radius_consistency_error_m=radius_consistency,
        roll_rad=gravity_roll,
        roll_deg=float(np.degrees(gravity_roll)),
        roll_std_deg=roll_std_deg,
        euler_roll_rad=euler_roll,
        euler_roll_deg=float(np.degrees(euler_roll)),
        roll_method_difference_deg=roll_method_difference_deg,
        pitch_diagnostic_rad=gravity_pitch,
        pitch_diagnostic_deg=float(np.degrees(gravity_pitch)),
        pitch_std_deg=pitch_std_deg,
        mean_up_body=[float(value) for value in mean_up],
        warnings=warnings,
        errors=errors,
        encoder_check=encoder_check,
    )
    debug = {
        "timestamps": timestamps,
        "positions": positions,
        "roll": roll,
        "pitch": pitch,
        "yaw": yaw,
        "yaw_unwrapped": yaw_unwrapped,
        "inliers": inliers,
        "residuals": residuals,
        "local_x": local_x,
        "local_y": local_y,
    }
    return result, debug
