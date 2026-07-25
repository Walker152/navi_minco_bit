#!/usr/bin/env python3
"""Static synthetic-data checks for the standalone LiDAR calibrator."""

from __future__ import annotations

import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np

REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPOSITORY_ROOT / "tools"))

from lidar_calibration.calibrator_core import CalibrationThresholds, calibrate
from lidar_calibration.report_writer import write_reports


def quaternion_from_yaw_roll(yaw: np.ndarray, roll: float) -> np.ndarray:
    """Quaternion for Rz(yaw) Rx(roll), returned as (x, y, z, w)."""
    half_yaw = 0.5 * yaw
    half_roll = 0.5 * roll
    return np.column_stack(
        (
            np.cos(half_yaw) * np.sin(half_roll),
            np.sin(half_yaw) * np.sin(half_roll),
            np.sin(half_yaw) * np.cos(half_roll),
            np.cos(half_yaw) * np.cos(half_roll),
        )
    )


class LidarCalibratorTest(unittest.TestCase):
    def test_recovers_offset_and_roll_with_outliers(self):
        generator = np.random.default_rng(20260724)
        count = 900
        yaw = np.linspace(0.0, 2.0 * np.pi, count, endpoint=False)
        true_x = -0.027
        true_y = 0.153
        true_roll = -0.2618
        center = np.array([1.25, -0.42])

        local = np.column_stack(
            (
                np.cos(yaw) * true_x - np.sin(yaw) * true_y,
                np.sin(yaw) * true_x + np.cos(yaw) * true_y,
            )
        )
        xy = center + local + generator.normal(0.0, 0.0012, size=(count, 2))
        outlier_indices = generator.choice(count, size=18, replace=False)
        xy[outlier_indices] += generator.normal(0.0, 0.04, size=(len(outlier_indices), 2))

        positions = np.column_stack((xy, np.full(count, 0.45)))
        quaternions = quaternion_from_yaw_roll(yaw, true_roll)
        timestamps = 1000.0 + np.arange(count) * 0.02
        thresholds = CalibrationThresholds(
            min_samples=100,
            recommended_samples=500,
            min_yaw_coverage_deg=270.0,
        )

        result, debug = calibrate(
            timestamps,
            positions,
            quaternions,
            discard_seconds=0.0,
            thresholds=thresholds,
        )

        self.assertNotEqual(result.status, "FAIL")
        self.assertAlmostEqual(result.lidar_offset_x_m, true_x, delta=0.0025)
        self.assertAlmostEqual(result.lidar_offset_y_m, true_y, delta=0.0025)
        self.assertAlmostEqual(result.roll_rad, true_roll, delta=np.deg2rad(0.2))
        self.assertGreater(result.yaw_coverage_deg, 350.0)
        self.assertLess(result.circle_rmse_m, 0.003)

        with tempfile.TemporaryDirectory() as temporary_directory:
            report_paths = write_reports(
                Path(temporary_directory),
                result,
                debug,
                "/aft_mapped_to_init",
                {"parent": "camera_init", "child": "body"},
                REPOSITORY_ROOT / "src/navigation/navi2_bringup/params/sentry1.yaml",
            )
            self.assertTrue(report_paths["markdown"].is_file())
            self.assertTrue(report_paths["candidate_yaml"].is_file())
            self.assertTrue(report_paths["svg"].is_file())
            candidate = report_paths["candidate_yaml"].read_text(encoding="utf-8")
            self.assertIn("frame_definition: center_to_body_lidar", candidate)
            self.assertIn("lidar_roll_offset:", candidate)


if __name__ == "__main__":
    unittest.main()
