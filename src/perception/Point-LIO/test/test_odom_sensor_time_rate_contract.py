#!/usr/bin/env python3

import unittest
from pathlib import Path


POINT_LIO_DIR = Path(__file__).resolve().parents[1]


class OdomSensorTimeRateContractTest(unittest.TestCase):
    def test_odom_rate_uses_sensor_time_and_configured_hz(self):
        limiter_header = POINT_LIO_DIR / "include" / "sensor_time_rate_limiter.h"
        self.assertTrue(
            limiter_header.exists(), "sensor-time rate limiter is missing"
        )

        parameters_cpp = (POINT_LIO_DIR / "src" / "parameters.cpp").read_text()
        mapping_cpp = (POINT_LIO_DIR / "src" / "laserMapping.cpp").read_text()
        mid360_yaml = (POINT_LIO_DIR / "config" / "mid360.yaml").read_text()
        cmake = (POINT_LIO_DIR / "CMakeLists.txt").read_text()

        self.assertIn(
            'declare_parameter<int>("odometry.publish_frequency_hz", 200)',
            parameters_cpp,
        )
        self.assertIn("publish_frequency_hz: 100", mid360_yaml)
        self.assertIn("SensorTimeRateLimiter", mapping_cpp)
        self.assertIn(
            "should_publish(odom_sensor_time, orig_odom_freq)", mapping_cpp
        )
        self.assertNotIn(
            "current_time - last_time < std::chrono::milliseconds(5)",
            mapping_cpp,
        )
        self.assertIn(
            "ament_add_gtest(sensor_time_rate_limiter_test",
            cmake,
        )


if __name__ == "__main__":
    unittest.main()
