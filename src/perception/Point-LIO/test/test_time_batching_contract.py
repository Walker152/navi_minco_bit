#!/usr/bin/env python3

import unittest
from pathlib import Path


POINT_LIO_DIR = Path(__file__).resolve().parents[1]


class TimeBatchingContractTest(unittest.TestCase):
    def test_batching_is_wired_without_reducing_full_cloud(self):
        header = POINT_LIO_DIR / "include" / "time_batching.h"
        self.assertTrue(header.exists(), "time batching helper is missing")

        common_lib = (POINT_LIO_DIR / "include" / "common_lib.h").read_text()
        parameters_h = (POINT_LIO_DIR / "src" / "parameters.h").read_text()
        parameters_cpp = (POINT_LIO_DIR / "src" / "parameters.cpp").read_text()
        mapping_cpp = (POINT_LIO_DIR / "src" / "laserMapping.cpp").read_text()
        mid360_yaml = (POINT_LIO_DIR / "config" / "mid360.yaml").read_text()
        cmake = (POINT_LIO_DIR / "CMakeLists.txt").read_text()

        self.assertIn("pose_update_time_bin_ms", parameters_h)
        self.assertIn(
            'declare_parameter<double>("mapping.pose_update_time_bin_ms", 0.0)',
            parameters_cpp,
        )
        self.assertIn(
            "time_compressing<int>(feats_down_body, pose_update_time_bin_ms)",
            mapping_cpp,
        )
        self.assertIn("build_time_batches", common_lib)
        self.assertIn("pose_update_time_bin_ms: 2.0", mid360_yaml)
        self.assertIn("point_filter_num: 1", mid360_yaml)
        self.assertIn(
            "ament_add_gtest(time_batching_test test/time_batching_test.cpp)",
            cmake,
        )


if __name__ == "__main__":
    unittest.main()
