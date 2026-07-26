#!/usr/bin/env python3

import re
import unittest
from pathlib import Path


POINT_LIO_DIR = Path(__file__).resolve().parents[1]


class RuntimeStatisticsContractTest(unittest.TestCase):
    def test_runtime_statistics_owns_all_point_lio_diagnostics(self):
        header = POINT_LIO_DIR / "include" / "runtime_statistics.h"
        source = POINT_LIO_DIR / "src" / "runtime_statistics.cpp"
        self.assertTrue(header.exists(), "runtime statistics header is missing")
        self.assertTrue(source.exists(), "runtime statistics source is missing")

        header_text = header.read_text()
        source_text = source.read_text()
        mapping_text = (POINT_LIO_DIR / "src" / "laserMapping.cpp").read_text()
        initialization_text = (
            POINT_LIO_DIR / "src" / "li_initialization.cpp"
        ).read_text()
        parameters_text = (POINT_LIO_DIR / "src" / "parameters.cpp").read_text()
        parameters_header = (POINT_LIO_DIR / "src" / "parameters.h").read_text()
        ivox_text = (POINT_LIO_DIR / "include" / "ivox" / "ivox3d.h").read_text()
        cmake_text = (POINT_LIO_DIR / "CMakeLists.txt").read_text()

        for filename in (
            "imu_pbp.txt",
            "mat_out.txt",
            "pos_log.txt",
            "performance.csv",
        ):
            self.assertIn(filename, source_text)

        for field in (
            "latest_lidar_stamp",
            "lidar_backlog_ms",
            "pending_lidar_frames",
            "lidar_buffer_frames",
            "pending_imu_samples",
            "imu_buffer_samples",
            "process_ms",
            "odom_publish_age_ms",
            "global_map_points",
            "pcd_wait_points",
            "path_pose_count",
            "pose_update_attempt_count",
            "pose_update_failure_count",
            "pose_update_points_avg",
            "pose_update_points_min",
            "pose_update_points_max",
            "ekf_update_ms",
            "ekf_update_max_ms",
            "effective_feature_points",
            "ivox_stats_sampled",
            "ivox_valid_grids",
            "ivox_points_per_grid_avg",
            "ivox_points_per_grid_max",
            "ivox_points_per_grid_stddev",
            "ivox_stats_ms",
        ):
            self.assertIn(field, header_text + source_text)

        self.assertRegex(
            header_text,
            re.compile(
                r"recordPoseUpdate\s*\(\s*uint64_t points_per_update,\s*"
                r"double sensor_time,\s*double update_time_ms,\s*bool successful\s*\)"
            ),
        )
        self.assertIn("ivox_->StatGridPoints()", mapping_text)
        self.assertIn("ivox_statistics_frame_counter_", mapping_text)
        self.assertIn("kIvoxStatisticsPeriodFrames = 20", mapping_text)
        self.assertIn("if (grid_count == 0)", ivox_text)
        self.assertIn("double sum_square", ivox_text)
        self.assertIn("RuntimeStatistics::instance()", mapping_text)
        self.assertIn("runtime_pos_log, log_dir", mapping_text)
        self.assertIn("resolveLogDirectory", mapping_text)
        self.assertIn(
            "RuntimeStatistics::instance().enabled() && print_cloud_input_fps",
            mapping_text,
        )
        self.assertIn("recordImu", initialization_text)
        self.assertIn("runtime_pos_log", mapping_text)
        self.assertIn('"runtime_log_path"', parameters_text)
        self.assertIn("runtime_log_path", parameters_header)
        self.assertIn("resolveLogDirectory", header_text)
        self.assertIn("configured_path.is_absolute()", source_text)
        self.assertNotIn("ofstream fout_out", parameters_text)
        self.assertNotIn("extern ofstream fout_out", parameters_header)
        self.assertNotIn("void open_file()", parameters_text)
        self.assertNotIn("struct RuntimeRateStats", mapping_text)
        self.assertNotIn("struct PoseUpdateDebugStats", mapping_text)
        self.assertNotIn("dump_lio_state_to_log", mapping_text)
        self.assertIn("src/runtime_statistics.cpp", cmake_text)
        self.assertIn("ament_add_gtest(", cmake_text)
        self.assertIn("runtime_statistics_test", cmake_text)


if __name__ == "__main__":
    unittest.main()
