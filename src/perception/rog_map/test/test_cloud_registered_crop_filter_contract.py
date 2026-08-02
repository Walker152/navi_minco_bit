#!/usr/bin/env python3

import re
import unittest
from pathlib import Path

import yaml


REPO_ROOT = Path(__file__).resolve().parents[4]


class CloudRegisteredCropFilterContractTest(unittest.TestCase):
    def read(self, relative_path: str) -> str:
        return (REPO_ROOT / relative_path).read_text(encoding="utf-8")

    def test_filter_is_an_independent_class(self) -> None:
        header = self.read(
            "src/perception/rog_map/include/rog_map_ros/cloud_registered_crop_filter.hpp"
        )
        self.assertIn("class CloudRegisteredCropFilter", header)
        self.assertRegex(header, r"bool\s+filter\s*\(\s*PointCloud\s*&")

    def test_config_keeps_original_filter_parameters(self) -> None:
        config = self.read(
            "src/perception/rog_map/include/rog_map/rog_map_core/config.hpp"
        )
        required_keys = [
            "cloud_filter.enable",
            "cloud_filter.z_offset",
            "cloud_filter.position_frame",
            "cloud_filter.filter_mode",
            "cloud_filter.remove_inside",
            "cloud_filter.log_stats",
            "cloud_filter.stats_log_period_ms",
            "cloud_filter.publish_visualization",
            "cloud_filter.visualization_topic",
            "cloud_filter.visualization_period_ms",
            "cloud_filter.z_plane_size",
            "cloud_filter.z_plane_thickness",
            "cloud_filter.box_padding",
            "cloud_filter.position.x",
            "cloud_filter.position.y",
            "cloud_filter.position.z",
            "cloud_filter.box_size.x",
            "cloud_filter.box_size.y",
            "cloud_filter.box_size.z",
            "cloud_filter.positions.x",
            "cloud_filter.positions.y",
            "cloud_filter.positions.z",
            "cloud_filter.box_sizes.x",
            "cloud_filter.box_sizes.y",
            "cloud_filter.box_sizes.z",
        ]
        for key in required_keys:
            self.assertIn(f'load("{key}"', config)

    def test_filter_pointer_runs_before_cloud_enters_map_queue(self) -> None:
        ros = self.read(
            "src/perception/rog_map/include/rog_map_ros/rog_map_ros2.hpp"
        )
        self.assertIn("std::unique_ptr<CloudRegisteredCropFilter> cloud_filter_", ros)
        callback = re.search(
            r"void cloudCallback\(.*?\n  \}\n\n  void updateCallback",
            ros,
            re.DOTALL,
        )
        self.assertIsNotNone(callback)
        body = callback.group(0)
        filter_call = body.index("cloud_filter_->filter")
        queue_write = body.index("rc_.pc = temp_pc")
        self.assertLess(filter_call, queue_write)

    def test_sentry_config_is_disabled_by_default_and_contains_map_boxes(self) -> None:
        params_path = REPO_ROOT / "src/navigation/navi2_bringup/params/sentry1.yaml"
        params = yaml.safe_load(params_path.read_text(encoding="utf-8"))
        cloud_filter = params["planner_server"]["ros__parameters"]["MincoPlanner"][
            "rog_map"
        ]["cloud_filter"]
        self.assertFalse(cloud_filter["enable"])
        self.assertEqual(cloud_filter["position_frame"], "map")
        self.assertEqual(len(cloud_filter["positions"]["x"]), 4)
        self.assertEqual(len(cloud_filter["box_sizes"]["x"]), 4)
        self.assertTrue(cloud_filter["publish_visualization"])


if __name__ == "__main__":
    unittest.main()
