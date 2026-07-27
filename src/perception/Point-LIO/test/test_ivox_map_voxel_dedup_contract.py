#!/usr/bin/env python3

import re
import unittest
from pathlib import Path


POINT_LIO_DIR = Path(__file__).resolve().parents[1]


class IVoxMapVoxelDedupContractTest(unittest.TestCase):
    def test_default_ivox_keeps_one_representative_per_map_voxel(self):
        ivox_header = (
            POINT_LIO_DIR / "include" / "ivox" / "ivox3d.h"
        ).read_text()
        node_header = (
            POINT_LIO_DIR / "include" / "ivox" / "ivox3d_node.hpp"
        ).read_text()
        parameters = (POINT_LIO_DIR / "src" / "parameters.cpp").read_text()
        config = (POINT_LIO_DIR / "config" / "mid360.yaml").read_text()
        cmake = (POINT_LIO_DIR / "CMakeLists.txt").read_text()

        self.assertIn("point_resolution_", ivox_header)
        self.assertIn(
            "ivox_options_.point_resolution_ = "
            "static_cast<float>(filter_size_map_min)",
            parameters,
        )
        self.assertIn("subcell_to_point_index_", node_header)
        self.assertIn("subcells_per_axis_", node_header)
        self.assertIn(
            "std::ceil(side_length / point_resolution_)",
            node_header,
        )
        self.assertRegex(
            node_header,
            re.compile(
                r"if\s*\(\s*subcell_to_point_index_\[subcell_index\]\s*"
                r"==\s*kEmptyPointIndex\s*\)"
            ),
        )
        self.assertIn(
            "points_[point_index] = pt",
            node_header,
        )
        self.assertIn(
            "if constexpr (node_type == IVoxNodeType::DEFAULT)",
            ivox_header,
        )
        self.assertRegex(
            cmake,
            re.compile(
                r"ament_add_gtest\s*\(\s*ivox_map_voxel_dedup_test"
            ),
        )

        ivox_resolution = float(
            re.search(r"ivox_grid_resolution:\s*([0-9.]+)", config).group(1)
        )
        map_resolution = float(
            re.search(r"filter_size_map:\s*([0-9.]+)", config).group(1)
        )
        points_per_axis = round(ivox_resolution / map_resolution)
        self.assertEqual(points_per_axis ** 3, 64)

        self.assertRegex(
            config,
            re.compile(r"pose_update_time_bin_ms:\s*2\.0\b"),
        )
        self.assertRegex(
            config,
            re.compile(r"publish_frequency_hz:\s*100\b"),
        )


if __name__ == "__main__":
    unittest.main()
