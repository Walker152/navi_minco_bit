#!/usr/bin/env python3

import re
import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[4]


class DynamicGlobalHardObstacleContractTest(unittest.TestCase):
    def read(self, relative_path: str) -> str:
        return (REPO_ROOT / relative_path).read_text(encoding="utf-8")

    def test_validation_collision_distance_is_shared_with_both_global_searchers(self) -> None:
        planner = self.read(
            "src/navigation/minco_planner/src/minco_core/minco_planner.cpp"
        )
        self.assertIn(
            "astar_planner_->setCollisionDistance(collision_dist);", planner
        )
        self.assertIn(
            "smac_planner_->setCollisionDistance(collision_dist);", planner
        )

    def test_astar_rejects_dynamic_esdf_collision_cells(self) -> None:
        header = self.read(
            "src/navigation/minco_planner/include/minco_core/astar.hpp"
        )
        implementation = self.read(
            "src/navigation/minco_planner/src/minco_core/astar.cpp"
        )
        self.assertIn("void setESDFQuery", header)
        self.assertIn("bool isDynamicCollision", header)
        self.assertRegex(
            implementation,
            re.compile(
                r"costarr\[nc\]\s*>=\s*COST_OBS_ROS.*?isDynamicCollision\(nc\)",
                re.DOTALL,
            ),
        )
        self.assertRegex(
            implementation,
            re.compile(r"if\s*\(.*?!result\.ok.*?return false;", re.DOTALL),
        )

    def test_smac_rejects_dynamic_esdf_collision_cells(self) -> None:
        header = self.read(
            "src/navigation/minco_planner/include/smac_search/smac_planner_2d_simple.hpp"
        )
        implementation = self.read(
            "src/navigation/minco_planner/src/smac_search/smac_planner_2d_simple.cpp"
        )
        self.assertIn("void setCollisionDistance", header)
        self.assertIn("bool isESDFCollision", header)
        traversable = re.search(
            r"const auto is_traversable\s*=.*?\n\s*\};",
            implementation,
            re.DOTALL,
        )
        self.assertIsNotNone(traversable)
        self.assertIn("isESDFCollision", traversable.group(0))

    def test_dynamic_query_failure_does_not_create_a_hard_obstacle(self) -> None:
        implementation = self.read(
            "src/navigation/minco_planner/src/smac_search/smac_planner_2d_simple.cpp"
        )
        distance_query = re.search(
            r"bool SmacPlanner2DSimple::getESDFDistance\(.*?\n\}",
            implementation,
            re.DOTALL,
        )
        self.assertIsNotNone(distance_query)
        self.assertRegex(
            distance_query.group(0),
            re.compile(r"!result\.ok.*?return false;", re.DOTALL),
        )

    def test_smac_query_setters_invalidate_distance_cache_ids(self) -> None:
        implementation = self.read(
            "src/navigation/minco_planner/src/smac_search/smac_planner_2d_simple.cpp"
        )
        for setter in ["setMap", "setESDFQuery", "setCollisionDistance"]:
            body = re.search(
                rf"void SmacPlanner2DSimple::{setter}\(.*?\n\}}",
                implementation,
                re.DOTALL,
            )
            self.assertIsNotNone(body)
            self.assertIn(
                "std::fill(esdf_distance_cache_id_.begin(), "
                "esdf_distance_cache_id_.end(), 0u);",
                body.group(0),
            )


if __name__ == "__main__":
    unittest.main()
