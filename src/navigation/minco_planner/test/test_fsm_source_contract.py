#!/usr/bin/env python3

import re
import unittest
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
FSM_SOURCE = PACKAGE_ROOT / "src/minco_core/minco_fsm.cpp"
PLANNER_HEADER = PACKAGE_ROOT / "include/minco_core/minco_planner.hpp"


class MincoFsmSourceContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.fsm_source = FSM_SOURCE.read_text(encoding="utf-8")
        cls.planner_header = PLANNER_HEADER.read_text(encoding="utf-8")

    def test_duplicate_goal_uses_trajectory_goal_tolerance(self):
        self.assertIn("getTrajGoalTolerance", self.planner_header)
        self.assertRegex(
            self.fsm_source,
            re.compile(
                r"consumePendingGoal\(new_goal\).*?"
                r"hypot\(.*?new_goal\.pose\.position\.x.*?goal_\.pose\.position\.x.*?"
                r"new_goal\.pose\.position\.y.*?goal_\.pose\.position\.y.*?\).*?"
                r"getTrajGoalTolerance\(\).*?"
                r"if \(!same_goal\).*?changeState\(\"NewGoal\", State::GENERATE_TRAJ\)",
                re.DOTALL,
            ),
        )

    def test_unsafe_trajectory_does_not_wait_for_expiration(self):
        follow_start = self.fsm_source.index("case State::FOLLOW_TRAJ")
        follow_source = self.fsm_source[follow_start:]

        self.assertIn("current_traj_safe = planner_->isTrajSafe()", follow_source)
        self.assertIn("trajectory_expired = planner_->isTrajectoryTimeExpired(now_s)", follow_source)
        self.assertIn("current_traj_safe && !trajectory_expired", follow_source)

        safe_snapshot = follow_source.index("current_traj_safe = planner_->isTrajSafe()")
        expiration_snapshot = follow_source.index("trajectory_expired = planner_->isTrajectoryTimeExpired(now_s)")
        local_replan = follow_source.index("planner_->ReplanLocal(current_pose)")
        safe_and_unexpired = follow_source.index("current_traj_safe && !trajectory_expired")

        self.assertLess(safe_snapshot, local_replan)
        self.assertLess(expiration_snapshot, local_replan)
        self.assertGreater(safe_and_unexpired, local_replan)


if __name__ == "__main__":
    unittest.main()
