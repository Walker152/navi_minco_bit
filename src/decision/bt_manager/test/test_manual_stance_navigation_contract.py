#!/usr/bin/env python3

import re
import unittest
from pathlib import Path


BT_MANAGER_DIR = Path(__file__).resolve().parents[1]


def extract_case(source: str, key: int) -> str:
    match = re.search(
        rf"  case {key}:.*?(?=\n  case |\n  default:)",
        source,
        re.DOTALL,
    )
    if match is None:
        raise AssertionError(f"manual_key case {key} is missing")
    return match.group(0)


class ManualStanceNavigationContractTest(unittest.TestCase):
    def test_only_a_enters_manual_navigation(self):
        source = (BT_MANAGER_DIR / "src" / "ros_interface.cpp").read_text()

        self.assertIn("ControlMode::MANUAL_CONTROL", extract_case(source, 65))
        self.assertNotIn("ControlMode::MANUAL_CONTROL", extract_case(source, 67))
        self.assertNotIn("ControlMode::MANUAL_CONTROL", extract_case(source, 68))

    def test_stance_timeout_does_not_change_navigation_mode(self):
        source = (
            BT_MANAGER_DIR / "src" / "condition" / "change_stance_condition.cpp"
        ).read_text()
        timeout_block = re.search(
            r"  if \(remaining_sec <= 0\) \{(.*?)\n  \}",
            source,
            re.DOTALL,
        )

        self.assertIsNotNone(timeout_block, "enhanced stance timeout block is missing")
        self.assertNotIn("control_mode", timeout_block.group(1))


if __name__ == "__main__":
    unittest.main()
