#!/usr/bin/env python3

import re
import unittest
from pathlib import Path


HEADER = (
    Path(__file__).resolve().parents[1] / "include" / "com_interface_ros.hpp"
).read_text(encoding="utf-8")


class OdomSubscriptionConfigurationTest(unittest.TestCase):
    def test_odom_uses_dedicated_callback_group_and_latest_best_effort_qos(self):
        self.assertIn(
            "rclcpp::CallbackGroup::SharedPtr odom_cb_group_;",
            HEADER,
        )
        self.assertRegex(
            HEADER,
            re.compile(
                r"odom_cb_group_\s*=\s*this->create_callback_group\("
                r"\s*rclcpp::CallbackGroupType::MutuallyExclusive\s*\)"
            ),
        )
        self.assertRegex(
            HEADER,
            re.compile(
                r"rclcpp::SubscriptionOptions\s+odom_sub_opt\s*;"
                r".*?odom_sub_opt\.callback_group\s*=\s*odom_cb_group_\s*;",
                re.DOTALL,
            ),
        )
        self.assertRegex(
            HEADER,
            re.compile(
                r"auto\s+odom_qos\s*=\s*rclcpp::QoS\(rclcpp::KeepLast\(1\)\)"
                r"\.best_effort\(\)\.durability_volatile\(\)\s*;"
            ),
        )
        self.assertRegex(
            HEADER,
            re.compile(
                r"odom_sub_\s*=\s*create_subscription<nav_msgs::msg::Odometry>\("
                r".*?odom_qos,.*?odom_sub_opt\s*\);",
                re.DOTALL,
            ),
        )


if __name__ == "__main__":
    unittest.main()
