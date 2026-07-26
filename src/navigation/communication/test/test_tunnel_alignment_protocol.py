#!/usr/bin/env python3

import re
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[4]


class TunnelAlignmentProtocolTest(unittest.TestCase):
    def read(self, relative_path: str) -> str:
        return (REPO_ROOT / relative_path).read_text(encoding="utf-8")

    def assert_fields_at_tail(self, relative_path: str, fields: list[str]) -> None:
        content = self.read(relative_path)
        declarations = [
            line.split("#", 1)[0].strip()
            for line in content.splitlines()
            if line.split("#", 1)[0].strip()
        ]
        self.assertEqual(declarations[-len(fields) :], fields)

    def test_ros_messages_append_alignment_fields(self) -> None:
        self.assert_fields_at_tail(
            "src/ros_interfaces/msg/Behavior.msg",
            ["bool tunnel_align_active", "float32 tunnel_align_angle_deg"],
        )
        self.assert_fields_at_tail(
            "src/ros_interfaces/msg/SentryInfoOffline.msg",
            ["bool tunnel_yaw_aligned"],
        )

    def test_packed_protocol_appends_alignment_fields(self) -> None:
        protocol = self.read(
            "src/navigation/communication/include/utils/custom_protocol.hpp"
        )
        behavior_body = re.search(
            r"struct __attribute__\(\(packed, aligned\(1\)\)\) _BehaviorData\s*\{(.*?)"
            r"\n\s*_BehaviorData\(",
            protocol,
            re.DOTALL,
        )
        self.assertIsNotNone(behavior_body)
        self.assertRegex(
            behavior_body.group(1),
            r"bool tunnel_align_active\{\};\s*"
            r"float tunnel_align_angle_deg\{\};\s*$",
        )

        offline_body = re.search(
            r"struct __attribute__\(\(packed\)\) _SentryInfoOffline\s*\{(.*?)"
            r"\n\s*_SentryInfoOffline\(",
            protocol,
            re.DOTALL,
        )
        self.assertIsNotNone(offline_body)
        self.assertRegex(
            offline_body.group(1),
            r"bool tunnel_yaw_aligned\{\};\s*$",
        )

    def test_ros_protocol_bridge_carries_alignment_request_and_result(self) -> None:
        decision_bridge = self.read(
            "src/decision/bt_manager/src/ros_interface.cpp"
        )
        communication_bridge = self.read(
            "src/navigation/communication/include/com_interface_ros.hpp"
        )
        self.assertIn(
            "behavior_msg.tunnel_align_active = tunnel_align_active;",
            decision_bridge,
        )
        self.assertIn(
            "behavior_msg.tunnel_align_angle_deg = tunnel_align_angle_deg;",
            decision_bridge,
        )
        self.assertIn(
            'blackboard_->set<bool>("tunnel_yaw_aligned", msg->tunnel_yaw_aligned);',
            decision_bridge,
        )
        self.assertIn(
            "msg.tunnel_yaw_aligned = in.tunnel_yaw_aligned;",
            communication_bridge,
        )
        self.assertIn(
            "tunnel_align_active = behavior_.tunnel_align_active;",
            communication_bridge,
        )
        self.assertIn(
            "tunnel_align_angle_deg = behavior_.tunnel_align_angle_deg;",
            communication_bridge,
        )

    def test_stance_action_delegates_alignment_to_chassis(self) -> None:
        action = self.read(
            "src/decision/bt_manager/src/action/change_stance_action.cpp"
        )
        header = self.read(
            "src/decision/bt_manager/include/bt_manager/action/change_stance_action.hpp"
        )
        self.assertNotIn("computeTunnelGyroVelPid", action)
        self.assertNotIn("computeTunnelGyroVelPid", header)
        self.assertNotIn('blackboard->set("use_gyro_mode", true);', action)
        self.assertIn(
            'blackboard->set("tunnel_align_active", true);',
            action,
        )
        self.assertIn(
            'blackboard->set("tunnel_align_angle_deg", target_yaw_deg);',
            action,
        )
        self.assertIn("180.0 / M_PI", action)
        self.assertNotRegex(
            action,
            r'blackboard->set\(\s*"tunnel_yaw_aligned",\s*'
            r"std::fabs\(yaw_error\)",
        )

    def test_attack_fort_approach_holds_tunnel_alignment_after_move_stance(self) -> None:
        tree = ET.parse(
            REPO_ROOT / "src/decision/bt_manager/tree/stance_tree.xml"
        )
        approach = tree.find(".//Sequence[@name='ApproachEnemyFortMoveZeroGyro']")
        self.assertIsNotNone(approach)
        children = list(approach)
        self.assertEqual(
            [child.tag for child in children],
            ["Inverter", "ChangeStance", "TunnelGyroAlignAction"],
        )
        self.assertEqual(children[1].attrib.get("stance"), "MOVE")
        self.assertEqual(
            children[2].attrib.get("reuse_existing_target"),
            "true",
        )

    def test_chassis_alignment_disables_host_gyro_command(self) -> None:
        action = self.read(
            "src/decision/bt_manager/src/action/change_stance_action.cpp"
        )
        self.assertRegex(
            action,
            r'BT::InputPort<bool>\(\s*"reuse_existing_target"',
        )
        self.assertIn(
            'blackboard->set("use_gyro_mode", false);',
            action,
        )
        self.assertIn(
            'blackboard->set("gyro_vel", 0.0f);',
            action,
        )

    def test_odd_length_checksum_treats_tail_byte_as_unsigned(self) -> None:
        protocol = self.read(
            "src/navigation/communication/include/utils/protocol.hpp"
        )
        self.assertIn(
            "uint16_t tmp = static_cast<uint8_t>(__data[__len - 1]);",
            protocol,
        )


if __name__ == "__main__":
    unittest.main()
