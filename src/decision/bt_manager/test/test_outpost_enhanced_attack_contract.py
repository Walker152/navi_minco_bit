#!/usr/bin/env python3

import unittest
import xml.etree.ElementTree as ET
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[4]


class OutpostEnhancedAttackContractTest(unittest.TestCase):
    def test_outpost_response_prefers_enhanced_attack_with_normal_fallback(self) -> None:
        tree = ET.parse(
            REPO_ROOT / "src/decision/bt_manager/tree/stance_tree.xml"
        )
        absolute = tree.find(".//ReactiveFallback[@name='AbsoluteAttackBranch']")
        self.assertIsNotNone(absolute)

        outpost = absolute.find("./Sequence[@name='OutpostEnhancedAttackBranch']")
        self.assertIsNotNone(outpost)
        children = list(outpost)
        self.assertEqual(
            [child.tag for child in children],
            ["CheckOutpostTarget", "SetGyroState", "Fallback"],
        )
        self.assertEqual(children[0].attrib.get("require_pitch_up"), "true")

        selector = children[2]
        enhanced = selector.find("./Sequence[@name='TryOutpostEnhancedAttack']")
        self.assertIsNotNone(enhanced)
        enhance_children = list(enhanced)
        self.assertEqual(
            [child.tag for child in enhance_children],
            ["CheckShouldEnhanceStance", "ChangeStance"],
        )
        self.assertEqual(enhance_children[0].attrib.get("target_stance"), "ATTACK")
        self.assertEqual(enhance_children[0].attrib.get("min_remaining_sec"), "1")
        self.assertEqual(enhance_children[0].attrib.get("force_if_available"), "true")
        self.assertEqual(enhance_children[1].attrib.get("stance"), "ENHANCED_ATTACK")

        fallback = list(selector)[1]
        self.assertEqual(fallback.tag, "ChangeStance")
        self.assertEqual(fallback.attrib.get("stance"), "ATTACK")

    def test_heat_attack_path_remains_separate(self) -> None:
        tree = ET.parse(
            REPO_ROOT / "src/decision/bt_manager/tree/stance_tree.xml"
        )
        absolute = tree.find(".//ReactiveFallback[@name='AbsoluteAttackBranch']")
        self.assertIsNotNone(absolute)
        heat = absolute.find("./Sequence[@name='HeatAttackBranch']")
        self.assertIsNotNone(heat)
        self.assertEqual(list(heat)[0].tag, "CheckHeat")
        self.assertIsNotNone(heat.find("./Fallback[@name='AbsoluteAttackStanceSelector']"))

    def test_force_if_available_only_bypasses_automatic_trigger(self) -> None:
        source = (
            REPO_ROOT
            / "src/decision/bt_manager/src/condition/change_stance_condition.cpp"
        ).read_text(encoding="utf-8")
        self.assertIn('BT::InputPort<bool>("force_if_available", false', source)
        self.assertIn(
            'getInput<bool>("force_if_available").value_or(false)', source
        )
        self.assertIn("if (force_if_available)", source)
        self.assertIn('BT::InputPort<bool>("require_pitch_up", false', source)
        self.assertIn('blackboard->get<PitchPos>("pitch_mode")', source)


if __name__ == "__main__":
    unittest.main()
