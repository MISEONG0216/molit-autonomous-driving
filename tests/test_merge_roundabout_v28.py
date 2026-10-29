"""Merge camera gating and far-vehicle sensitivity regression checks."""
import ast
import json
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from integration.merge_side_io import (
    active_merge_cameras, attenuate_distant_risk,
    merge_camera_mode, vehicle_risk, format_result,
)
from integration.mission_io import parse_merge_message


class MergeRoundaboutV28Test(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.cfg = json.loads((ROOT / "config/integration_missions.json")
                             .read_text(encoding="utf-8"))["merge"]

    def test_roundabout_left_only_at_boundaries(self):
        for wp in (1770, 1780, 1820):
            mode = merge_camera_mode(wp, self.cfg)
            self.assertEqual(mode, "roundabout")
            self.assertEqual(active_merge_cameras(mode), ("left",))
            # --merge-all must NOT override roundabout rear camera prohibition.
            self.assertEqual(merge_camera_mode(wp, self.cfg, True), "roundabout")

    def test_highway_merge_both_cameras_enabled(self):
        for wp in (2480, 2520, 2537, 3050, 3120, 3190):
            mode = merge_camera_mode(wp, self.cfg)
            self.assertEqual(mode, "highway_merge")
            self.assertEqual(active_merge_cameras(mode), ("left", "left_rear"))
        # V29: user-observed completion WP2538, rear no longer used.
        for wp in (2538, 2540, 2560):
            mode = merge_camera_mode(wp, self.cfg)
            self.assertEqual(mode, "post_merge")
            self.assertEqual(active_merge_cameras(mode), ("left",))
        # V30: perception is pre-armed 25 WPs before zone boundaries.
        for wp, expected in ((1769, "roundabout_prearm"),
                             (2479, "highway_prearm"),
                             (3049, "highway_prearm"),
                             (1821, "off"), (2561, "off"),
                             (3191, "off")):
            self.assertEqual(merge_camera_mode(wp, self.cfg), expected)
        self.assertEqual(active_merge_cameras("off"), ())

    def test_far_boxes_are_less_sensitive_and_near_unchanged(self):
        self.assertEqual(self.cfg["risk_stop_threshold"], 0.6)
        self.assertEqual(self.cfg["rear_left_camera_port"], 1118)
        self.assertEqual(self.cfg["side_camera_ports"], [9293])
        far = attenuate_distant_risk(1.0, 0.005, self.cfg)
        intermediate = attenuate_distant_risk(1.0, 0.015, self.cfg)
        near = attenuate_distant_risk(1.0, 0.050, self.cfg)
        self.assertLess(far, 0.60)
        self.assertLess(far, intermediate)
        self.assertLess(intermediate, near)
        self.assertEqual(near, 1.0)
        self.assertEqual(attenuate_distant_risk(0.9, 0.05, self.cfg), 0.9)
        self.assertEqual(attenuate_distant_risk(0.5, 0.05, self.cfg), 0.5)
        off = dict(self.cfg, distance_sensitivity_enabled=False)
        self.assertEqual(attenuate_distant_risk(0.8, 0, off), 0.8)

    def test_old_risk_function_and_wire_protocol_preserved(self):
        small = vehicle_risk(10, 20, 30, 50, 640, 480)
        self.assertLess(small, 0.60)
        accepted = parse_merge_message(format_result(0.1, 0.0, "MERGE_ACCEPT"))
        self.assertTrue(accepted["valid"])
        self.assertEqual(accepted["decision"], "MERGE_ACCEPT")

    def test_worker_does_not_infer_disabled_rear_or_send_control(self):
        script = (ROOT / "src/merge_side_camera_worker.py").read_text(encoding="utf-8")
        ast.parse(script)
        self.assertIn('if side not in active_sides:', script)
        self.assertIn('if "left_rear" in active_sides else 0.0', script)
        self.assertIn("mode != previous_mode", script)
        self.assertIn("attenuate_distant_risk(current, area, cfg)", script)
        self.assertNotIn("EgoCtrlCmd(", script)
        self.assertNotIn('("right", 9295)', script)

if __name__ == "__main__":
    unittest.main()
