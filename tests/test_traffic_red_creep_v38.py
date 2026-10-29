#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""V38 regression: limited RED creep must stop at the user's WP gates."""

import json
import unittest
from pathlib import Path

from src.integration.mission_manager import MissionManager

ROOT = Path(__file__).resolve().parents[1]
STOP_INDICES = [414, 620, 879, 1104, 1224]


class RedCreepV38Tests(unittest.TestCase):
    def setUp(self):
        cfg = json.loads(
            (ROOT / "config" / "integration_missions.json").read_text(encoding="utf-8")
        )
        self.assertEqual(cfg["traffic"]["red_creep_targets_wp"], {
            "414": 410, "620": 610, "1224": 1215,
        })
        self.assertEqual(cfg["traffic"]["red_creep_speed_kmh"], 5)
        self.m = MissionManager(cfg["traffic"], cfg["merge"], STOP_INDICES)
        self.m.set_global_path([(i * 0.5, 0.0) for i in range(1500)])

    def step(self, wp, now=100.1, speed=57.0):
        return self.m.apply(
            nearest_index=wp,
            base_target_speed_kmh=speed,
            base_steering=0.12,
            traffic_enabled=True,
            now=now,
        )

    def red(self, wp):
        self.m.update_traffic_input({"signal": "STOP", "left_arrow": False}, 100.0)
        return self.step(wp)

    def assert_creep_then_stop(self, initial_wp, last_creep_wp, gate_wp, stop_wp):
        r = self.red(initial_wp)
        self.assertEqual(r["active_stop_index"], stop_wp)
        self.assertTrue(r["red_latched"])
        self.assertEqual(r["target_speed_kmh"], 5.0)
        self.assertFalse(r["traffic_full_brake"])
        self.assertTrue(r["traffic_approach"])
        self.assertEqual(r["traffic_state"], f"RED_CREEP_{stop_wp}")
        self.assertEqual(r["steering"], 0.12)

        r = self.step(last_creep_wp)
        self.assertEqual(r["target_speed_kmh"], 5.0)
        self.assertFalse(r["traffic_full_brake"])

        r = self.step(gate_wp)
        self.assertTrue(r["traffic_full_brake"])
        self.assertTrue(r["red_latched"])
        self.assertEqual(r["target_speed_kmh"], 0.0)
        self.assertEqual(r["traffic_state"], "FULL_BRAKE_RED")

        # Nearest waypoint may move backward while standing still.
        r = self.step(last_creep_wp)
        self.assertTrue(r["traffic_full_brake"])
        self.assertEqual(r["target_speed_kmh"], 0.0)

    def test_first_wp410(self):
        self.assert_creep_then_stop(390, 409, 410, 414)

    def test_second_wp610(self):
        self.assert_creep_then_stop(590, 609, 610, 620)

    def test_fifth_wp1215(self):
        self.assert_creep_then_stop(1190, 1214, 1215, 1224)

    def test_third_signal_wp879_unchanged(self):
        r = self.red(870)
        self.assertEqual(r["active_stop_index"], 879)
        self.assertTrue(r["traffic_full_brake"])
        self.assertEqual(r["target_speed_kmh"], 0.0)

    def test_fourth_wp1104_still_ignored(self):
        r = self.red(1104)
        self.assertEqual(r["active_stop_index"], 1224)
        self.assertFalse(r["red_latched"])
        self.assertFalse(r["traffic_full_brake"])

    def test_stale_unknown_cannot_release_red(self):
        self.red(600)
        self.m.update_traffic_input({"signal": "UNKNOWN"}, 100.2)
        r = self.step(609, 100.21)
        self.assertTrue(r["red_latched"])
        self.assertEqual(r["target_speed_kmh"], 5.0)
        r = self.step(610, 104.0)
        self.assertTrue(r["red_latched"])
        self.assertTrue(r["traffic_full_brake"])

    def test_lower_base_speed_is_not_raised(self):
        self.red(600)
        r = self.step(601, speed=3.0)
        self.assertEqual(r["target_speed_kmh"], 3.0)

    def test_red_requires_eight_new_green_packets_to_release(self):
        self.red(600)
        for i in range(7):
            t = 100.2 + i * 0.05
            self.m.update_traffic_input({"signal": "GO"}, t)
            r = self.step(605, now=t + 0.01)
            self.assertTrue(r["red_latched"])
            self.assertEqual(r["target_speed_kmh"], 5.0)
        self.m.update_traffic_input({"signal": "GO"}, 100.6)
        r = self.step(605, now=100.61)
        self.assertFalse(r["red_latched"])
        self.assertFalse(r["traffic_full_brake"])


if __name__ == "__main__":
    unittest.main()
