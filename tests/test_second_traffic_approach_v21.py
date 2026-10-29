#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""V21 regression: only WP620 traffic approach gets less conservative caps.

The red-light full-brake latch is intentionally unchanged.
"""
import json
import unittest
from pathlib import Path

from src.integration.mission_manager import MissionManager

ROOT = Path(__file__).resolve().parents[1]


class SecondTrafficApproachTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.config = json.loads(
            (ROOT / "config" / "integration_missions.json").read_text(encoding="utf-8")
        )

    def setUp(self):
        traffic = self.config["traffic"]
        self.manager = MissionManager(
            traffic_cfg=traffic,
            merge_cfg=self.config["merge"],
            stop_indices=[414, 620, 879, 1104, 1224],
        )
        # 0.5m waypoint spacing, as in the experiment's route.
        self.manager.set_global_path([(i * 0.5, 0.0) for i in range(4430)])
        self.manager.update_traffic_input({"signal": "GO"}, 100.0)

    def run_at(self, wp, now=100.1):
        return self.manager.apply(
            nearest_index=wp,
            base_target_speed_kmh=57.0,
            base_steering=0.0,
            traffic_enabled=True,
            now=now,
        )

    def test_approach_override_exact_config(self):
        expected = {
            "approach_distance_m": 65,
            "approach_speed_kmh": 35,
            "near_distance_m": 25,
            "near_speed_kmh": 20,
        }
        settings = self.config["traffic"]["stop_overrides"]["620"]
        self.assertEqual(
            {key: settings[key] for key in expected}, expected
        )

    def test_before_65m_preserves_v7_target(self):
        result = self.run_at(489)  # 65.5m before WP620
        self.assertEqual(result["active_stop_index"], 620)
        self.assertFalse(result["traffic_approach"])
        self.assertEqual(result["target_speed_kmh"], 57.0)

    def test_at_65m_limits_to_35kmh(self):
        result = self.run_at(490)  # 65m before WP620
        self.assertTrue(result["traffic_approach"])
        self.assertAlmostEqual(result["stop_distance_m"], 65.0)
        self.assertEqual(result["target_speed_kmh"], 35.0)

    def test_at_25_5m_still_35kmh(self):
        result = self.run_at(569)  # 25.5m before WP620
        self.assertEqual(result["target_speed_kmh"], 35.0)

    def test_at_25m_limits_to_20kmh(self):
        result = self.run_at(570)  # 25m before WP620
        self.assertAlmostEqual(result["stop_distance_m"], 25.0)
        self.assertEqual(result["target_speed_kmh"], 20.0)

    def test_still_at_20kmh_until_stopline_if_go(self):
        result = self.run_at(618)  # 1m before WP620
        self.assertEqual(result["target_speed_kmh"], 20.0)
        self.assertFalse(result["traffic_full_brake"])

    def test_red_creeps_to_wp610_then_full_stop(self):
        self.manager.update_traffic_input({"signal": "STOP"}, 101.0)
        result = self.run_at(600, now=101.1)  # 10m before WP620
        self.assertEqual(result["target_speed_kmh"], 5.0)
        self.assertFalse(result["traffic_full_brake"])
        self.assertTrue(result["red_latched"])
        at_gate = self.run_at(610, now=101.2)
        self.assertEqual(at_gate["target_speed_kmh"], 0.0)
        self.assertTrue(at_gate["traffic_full_brake"])

    def test_earlier_signal_config_unchanged(self):
        self.assertEqual(self.config["traffic"]["approach_distance_m"], 60)
        self.assertEqual(self.config["traffic"]["approach_speed_kmh"], 30)

    def test_fifth_left_signal_override_unchanged(self):
        self.assertEqual(
            self.config["traffic"]["stop_overrides"]["1224"], {
                "approach_distance_m": 35,
                "approach_speed_kmh": 20,
                "near_distance_m": 15,
                "near_speed_kmh": 8,
                "signal_activation_index": 1150,
            },
        )

    def test_static_25kmh_and_zone_unchanged(self):
        self.assertEqual(self.config["static"]["avoidance_speed_kmh"], 25)
        self.assertEqual(self.config["static"]["avoidance_zones_wp"], [[590, 760]])


if __name__ == "__main__":
    unittest.main()
