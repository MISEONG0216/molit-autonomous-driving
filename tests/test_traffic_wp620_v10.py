#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Offline per-stop V10 approach tests; no simulator connection required."""

import unittest
from src.integration.mission_manager import MissionManager


class WP620ApproachTests(unittest.TestCase):
    def setUp(self):
        self.manager = MissionManager(
            {
                "signal_timeout_sec": 1.2,
                "go_release_confirm_loops": 8,
                "approach_distance_m": 60.0,
                "approach_speed_kmh": 30.0,
                "stop_overrides": {
                    "620": {
                        "approach_distance_m": 100.0,
                        "approach_speed_kmh": 25.0,
                        "near_distance_m": 35.0,
                        "near_speed_kmh": 10.0,
                    }
                },
            },
            {},
            [414, 620, 879, 1104, 1224],
        )
        self.manager.set_global_path([(float(i), 0.0) for i in range(1500)])

    def decision(self, index, now=100.5):
        return self.manager.apply(
            nearest_index=index,
            base_target_speed_kmh=57.0,
            base_steering=0.13,
            traffic_enabled=True,
            now=now,
        )

    def test_first_light_keeps_60m_30kmh(self):
        self.assertEqual(self.decision(354)["target_speed_kmh"], 30.0)
        self.assertEqual(self.decision(340)["target_speed_kmh"], 57.0)

    def test_second_light_special_caps(self):
        self.manager.completed_stop_indices.add(414)
        self.assertEqual(self.decision(510)["target_speed_kmh"], 57.0)
        self.assertEqual(self.decision(520)["target_speed_kmh"], 25.0)
        self.assertEqual(self.decision(570)["target_speed_kmh"], 25.0)
        self.assertEqual(self.decision(585)["target_speed_kmh"], 10.0)
        self.assertEqual(self.decision(615)["target_speed_kmh"], 10.0)

    def test_red_only_full_brake_and_green_release(self):
        self.manager.completed_stop_indices.add(414)
        self.manager.update_traffic_input({"signal": "STOP"}, 100.1)
        r = self.decision(600, 100.2)
        self.assertTrue(r["traffic_full_brake"])
        self.assertEqual(r["target_speed_kmh"], 0.0)
        self.assertEqual(r["traffic_state"], "FULL_BRAKE_RED")

        self.manager.update_traffic_input({"signal": "UNKNOWN"}, 100.3)
        self.assertTrue(self.decision(630, 100.4)["traffic_full_brake"])

        for i in range(8):
            t = 100.5 + 0.05*i
            self.manager.update_traffic_input({"signal": "GO"}, t)
            r = self.decision(630, t + 0.001)
        self.assertFalse(r["traffic_full_brake"])
        self.assertIn(620, self.manager.completed_stop_indices)

    def test_green_and_unknown_are_not_automatic_stops(self):
        self.manager.completed_stop_indices.add(414)
        self.assertFalse(self.decision(615)["traffic_full_brake"])
        self.manager.update_traffic_input({"signal": "GO"}, 100.1)
        self.assertFalse(self.decision(615, 100.2)["traffic_full_brake"])
        self.assertEqual(self.decision(615, 100.2)["target_speed_kmh"], 10.0)


if __name__ == "__main__":
    unittest.main()
