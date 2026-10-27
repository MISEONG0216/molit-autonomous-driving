#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Offline tests for V9: 60m cap, RED-only full brake, confirmed GO release."""

import unittest

from src.integration.mission_manager import MissionManager


class TrafficApproachV9Tests(unittest.TestCase):
    def setUp(self):
        self.manager = MissionManager(
            {
                "signal_timeout_sec": 1.2,
                "go_release_confirm_loops": 8,
                "approach_distance_m": 60.0,
                "approach_speed_kmh": 30.0,
            },
            {},
            [100, 260],
        )
        self.manager.set_global_path([(0.5 * i, 0.0) for i in range(400)])

    def decision(self, wp, now=100.2):
        return self.manager.apply(
            nearest_index=wp,
            base_target_speed_kmh=45.0,
            base_steering=0.15,
            traffic_enabled=True,
            now=now,
        )

    def test_approach_cap_uses_actual_path_distance(self):
        r = self.decision(0)
        self.assertEqual(r["active_stop_index"], 100)
        self.assertAlmostEqual(r["stop_distance_m"], 50.0)
        self.assertEqual(r["target_speed_kmh"], 30.0)
        self.assertTrue(r["traffic_approach"])
        self.assertFalse(r["traffic_full_brake"])
        self.assertEqual(r["steering"], 0.15)

        # UNKNOWN may not trigger an unrequested full stop, even near the line.
        r = self.decision(99)
        self.assertAlmostEqual(r["stop_distance_m"], 0.5)
        self.assertEqual(r["target_speed_kmh"], 30.0)
        self.assertFalse(r["traffic_full_brake"])

    def test_green_and_unknown_only_cap_to_30(self):
        self.manager.update_traffic_input({"signal": "GO"}, 100.1)
        r = self.decision(80, 100.2)
        self.assertEqual(r["target_speed_kmh"], 30.0)
        self.assertFalse(r["traffic_full_brake"])
        self.assertEqual(r["traffic_state"], "APPROACH_GO_100")

        self.manager.update_traffic_input({"signal": "UNKNOWN"}, 100.3)
        r = self.decision(90, 100.4)
        self.assertEqual(r["target_speed_kmh"], 30.0)
        self.assertFalse(r["traffic_full_brake"])

    def test_red_full_brake_even_after_stop_index(self):
        self.manager.update_traffic_input({"signal": "STOP"}, 100.1)
        r = self.decision(0, 100.2)
        self.assertTrue(r["red_latched"])
        self.assertTrue(r["traffic_full_brake"])
        self.assertEqual(r["target_speed_kmh"], 0.0)

        self.manager.update_traffic_input({"signal": "UNKNOWN"}, 100.3)
        r = self.decision(120, 100.4)
        self.assertTrue(r["traffic_full_brake"])
        self.assertTrue(r["red_latched"])

    def test_green_release_needs_eight_distinct_packets(self):
        self.manager.update_traffic_input({"signal": "STOP"}, 100.1)
        self.decision(0, 100.2)

        self.manager.update_traffic_input({"signal": "GO"}, 100.5)
        for _ in range(20):
            r = self.decision(120, 100.51)
        self.assertTrue(r["traffic_full_brake"])
        self.assertEqual(r["go_release_count"], 1)

        for i in range(1, 8):
            t = 100.5 + i * 0.05
            self.manager.update_traffic_input({"signal": "GO"}, t)
            r = self.decision(120, t + 0.001)
        self.assertFalse(r["red_latched"])
        self.assertFalse(r["traffic_full_brake"])
        self.assertEqual(r["target_speed_kmh"], 45.0)
        self.assertIn(100, self.manager.completed_stop_indices)

    def test_red_outside_approach_is_ignored(self):
        self.manager.completed_stop_indices.add(100)
        self.manager.update_traffic_input({"signal": "STOP"}, 100.1)
        r = self.decision(0, 100.2)
        self.assertFalse(r["red_latched"])
        self.assertFalse(r["traffic_full_brake"])
        self.assertEqual(r["target_speed_kmh"], 45.0)


if __name__ == "__main__":
    unittest.main()
