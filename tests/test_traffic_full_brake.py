#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Offline tests for RED full-brake mission decisions (no MORAI required)."""

import unittest

from src.integration.mission_manager import MissionManager


class TrafficFullBrakeTests(unittest.TestCase):
    def setUp(self):
        self.manager = MissionManager(
            {"signal_timeout_sec": 1.2, "go_release_confirm_loops": 8},
            {},
            [414, 620, 879, 1104, 1224],
        )

    def decision(self, index, now):
        return self.manager.apply(
            nearest_index=index,
            base_target_speed_kmh=45.0,
            base_steering=0.15,
            traffic_enabled=True,
            now=now,
        )

    def test_stop_full_brake_anywhere_not_just_near_waypoint(self):
        self.manager.update_traffic_input({"signal": "STOP"}, 100.0)
        for index in [3, 407, 414, 430, 700]:
            r = self.decision(index, 100.1)
            self.assertEqual(r["target_speed_kmh"], 0.0)
            self.assertEqual(r["traffic_state"], "FULL_BRAKE_RED")
            self.assertTrue(r["red_latched"])
            self.assertIn("traffic_full_brake", r["reasons"])
            # Keep V7 lateral control unchanged while braking.
            self.assertEqual(r["steering"], 0.15)

    def test_unknown_and_stale_do_not_release_stop(self):
        self.manager.update_traffic_input({"signal": "STOP"}, 100.0)
        self.decision(3, 100.01)
        self.manager.update_traffic_input({"signal": "UNKNOWN"}, 100.1)
        self.assertTrue(self.decision(999, 100.11)["red_latched"])
        self.assertTrue(self.decision(999, 103.0)["red_latched"])

    def test_green_needs_eight_distinct_fresh_packets(self):
        self.manager.update_traffic_input({"signal": "STOP"}, 100.0)
        self.decision(3, 100.01)
        self.manager.update_traffic_input({"signal": "GO"}, 100.10)
        for _ in range(20):
            r = self.decision(430, 100.11)
        self.assertTrue(r["red_latched"])
        self.assertEqual(r["go_release_count"], 1)

        for i in range(1, 8):
            t = 100.10 + 0.05 * i
            self.manager.update_traffic_input({"signal": "GO"}, t)
            r = self.decision(430, t + 0.01)
        self.assertFalse(r["red_latched"])
        self.assertEqual(r["target_speed_kmh"], 45.0)
        self.assertEqual(r["steering"], 0.15)


if __name__ == "__main__":
    unittest.main()
