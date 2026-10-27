#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Offline V9 traffic-approach state-machine tests; no MORAI needed."""

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
                "slow_distance_m": 25.0,
                "slow_speed_kmh": 12.0,
                "crawl_distance_m": 10.0,
                "crawl_speed_kmh": 4.0,
                "unknown_hold_distance_m": 0.5,
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

    def test_staged_approach_in_metres(self):
        r = self.decision(0)
        self.assertEqual(r["active_stop_index"], 100)
        self.assertAlmostEqual(r["stop_distance_m"], 50.0)
        self.assertEqual(r["target_speed_kmh"], 30.0)
        self.assertTrue(r["traffic_approach"])

        r = self.decision(50)
        self.assertAlmostEqual(r["stop_distance_m"], 25.0)
        self.assertEqual(r["target_speed_kmh"], 12.0)
        r = self.decision(80)
        self.assertAlmostEqual(r["stop_distance_m"], 10.0)
        self.assertEqual(r["target_speed_kmh"], 4.0)

    def test_unknown_holds_near_mapped_stop(self):
        r = self.decision(99)
        self.assertEqual(r["target_speed_kmh"], 0.0)
        self.assertTrue(r["traffic_full_brake"])
        self.assertEqual(r["traffic_state"], "FULL_BRAKE_UNKNOWN_100")
        # Even after overshooting the stop index, hold must persist.
        r = self.decision(120)
        self.assertTrue(r["traffic_full_brake"])

    def test_green_continues_at_approach_cap(self):
        self.manager.update_traffic_input({"signal": "GO"}, 100.1)
        r = self.decision(80, 100.2)
        self.assertEqual(r["target_speed_kmh"], 30.0)
        self.assertFalse(r["traffic_full_brake"])
        self.assertEqual(r["traffic_state"], "APPROACH_GO_100")

    def test_red_brakes_and_green_requires_new_packets(self):
        self.manager.update_traffic_input({"signal": "STOP"}, 100.1)
        r = self.decision(0, 100.2)
        self.assertTrue(r["red_latched"])
        self.assertEqual(r["target_speed_kmh"], 0.0)
        self.assertEqual(r["traffic_state"], "FULL_BRAKE_RED")
        self.assertEqual(r["steering"], 0.15)

        # Camera loses light: no blind departure.
        self.manager.update_traffic_input({"signal": "UNKNOWN"}, 100.3)
        self.assertTrue(self.decision(120, 100.4)["traffic_full_brake"])

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

    def test_ignore_red_outside_approach_zone(self):
        # At wp 0, second stop 260 is 130m away.
        self.manager.completed_stop_indices.add(100)
        self.manager.update_traffic_input({"signal": "STOP"}, 100.1)
        r = self.decision(0, 100.2)
        self.assertFalse(r["red_latched"])
        self.assertFalse(r["traffic_full_brake"])
        self.assertEqual(r["target_speed_kmh"], 45.0)


if __name__ == "__main__":
    unittest.main()
