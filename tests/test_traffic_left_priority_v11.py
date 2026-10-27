#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Mapped left-arrow precedence and passed-GO-stop regression tests."""
import unittest
from src.integration.mission_manager import MissionManager
from src.integration.mission_io import parse_traffic_message


class LeftPriorityTest(unittest.TestCase):
    def setUp(self):
        self.m = MissionManager(
            {
                "signal_timeout_sec": 1.2,
                "go_release_confirm_loops": 8,
                "approach_distance_m": 60,
                "approach_speed_kmh": 30,
                "left_turn_stops": [1104],
            },
            {}, [414, 620, 879, 1104, 1224],
        )
        self.m.set_global_path([(i * 0.5, 0.0) for i in range(1500)])

    def step(self, idx, t=100.2):
        return self.m.apply(
            nearest_index=idx, base_target_speed_kmh=57,
            base_steering=0.2, traffic_enabled=True, now=t,
        )

    def test_legacy_and_left_udp_parsing(self):
        self.assertFalse(parse_traffic_message(b"STOP|-1.0000|0.00")["left_arrow"])
        self.assertTrue(parse_traffic_message(b"STOP|-1.0000|0.00|1")["left_arrow"])

    def test_skip_green_intersection_when_passed(self):
        result = self.step(900)
        self.assertEqual(result["active_stop_index"], 1104)

    def test_red_plus_left_does_not_brake_at_fourth(self):
        self.m.update_traffic_input({"signal": "STOP", "left_arrow": True}, 100.1)
        result = self.step(1100)
        self.assertEqual(result["active_stop_index"], 1104)
        self.assertTrue(result["left_arrow_override"])
        self.assertFalse(result["traffic_full_brake"])
        self.assertFalse(result["red_latched"])
        self.assertEqual(result["target_speed_kmh"], 30)
        self.assertEqual(result["steering"], 0.2)

    def test_red_still_brakes_without_left(self):
        self.m.update_traffic_input({"signal": "STOP", "left_arrow": False}, 100.1)
        self.assertTrue(self.step(1100)["traffic_full_brake"])

    def test_red_with_left_does_not_override_other_lights(self):
        self.m.update_traffic_input({"signal": "STOP", "left_arrow": True}, 100.1)
        result = self.step(410)
        self.assertTrue(result["traffic_full_brake"])
        self.assertFalse(result["left_arrow_override"])

    def test_left_releases_latched_red_after_eight_fresh_packets(self):
        self.m.update_traffic_input({"signal": "STOP", "left_arrow": False}, 100.1)
        self.assertTrue(self.step(1100, 100.11)["red_latched"])
        for i in range(8):
            t = 100.2 + i * 0.05
            self.m.update_traffic_input({"signal": "STOP", "left_arrow": True}, t)
            result = self.step(1100, t + 0.001)
        self.assertFalse(result["red_latched"])
        self.assertFalse(result["traffic_full_brake"])


if __name__ == "__main__":
    unittest.main()
