#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""V16: combined red + green-left evidence must release only mapped left turn."""
import unittest

from src.integration.mission_io import parse_traffic_message
from src.integration.mission_manager import MissionManager


class CombinedLeftSignalTests(unittest.TestCase):
    def setUp(self):
        self.cfg = {
            "signal_timeout_sec": 1.2,
            "go_release_confirm_loops": 8,
            "approach_distance_m": 60,
            "approach_speed_kmh": 30,
            "left_turn_stops": [1224],
            "left_only_stops": [1224],
            "ignore_signal_stops": [1104],
            "stop_overrides": {"1224": {
                "approach_distance_m": 35,
                "approach_speed_kmh": 20,
                "signal_activation_index": 1150,
            }},
        }
        self.m = MissionManager(self.cfg, {}, [414, 620, 879, 1104, 1224])
        self.m.set_global_path([(0.5 * i, 0.0) for i in range(1400)])

    def step(self, raw, t, idx=1206):
        self.m.update_traffic_input(parse_traffic_message(raw), t)
        return self.m.apply(
            nearest_index=idx,
            base_target_speed_kmh=25.0,
            base_steering=0.0,
            traffic_enabled=True,
            now=t + 0.005,
        )

    def test_red_and_plain_green_cannot_release_fifth(self):
        first = self.step(b"STOP|-1|0|0", 100.0)
        self.assertTrue(first["red_latched"])
        self.assertTrue(first["traffic_full_brake"])
        for i in range(12):
            result = self.step(b"GO|-1|0|0", 100.1 + i * 0.05)
            self.assertTrue(result["red_latched"])
            self.assertEqual(result["go_release_count"], 0)

    def test_red_with_stable_left_flag_releases_after_distinct_frames(self):
        self.step(b"STOP|-1|0|0", 100.0)
        for i in range(7):
            result = self.step(b"STOP|-1|0|1", 100.1 + i * 0.05)
            self.assertTrue(result["red_latched"])
        result = self.step(b"STOP|-1|0|1", 100.5)
        self.assertFalse(result["red_latched"])
        self.assertFalse(result["traffic_full_brake"])
        self.assertFalse(result["left_arrow_override"] is False)
        self.assertIn(1224, self.m.completed_stop_indices)

    def test_unknown_does_not_release(self):
        self.step(b"STOP|-1|0|0", 100.0)
        for i in range(12):
            result = self.step(b"UNKNOWN|-1|0|0", 100.1 + i * 0.05)
        self.assertTrue(result["red_latched"])
        self.assertTrue(result["traffic_full_brake"])

    def test_left_flag_before_activation_gate_does_not_release(self):
        self.step(b"STOP|-1|0|0", 100.0)
        # A left-arrow observation from the previous junction must be ignored.
        for i in range(10):
            result = self.step(b"STOP|-1|0|1", 100.1 + i * 0.05, idx=1149)
        self.assertTrue(result["red_latched"])


if __name__ == "__main__":
    unittest.main()
