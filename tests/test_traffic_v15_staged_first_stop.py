#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""V15 regression tests: early 1st light red should approach before stopping."""
import unittest

from src.integration.mission_manager import MissionManager


class FirstSignalStagedStopTests(unittest.TestCase):
    def setUp(self):
        self.m = MissionManager({
            "signal_timeout_sec": 1.2,
            "go_release_confirm_loops": 8,
            "approach_distance_m": 60.0,
            "approach_speed_kmh": 30.0,
            "ignore_signal_stops": [1104],
            "left_turn_stops": [1224],
            "left_only_stops": [1224],
            "stop_overrides": {
                "414": {
                    "approach_distance_m": 60,
                    "approach_speed_kmh": 30,
                    "red_stop_offset_m": 8,
                    "red_slow_start_m": 45,
                    "red_slow_speed_kmh": 20,
                    "red_near_distance_m": 20,
                    "red_near_speed_kmh": 9,
                    "red_final_distance_m": 12,
                    "red_final_speed_kmh": 5,
                },
                "620": {
                    "approach_distance_m": 100,
                    "approach_speed_kmh": 25,
                    "near_distance_m": 35,
                    "near_speed_kmh": 10,
                },
                "1224": {
                    "approach_distance_m": 35,
                    "approach_speed_kmh": 20,
                    "signal_activation_index": 1150,
                },
            }
        }, {}, [414, 620, 879, 1104, 1224])
        self.m.set_global_path([(float(i) * 0.5, 0.0) for i in range(1500)])

    def step(self, idx, now=100.2):
        return self.m.apply(
            nearest_index=idx, base_target_speed_kmh=57,
            base_steering=0.0, traffic_enabled=True, now=now,
        )

    def test_red_50m_away_staged_not_full_stop(self):
        self.m.update_traffic_input({"signal": "STOP"}, 100.1)
        # 414-100=314 => 50 metres to first signal.
        r = self.step(314)
        self.assertEqual(r["active_stop_index"], 414)
        self.assertTrue(r["red_latched"])
        self.assertFalse(r["traffic_full_brake"])
        self.assertTrue(r["traffic_approach"])
        self.assertEqual(r["target_speed_kmh"], 30.0)

    def test_red_19m_away_slows_to_9(self):
        self.m.update_traffic_input({"signal": "STOP"}, 100.1)
        r = self.step(376)  # 19m remaining
        self.assertEqual(r["target_speed_kmh"], 9.0)
        self.assertFalse(r["traffic_full_brake"])

    def test_red_8m_away_full_brake(self):
        self.m.update_traffic_input({"signal": "STOP"}, 100.1)
        r = self.step(398)  # 8m remaining
        self.assertEqual(r["target_speed_kmh"], 0)
        self.assertTrue(r["traffic_full_brake"])

    def test_red_latch_holds_if_signal_turns_unknown(self):
        self.m.update_traffic_input({"signal": "STOP"}, 100.1)
        self.step(314)
        self.m.update_traffic_input({"signal": "UNKNOWN"}, 100.3)
        r = self.step(398, 100.4)
        self.assertTrue(r["traffic_full_brake"])

    def test_green_eight_fresh_packets_releases(self):
        self.m.update_traffic_input({"signal": "STOP"}, 100.1)
        self.step(398)
        for i in range(8):
            t = 100.3 + i * 0.05
            self.m.update_traffic_input({"signal": "GO"}, t)
            r = self.step(398, t + 0.01)
        self.assertFalse(r["traffic_full_brake"])
        self.assertFalse(r["red_latched"])

    def test_ignored_fourth_not_overridden(self):
        self.m.update_traffic_input({"signal": "STOP"}, 100.1)
        r = self.step(1104)
        self.assertNotEqual(r["active_stop_index"], 1104)
        self.assertFalse(r["traffic_full_brake"])


if __name__ == "__main__":
    unittest.main()
