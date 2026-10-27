#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""V12: fourth signal ignored; fifth light moves only on true left-green."""
import unittest

from src.integration.mission_manager import MissionManager


class FourthBypassFifthLeftTests(unittest.TestCase):
    def setUp(self):
        self.m = MissionManager({
            "signal_timeout_sec": 1.2,
            "go_release_confirm_loops": 8,
            "approach_distance_m": 60,
            "approach_speed_kmh": 30,
            "left_turn_stops": [1224],
            "left_only_stops": [1224],
            "ignore_signal_stops": [1104],
            "stop_overrides": {
                "1224": {
                    "approach_distance_m": 35,
                    "approach_speed_kmh": 20,
                    "near_distance_m": 15,
                    "near_speed_kmh": 8,
                    "signal_activation_index": 1150,
                },
            },
        }, {}, [414, 620, 879, 1104, 1224])
        self.m.set_global_path([(float(i) * 0.5, 0.0) for i in range(1500)])

    def step(self, idx, t=100.2):
        return self.m.apply(
            nearest_index=idx,
            base_target_speed_kmh=57,
            base_steering=0.12,
            traffic_enabled=True,
            now=t,
        )

    def test_fourth_red_is_ignored(self):
        self.m.update_traffic_input({"signal": "STOP"}, 100.1)
        r = self.step(1104)
        self.assertEqual(r["active_stop_index"], 1224)
        self.assertFalse(r["red_latched"])
        self.assertFalse(r["traffic_full_brake"])
        self.assertEqual(r["target_speed_kmh"], 57.0)

    def test_fifth_normal_green_triggers_stop(self):
        self.m.update_traffic_input({"signal": "GO"}, 100.1)
        r = self.step(1200)
        self.assertEqual(r["traffic_state"], "FULL_BRAKE_RED")
        self.assertEqual(r["target_speed_kmh"], 0.0)

    def test_fifth_redleft_is_stop_not_go_left(self):
        # Camera's redleft classifier produces STOP, NOT GO_LEFT.
        self.m.update_traffic_input({"signal": "STOP", "left_arrow": False}, 100.1)
        self.assertTrue(self.step(1200)["traffic_full_brake"])

    def test_fifth_red_and_greenleft_go_priority(self):
        self.m.update_traffic_input({"signal": "STOP", "left_arrow": True}, 100.1)
        r = self.step(1200)
        self.assertFalse(r["traffic_full_brake"])
        self.assertTrue(r["left_arrow_override"])
        self.assertEqual(r["target_speed_kmh"], 8.0)

    def test_fifth_greenleft_releases_after_eight_new_frames(self):
        self.m.update_traffic_input({"signal": "STOP", "left_arrow": False}, 100.1)
        self.assertTrue(self.step(1200, 100.11)["red_latched"])
        for i in range(8):
            t = 100.2 + i * 0.05
            self.m.update_traffic_input({"signal": "STOP", "left_arrow": True}, t)
            r = self.step(1200, t + 0.001)
        self.assertFalse(r["red_latched"])
        self.assertFalse(r["traffic_full_brake"])

    def test_past_green_only_does_not_release(self):
        self.m.update_traffic_input({"signal": "STOP", "left_arrow": False}, 100.1)
        self.assertTrue(self.step(1200, 100.11)["red_latched"])
        for i in range(9):
            t=100.2+i*0.05
            self.m.update_traffic_input({"signal": "GO", "left_arrow": False}, t)
            r=self.step(1200,t+0.001)
        self.assertTrue(r["red_latched"])
        self.assertTrue(r["traffic_full_brake"])


if __name__ == "__main__":
    unittest.main()
