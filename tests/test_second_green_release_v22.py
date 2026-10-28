#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""V22: WP620 confirmed-green bypass of traffic approach speed caps."""
import ast
import json
import unittest
from pathlib import Path

from src.integration.mission_manager import MissionManager

ROOT = Path(__file__).resolve().parents[1]


class SecondGreenReleaseTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.config = json.loads(
            (ROOT / "config" / "integration_missions.json").read_text(encoding="utf-8")
        )

    def setUp(self):
        self.manager = MissionManager(
            traffic_cfg=self.config["traffic"],
            merge_cfg=self.config["merge"],
            stop_indices=[414, 620, 879, 1104, 1224],
        )
        self.manager.set_global_path([(0.5 * i, 0.0) for i in range(4430)])

    def signal(self, label, timestamp):
        self.manager.update_traffic_input(
            {"signal": label, "left_arrow": False}, timestamp
        )

    def approach(self, wp=530, now=100.1, v7_speed=57.0, **kwargs):
        return self.manager.apply(
            nearest_index=wp, base_target_speed_kmh=v7_speed,
            base_steering=0.07, traffic_enabled=True, now=now, **kwargs
        )

    def confirm_green(self, wp=530):
        # 530 is 45m before WP620, within 55m confirmation gate.
        for timestamp in (100.0, 100.1, 100.2):
            self.signal("GO", timestamp)
            outcome = self.approach(wp, now=timestamp + 0.01)
        return outcome

    def test_exact_new_config(self):
        override = self.config["traffic"]["stop_overrides"]["620"]
        self.assertEqual(override["approach_distance_m"], 65)
        self.assertEqual(override["approach_speed_kmh"], 35)
        self.assertEqual(override["near_distance_m"], 25)
        self.assertEqual(override["near_speed_kmh"], 20)
        self.assertEqual(override["green_release_confirm_packets"], 3)
        self.assertEqual(override["green_release_start_distance_m"], 55)

    def test_start_approach_65m_without_green(self):
        r = self.approach(490)
        self.assertEqual(r["target_speed_kmh"], 35.0)
        self.assertTrue(r["traffic_approach"])
        self.assertFalse(r["second_green_bypass"])

    def test_gate_blocks_green_release_until_within_55m(self):
        for t in (100.0, 100.1, 100.2, 100.3):
            self.signal("GO", t)
            r = self.approach(505, now=t + 0.01)  # 57.5m before WP620
        self.assertFalse(r["second_green_bypass"])
        self.assertEqual(r["second_green_count"], 0)
        self.assertEqual(r["target_speed_kmh"], 35.0)

    def test_only_distinct_green_packets_count(self):
        self.signal("GO", 100.0)
        one = self.approach(530, now=100.01)
        for moment in (100.02, 100.03, 100.04):
            repeat = self.approach(530, now=moment)
        self.assertEqual(one["second_green_count"], 1)
        self.assertEqual(repeat["second_green_count"], 1)
        self.assertEqual(repeat["target_speed_kmh"], 35.0)

    def test_confirmed_go_releases_only_traffic_cap(self):
        r = self.confirm_green()
        self.assertTrue(r["second_green_bypass"])
        self.assertEqual(r["second_green_count"], 3)
        self.assertFalse(r["traffic_approach"])
        self.assertEqual(r["traffic_state"], "APPROACH_GREEN_RELEASE_620")
        self.assertIn("traffic_green_release_620", r["reasons"])
        self.assertEqual(r["target_speed_kmh"], 57.0)
        self.assertAlmostEqual(r["steering"], 0.07)

    def test_confirmed_go_25m_near_cap_is_also_released(self):
        r = self.confirm_green(wp=575)  # WP620 ahead by 22.5m
        self.assertEqual(r["target_speed_kmh"], 57.0)
        self.assertTrue(r["second_green_bypass"])

    def test_v7_curve_speed_remains_lower(self):
        r = self.confirm_green()
        curved = self.approach(540, now=100.22, v7_speed=28.0)
        self.assertEqual(curved["target_speed_kmh"], 28.0)
        self.assertTrue(curved["second_green_bypass"])

    def test_red_immediately_cancels_bypass_and_brakes(self):
        self.confirm_green()
        self.signal("STOP", 100.3)
        r = self.approach(560, now=100.31)
        self.assertTrue(r["traffic_full_brake"])
        self.assertTrue(r["red_latched"])
        self.assertFalse(r["second_green_bypass"])
        self.assertEqual(r["second_green_count"], 0)
        self.assertEqual(r["target_speed_kmh"], 0)

    def test_red_latch_requires_eight_new_go_packets_to_release(self):
        self.confirm_green()
        self.signal("STOP", 100.3)
        self.approach(560, now=100.31)
        for i in range(1, 8):
            ts = 100.3 + i * 0.08
            self.signal("GO", ts)
            r = self.approach(565, now=ts + 0.01)
            self.assertTrue(r["red_latched"])
            self.assertEqual(r["target_speed_kmh"], 0)
            self.assertFalse(r["second_green_bypass"])
        self.signal("GO", 101.0)
        r = self.approach(566, now=101.01)
        self.assertFalse(r["red_latched"])
        self.assertFalse(r["traffic_full_brake"])

    def test_unknown_resets_bypass_and_restores_35kmh_cap(self):
        self.confirm_green()
        self.signal("UNKNOWN", 100.3)
        r = self.approach(530, now=100.31)
        self.assertEqual(r["target_speed_kmh"], 35.0)
        self.assertFalse(r["second_green_bypass"])
        self.assertEqual(r["second_green_count"], 0)

    def test_stale_green_resets_bypass_and_restores_20kmh_cap(self):
        self.confirm_green(wp=575)
        r = self.approach(575, now=102.0)
        self.assertEqual(r["target_speed_kmh"], 20.0)
        self.assertFalse(r["second_green_bypass"])
        self.assertEqual(r["second_green_count"], 0)

    def test_other_signals_cannot_bypass(self):
        for signal in ("GO_LEFT", "STOP", "UNKNOWN"):
            m = MissionManager(
                self.config["traffic"], self.config["merge"],
                [414, 620, 879, 1104, 1224],
            )
            m.set_global_path([(0.5*i, 0.0) for i in range(4430)])
            for i in range(3):
                t = 100.0 + 0.1 * i
                m.update_traffic_input({"signal": signal}, t)
                r = m.apply(
                    nearest_index=530, base_target_speed_kmh=57,
                    base_steering=0, traffic_enabled=True, now=t + 0.01,
                )
            self.assertFalse(r["second_green_bypass"])

    def test_bypass_not_reused_for_next_light(self):
        self.confirm_green(wp=530)
        self.signal("GO", 100.4)
        r = self.approach(700, now=100.41)  # target now WP879
        self.assertEqual(r["active_stop_index"], 879)
        self.assertFalse(r["second_green_bypass"])
        self.assertEqual(r["second_green_count"], 0)

    def test_regular_other_signal_unchanged(self):
        self.signal("GO", 100.0)
        r = self.approach(350, now=100.01)  # 32m before WP414
        self.assertEqual(r["active_stop_index"], 414)
        self.assertEqual(r["target_speed_kmh"], 30)
        self.assertFalse(r["second_green_bypass"])

    def test_static_25kmh_and_main_priority_not_modified(self):
        self.assertEqual(self.config["static"]["avoidance_speed_kmh"], 25)
        self.assertEqual(self.config["static"]["avoidance_zones_wp"], [[590, 760]])
        source = (ROOT / "src" / "main_integrated_v1.py").read_text(encoding="utf-8")
        self.assertIn("target_speed_kmh = min(target_speed_kmh, static_speed_limit)", source)
        self.assertIn('mission["traffic_full_brake"]', source)

    def test_syntax(self):
        for rel in (
            "src/integration/mission_manager.py", "src/main_integrated_v1.py"
        ):
            ast.parse((ROOT / rel).read_text(encoding="utf-8-sig"), filename=rel)


if __name__ == "__main__":
    unittest.main()
