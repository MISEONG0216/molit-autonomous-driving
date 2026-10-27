#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Offline integration tests for UDP merge risk + main mission arbitration."""
import unittest
from src.integration.mission_manager import MissionManager
from src.integration.mission_io import parse_merge_message


class MergeIntegrationTests(unittest.TestCase):
    def setUp(self):
        self.m = MissionManager({}, {
            "result_timeout_sec": 2.0,
            "zones": [[200, 300]],
            "roundabout_zones": [[500, 600]],
            "zone_speed_kmh": 25.0,
            "risk_stop_threshold": 0.6,
            "accept_confirm_frames": 3,
        }, [414, 620])

    def step(self, idx, result, when, updated=None, enabled=True, control=True):
        return self.m.apply(
            nearest_index=idx, base_target_speed_kmh=57,
            base_steering=0.18, traffic_enabled=False,
            merge_enabled=enabled, merge_control_enabled=control,
            merge_result=result,
            merge_updated_at=when - 0.01 if updated is None else updated,
            now=when,
        )

    def test_parse_valid_and_unknown(self):
        self.assertTrue(parse_merge_message(
            b"MERGE|0.500|0.650|0.650|MERGE_REJECT")["valid"])
        self.assertFalse(parse_merge_message(
            b"MERGE|0|0|0|MERGE_UNKNOWN")["valid"])

    def test_outside_zone_does_not_stop(self):
        r = self.step(100, parse_merge_message(
            b"MERGE|0.9|0.9|0.9|MERGE_REJECT"), 100)
        self.assertFalse(r["merge_stop"])
        self.assertEqual(r["target_speed_kmh"], 57)
        self.assertEqual(r["steering"], 0.18)

    def test_reject_stops_in_highway_zone(self):
        r = self.step(250, parse_merge_message(
            b"MERGE|0.8|0.7|0.8|MERGE_REJECT"), 100)
        self.assertTrue(r["merge_zone"])
        self.assertTrue(r["merge_stop"])
        self.assertEqual(r["target_speed_kmh"], 0.0)

    def test_roundabout_zone_caps_speed(self):
        r = self.step(550, parse_merge_message(
            b"MERGE|0.1|0.1|0.1|MERGE_ACCEPT"), 100)
        self.assertTrue(r["merge_zone"])
        self.assertFalse(r["merge_stop"])
        self.assertEqual(r["target_speed_kmh"], 25)

    def test_unknown_never_releases_latch(self):
        reject = parse_merge_message(
            b"MERGE|0.8|0.8|0.8|MERGE_REJECT")
        self.step(250, reject, 100.0)
        unknown = parse_merge_message(b"MERGE|0|0|0|MERGE_UNKNOWN")
        r = self.step(250, unknown, 100.1)
        self.assertTrue(r["merge_stop"])

    def test_three_distinct_accept_updates_release(self):
        reject = parse_merge_message(
            b"MERGE|0.8|0.8|0.8|MERGE_REJECT")
        accept = parse_merge_message(
            b"MERGE|0.1|0.1|0.1|MERGE_ACCEPT")
        self.step(250, reject, 100.0)
        a = self.step(250, accept, 100.10)
        self.assertTrue(a["merge_stop"])
        for i in range(10):
            a = self.step(250, accept, 100.11, updated=100.09)
        self.assertTrue(a["merge_stop"])
        self.assertEqual(a["merge_accept_count"], 1)
        a = self.step(250, accept, 100.20)
        self.assertTrue(a["merge_stop"])
        a = self.step(250, accept, 100.30)
        self.assertFalse(a["merge_stop"])
        self.assertEqual(a["target_speed_kmh"], 25)


if __name__ == "__main__":
    unittest.main()
