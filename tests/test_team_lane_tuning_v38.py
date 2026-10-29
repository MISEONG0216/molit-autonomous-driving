#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Offline regression for the opt-in V38 team path. Does not control MORAI."""
import math
import sys
import unittest
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from integration.team_lane_tuning import (
    EXPECTED_WAYPOINTS, build_team_lane_plan,
)


class TeamLaneTuningV38Tests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.original = np.loadtxt(
            str(ROOT / "path" / "2026_molit_comp_global_path.txt"),
            usecols=(0, 1),
        )
        cls.baseline = cls.original.copy()
        cls.tracking, cls.caps, cls.shifts = build_team_lane_plan(
            cls.original, 30.0,
        )

    def right_projection(self, wp):
        tangent = self.baseline[wp + 3] - self.baseline[wp - 3]
        tangent = tangent / np.linalg.norm(tangent)
        right = np.array([tangent[1], -tangent[0]])
        return float(np.dot(self.tracking[wp] - self.baseline[wp], right))

    def test_official_waypoints_untouched(self):
        self.assertEqual(len(self.original), EXPECTED_WAYPOINTS)
        self.assertTrue(np.array_equal(self.original, self.baseline))
        self.assertFalse(np.shares_memory(self.tracking, self.original))

    def test_known_first_corner_left(self):
        self.assertLess(self.right_projection(160), -0.25)

    def test_second_corner_uses_delayed_right_with_apex(self):
        self.assertGreater(self.right_projection(265), 0.45)
        self.assertLess(self.shifts[265], 0.501)

    def test_guardrail_roundabout_and_tunnel_bias_are_right(self):
        for wp, minimum in ((1578, 0.20), (1877, 0.20),
                            (3750, 0.10), (4150, 0.20)):
            with self.subTest(wp=wp):
                self.assertGreater(self.right_projection(wp), minimum)

    def test_no_changes_outside_selected_zones(self):
        for wp in (0, 120, 125, 215, 220, 231, 300, 301,
                   1480, 1715, 1750, 1859, 1902, 2000,
                   3640, 3820, 4000, 4050, 4300, 4301,
                   EXPECTED_WAYPOINTS - 1):
            with self.subTest(wp=wp):
                self.assertTrue(np.array_equal(
                    self.tracking[wp], self.baseline[wp],
                ))

    def test_all_shifts_bounded_and_finite(self):
        self.assertTrue(np.all(np.isfinite(self.tracking)))
        self.assertTrue(np.all(np.isfinite(self.shifts)))
        self.assertLessEqual(float(np.max(self.shifts)), 0.501)

    def test_speed_caps_only_reduce_targets(self):
        self.assertEqual(len(self.caps), EXPECTED_WAYPOINTS)
        self.assertTrue(np.all((0.0 < self.caps) & (self.caps <= 30.0)))
        self.assertLessEqual(float(self.caps[1578]), 20.0)
        self.assertLessEqual(float(self.caps[160]), 20.0)
        self.assertAlmostEqual(float(self.caps[1800]), 8.0)
        self.assertAlmostEqual(float(self.caps[1877]), 8.0)
        self.assertAlmostEqual(float(self.caps[3500]), 30.0)

    def test_high_speed_and_different_path_rejected(self):
        for bad in (0.0, 30.01, 57.0, float("nan"), float("inf")):
            with self.subTest(kmh=bad), self.assertRaises(ValueError):
                build_team_lane_plan(self.original, bad)
        with self.assertRaises(ValueError):
            build_team_lane_plan(self.original[:-1], 10.0)

    def test_five_kmh_stage_never_accelerates_above_request(self):
        _path, caps, _shift = build_team_lane_plan(self.original, 5.0)
        self.assertTrue(np.all(caps <= 5.0 + 1e-10))

    def test_existing_safety_arbitration_is_still_present(self):
        source = (ROOT / "src" / "main_integrated_v1.py").read_text(
            encoding="utf-8",
        )
        self.assertIn("tracking_path = global_path", source)
        self.assertIn("if args.team_lane_tuning:", source)
        self.assertIn("tracking_path, nearest_index, current_x, current_y, yaw,", source)
        self.assertIn("global_path,\n                current_x,\n                current_y,", source)
        self.assertIn("traffic_enabled=bool(args.traffic and not shadow_active)", source)
        self.assertIn("mission[\"traffic_full_brake\"]", source)
        self.assertIn("or (args.static_control and static_stop)", source)
        self.assertIn("or pedestrian_stop or shadow_stop", source)
        self.assertIn('static_plan["path"]', source)


if __name__ == "__main__":
    unittest.main()
