#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Non-driving geometry regressions for the opt-in yellow-line experiment.

Checks that the original official path, cone mission and unaffected route
remain unchanged. Does NOT certify camera-lane/tire/wall clearance in MORAI.
"""
import unittest
from pathlib import Path
import sys
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from integration.beomsu_lane_guard import (
    make_zones, make_tracking_path, offset_at_waypoint, TEST_SPEED_LIMIT_KMH,
)
from integration.roundabout_exit_offset import make_roundabout_exit_path

EXTRA = ("3640:3820:0.12", "4050:4300:0.25")


class YellowLineTestProfile(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.original = np.loadtxt(
            str(ROOT / "path" / "2026_molit_comp_global_path.txt"),
            usecols=(0, 1),
        )
        cls.zones = make_zones(len(cls.original), 0.25, EXTRA)
        cls.tracking = make_roundabout_exit_path(
            make_tracking_path(cls.original, cls.zones), 0.25
        )

    def test_three_different_nonoverlapping_zone_ranges(self):
        self.assertEqual(len(self.zones), 3)
        self.assertEqual([(z[0], z[3]) for z in self.zones],
                         [(1480, 1715), (3640, 3820), (4050, 4300)])
        for a, b in zip(self.zones, self.zones[1:]):
            self.assertLess(a[3], b[0])
        self.assertEqual(TEST_SPEED_LIMIT_KMH, 20.)

    def test_original_global_route_does_not_mutate(self):
        official = self.original.copy()
        _ = make_tracking_path(official, self.zones)
        self.assertTrue(np.array_equal(official, self.original))

    def test_cone_mission_is_exactly_unchanged(self):
        self.assertTrue(np.array_equal(self.tracking[590:761],
                                      self.original[590:761]))

    def test_unrelated_route_stays_identical(self):
        for wp in (0, 414, 620, 879, 1104, 1224, 1770, 1820,
                   1930, 2100, 2570, 3200, 4400):
            if wp < len(self.original):
                self.assertTrue(np.array_equal(self.tracking[wp],
                                               self.original[wp]), wp)

    def test_shifts_have_correct_sign_and_limits(self):
        for wp, value in ((1550, 0.25), (3700, 0.12), (4130, 0.25)):
            self.assertAlmostEqual(
                offset_at_waypoint(wp, self.zones), value, places=5
            )
            tangent = self.original[wp + 3] - self.original[wp - 3]
            tangent /= np.linalg.norm(tangent)
            right = np.array([tangent[1], -tangent[0]])
            signed = float(np.dot(
                self.tracking[wp] - self.original[wp], right))
            self.assertAlmostEqual(signed, value, places=5)

    def test_changes_are_restricted_to_expected_wp_ranges(self):
        for wp in (1350, 1715, 1800, 1902, 3640, 3820, 4050, 4300):
            self.assertTrue(np.array_equal(self.tracking[wp],
                                           self.original[wp]), wp)


if __name__ == "__main__":
    unittest.main()
