#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Offline-only geometry checks for beomsu V20 branch (no MORAI required)."""
import sys
import unittest
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from integration.beomsu_lane_guard import (
    FIRST_ZONE, make_tracking_path, make_zones, offset_at_waypoint,
    parse_extra_zone, zone_weight,
)


class BeomsuLaneGuardTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.path = np.loadtxt(
            str(ROOT / "path" / "2026_molit_comp_global_path.txt"),
            usecols=(0, 1),
        )

    def test_no_change_without_opt_in(self):
        zones = make_zones(len(self.path))
        actual = make_tracking_path(self.path, zones)
        self.assertTrue(np.array_equal(self.path, actual))
        self.assertFalse(np.shares_memory(self.path, actual))

    def test_first_zone_right_shift_only(self):
        zones = make_zones(len(self.path), 0.25)
        actual = make_tracking_path(self.path, zones)
        for wp in (0, 879, 1480, 1715, 1770, 3827):
            self.assertTrue(np.array_equal(self.path[wp], actual[wp]))
        for wp in (1533, 1578, 1655):
            tangent = self.path[wp + 3] - self.path[wp - 3]
            tangent /= np.linalg.norm(tangent)
            right = np.array([tangent[1], -tangent[0]])
            delta = actual[wp] - self.path[wp]
            self.assertAlmostEqual(float(np.dot(delta, right)), 0.25, places=6)
        self.assertTrue(np.array_equal(self.path, np.loadtxt(
            str(ROOT / "path" / "2026_molit_comp_global_path.txt"),
            usecols=(0, 1))))

    def test_extra_zone_signed_and_smooth(self):
        zones = make_zones(len(self.path), 0.0, ("3650:3750:-0.2",))
        self.assertEqual(len(zones), 1)
        z = zones[0]
        self.assertEqual(offset_at_waypoint(3650, zones), 0.0)
        self.assertAlmostEqual(offset_at_waypoint(3690, zones), -0.2)
        self.assertEqual(offset_at_waypoint(3750, zones), 0.0)
        actual = make_tracking_path(self.path, zones)
        wp = 3700
        tangent = self.path[wp + 3] - self.path[wp - 3]
        tangent /= np.linalg.norm(tangent)
        right = np.array([tangent[1], -tangent[0]])
        self.assertAlmostEqual(float(np.dot(actual[wp] - self.path[wp], right)), -0.2)
        self.assertEqual(zone_weight(3650, z), 0.0)

    def test_reject_bad_zone(self):
        for spec in ("bad", "10:20:0.2", "3650:3750:1.0", "3650:3750:nan"):
            with self.assertRaises(ValueError):
                make_zones(len(self.path), 0, (spec,))
        with self.assertRaises(ValueError):
            make_zones(len(self.path), 0.25, ("1550:1680:0.2",))
        with self.assertRaises(ValueError):
            make_zones(len(self.path), 0.5 + 0.01)


if __name__ == "__main__":
    unittest.main()
