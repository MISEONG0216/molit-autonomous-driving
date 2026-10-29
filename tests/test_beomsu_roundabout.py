#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Roundabout opt-in geometry and speed tests. No MORAI control packets."""
import sys
import unittest
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from integration.beomsu_lane_guard import make_zones, make_tracking_path
from integration.roundabout_exit_offset import make_roundabout_exit_path, offset_at_waypoint
from integration.roundabout_speed_cap import build_roundabout_speed_caps

class BeomsuRoundaboutTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.path = np.loadtxt(
            str(ROOT / "path" / "2026_molit_comp_global_path.txt"),
            usecols=(0, 1),
        )

    def test_early_guardrail_fix_preserved(self):
        reference = self.path.copy()
        first = make_tracking_path(reference, make_zones(len(reference), 0.25))
        roundabout = make_roundabout_exit_path(first, 0.20)
        self.assertTrue(np.array_equal(reference, self.path))
        for wp in (0, 1224, 1480, 1533, 1578, 1655, 1715, 1770, 1840, 1859, 1902, 3827):
            self.assertTrue(np.array_equal(first[wp], roundabout[wp]), wp)
        for wp in (1870, 1876, 1885):
            tangent = first[wp+3] - first[wp-3]
            tangent /= np.linalg.norm(tangent)
            right = np.array([tangent[1], -tangent[0]])
            self.assertAlmostEqual(
                float(np.dot(roundabout[wp] - first[wp], right)), 0.20,
                places=6,
            )
            self.assertAlmostEqual(offset_at_waypoint(wp, 0.20), 0.20, places=6)

    def test_zero_extra_offset_preserves_baseline(self):
        first = make_tracking_path(self.path, make_zones(len(self.path), 0.25))
        self.assertTrue(np.array_equal(first, make_roundabout_exit_path(first, 0.0)))

    def test_preview_speed_cap(self):
        unchanged, zones_off = build_roundabout_speed_caps(self.path, 30, 0)
        self.assertTrue(all(v == 30 for v in unchanged))
        self.assertTrue(all(z == "OFF" for z in zones_off))
        caps, zones = build_roundabout_speed_caps(self.path, 30, 12)
        for i in range(1835, 1893):
            self.assertAlmostEqual(caps[i], 12.0)
            self.assertEqual(zones[i], "HOLD")
        self.assertGreater(caps[1800], caps[1820])
        self.assertGreater(caps[1820], caps[1835])
        self.assertLess(caps[1892], caps[1910])
        self.assertLessEqual(max(caps), 30.0)
        self.assertEqual(zones[1700], "OFF")
        for bad in (-1, 5, 21):
            with self.assertRaises(ValueError):
                build_roundabout_speed_caps(self.path, 30, bad)

if __name__ == "__main__":
    unittest.main()
