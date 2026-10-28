#!/usr/bin/env python3
"""Unit tests for read-only global-path diagnostics; MORAI is not required."""
import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from integration.lane_diagnostics import path_tracking_metrics


class LaneDiagnosticsTests(unittest.TestCase):
    def setUp(self):
        self.path = [(0.0, 0.0), (10.0, 0.0), (20.0, 0.0)]

    def test_centerline(self):
        m = path_tracking_metrics(self.path, 0, 5.0, 0.0, 0.0)
        self.assertAlmostEqual(m["signed_cte_m"], 0)
        self.assertAlmostEqual(m["heading_error_deg"], 0)

    def test_positive_left_negative_right(self):
        a = path_tracking_metrics(self.path, 1, 5.0, 1.5, 0)
        b = path_tracking_metrics(self.path, 1, 5.0, -1.0, 0)
        self.assertAlmostEqual(a["signed_cte_m"], 1.5)
        self.assertAlmostEqual(b["signed_cte_m"], -1.0)

    def test_projection_not_nearest_waypoint(self):
        m = path_tracking_metrics(self.path, 1, 4.8, 0.3, 0)
        self.assertAlmostEqual(m["abs_cte_m"], 0.3)
        self.assertAlmostEqual(m["signed_cte_m"], 0.3)

    def test_heading_wraparound(self):
        west = [(0.0, 0.0), (-10.0, 0.0)]
        m = path_tracking_metrics(west, 0, -5, 0, -math.pi + 0.1)
        self.assertAlmostEqual(m["heading_error_deg"], math.degrees(0.1), places=5)

    def test_end_of_path(self):
        m = path_tracking_metrics(self.path, 2, 19.5, -0.2, 0)
        self.assertAlmostEqual(m["signed_cte_m"], -0.2)


if __name__ == "__main__":
    unittest.main()
