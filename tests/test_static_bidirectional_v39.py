#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Bidirectional static planner regression; Python 3.8 compatible."""
import json
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "lidar"))
from static_obstacle_planner import StaticObstaclePlanner, Obstacle


def obstacle(i, x, y, width=1.0):
    return Obstacle(
        id=i, x=float(x), y=float(y), width=float(width), length=1.0,
        distance=float(x), velocity_x=0.0, velocity_y=0.0,
        is_dynamic=False, confidence=1.0, height=1.0,
    )


class BidirectionalStaticTests(unittest.TestCase):
    def setUp(self):
        x = np.arange(0.0, 150.5, 0.5)
        route = np.column_stack((x, np.zeros_like(x), np.zeros_like(x)))
        original = json.loads(
            (ROOT / "src/lidar/static_obstacle_config.json").read_text(encoding="utf-8")
        )
        self.temp = tempfile.TemporaryDirectory()
        self.config_path = Path(self.temp.name) / "planner.json"
        self.config = original
        self.route = route
        self.write_config()
        self.planner = StaticObstaclePlanner(route, str(self.config_path),
                                             avoidance_speed_kph=25.0)

    def tearDown(self):
        self.temp.cleanup()

    def write_config(self):
        self.config_path.write_text(json.dumps(self.config), encoding="utf-8")

    def test_left_and_right_configured(self):
        offsets = self.config["candidate_offsets_m"]
        self.assertTrue(any(v > 0 for v in offsets))
        self.assertTrue(any(v < 0 for v in offsets))

    def test_center_obstacle_evaluates_both(self):
        result = self.planner.plan(10.0, 0.0, [obstacle(1, 20, 0)])
        self.assertEqual(result.status, "AVOIDING")
        offsets = [d["offset_m"] for d in result.candidate_diagnostics]
        self.assertTrue(any(v > 0 for v in offsets))
        self.assertTrue(any(v < 0 for v in offsets))
        self.assertEqual(result.recommended_speed_kph, 25.0)

    def test_right_side_blocker_uses_left_if_clear(self):
        # Primary obstacle on centreline, second obstacle in right bypass.
        result = self.planner.plan(10.0, 0.0, [
            obstacle(1, 20, 0), obstacle(2, 20, -2.4),
        ])
        self.assertEqual(result.status, "AVOIDING")
        self.assertEqual(result.avoidance_side, "LEFT")

    def test_left_side_blocker_uses_right_if_clear(self):
        result = self.planner.plan(10.0, 0.0, [
            obstacle(1, 20, 0), obstacle(2, 20, 2.4),
        ])
        self.assertEqual(result.status, "AVOIDING")
        self.assertEqual(result.avoidance_side, "RIGHT")

    def test_both_sides_blocked_stops(self):
        result = self.planner.plan(10.0, 0.0, [
            obstacle(1, 20, 0), obstacle(2, 20, -2.4),
            obstacle(3, 20, 2.4),
        ])
        self.assertEqual(result.status, "STOP_REQUIRED")
        self.assertEqual(result.recommended_speed_kph, 0.0)

    def test_too_large_obstacle_stops(self):
        result = self.planner.plan(10.0, 0.0, [
            obstacle(1, 20, 0, width=5.0),
        ])
        self.assertEqual(result.status, "STOP_REQUIRED")


if __name__ == "__main__":
    unittest.main()
