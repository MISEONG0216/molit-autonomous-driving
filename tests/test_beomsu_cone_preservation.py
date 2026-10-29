#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Regression guard for the V20 cone/static mission when tuning V39.

This is an offline geometry/control-unit check, NOT proof of physical cone
clearance in MORAI. The supplied 2026-10-30 CSVs did not cover WP590-760.
"""
import json
import sys
import unittest
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT / "src" / "lidar"))

from integration.beomsu_lane_guard import make_zones, make_tracking_path
from integration.roundabout_exit_offset import make_roundabout_exit_path
from integration.static_zone import load_static_zones, in_static_zone
from integration.static_avoidance_io import configured_avoidance_speed_kmh
from static_obstacle_planner import StaticObstaclePlanner, Obstacle


class V20ConePreservationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.original = np.loadtxt(
            str(ROOT / "path" / "2026_molit_comp_global_path.txt"),
            usecols=(0, 1),
        )
        cls.missions = json.loads(
            (ROOT / "config" / "integration_missions.json").read_text(
                encoding="utf-8"
            )
        )

    def test_course_tuning_never_moves_cone_waypoints(self):
        official = self.original.copy()
        biased = make_tracking_path(
            official, make_zones(len(official), first_right_m=0.25)
        )
        biased = make_roundabout_exit_path(biased, 0.20)
        self.assertTrue(np.array_equal(official, self.original))
        self.assertTrue(np.array_equal(
            biased[590:761, :2], official[590:761, :2]
        ), "Cone WP590-760 must be untouched by guardrail/roundabout offsets")

    def test_static_mission_zone_speed_and_size_filters_preserved(self):
        cfg = self.missions["static"]
        zones = load_static_zones(cfg, len(self.original))
        self.assertEqual(zones, ((590, 760),))
        self.assertTrue(in_static_zone(635, zones))
        self.assertFalse(in_static_zone(862, zones))
        self.assertEqual(configured_avoidance_speed_kmh(cfg), 25.0)
        planner_cfg = json.loads(
            (ROOT / "src/lidar/static_obstacle_config.json").read_text(
                encoding="utf-8"
            )
        )
        self.assertEqual(planner_cfg["candidate_offsets_m"], [-2.2, -1.8, 1.8, 2.2])
        self.assertEqual(planner_cfg["min_static_obstacle_width_m"], 0.6)
        self.assertEqual(planner_cfg["min_static_obstacle_height_m"], 0.2)

    def test_original_normal_gps_planner_can_detour_box(self):
        x = np.arange(0.0, 150.5, 0.5)
        straight = np.column_stack((x, np.zeros_like(x), np.zeros_like(x)))
        planner = StaticObstaclePlanner(
            straight,
            str(ROOT / "src/lidar/static_obstacle_config.json"),
            avoidance_speed_kph=25.0,
        )
        cone_like_box = Obstacle(
            id=184, x=20.0, y=0.0, width=1.0, length=1.0,
            distance=10.0, velocity_x=0.0, velocity_y=0.0,
            is_dynamic=False, confidence=1.0, height=0.6,
        )
        result = planner.plan(10.0, 0.0, [cone_like_box])
        self.assertEqual(result.status, "AVOIDING")
        self.assertEqual(result.recommended_speed_kph, 25.0)
        self.assertIn(result.avoidance_side, ("LEFT", "RIGHT"))
        self.assertGreater(len(result.local_path), 2)


if __name__ == "__main__":
    unittest.main()
