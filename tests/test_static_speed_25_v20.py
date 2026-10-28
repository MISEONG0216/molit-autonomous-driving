#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""V20 regression tests for 25-km/h static avoidance speed propagation."""
import ast
import json
import math
import sys
import unittest
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "lidar"))

from src.integration.static_avoidance_io import configured_avoidance_speed_kmh
from src.integration.static_zone import load_static_zones, in_static_zone
from static_obstacle_planner import StaticObstaclePlanner, Obstacle, PlanResult
from static_avoidance_path_latch import AvoidancePathLatch


def global_path():
    x = np.arange(0.0, 150.5, 0.5)
    return np.column_stack((x, np.zeros_like(x), np.zeros_like(x)))


def box(obstacle_id, x, y):
    return Obstacle(
        id=obstacle_id, x=float(x), y=float(y),
        width=1.0, length=1.0, distance=10.0,
        velocity_x=0.0, velocity_y=0.0, is_dynamic=False,
        confidence=1.0, height=1.0,
    )


class StaticSpeedConfigurationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.missions = json.loads(
            (ROOT / "config" / "integration_missions.json").read_text(encoding="utf-8")
        )

    def test_mission_uses_25_kmh(self):
        self.assertEqual(
            configured_avoidance_speed_kmh(self.missions["static"]), 25.0
        )

    def test_legacy_5_10_limit_removed(self):
        main = (ROOT / "src" / "main_integrated_v1.py").read_text(encoding="utf-8")
        self.assertIn(
            "static_speed_limit = configured_avoidance_speed_kmh(static_cfg)", main
        )
        self.assertNotIn("10.0, max(0.1", main)
        self.assertNotIn('static_cfg.get("test_max_speed_kmh"', main)

    def test_invalid_speeds_rejected(self):
        for bad in (0, -1, 25.01, 57, None, "bad", math.inf, math.nan):
            with self.subTest(speed=bad):
                with self.assertRaises(ValueError):
                    configured_avoidance_speed_kmh({"avoidance_speed_kmh": bad})

    def test_existing_static_wp_zone_unchanged(self):
        zones = load_static_zones(self.missions["static"], 4430)
        self.assertEqual(zones, ((590, 760),))
        self.assertTrue(in_static_zone(635, zones))
        self.assertFalse(in_static_zone(862, zones))

    def test_traffic_configuration_preserved(self):
        traffic = self.missions["traffic"]
        self.assertEqual(traffic["left_turn_stops"], [1224])
        self.assertEqual(traffic["stop_overrides"]["620"]["approach_speed_kmh"], 35)


class AvoidanceSpeedPropagationTests(unittest.TestCase):
    def test_planner_first_route_is_25_kmh(self):
        planner = StaticObstaclePlanner(
            global_path(),
            str(ROOT / "src" / "lidar" / "static_obstacle_config.json"),
            avoidance_speed_kph=25.0,
        )
        result = planner.plan(ego_x=10.0, ego_y=0.0, obstacles=[box(184, 20, 0)])
        self.assertEqual(result.status, "AVOIDING")
        self.assertEqual(result.recommended_speed_kph, 25.0)

    def test_latched_route_remains_25_kmh_after_track_id_change(self):
        path = global_path()
        latch = AvoidancePathLatch(
            path, avoidance_speed_kmh=25.0
        )
        local = np.column_stack((
            np.arange(0.0, 61.0),
            np.full(61, 2.5),
            np.zeros(61),
        ))
        first = PlanResult("AVOIDING", local, 184, "LEFT", 25.0, "test")
        accepted, state = latch.apply(first, 0.0, 2.5, [box(184, 20, 0)])
        self.assertEqual(accepted.recommended_speed_kph, 25.0)
        self.assertEqual(state, "LOCKED_NEW_PATH")

        changed_id = PlanResult(
            "STOP_REQUIRED", np.empty((0, 3)), 195, None, 0.0, "test"
        )
        held, state = latch.apply(changed_id, 10.0, 2.5, [box(195, 20, 0)])
        self.assertEqual(held.status, "AVOIDING")
        self.assertEqual(held.recommended_speed_kph, 25.0)
        self.assertTrue(state.startswith("LOCKED_RAW_STOP_IGNORED"))

    def test_path_blocked_still_requests_stop(self):
        latch = AvoidancePathLatch(global_path(), avoidance_speed_kmh=25.0)
        local = np.column_stack((
            np.arange(0.0, 61.0),
            np.full(61, 2.5),
            np.zeros(61),
        ))
        first = PlanResult("AVOIDING", local, 184, "LEFT", 25.0, "test")
        latch.apply(first, 0, 2.5, [box(184, 20, 0)])
        stopped, state = latch.apply(first, 10, 2.5, [box(195, 30, 2.5)])
        self.assertEqual(stopped.status, "STOP_REQUIRED")
        self.assertEqual(stopped.recommended_speed_kph, 0.0)
        self.assertEqual(state, "STOP_REQUIRED_PATH_BLOCKED")

    def test_modified_files_parse(self):
        for name in (
            "src/main_integrated_v1.py",
            "src/lidar/static_obstacle_planner.py",
            "src/lidar/static_avoidance_path_latch.py",
            "src/lidar/static_avoidance_module.py",
            "src/integration/static_avoidance_io.py",
        ):
            with self.subTest(file=name):
                ast.parse((ROOT / name).read_text(encoding="utf-8-sig"), filename=name)


if __name__ == "__main__":
    unittest.main()
