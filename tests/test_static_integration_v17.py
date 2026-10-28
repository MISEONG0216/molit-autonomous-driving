#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""No-hardware regression tests for V17 LiDAR static integration."""
import ast
import json
import math
import unittest
from pathlib import Path

from src.integration.static_avoidance_io import (
    parse_static_message, local_path_pure_pursuit,
)

ROOT = Path(__file__).resolve().parents[1]


class StaticMessageTests(unittest.TestCase):
    def test_clear(self):
        result = parse_static_message(json.dumps({
            "status": "CLEAR", "path": [], "recommended_speed_kmh": 20,
            "scan": 1, "detected": 0,
        }).encode())
        self.assertTrue(result["valid"])
        self.assertEqual(result["status"], "CLEAR")
        self.assertEqual(result["path"], [])

    def test_valid_avoidance_path(self):
        payload = {
            "status": "AVOIDING", "path": [[0, 0], [1, 0.5], [2, 1]],
            "recommended_speed_kmh": 8, "obstacle_id": 7,
            "avoidance_side": "LEFT", "scan": 3, "detected": 1,
        }
        result = parse_static_message(json.dumps(payload).encode())
        self.assertTrue(result["valid"])
        self.assertEqual(result["status"], "AVOIDING")
        self.assertEqual(result["obstacle_id"], 7)
        self.assertEqual(len(result["path"]), 3)

    def test_bad_path_rejected(self):
        for path in ([], [[0, 0]], [[0, 0], [float("nan"), 1]],
                     [[0, 0], [float("inf"), 1]], [[0, 0], [1e10, 0]],
                     [[0, 0] for _ in range(401)]):
            with self.subTest(path_len=len(path)):
                raw = json.dumps({
                    "status": "AVOIDING",
                    "path": path, "recommended_speed_kmh": 8,
                }).encode()
                self.assertFalse(parse_static_message(raw)["valid"])

    def test_unknown_and_negative_speed_fail_closed(self):
        for status, speed in (("UNKNOWN", 0), ("AVOIDING", -1), ("bad", 4)):
            raw = json.dumps({
                "status": status, "path": [[0, 0], [1, 0]],
                "recommended_speed_kmh": speed,
            }).encode()
            self.assertFalse(parse_static_message(raw)["valid"])

    def test_stop_required_no_path(self):
        raw = json.dumps({"status": "STOP_REQUIRED", "recommended_speed_kmh": 0}).encode()
        self.assertTrue(parse_static_message(raw)["valid"])
        self.assertEqual(parse_static_message(raw)["status"], "STOP_REQUIRED")


class LocalSteeringTests(unittest.TestCase):
    def test_straight_ahead_near_zero_steer(self):
        path = [[i * 0.5, 0] for i in range(20)]
        result = local_path_pure_pursuit(path, 0.0, 0.0, 0.0, 1.0)
        self.assertIsNotNone(result)
        self.assertAlmostEqual(result["steering"], 0.0, places=6)
        self.assertGreater(result["local_x"], 2.0)

    def test_left_side_is_left_steering(self):
        path = [[0.5*i, 0.2*i] for i in range(15)]
        result = local_path_pure_pursuit(path, 0, 0, 0, 1)
        self.assertIsNotNone(result)
        self.assertGreater(result["steering"], 0.0)

    def test_no_forward_point(self):
        result = local_path_pure_pursuit([[-4, 0], [-3, 0], [-2, 0]], 0, 0, 0, 1)
        self.assertIsNone(result)


class BranchSanityTests(unittest.TestCase):
    def test_syntax_all_touched_python(self):
        files = [
            "src/main_integrated_v1.py",
            "src/integration/static_avoidance_io.py",
            "src/lidar/static_avoidance_module.py",
            "src/lidar/static_avoidance_path_latch.py",
            "src/lidar/static_obstacle_planner.py",
            "src/lidar/obstacle_adapter.py",
            "src/lidar/object_detection/object_detector.py",
            "src/lidar/object_detection/obstacle.py",
            "src/lidar/object_detection/tracker.py",
        ]
        for path in files:
            with self.subTest(path=path):
                ast.parse((ROOT / path).read_text(encoding="utf-8-sig"), filename=path)

    def test_network_channels_distinct(self):
        network = json.loads((ROOT / "config/integration_network.json").read_text())
        self.assertEqual(network["lidar"]["port"], 1116)
        self.assertNotEqual(network["static_pose"]["port"], network["static_result"]["port"])
        self.assertNotEqual(network["static_result"]["port"], network["control"]["port"])
        self.assertEqual(network["static_pose"]["bind_ip"], "127.0.0.1")
        self.assertEqual(network["static_result"]["bind_ip"], "127.0.0.1")


if __name__ == "__main__":
    unittest.main()
