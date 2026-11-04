"""V48 first-left / second-right fixed-shape shadow template tests."""
import ast
import json
import sys
import unittest
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src/lidar"))
from shadow_predefined_templates import choose_template_detour

CFG = json.loads((ROOT / "config/tunnel_predefined_templates.json").read_text(
    encoding="utf-8"))


def scan(obstacle_left=0.0, left_wall=7.0, right_wall=-6.0,
         obstacle_forward=10.0, wall_end=25.0):
    x = np.arange(2.0, wall_end, 0.25)
    wall = np.vstack((
        np.column_stack((x, np.full_like(x, left_wall), np.zeros_like(x))),
        np.column_stack((x, np.full_like(x, right_wall), np.zeros_like(x))),
    ))
    xs, ys = np.meshgrid(
        np.arange(obstacle_forward, obstacle_forward + 2.01, 0.25),
        np.linspace(obstacle_left - .35, obstacle_left + .35, 5),
    )
    obstacle = np.column_stack((xs.ravel(), ys.ravel(),
                                np.zeros(xs.size)))
    return np.vstack((wall, obstacle))


class V48Templates(unittest.TestCase):
    def test_first_template_is_left(self):
        result = choose_template_detour(scan(obstacle_left=-0.5), CFG, 0)
        self.assertEqual(result["status"], "AVOIDING", result["reason"])
        self.assertEqual(result["template_id"], "FIRST_LEFT")
        self.assertEqual(result["side"], 1)
        self.assertGreater(np.max(np.asarray(result["path"])[:, 1]), 0)

    def test_second_template_only_after_first_completed(self):
        result = choose_template_detour(scan(obstacle_left=2.0), CFG, 1)
        self.assertEqual(result["status"], "AVOIDING", result["reason"])
        self.assertEqual(result["template_id"], "SECOND_RIGHT")
        self.assertEqual(result["side"], -1)
        self.assertLess(np.min(np.asarray(result["path"])[:, 1]), 0)

    def test_no_obstacle_keeps_camera_lane_tracking(self):
        point_cloud = scan()[:184]
        result = choose_template_detour(point_cloud, CFG, 0)
        self.assertEqual(result["status"], "CLEAR", result["reason"])
        self.assertEqual(result["path"], [])

    def test_both_sides_close_is_rejected(self):
        result = choose_template_detour(
            scan(left_wall=1.8, right_wall=-1.8), CFG, 0)
        self.assertEqual(result["status"], "STOP_REQUIRED")
        self.assertEqual(result["path"], [])

    def test_missing_return_visibility_rejected(self):
        result = choose_template_detour(
            scan(wall_end=14.0), CFG, 0)
        self.assertEqual(result["status"], "STOP_REQUIRED")
        self.assertEqual(result["path"], [])

    def test_too_close_obstacle_fails_stop(self):
        result = choose_template_detour(
            scan(obstacle_forward=4.0), CFG, 0)
        self.assertEqual(result["status"], "STOP_REQUIRED")
        self.assertEqual(result["reason"], "OBSTACLE_TOO_CLOSE_TO_TEMPLATE")

    def test_after_second_template_ends(self):
        result = choose_template_detour(scan(), CFG, 2)
        self.assertEqual(result["status"], "CLEAR")

    def test_worker_leaves_camera_and_main_unchanged(self):
        worker = (ROOT / "src/lidar/static_avoidance_module.py").read_text(
            encoding="utf-8")
        ast.parse(worker)
        self.assertIn("choose_template_detour(", worker)
        self.assertIn("template_completed_count += 1", worker)
        self.assertIn("check_wall_clearance(", worker)
        self.assertIn("close_forward_collision(", worker)
        self.assertEqual(CFG["speed_kmh"], 5.0)


if __name__ == "__main__":
    unittest.main()
