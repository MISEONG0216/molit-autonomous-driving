"""V45 rightward lane shift verification and guard tests (no MORAI)."""
import ast
import json
import sys
import unittest
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src/lidar"))
from shadow_timed_right_shift import (
    right_shift_local_path, plan_verified_right_shift,
)

MISSIONS = json.loads((ROOT / "config/integration_missions.json").read_text(
    encoding="utf-8"))
CFG = MISSIONS["gps_shadow_right_shift"]
WALL = MISSIONS["gps_shadow_wall_guard"]


def scan(left=5.0, right=-5.0, near_obstacle=False):
    x = np.linspace(3.0, 19.0, 80)
    points = [
        np.column_stack((x, np.full(len(x), left), np.zeros(len(x)))),
        np.column_stack((x, np.full(len(x), right), np.zeros(len(x)))),
    ]
    if near_obstacle:
        points.append(np.column_stack((
            np.full(8, 3.0), np.zeros(8), np.zeros(8))))
    return np.vstack(points)


class RightShiftV45Test(unittest.TestCase):
    def test_right_shift_holds_right_lane_at_endpoint(self):
        path = right_shift_local_path(2.4, CFG)
        self.assertAlmostEqual(path[0, 1], 0, places=6)
        self.assertAlmostEqual(path[-1, 1], -2.4, places=6)
        self.assertLessEqual(float(np.max(np.abs(path[:, 1]))), 2.8)

    def test_open_corridor_is_verified(self):
        result = plan_verified_right_shift(
            scan(), (0.0, 0.0, 0.0), WALL, CFG)
        self.assertEqual(result["status"], "AVOIDING", result["reason"])
        self.assertEqual(result["offset_m"], 2.4)
        self.assertTrue(all(d["verified"] for d in result["diagnostics"]))

    def test_right_wall_is_not_ignored(self):
        result = plan_verified_right_shift(
            scan(right=-3.0), (0.0, 0.0, 0.0), WALL, CFG)
        self.assertEqual(result["status"], "STOP_REQUIRED")
        self.assertEqual(result["path"], [])

    def test_close_forward_obstacle_prevents_shift(self):
        result = plan_verified_right_shift(
            scan(near_obstacle=True), (0.0, 0.0, 0.0), WALL, CFG)
        self.assertEqual(result["status"], "STOP_REQUIRED")
        self.assertEqual(result["reason"], "RIGHT_SHIFT_FORWARD_BLOCKED")

    def test_timing_is_one_shot_after_left_tail_release(self):
        worker = (ROOT / "src/lidar/static_avoidance_module.py").read_text(
            encoding="utf-8")
        ast.parse(worker)
        self.assertIn("first_left_release_at = time.monotonic()", worker)
        self.assertIn('shadow_side == 1', worker)
        self.assertIn('time.monotonic() - first_left_release_at >= wait_s', worker)
        self.assertIn('local["status"] == "AVOIDING"', worker)
        self.assertIn('timed_right_shift_started = True', worker)
        self.assertIn('latch_state = "EGO_LIDAR_NEW_RIGHT_TIMED"', worker)
        self.assertEqual(CFG["delay_sec"], 5.0)


if __name__ == "__main__":
    unittest.main()
