"""V40 GPS-shadow adaptive bidirectional LiDAR detour tests (no MORAI)."""
import ast
import json
import sys
import unittest
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from lidar.shadow_ego_planner import choose_local_detour

CFG = json.loads((ROOT / "config/integration_missions.json").read_text(
    encoding="utf-8"))["gps_shadow_ego_planner"]


def scan(left=5.0, right=-2.0, obstacle_length=2.0,
         obstacle_width=1.0, end=27.0):
    x = np.arange(3.0, end, 0.25)
    wall_left = np.column_stack((x, np.full_like(x, left), np.zeros_like(x)))
    wall_right = np.column_stack((x, np.full_like(x, right), np.zeros_like(x)))
    obstacle_x = np.linspace(10.0, 10.0 + obstacle_length, 18)
    obstacle_y = np.linspace(-obstacle_width / 2.0,
                             obstacle_width / 2.0, 9)
    xx, yy = np.meshgrid(obstacle_x, obstacle_y)
    block = np.column_stack((xx.ravel(), yy.ravel(),
                             np.zeros(xx.size)))
    return np.vstack((wall_left, wall_right, block))


class ShadowV40Tests(unittest.TestCase):
    def test_config_contains_both_sides_and_multiple_offsets(self):
        self.assertGreater(len(CFG["candidate_offsets_m"]), 1)

    def test_right_wall_obstruction_chooses_left(self):
        result = choose_local_detour(scan(left=5.0, right=-2.0), CFG)
        self.assertEqual(result["status"], "AVOIDING", result["reason"])
        self.assertEqual(result["side"], 1)
        self.assertTrue(any(d["side"] == "RIGHT" for d in result["diagnostics"]))
        self.assertTrue(any(d["side"] == "LEFT" and d["reason"] == "SAFE"
                            for d in result["diagnostics"]))

    def test_left_wall_obstruction_chooses_right(self):
        result = choose_local_detour(scan(left=2.0, right=-5.0), CFG)
        self.assertEqual(result["status"], "AVOIDING", result["reason"])
        self.assertEqual(result["side"], -1)

    def test_both_sides_blocked_stops(self):
        result = choose_local_detour(scan(left=1.8, right=-1.8), CFG)
        self.assertEqual(result["status"], "STOP_REQUIRED")
        self.assertEqual(result["reason"], "NO_FREE_SIDE")

    def test_adaptive_return_after_longer_obstacle(self):
        result = choose_local_detour(
            scan(left=6.0, right=-2.0, obstacle_length=4.0), CFG)
        self.assertEqual(result["status"], "AVOIDING", result["reason"])
        chosen = [d for d in result["diagnostics"] if d["reason"] == "SAFE"]
        self.assertTrue(chosen)
        self.assertGreaterEqual(chosen[0]["return_start_m"], 16.5)
        self.assertGreater(chosen[0]["return_end_m"], 15.0)

    def test_unseen_return_stops(self):
        result = choose_local_detour(
            scan(left=6.0, right=-2.0, obstacle_length=4.0, end=14.0), CFG)
        self.assertEqual(result["status"], "STOP_REQUIRED")
        self.assertEqual(result["reason"], "RETURN_CORRIDOR_UNOBSERVED")

    def test_immediate_obstacle_stops(self):
        cloud = scan()
        close = np.column_stack((np.full(8, 2.0),
                                 np.zeros(8), np.zeros(8)))
        result = choose_local_detour(np.vstack((cloud, close)), CFG)
        self.assertEqual(result["reason"], "IMMEDIATE_OBSTACLE")

    def test_syntax_python38(self):
        for name in ("src/lidar/shadow_ego_planner.py",
                     "src/lidar/static_avoidance_module.py"):
            ast.parse((ROOT / name).read_text(encoding="utf-8"), filename=name)


if __name__ == "__main__":
    unittest.main()
