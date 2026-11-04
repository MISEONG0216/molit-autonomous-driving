"""V43: shadow path continuous alignment regression (Python 3.8)."""
import sys
import unittest
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src/lidar"))
from shadow_wall_guard import check_wall_clearance, path_alignment_metrics

CFG = dict(
    max_path_pose_error_m=1.8, check_ahead_m=18,
    minimum_path_ahead_m=6, minimum_obstacle_points=40,
    side_observation_min_m=1.3, side_observation_max_m=8,
    min_side_observation_points=8, vehicle_half_width_m=0.9,
    safety_margin_m=0.45, z_min_m=-1.45, z_max_m=1.25,
    allow_sweep_hit_points=0,
)


def walls(left=5.0, right=-5.0):
    xx = np.linspace(3, 17, 50)
    return np.vstack([
        np.column_stack((xx, np.full_like(xx, left), np.zeros(len(xx)))),
        np.column_stack((xx, np.full_like(xx, right), np.zeros(len(xx)))),
    ])


class ContinuousAlignmentTests(unittest.TestCase):
    def test_between_two_waypoints_is_aligned(self):
        path = np.array([[0, 0, 0], [4, 0, 0], [8, 0, 0], [12, 0, 0]])
        metrics = path_alignment_metrics(path, (2, 0, 0))
        self.assertLess(metrics["segment_error_m"], 1e-6)
        self.assertGreater(metrics["vertex_error_m"], 1.8)
        data = {}
        good, why = check_wall_clearance(
            path, (2, 0, 0), walls(), CFG, alignment_diagnostics=data
        )
        self.assertTrue(good, why)
        self.assertAlmostEqual(data["segment_error_m"], 0.0, places=6)

    def test_true_deviation_over_same_limit_stops(self):
        path = np.array([[0, 0, 0], [4, 0, 0], [8, 0, 0], [12, 0, 0]])
        data = {}
        good, why = check_wall_clearance(
            path, (2, 2.1, 0), walls(), CFG, alignment_diagnostics=data
        )
        self.assertFalse(good)
        self.assertEqual(why, "PATH_NOT_ALIGNED")
        self.assertGreater(data["segment_error_m"], 1.8)
        self.assertEqual(data["alignment_limit_m"], 1.8)

    def test_wall_collision_still_rejected(self):
        path = np.array([[0, 0, 0], [4, 0, 0], [8, 0, 0], [12, 0, 0]])
        scan = np.vstack((
            walls(),
            np.column_stack((
                np.linspace(3.0, 5.0, 10),
                np.zeros(10),
                np.zeros(10),
            )),
        ))
        good, why = check_wall_clearance(
            path, (0, 0, 0), scan, CFG
        )
        self.assertFalse(good)
        self.assertEqual(why, "WALL_OR_OBSTACLE_IN_SWEPT_PATH")

    def test_worker_keeps_previous_segment_and_logs(self):
        worker = (ROOT / "src/lidar/static_avoidance_module.py").read_text(
            encoding="utf-8")
        self.assertIn("shadow_locked_path[max(0, nearest - 1):]", worker)
        self.assertIn("[SHADOW ALIGN]", worker)
        self.assertIn("alignment_diagnostics=alignment", worker)


if __name__ == "__main__":
    unittest.main()
