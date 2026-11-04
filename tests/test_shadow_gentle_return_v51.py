"""V51: return steering gentler than original while approach is unchanged."""
import json
import sys
import unittest
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from integration.gentle_return import (
    gentle_return_pure_pursuit, limit_return_steering,
)
from integration.static_avoidance_io import local_path_pure_pursuit

CFG = json.loads((ROOT / "config/integration_missions.json").read_text(
    encoding="utf-8"))["gps_shadow_ego_planner"]


class V51GentleReturnTests(unittest.TestCase):
    def test_gentle_return_steering_lower_for_same_lane_change(self):
        x = np.arange(0., 24., 0.5)
        y = np.interp(x, [0., 14., 19., 23.5], [2.8, 2.8, 1.4, 0.])
        path = np.column_stack((x, y))
        normal = local_path_pure_pursuit(
            path, 15.0, 2.8, 0.0, 2.7)
        gentle = gentle_return_pure_pursuit(
            path, 15.0, 2.8, 0.0, 2.7, 3.8)
        self.assertIsNotNone(normal)
        self.assertIsNotNone(gentle)
        self.assertLess(abs(gentle["steering"]),
                        abs(normal["steering"]))

    def test_right_steer_slew_bound(self):
        output = limit_return_steering(-0.45, 0.0, 0.1, 0.22, 0.35)
        self.assertAlmostEqual(output, -0.035)

    def test_absolute_cap(self):
        self.assertAlmostEqual(
            limit_return_steering(-0.8, None, 0.1, 0.22, 0.35), -0.22)

    def test_nan_cannot_propagate(self):
        with self.assertRaises(ValueError):
            limit_return_steering(float("nan"), 0.0, 0.1)

    def test_worker_phase_is_not_global(self):
        worker = (ROOT / "src/lidar/static_avoidance_module.py").read_text(
            encoding="utf-8")
        main = (ROOT / "src/main_integrated_v1.py").read_text(
            encoding="utf-8")
        self.assertIn('shadow_return_start_index', worker)
        self.assertIn('"shadow_returning": bool(', worker)
        self.assertIn('if gentle_return_active:', main)
        self.assertIn('else:\n                            local_pp = local_path_pure_pursuit(', main)

    def test_current_detour_collision_gate_preserved(self):
        self.assertEqual(CFG["allowed_points"], 0)
        self.assertEqual(CFG["candidate_offsets_m"], [1.8, 2.4, 2.8])
        self.assertEqual(CFG["return_delay_options_m"], [0, 1.5, 3])


if __name__ == "__main__":
    unittest.main()
