"""V41 regression checks for shadow path-release state and emergency stop.

Synthetic logic check; a real MORAI scan is needed to establish road safety.
"""
import ast
import sys
import unittest
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "lidar"))
from shadow_wall_guard import close_forward_collision

class ShadowReleaseRegression(unittest.TestCase):
    def test_release_is_not_sticky_across_scans(self):
        # Regression for the original V40 bug: release status persisted and
        # suppressed planning on every subsequent scan indefinitely.
        source = (ROOT / "src/lidar/static_avoidance_module.py").read_text(encoding="utf-8")
        tree = ast.parse(source)
        shadow_branches = [
            node for node in ast.walk(tree) if isinstance(node, ast.If)
            and isinstance(node.test, ast.Name) and node.test.id == "gps_shadow"
            and any(isinstance(stmt, ast.Assign) and any(
                isinstance(target, ast.Name) and target.id == "latch_state"
                for target in stmt.targets
            ) and isinstance(stmt.value, ast.Constant)
                and stmt.value.value == "SHADOW_SCAN_PENDING"
                for stmt in node.body[:3])
        ]
        self.assertTrue(shadow_branches, "shadow latch state must reset each scan")
        self.assertIn('latch_state != "EGO_LIDAR_TAIL_RELEASED"', source)
        self.assertIn('close_forward_collision(vehicle_points, wall_cfg)', source)

    def test_emergency_guard_stops_close_forward_obstacle(self):
        cfg = {
            "vehicle_half_width_m": 0.9, "safety_margin_m": 0.45,
            "emergency_min_forward_m": 1.5, "emergency_stop_ahead_m": 6.0,
            "emergency_min_points": 5, "z_min_m": -1.45, "z_max_m": 1.25,
        }
        pts = np.column_stack((np.full(8, 3.0), np.zeros(8), np.zeros(8)))
        self.assertTrue(close_forward_collision(pts, cfg))
        clear = np.column_stack((np.full(8, 3.0), np.full(8, 4.0), np.zeros(8)))
        self.assertFalse(close_forward_collision(clear, cfg))

if __name__ == "__main__":
    unittest.main()
