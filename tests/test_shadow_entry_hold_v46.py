"""V46: stop, observe LiDAR, then resume existing V44 shadow detours."""
import ast
import json
import sys
import unittest
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src/lidar"))
from shadow_entry_hold import EntryObservationGate

CONFIG = json.loads((ROOT / "config/integration_missions.json").read_text(
    encoding="utf-8"))["gps_shadow_entry_hold"]


def cloud(left=4.0, right=-4.0, close=False):
    x = np.linspace(3.0, 16.0, 50)
    points = [
        np.column_stack((x, np.full(len(x), left), np.zeros(len(x)))),
        np.column_stack((x, np.full(len(x), right), np.zeros(len(x)))),
    ]
    if close:
        points.append(np.column_stack((
            np.full(10, 5.0), np.zeros(10), np.zeros(10))))
    return np.vstack(points)


class EntryHoldTests(unittest.TestCase):
    def test_stopped_and_observed_can_release(self):
        gate = EntryObservationGate(CONFIG)
        result = None
        for t in np.arange(0, 3.1, 0.1):
            result = gate.check(float(t), (0, 0, 0), cloud())
        self.assertTrue(result[0], result)
        self.assertEqual(result[1], "READY")

    def test_missing_right_boundary_stays_stopped(self):
        gate = EntryObservationGate(CONFIG)
        result = None
        x = np.linspace(3, 16, 50)
        left_only = np.column_stack(
            (x, np.full(len(x), 4.0), np.zeros(len(x))))
        for t in np.arange(0, 3.1, 0.1):
            result = gate.check(float(t), (0, 0, 0), left_only)
        self.assertFalse(result[0])
        self.assertEqual(result[1], "INSUFFICIENT_LIDAR")

    def test_forward_obstacle_blocks_release(self):
        gate = EntryObservationGate(CONFIG)
        result = None
        for t in np.arange(0, 3.1, 0.1):
            result = gate.check(float(t), (0, 0, 0), cloud(close=True))
        self.assertFalse(result[0])
        self.assertEqual(result[1], "CLOSE_OBSTACLE")

    def test_motion_does_not_pass_stationary_gate(self):
        gate = EntryObservationGate(CONFIG)
        result = None
        for i in range(31):
            result = gate.check(i * 0.1, (i * 0.25, 0, 0), cloud())
        self.assertFalse(result[0])
        self.assertEqual(result[1], "WAIT_STOP")

    def test_worker_preserves_v44_planner_after_gate(self):
        worker = (ROOT / "src/lidar/static_avoidance_module.py").read_text(
            encoding="utf-8")
        ast.parse(worker)
        self.assertIn("entry_gate.check(", worker)
        self.assertIn('latch_state = "ENTRY_OBSERVE_" + entry_reason', worker)
        self.assertIn("shadow_locked_path = None", worker)
        self.assertIn("choose_local_detour(vehicle_points, ego_cfg)", worker)
        self.assertEqual(CONFIG["minimum_hold_sec"], 2.0)


if __name__ == "__main__":
    unittest.main()
