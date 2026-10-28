#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""V18 regression tests: locked path ignores ID churn but not path collisions."""
import sys
import unittest
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "lidar"))

from static_avoidance_path_latch import AvoidancePathLatch
from static_obstacle_planner import Obstacle, PlanResult


def obstacle(ident, x, y, width=1.0, height=1.0, dynamic=False):
    return Obstacle(
        id=ident, x=float(x), y=float(y), width=float(width),
        length=1.0, distance=10.0, velocity_x=0.0, velocity_y=0.0,
        is_dynamic=dynamic, confidence=1.0, height=float(height),
    )


def plan(status, ident=None):
    path = np.column_stack((
        np.arange(0.0, 61.0, 1.0),
        np.full(61, 2.5),
        np.zeros(61),
    ))
    return PlanResult(
        status, path if status == "AVOIDING" else np.empty((0, 3)),
        ident, "LEFT" if status == "AVOIDING" else None,
        8.0 if status == "AVOIDING" else 0.0, "test",
    )


class LockedPathTests(unittest.TestCase):
    def setUp(self):
        global_path = np.column_stack((
            np.arange(0.0, 101.0),
            np.zeros(101),
            np.zeros(101),
        ))
        self.latch = AvoidancePathLatch(global_path)
        p, state = self.latch.apply(plan("AVOIDING", 184), 0.0, 2.5,
                                     [obstacle(184, 20, 0)])
        self.assertEqual(p.status, "AVOIDING")
        self.assertEqual(state, "LOCKED_NEW_PATH")
        self.assertTrue(self.latch.active)

    def test_tracking_id_change_outside_path_does_not_stop(self):
        raw = plan("STOP_REQUIRED", 195)
        kept, state = self.latch.apply(raw, 10, 2.5, [
            obstacle(195, 20, 0),
        ])
        self.assertEqual(kept.status, "AVOIDING")
        self.assertIn("LOCKED_RAW_STOP_IGNORED", state)
        self.assertEqual(kept.obstacle_id, 184)

    def test_unrelated_off_path_object_ignored(self):
        kept, state = self.latch.apply(plan("AVOIDING", 777), 10, 2.5, [
            obstacle(777, 30, -5),
        ])
        self.assertEqual(kept.status, "AVOIDING")
        self.assertIn("LOCKED", state)
        self.assertEqual(kept.obstacle_id, 184)

    def test_new_obstacle_on_path_must_stop(self):
        kept, state = self.latch.apply(plan("STOP_REQUIRED", 195), 10, 2.5, [
            obstacle(195, 30, 2.5),
        ])
        self.assertEqual(kept.status, "STOP_REQUIRED")
        self.assertEqual(state, "STOP_REQUIRED_PATH_BLOCKED")
        self.assertEqual(kept.obstacle_id, 195)
        self.assertTrue(self.latch.active)

    def test_obstacle_between_waypoints_stops(self):
        kept, state = self.latch.apply(plan("CLEAR"), 10, 2.5, [
            obstacle(195, 30.5, 2.5, width=0.6),
        ])
        self.assertEqual((kept.status, state),
                         ("STOP_REQUIRED", "STOP_REQUIRED_PATH_BLOCKED"))

    def test_moving_obstacle_blocking_path_stops(self):
        kept, state = self.latch.apply(plan("CLEAR"), 10, 2.5, [
            obstacle(195, 30, 2.5, dynamic=True),
        ])
        self.assertEqual(kept.status, "STOP_REQUIRED")
        self.assertEqual(state, "STOP_REQUIRED_PATH_BLOCKED")

    def test_resumes_same_path_after_blocker_clears(self):
        self.latch.apply(plan("STOP_REQUIRED", 195), 10, 2.5,
                         [obstacle(195, 30, 2.5)])
        kept, state = self.latch.apply(plan("CLEAR"), 10, 2.5,
                                       [obstacle(184, 20, 0)])
        self.assertEqual(kept.status, "AVOIDING")
        self.assertEqual(kept.obstacle_id, 184)
        self.assertTrue(state.startswith("LOCKED"))

    def test_clear_scan_count_never_releases_early(self):
        for i in range(50):
            kept, state = self.latch.apply(plan("CLEAR"), 10 + i * .1, 2.5,
                                           [obstacle(184, 20, 0)])
            self.assertEqual(kept.status, "AVOIDING")
            self.assertTrue(self.latch.active)

    def test_release_at_saved_geometric_return(self):
        kept, state = self.latch.apply(plan("CLEAR"), 60, 2.5, [])
        self.assertEqual(kept.status, "CLEAR")
        self.assertEqual(state, "RELEASED_BY_PATH")
        self.assertFalse(self.latch.active)

    def test_small_floor_noise_does_not_force_stop(self):
        kept, _ = self.latch.apply(plan("CLEAR"), 10, 2.5, [
            obstacle(202, 30, 2.5, width=0.2, height=0.03),
        ])
        self.assertEqual(kept.status, "AVOIDING")

    def test_missing_obstacle_geometry_fails_closed_on_new_stop(self):
        kept, state = self.latch.apply(plan("STOP_REQUIRED", 195),
                                       10, 2.5)
        self.assertEqual(kept.status, "STOP_REQUIRED")
        self.assertEqual(state, "STOP_REQUIRED_UNVERIFIED")

    def test_no_replan_during_lock(self):
        different = plan("AVOIDING", 777)
        different.local_path[:, 1] = -2.0
        kept, _ = self.latch.apply(different, 10, 2.5, [])
        self.assertEqual(kept.obstacle_id, 184)
        self.assertTrue(np.allclose(kept.local_path[:, 1], 2.5))


if __name__ == "__main__":
    unittest.main()
