"""V54: historical LiDAR points supplement visibility, not collision permissions."""
import json
import math
import sys
import unittest
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT / "src/lidar"))
from lidar.shadow_ego_planner import choose_local_detour
from temporal_shadow_observations import TemporalShadowObservations

CFG = json.loads((ROOT / "config/integration_missions.json").read_text(
    encoding="utf-8"))["gps_shadow_ego_planner"]


def scan(left=True):
    x = np.arange(3., 26., 0.25)
    segments = [np.column_stack((x, -2*np.ones_like(x), np.zeros_like(x)))]
    if left:
        segments.append(np.column_stack(
            (x, 5*np.ones_like(x), np.zeros_like(x))))
    a, b = np.meshgrid(np.linspace(10., 12., 12),
                       np.linspace(-.5, .5, 7))
    segments.append(np.column_stack(
        (a.ravel(), b.ravel(), np.zeros(a.size))))
    return np.vstack(segments)


class V54TemporalObservationTests(unittest.TestCase):
    def test_detection_config_and_safety_preserved(self):
        self.assertEqual(CFG["obstacle_max_forward_m"], 17)
        self.assertEqual(CFG["obstacle_point_threshold"], 3)
        self.assertEqual(CFG["allowed_points"], 0)
        self.assertEqual(CFG["candidate_offsets_m"], [1.8, 2.4, 2.8])

    def test_missing_left_visibility_stops_without_history(self):
        result = choose_local_detour(scan(left=False), CFG)
        self.assertEqual(result["status"], "STOP_REQUIRED")
        self.assertEqual(result["reason"], "RETURN_CORRIDOR_UNOBSERVED")

    def test_previous_observed_left_can_support_return(self):
        now = scan(left=False)
        prior = np.column_stack((
            np.arange(3., 26., .25),
            np.full(len(np.arange(3., 26., .25)), 5.),
            np.zeros(len(np.arange(3., 26., .25)))))
        result = choose_local_detour(now, CFG, visibility_history=prior)
        self.assertEqual(result["status"], "AVOIDING", result["reason"])
        self.assertEqual(result["side"], 1)

    def test_historical_obstacle_is_checked_for_collision(self):
        points = scan(left=False)
        prior = scan(left=True)
        a = np.repeat(np.array([[11., 2.8, 0.]]), 15, axis=0)
        history = np.vstack((prior, a))
        result = choose_local_detour(points, CFG, visibility_history=history)
        self.assertTrue(any(
            d.get("side") == "LEFT" and d.get("offset_m") == 2.8 and
            d.get("reason") == "SWEEP_COLLISION"
            for d in result["diagnostics"]
        ))

    def test_pose_compensation_and_expiry(self):
        buffer = TemporalShadowObservations(max_frames=5, max_age_sec=1.8)
        points = scan(left=True)
        buffer.update(points, (0., 0., 0.), 10.)
        buffer.update(points, (1., 0., 0.), 10.5)
        history = buffer.get_prior_in_vehicle((1., 0., 0.), 10.5)
        self.assertGreater(len(history), 0)
        self.assertAlmostEqual(float(history[:, 0].min()),
                               float(points[:, 0].min())-1., delta=.2)
        buffer.update(points, (2., 0., 0.), 13.)
        self.assertEqual(len(buffer.get_prior_in_vehicle(
            (2., 0., 0.), 13.)), 0)


if __name__ == "__main__":
    unittest.main()
