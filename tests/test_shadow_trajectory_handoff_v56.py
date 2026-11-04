"""V56 predicted swept-path filtering does not bypass an actual collision."""
import json
import sys
import unittest
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src/lidar"))
from post_detour_clearance import evaluate_handoff_clearance

CFG = json.loads((ROOT / "config/integration_missions.json").read_text(
    encoding="utf-8"))["gps_shadow_ego_planner"]


def observed(*returns):
    xx = np.linspace(4., 11.9, 90)
    left = np.column_stack((xx, np.full_like(xx, 5.), np.zeros_like(xx)))
    right = np.column_stack((xx, np.full_like(xx, -5.), np.zeros_like(xx)))
    objects = np.array([(x, y, 0.) for x, y in returns], dtype=float)
    return np.vstack((left, right, objects)) if len(objects) else np.vstack((left,right))


class V56TrajectoryTests(unittest.TestCase):
    def test_straight_many_centre_returns_remain_stop(self):
        cloud = observed(*[(10.5, 0.)]*5)
        d = evaluate_handoff_clearance(cloud, CFG, lane_steering=0.)
        self.assertFalse(d["verified"], d)
        self.assertEqual(d["reason"], "FAR_HITS_STRAIGHT_PATH")

    def test_no_lane_returns_keep_original_stop(self):
        cloud = observed(*[(10.5, 0.)]*5)
        d = evaluate_handoff_clearance(cloud, CFG)
        self.assertFalse(d["verified"])
        self.assertEqual(d["reason"], "FAR_CENTER_RETURNS")

    def test_returns_outside_valid_curved_path_can_pass(self):
        # 10.8m ahead on original straight axis; camera curvature 0.09rad
        # predicts a left arc passing well clear of those points.
        cloud = observed(*[(10.8, 0.)]*5)
        d = evaluate_handoff_clearance(cloud, CFG, lane_steering=0.09)
        self.assertTrue(d["verified"], d)
        self.assertEqual(d["reason"], "FAR_HITS_OUTSIDE_PREDICTED_PATH")
        self.assertGreater(d["trajectory_margin_m"], 0.15)

    def test_return_on_predicted_arc_still_stops(self):
        # Predicted arc with steer 0.09rad is about y=1.7m at x=10m.
        cloud = observed(*[(10.5, 0.)]*5, (10., 1.7))
        d = evaluate_handoff_clearance(cloud, CFG, lane_steering=0.09)
        self.assertFalse(d["verified"])
        self.assertEqual(d["reason"], "PREDICTED_PATH_OBSTRUCTED")

    def test_near_obstacle_still_stops_with_curving_lane(self):
        cloud = observed(*[(10.8, 0.)]*5, (2.2, 0.))
        d = evaluate_handoff_clearance(cloud, CFG, lane_steering=0.09)
        self.assertFalse(d["verified"])
        self.assertEqual(d["reason"], "NEAR_CENTER_RETURN")

    def test_untrusted_steering_stops(self):
        cloud = observed(*[(10.8, 0.)]*5)
        d = evaluate_handoff_clearance(cloud, CFG, lane_steering=0.5)
        self.assertFalse(d["verified"])
        self.assertEqual(d["reason"], "INVALID_LANE_TRAJECTORY")

    def test_candidate_generation_and_safety_unmodified(self):
        self.assertEqual(CFG["obstacle_max_forward_m"], 17)
        self.assertEqual(CFG["obstacle_point_threshold"], 3)
        self.assertEqual(CFG["allowed_points"], 0)
        self.assertEqual(len(CFG["candidate_offsets_m"]) *
                         len(CFG["return_delay_options_m"])*2, 18)


if __name__ == "__main__":
    unittest.main()
