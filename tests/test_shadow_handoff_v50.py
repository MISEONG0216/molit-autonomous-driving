"""V50 post-detour sparse far-field LiDAR returns are not all stops."""
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


def cloud(near=0, far=0, left=25, right=25):
    points = []
    points += [(float(x), 5.0, 0.0) for x in np.linspace(4., 11., left)]
    points += [(float(x), -5.0, 0.0) for x in np.linspace(4., 11., right)]
    points += [(2.5, 0., 0.) for _ in range(near)]
    points += [(8.0, 0., 0.) for _ in range(far)]
    # Additional observed sector points so ROI has >=80 valid returns
    points += [(float(x), 6.0, 0.0) for x in np.linspace(4., 11., 60)]
    return np.array(points)


class V50Tests(unittest.TestCase):
    def test_one_far_center_point_allowed(self):
        d = evaluate_handoff_clearance(cloud(far=1), CFG)
        self.assertTrue(d["verified"], d)
        self.assertEqual(d["far_hits"], 1)

    def test_two_far_center_points_allowed(self):
        self.assertTrue(evaluate_handoff_clearance(cloud(far=2), CFG)["verified"])

    def test_three_far_center_points_block(self):
        d = evaluate_handoff_clearance(cloud(far=3), CFG)
        self.assertFalse(d["verified"])
        self.assertEqual(d["reason"], "FAR_CENTER_RETURNS")

    def test_one_near_center_point_blocks(self):
        d = evaluate_handoff_clearance(cloud(near=1), CFG)
        self.assertFalse(d["verified"])
        self.assertEqual(d["reason"], "NEAR_CENTER_RETURN")

    def test_one_side_missing_no_longer_stops_if_forward_clear(self):
        d = evaluate_handoff_clearance(cloud(right=0, far=1), CFG)
        self.assertTrue(d["verified"], d)
        self.assertEqual(d["reason"], "ONE_SIDE_OBSERVED_FORWARD_CLEAR")

    def test_both_sides_missing_blocks(self):
        # Valid wide LiDAR ROI, but neither lateral wall sector is observed.
        wide = np.array([(float(x), 8.5, 0.0)
                         for x in np.linspace(1., 12., 100)])
        d = evaluate_handoff_clearance(wide, CFG)
        self.assertFalse(d["verified"])
        self.assertEqual(d["reason"], "BOTH_SIDES_UNOBSERVED")

    def test_one_side_missing_does_not_bypass_near_obstacle(self):
        d = evaluate_handoff_clearance(cloud(right=0, near=1), CFG)
        self.assertFalse(d["verified"])
        self.assertEqual(d["reason"], "NEAR_CENTER_RETURN")

    def test_one_side_missing_does_not_bypass_far_obstacle(self):
        d = evaluate_handoff_clearance(cloud(right=0, far=3), CFG)
        self.assertFalse(d["verified"])
        self.assertEqual(d["reason"], "FAR_CENTER_RETURNS")

    def test_sparse_cloud_blocks(self):
        d = evaluate_handoff_clearance(np.array([[8., 0., 0.]]), CFG)
        self.assertFalse(d["verified"])
        self.assertEqual(d["reason"], "SPARSE_FORWARD_SCAN")

    def test_existing_detour_collision_rule_unchanged(self):
        self.assertEqual(CFG["allowed_points"], 0)
        self.assertEqual(CFG["candidate_offsets_m"], [1.8, 2.4, 2.8])
        self.assertEqual(CFG["post_detour_handoff_distance_m"], 8)


if __name__ == "__main__":
    unittest.main()
