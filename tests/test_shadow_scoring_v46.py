"""V46: expanded GPS-shadow candidates and ranked verified paths."""
import json
import sys
import unittest
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from lidar.shadow_ego_planner import (
    choose_local_detour, score_verified_candidate, _max_discrete_curvature,
)

CFG = json.loads((ROOT / "config/integration_missions.json").read_text(
    encoding="utf-8"))["gps_shadow_ego_planner"]


def sample_cloud(observe_left=True):
    xx = np.arange(3., 29., .25)
    pts = [np.column_stack((xx, np.full_like(xx, -2.0),
                            np.zeros_like(xx)))]
    if observe_left:
        pts.append(np.column_stack((xx, np.full_like(xx, 5.0),
                                    np.zeros_like(xx))))
    ox, oy = np.meshgrid(np.linspace(10., 12., 18),
                         np.linspace(-.5, .5, 9))
    pts.append(np.column_stack((ox.ravel(), oy.ravel(),
                                np.zeros(ox.size))))
    return np.vstack(pts)


class V46Tests(unittest.TestCase):
    def test_three_by_three_offsets_and_delays(self):
        self.assertEqual(len(CFG["candidate_offsets_m"]), 3)
        self.assertEqual(len(CFG["return_delay_options_m"]), 3)
        self.assertEqual(len(CFG["candidate_offsets_m"]) *
                         len(CFG["return_delay_options_m"]) * 2, 18)
        self.assertEqual(CFG["allowed_points"], 0)
        self.assertEqual(CFG["avoidance_speed_kmh"], 5)

    def test_score_rewards_clearance_and_penalizes_curvature(self):
        a = score_verified_candidate(1.4, 1.35, .12, 1.6, 0, CFG)
        b = score_verified_candidate(1.8, 1.35, .12, 1.6, 0, CFG)
        c = score_verified_candidate(1.8, 1.35, .52, 1.6, 0, CFG)
        self.assertGreater(b, a)
        self.assertGreater(b, c)

    def test_score_clearance_reward_saturates(self):
        a = score_verified_candidate(2.35, 1.35, .1, 2, 0, CFG)
        b = score_verified_candidate(20., 1.35, .1, 2, 0, CFG)
        self.assertAlmostEqual(a, b)

    def test_straight_path_zero_curvature(self):
        straight = np.column_stack((np.arange(0., 5., .5),
                                    np.zeros(10)))
        self.assertAlmostEqual(_max_discrete_curvature(straight), 0.)

    def test_verified_path_has_score_and_is_not_diagnostic(self):
        result = choose_local_detour(sample_cloud(), CFG)
        self.assertEqual(result["status"], "AVOIDING", result["reason"])
        self.assertIn("selected_score", result)
        self.assertIn("selected_min_distance_m", result)
        self.assertIn("selected_max_curvature_1pm", result)
        self.assertLessEqual(result["selected_offset_m"], 2.8)
        selected = [x for x in result["diagnostics"]
                    if (x.get("side") == ("LEFT" if result["side"] == 1 else "RIGHT")
                        and x.get("offset_m") == result["selected_offset_m"]
                        and x.get("return_delay_m") == result["selected_return_delay_m"])]
        self.assertEqual(len(selected), 1)
        self.assertEqual(selected[0]["hit_points"], 0)
        self.assertTrue(selected[0]["visibility_ok"])
        self.assertFalse(selected[0]["diagnostic_only"])

    def test_unobserved_return_still_rejected(self):
        result = choose_local_detour(sample_cloud(observe_left=False), CFG)
        self.assertEqual(result["status"], "STOP_REQUIRED")
        self.assertEqual(result["path"], [])


if __name__ == "__main__":
    unittest.main()
