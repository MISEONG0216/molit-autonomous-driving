"""V44 GPS-shadow variable return position, safety and regression tests."""
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


def cloud(left_wall=5.0, right_wall=-2.0, wall_end=28.0,
          distant_block=False):
    x = np.arange(3.0, wall_end, 0.25)
    side = [
        np.column_stack((x, np.full_like(x, left_wall), np.zeros(len(x)))),
        np.column_stack((x, np.full_like(x, right_wall), np.zeros(len(x)))),
    ]
    xs = np.linspace(10.0, 12.0, 18)
    ys = np.linspace(-0.5, 0.5, 9)
    xx, yy = np.meshgrid(xs, ys)
    side.append(np.column_stack((xx.ravel(), yy.ravel(),
                                 np.zeros(xx.size))))
    if distant_block:
        # Another independent obstacle in the early merge-back corridor.
        # Longitudinal gap prevents merging into the primary object's extent.
        side.append(np.column_stack((
            np.full(12, 18.0), np.zeros(12), np.zeros(12))))
    return np.vstack(side)


class AdaptiveReturnTests(unittest.TestCase):
    def test_multiple_return_times_configured(self):
        self.assertEqual(CFG["return_delay_options_m"], [0, 1.5, 3])
        self.assertEqual(CFG["candidate_offsets_m"], [1.8, 2.4, 2.8])
        self.assertEqual(CFG["diagnostic_offsets_m"], [])

    def test_first_left_avoidance_remains(self):
        result = choose_local_detour(cloud(), CFG)
        self.assertEqual(result["status"], "AVOIDING", result["reason"])
        self.assertEqual(result["side"], 1)

    def test_early_return_collision_can_be_avoided_by_later_return(self):
        result = choose_local_detour(cloud(distant_block=True), CFG)
        self.assertEqual(result["status"], "AVOIDING", result["reason"])
        self.assertEqual(result["side"], 1)
        details = [d for d in result["diagnostics"]
                   if "offset_m" in d and d["side"] == "LEFT"]
        self.assertTrue(any(d["return_delay_m"] == 0 and
                            d["stage_hits"]["RETURN"] > 0
                            for d in details))
        self.assertTrue(any(d["return_delay_m"] > 0 and
                            d["reason"] == "SAFE"
                            for d in details))
        # Chosen route must use a verified candidate, not a diagnostic path.
        chosen = np.asarray(result["path"])
        self.assertLessEqual(float(np.max(np.abs(chosen[:, 1]))), 2.8 + 1e-6)

    def test_missing_endpoint_visibility_still_stops(self):
        result = choose_local_detour(cloud(wall_end=14.0), CFG)
        self.assertEqual(result["status"], "STOP_REQUIRED")
        self.assertEqual(result["reason"], "RETURN_CORRIDOR_UNOBSERVED")
        self.assertEqual(result["path"], [])
        details = [d for d in result["diagnostics"] if "offset_m" in d]
        self.assertTrue(details)
        self.assertTrue(all(not d["visibility_ok"] for d in details))

    def test_close_wall_still_stops(self):
        result = choose_local_detour(
            cloud(left_wall=1.8, right_wall=-1.8), CFG)
        self.assertEqual(result["status"], "STOP_REQUIRED")

    def test_diagnostic_only_offsets_never_selected(self):
        result = choose_local_detour(cloud(), CFG)
        diagnostics = [d for d in result["diagnostics"]
                       if "offset_m" in d and d["offset_m"] >= 2.9]
        self.assertEqual(diagnostics, [])
        self.assertLessEqual(
            float(np.max(np.abs(np.asarray(result["path"])[:, 1]))), 2.8 + 1e-6)


if __name__ == "__main__":
    unittest.main()
