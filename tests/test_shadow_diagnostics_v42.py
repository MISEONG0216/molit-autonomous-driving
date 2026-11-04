"""V42: diagnostics-only lateral offsets and sweep-stage regression tests."""
import json
import sys
import unittest
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from lidar.shadow_ego_planner import (
    _path, _sweep_metrics, choose_local_detour,
)

CFG = json.loads((ROOT / "config/integration_missions.json").read_text(
    encoding="utf-8"))["gps_shadow_ego_planner"]


def scan(left=6.5, right=-2.0, end=27.0):
    x = np.arange(3.0, end, 0.25)
    left_wall = np.column_stack((x, np.full_like(x, left), np.zeros_like(x)))
    right_wall = np.column_stack((x, np.full_like(x, right), np.zeros_like(x)))
    ox = np.linspace(10.0, 12.0, 17)
    oy = np.linspace(-0.5, 0.5, 7)
    xx, yy = np.meshgrid(ox, oy)
    block = np.column_stack((xx.ravel(), yy.ravel(), np.zeros(xx.size)))
    return np.vstack((left_wall, right_wall, block))


class V42Tests(unittest.TestCase):
    def test_active_offsets_unchanged(self):
        self.assertEqual(CFG["candidate_offsets_m"], [1.6, 2.0, 2.4, 2.8])
        self.assertEqual(CFG["diagnostic_offsets_m"], [2.9, 3.0, 3.1, 3.2])

    def test_new_offsets_never_control_path(self):
        result = choose_local_detour(scan(), CFG)
        self.assertEqual(result["status"], "AVOIDING", result["reason"])
        self.assertEqual(result["side"], 1)
        extras = [d for d in result["diagnostics"]
                  if d.get("offset_m") in CFG["diagnostic_offsets_m"]]
        self.assertEqual(len(extras), 8)
        self.assertTrue(all(d["diagnostic_only"] for d in extras))
        # At least one additional diagnostic candidate geometrically clears
        # the scan, but it still is not eligible as a selected path.
        self.assertTrue(any(d["reason"] == "DIAGNOSTIC_ONLY" for d in extras))
        # Selected path is bounded by the original maximum 2.8m offset.
        local_y = np.asarray(result["path"])[:, 1]
        self.assertLessEqual(float(np.max(np.abs(local_y))), 2.8 + 1e-6)

    def test_stage_hit_counts_sum_to_total(self):
        path = _path(1, CFG, 2.8, 7.0, 14.0, 19.0)
        points = np.array([
            [3.0, 0.0],
            [10.0, 0.0],
            [18.0, 0.0],
            [10.0, 6.0],
        ])
        distances, counts, minima = _sweep_metrics(points, path, 1.35, 7.0, 14.0)
        self.assertEqual(sum(counts.values()), int(np.count_nonzero(distances < 1.35)))
        self.assertEqual(set(counts), {"ENTER", "PASS", "RETURN"})
        self.assertEqual(set(minima), {"ENTER", "PASS", "RETURN"})

    def test_missing_return_still_diagnoses_but_stops(self):
        result = choose_local_detour(scan(end=14.0), CFG)
        self.assertEqual(result["status"], "STOP_REQUIRED")
        self.assertEqual(result["reason"], "RETURN_CORRIDOR_UNOBSERVED")
        offsets = [d for d in result["diagnostics"] if "offset_m" in d]
        self.assertEqual(len(offsets), 16)
        self.assertTrue(any(d.get("diagnostic_only") for d in offsets))
        self.assertEqual(result["path"], [])

    def test_both_wall_blocked_still_stops(self):
        result = choose_local_detour(scan(left=1.8, right=-1.8), CFG)
        self.assertEqual(result["status"], "STOP_REQUIRED")


if __name__ == "__main__":
    unittest.main()
