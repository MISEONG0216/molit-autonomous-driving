"""V48 candidate count and early-detection configuration regression."""
import json
import unittest
from pathlib import Path
CFG=json.loads((Path(__file__).resolve().parents[1]/"config/integration_missions.json").read_text(encoding="utf-8"))["gps_shadow_ego_planner"]
class V48Tests(unittest.TestCase):
    def test_candidate_count(self):
        self.assertEqual(CFG["candidate_offsets_m"], [1.8, 2.4, 2.8])
        self.assertEqual(CFG["return_delay_options_m"], [0, 1.5, 3])
        self.assertEqual(CFG["diagnostic_offsets_m"], [])
        self.assertEqual(2*len(CFG["candidate_offsets_m"])*len(CFG["return_delay_options_m"]), 18)
    def test_detect_earlier_with_collision_gate(self):
        self.assertEqual(CFG["obstacle_max_forward_m"], 14)
        self.assertEqual(CFG["obstacle_point_threshold"], 3)
        self.assertEqual(CFG["min_side_points"], 5)
        self.assertEqual(CFG["allowed_points"], 0)
        self.assertEqual(CFG["end_visibility_min_points"], 3)
        self.assertEqual(CFG["avoidance_speed_kmh"], 5)
if __name__=="__main__":
    unittest.main()
