"""V27 highway following regression tests (offline, no MORAI needed)."""
import ast
import json
import math
import sys
import unittest
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from integration.highway_follow_io import (
    LANE_ROI_RATIO, grade_speed_cap, in_follow_zone,
    highway_speed_zones, parse_follow_message,
)
from integration.highway_speed_profile import build_highway_profile


class V27HighwayFollowTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.cfg = json.loads((ROOT / "config/integration_missions.json")
                             .read_text(encoding="utf-8"))["highway_follow"]

    def test_v30_near_field_roi_preserved(self):
        # V30 intentionally replaced the original V27 narrow ROI.
        self.assertEqual(LANE_ROI_RATIO, (
            (0.44, 0.38), (0.56, 0.38), (0.88, 0.97), (0.12, 0.97)
        ))

    def test_zones_include_second_until_tollgate_unknown(self):
        c = self.cfg
        for wp in (2570, 3040, 3200, 4429):
            self.assertTrue(in_follow_zone(wp, c, 4430))
        for wp in (2569, 3041, 3199, -1):
            self.assertFalse(in_follow_zone(wp, c, 4430))
        self.assertEqual(highway_speed_zones(c, 4430), [(2570,3040)])

    def test_verified_tollgate_enables_second_fast_zone(self):
        c = dict(self.cfg, tollgate_exit_wp=3650)
        self.assertEqual(highway_speed_zones(c, 4430),
                         [(2570,3040), (3200,3650)])

    def test_grades_are_decreasing_caps(self):
        self.assertEqual([grade_speed_cap(v) for v in
                         (0.00,0.20,0.40,0.60,0.80)],
                         [(0,120.0),(1,90.0),(2,65.0),(3,40.0),(4,20.0)])
        self.assertEqual(grade_speed_cap(float("nan")), (4,20.0))

    def test_follow_packet_validation(self):
        raw = json.dumps({"kind":"HIGHWAY_FOLLOW","risk":0.63,
                          "waypoint_index":2600,"frame":10,
                          "vehicles":1}).encode("utf-8")
        result = parse_follow_message(raw)
        self.assertTrue(result["valid"])
        self.assertEqual(result["grade"], 3)
        self.assertFalse(parse_follow_message(b"{}")["valid"])
        self.assertFalse(parse_follow_message(b'{"risk":NaN}')["valid"])

    def test_highway_profile_keeps_normal_speed_and_exits_slowly(self):
        n=4430
        path = np.zeros((n,2), dtype=float)
        path[:,0] = np.arange(n) * 0.5
        old = np.full(n, 57.0)
        radii = np.full(n, math.inf)
        profile=build_highway_profile(old,radii,path,
                                      highway_speed_zones(self.cfg,n),120.0)
        self.assertTrue(np.all(profile <= 120.00001))
        self.assertAlmostEqual(profile[2000],57.0)
        self.assertAlmostEqual(profile[3041],57.0)
        self.assertAlmostEqual(profile[3500],57.0)
        self.assertGreater(profile[2800],57.0)
        self.assertTrue(np.all(np.diff(profile[2560:3043]) < 30))
        old_profile=build_highway_profile(old,radii,path,
                                          highway_speed_zones(self.cfg,n),57.0)
        np.testing.assert_allclose(old_profile,old)

    def test_turns_cap_speed(self):
        n=4430
        path=np.column_stack((np.arange(n)*0.5, np.zeros(n)))
        radii=np.full(n, math.inf)
        radii[2800]=30.0
        speed=build_highway_profile(np.full(n,57.0),radii,path,
                                    [(2570,3040)],120.0)
        self.assertLess(speed[2800],30.0)

    def test_one_control_sender_and_shared_camera(self):
        worker=(ROOT/"src/dynamic_pedestrian_worker.py").read_text(encoding="utf-8")
        main=(ROOT/"src/main_integrated_v1.py").read_text(encoding="utf-8")
        launcher=(ROOT/"scripts/run_integrated_v25.sh").read_text(encoding="utf-8")
        for src in (worker,main):
            ast.parse(src)
        self.assertNotIn("EgoCtrlCmd(",worker)
        self.assertIn("--follow",launcher)
        self.assertIn("--follow-control",launcher)
        self.assertIn("highway_follow_result",worker)
        self.assertIn("highway_follow_result",main)
        self.assertIn('profile_test.build_test_profile(global_path, min(args.max_speed, 57.0))',main)
        self.assertIn('"follow_grade"',main)

if __name__=="__main__":
    unittest.main()
