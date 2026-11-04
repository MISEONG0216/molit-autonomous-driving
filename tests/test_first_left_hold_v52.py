"""V52: FIRST_LEFT must not include a built-in RIGHT rejoin before SECOND match."""
import ast
import json
import sys
import unittest
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/"src/lidar"))
from shadow_predefined_templates import choose_template_detour

CFG=json.loads((ROOT/"config/tunnel_predefined_templates.json").read_text(
    encoding="utf-8"))

def cloud(first_x=10.0, first_y=-0.5, left_wall=7.0, right_wall=-6.0,
          wall_end=25):
    x=np.arange(2.0,wall_end,.25)
    w=np.vstack((
        np.column_stack((x,np.full(len(x),left_wall),np.zeros(len(x)))),
        np.column_stack((x,np.full(len(x),right_wall),np.zeros(len(x))))
    ))
    xs,ys=np.meshgrid(
        np.arange(first_x, first_x+2.01,.25),
        np.linspace(first_y-.35,first_y+.35,5))
    return np.vstack((w,np.column_stack((xs.ravel(),ys.ravel(),
                                        np.zeros(xs.size)))))

class FirstLeftHoldTests(unittest.TestCase):
    def test_first_left_ends_left_not_right(self):
        r=choose_template_detour(cloud(),CFG,0)
        self.assertEqual(r["status"],"AVOIDING",r["reason"])
        self.assertEqual(r["template_id"],"FIRST_LEFT")
        path=np.asarray(r["path"])
        off=float(r["selected_offset_m"])
        self.assertGreater(off,0)
        self.assertAlmostEqual(float(path[-1,1]),off,places=5)
        self.assertTrue(np.all(np.diff(path[:,1])>=-1e-8))
        self.assertLessEqual(float(path[-1,0]),20.)

    def test_first_side_wall_collision_prevents_left(self):
        r=choose_template_detour(
            cloud(left_wall=3.0),CFG,0)
        self.assertEqual(r["status"],"STOP_REQUIRED",r["reason"])

    def test_missing_end_wall_observation_prevents_hold(self):
        r=choose_template_detour(cloud(wall_end=13),CFG,0)
        self.assertEqual(r["status"],"STOP_REQUIRED",r["reason"])

    def test_second_still_requires_survey_match(self):
        r=choose_template_detour(cloud(),CFG,1)
        self.assertNotEqual(r["status"],"AVOIDING")
        self.assertEqual(CFG["second_survey_match"]["target_forward_m"],17.5)
        self.assertEqual(CFG["second_survey_match"]["confirmation_scans"],2)

    def test_main_camera_following_and_shadow_guard_unmodified(self):
        main=(ROOT/"src/main_integrated_v1.py").read_text(encoding="utf-8")
        worker=(ROOT/"src/lidar/static_avoidance_module.py").read_text(
            encoding="utf-8")
        ast.parse(main); ast.parse(worker)
        self.assertIn('float(lane_result["steering"]) if lane_fresh',main)
        self.assertIn("check_wall_clearance(",worker)
        self.assertIn("close_forward_collision(",worker)
        self.assertIn("template_completed_count += 1",worker)

if __name__=="__main__":
    unittest.main()
