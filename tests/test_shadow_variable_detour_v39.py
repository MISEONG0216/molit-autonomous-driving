"""V39 regression tests for obstacle-dependent candidate geometry."""
import sys
import unittest
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/"src/lidar"))
from shadow_ego_planner import choose_local_detour

class VariableCandidateTests(unittest.TestCase):
    def setUp(self):
        self.cfg={"min_points":40,"min_side_points":8,
                  "obstacle_point_threshold":5,"minimum_offset_m":0.8,
                  "maximum_offset_m":2.6,"offset_step_m":0.2,
                  "max_horizon_m":24.0}
    def cloud(self, wall_left=4.8,wall_right=-4.8,box_start=8.,
              box_end=9.,box_y=0.):
        x=np.linspace(2.,24.,140)
        result=[np.column_stack((x,np.full(x.size,wall_left),np.zeros(x.size))),
                np.column_stack((x,np.full(x.size,wall_right),np.zeros(x.size)))]
        x_box=np.linspace(box_start,box_end,18)
        result.append(np.column_stack((x_box,np.full(x_box.size,box_y),
                                       np.zeros(x_box.size))))
        return np.vstack(result)
    def test_center_obstacle_selects_a_safe_side(self):
        p=choose_local_detour(self.cloud(),self.cfg)
        self.assertEqual(p["status"],"AVOIDING",p["reason"])
        self.assertGreaterEqual(p["offset_m"],1.4)
        self.assertLessEqual(p["offset_m"],2.6)
    def test_return_after_box(self):
        p=choose_local_detour(self.cloud(box_end=11.),self.cfg)
        self.assertEqual(p["status"],"AVOIDING",p["reason"])
        path=np.asarray(p["path"])
        index=np.argmin(abs(path[:,0]-11.))
        self.assertGreater(abs(path[index,1]),1.2)
    def test_box_too_close_rejected(self):
        p=choose_local_detour(self.cloud(box_start=3.2,box_end=4.),self.cfg)
        self.assertEqual(p["status"],"STOP_REQUIRED")
    def test_unseen_return_rejected(self):
        pts=self.cloud()
        pts=pts[pts[:,0]<11.]
        p=choose_local_detour(pts,self.cfg)
        self.assertEqual(p["status"],"STOP_REQUIRED")
    def test_no_box_keeps_lane(self):
        p=self.cloud()
        p=p[(p[:,1]>2)|(p[:,1]<-2)]
        self.assertEqual(choose_local_detour(p,self.cfg)["status"],"CLEAR")
if __name__=="__main__":
    unittest.main()
