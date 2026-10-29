import unittest
import sys
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/"src/lidar"))
from shadow_ego_planner import choose_local_detour

class ClearCorridorTest(unittest.TestCase):
    def setUp(self):
        self.cfg={"min_points":20,"min_side_points":8,"obstacle_point_threshold":5}
    def test_clear_forward_does_not_require_both_walls(self):
        x=np.linspace(3.,15.,40)
        p=np.column_stack((x,np.full_like(x,-2.),np.zeros_like(x)))
        result=choose_local_detour(p,self.cfg)
        self.assertEqual(result["status"],"CLEAR")
    def test_obstruction_with_missing_wall_stops(self):
        x=np.linspace(3.,15.,40)
        p=np.column_stack((x,np.full_like(x,-2.),np.zeros_like(x)))
        obstacle=np.column_stack((np.full(10,8.),np.zeros(10),np.zeros(10)))
        result=choose_local_detour(np.vstack((p,obstacle)),self.cfg)
        self.assertEqual(result["status"],"STOP_REQUIRED")
        self.assertEqual(result["reason"],"SIDE_WALL_NOT_OBSERVED")
    def test_immediate_obstacle_still_stops(self):
        x=np.linspace(3.,15.,40)
        p=np.column_stack((x,np.full_like(x,-2.),np.zeros_like(x)))
        obstacle=np.column_stack((np.full(10,2.),np.zeros(10),np.zeros(10)))
        result=choose_local_detour(np.vstack((p,obstacle)),self.cfg)
        self.assertEqual(result["status"],"STOP_REQUIRED")
        self.assertEqual(result["reason"],"IMMEDIATE_OBSTACLE")

if __name__=="__main__":
    unittest.main()
