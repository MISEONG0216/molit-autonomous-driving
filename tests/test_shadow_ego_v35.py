"""V35 GPS-free ego-frame tunnel avoidance regression, no MORAI needed."""
import ast
import json
import math
import sys
import unittest
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/"src"))
sys.path.insert(0,str(ROOT/"src/lidar"))
from lidar.shadow_ego_planner import choose_local_detour
from integration.gps_shadow_io import GpsShadowPose


def cloud(right=-2.0,left=4.8,obstacle=True):
    x=np.linspace(3,16,80)
    z=np.zeros(len(x))
    pts=[np.column_stack((x,np.full_like(x,right),z)),
         np.column_stack((x,np.full_like(x,left),z))]
    if obstacle:
        xx=np.linspace(8.5,9.5,15)
        pts.append(np.column_stack((xx,np.zeros_like(xx),np.zeros_like(xx))))
    return np.vstack(pts)


class EgoPlannerTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.cfg=json.loads((ROOT/"config/integration_missions.json").read_text())["gps_shadow_ego_planner"]

    def test_tunnel_without_center_obstacle_stays_in_lane(self):
        result=choose_local_detour(cloud(obstacle=False),self.cfg)
        self.assertEqual(result["status"],"CLEAR")
        self.assertEqual(result["path"],[])

    def test_right_wall_disallows_right_detour(self):
        result=choose_local_detour(cloud(),self.cfg)
        self.assertEqual(result["status"],"AVOIDING",result["reason"])
        self.assertEqual(result["side"],1)
        self.assertEqual(len(result["path"]),31)

    def test_both_close_walls_produce_stop(self):
        result=choose_local_detour(cloud(right=-1.6,left=1.8),self.cfg)
        self.assertEqual(result["status"],"STOP_REQUIRED")

    def test_missing_side_observations_fail_closed(self):
        pts=cloud()
        pts=pts[pts[:,1]>=0]
        result=choose_local_detour(pts,self.cfg)
        self.assertEqual(result["status"],"STOP_REQUIRED")

    def test_near_immediate_obstacle_means_stop(self):
        pts=cloud()
        q=np.column_stack((np.full(8,1.8),np.zeros(8),np.zeros(8)))
        result=choose_local_detour(np.vstack((pts,q)),self.cfg)
        self.assertEqual(result["status"],"STOP_REQUIRED")


class ShadowIntegrationTest(unittest.TestCase):
    def test_gps_loss_can_continue_beyond_old_twelve_second_timer(self):
        cfg={"allow_sensor_guided_extended":True,"stale_moving_sec":1}
        p=GpsShadowPose(cfg)
        p.update((0,0),0,3.6,0)
        for t in np.arange(.1,17,.1):
            status,_,_=p.update(None,0,3.6,float(t))
        self.assertEqual(status,"SHADOW")

    def test_control_loop_gap_still_stops(self):
        p=GpsShadowPose({"allow_sensor_guided_extended":True})
        p.update((0,0),0,3.6,0)
        p.update(None,0,3.6,.1)
        self.assertEqual(p.update(None,0,3.6,1.1)[0],"STOP")

    def test_main_has_fresh_lidar_gate_and_one_command_sender(self):
        source=(ROOT/"src/main_integrated_v1.py").read_text()
        worker=(ROOT/"src/lidar/static_avoidance_module.py").read_text()
        ast.parse(source);ast.parse(worker)
        self.assertIn('mission["reasons"].append("gps_shadow_lidar_stale")',source)
        self.assertIn('if not static_fresh:',source)
        self.assertIn("shadow_locked_path = global_xy",worker)
        self.assertIn("EGO_LIDAR_LOCKED_",worker)
        self.assertIn("choose_local_detour(",worker)
        self.assertNotIn("EgoCtrlCmd(",worker)
        self.assertEqual(source.count("ctrl_sender.send(cmd)"),1)


if __name__=="__main__":
    unittest.main()
