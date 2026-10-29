"""V34: tunnel wall avoidance clearance tests using synthetic vehicle-frame LiDAR."""
import ast
import json
import sys
import unittest
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/"src/lidar"))
sys.path.insert(0,str(ROOT/"src"))
from shadow_wall_guard import check_wall_clearance, pick_shadow_detour
from integration.static_avoidance_io import parse_static_message


def sample_cloud(right_y=-2.0,left_y=4.2,n=25):
    x=np.linspace(3.0,16.0,n)
    wall_right=np.column_stack((x,np.full(n,right_y),np.zeros(n)))
    wall_left=np.column_stack((x,np.full(n,left_y),np.zeros(n)))
    return np.vstack((wall_right,wall_left))


def route(offset):
    x=np.arange(0.,20.5,.5)
    # Return route gently into the selected side by 8m.
    t=np.clip((x-2.0)/6.0,0,1)
    y=offset*(t*t*(3-2*t))
    return np.column_stack((x,y))


class WallCheck(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.cfg=json.loads((ROOT/"config/integration_missions.json")
            .read_text())["gps_shadow_wall_guard"]

    def test_right_tunnel_wall_blocks_right_detour(self):
        clear,reason=check_wall_clearance(
            route(-1.8),(0,0,0),sample_cloud(),self.cfg)
        self.assertFalse(clear)
        self.assertEqual(reason,"WALL_OR_OBSTACLE_IN_SWEPT_PATH")

    def test_wider_left_clearance_allows_left_detour(self):
        clear,reason=check_wall_clearance(
            route(1.8),(0,0,0),sample_cloud(left_y=4.8),self.cfg)
        self.assertTrue(clear,reason)

    def test_both_walls_close_means_stop(self):
        points=sample_cloud(right_y=-1.9,left_y=2.0)
        self.assertFalse(check_wall_clearance(route(-1.8),(0,0,0),points,self.cfg)[0])
        self.assertFalse(check_wall_clearance(route(1.8),(0,0,0),points,self.cfg)[0])

    def test_sparse_lidar_cannot_authorize_turn(self):
        clear,reason=check_wall_clearance(
            route(1.8),(0,0,0),sample_cloud(n=2),self.cfg)
        self.assertFalse(clear)
        self.assertEqual(reason,"SCAN_SPARSE")

    def test_unseen_right_boundary_blocks_turn(self):
        x=np.linspace(3,16,30)
        cloud=np.column_stack((x,np.full(len(x),4.5),np.zeros(len(x))))
        clear,reason=check_wall_clearance(route(1.8),(0,0,0),cloud,self.cfg)
        self.assertFalse(clear)

    def test_pick_left_when_right_is_blocked_by_tunnel_wall(self):
        from types import SimpleNamespace
        class FakePlanner:
            path = np.column_stack((np.linspace(0, 30, 61), np.zeros(61)))
            cfg = {"candidate_offsets_m": [-1.8, 1.8]}
            avoidance_speed_kph = 12.0

            def _select_obstacle(self, ego_index, obstacles):
                return (SimpleNamespace(id=17), 8, 0.25)

            def _candidate_path(self, ego_index, obs_index, offset):
                path = route(offset)
                return path, np.full(len(path), offset)

            def _candidate_safety_diagnostic(self, candidate, offsets, obs):
                return {"safe": True, "reason": "SAFE"}

        plan, verified, reason = pick_shadow_detour(
            FakePlanner(), (0.0, 0.0, 0.0), [],
            sample_cloud(left_y=4.8), self.cfg
        )
        self.assertTrue(verified, reason)
        self.assertEqual(plan.status, "AVOIDING")
        self.assertEqual(plan.avoidance_side, "LEFT")
        self.assertLessEqual(plan.recommended_speed_kph, 6.0)

    def test_new_udp_flag_requires_bool(self):
        base=dict(status="AVOIDING",path=[[0,0],[5,0]],
                  zone_active=True,recommended_speed_kmh=6,
                  shadow_wall_verified=True)
        parsed=parse_static_message(json.dumps(base).encode())
        self.assertTrue(parsed["valid"])
        self.assertTrue(parsed["shadow_wall_verified"])
        base["shadow_wall_verified"]="true"
        self.assertFalse(parse_static_message(json.dumps(base).encode())["valid"])

    def test_main_only_accepts_verified_path(self):
        main=(ROOT/"src/main_integrated_v1.py").read_text()
        worker=(ROOT/"src/lidar/static_avoidance_module.py").read_text()
        ast.parse(main);ast.parse(worker)
        self.assertIn('not static_plan.get("shadow_wall_verified", False)',main)
        self.assertIn('pick_shadow_detour(',worker)
        self.assertIn('check_wall_clearance(',worker)
        self.assertIn('or (args.static_control and static_stop)',main)
        self.assertNotIn("EgoCtrlCmd(",worker)


if __name__=="__main__":
    unittest.main()
