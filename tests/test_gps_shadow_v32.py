"""Offline regression for V32 GPS-blackout lane + original obstacle controls."""
import ast
import json
import math
import sys
import unittest
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/"src"))
from integration.gps_shadow_io import GpsShadowPose
from integration.gps_shadow_lane import parse_lane_result


class GpsShadowTest(unittest.TestCase):
    def test_first_fix_required(self):
        p=GpsShadowPose({})
        mode,xy,_=p.update(None,0,10,0.0)
        self.assertEqual(mode,"NO_FIX")
        self.assertIsNone(xy)
        mode,xy,_=p.update((10.0,2.0),0,10,0.1)
        self.assertEqual(mode,"GPS")
        self.assertEqual(xy,(10.0,2.0))

    def test_zero_gps_switches_to_shadow_and_imu_speed_integrates(self):
        p=GpsShadowPose({"max_shadow_sec":12})
        p.update((0.,0.),0.,7.2,0.)
        mode,pose,age=p.update(None,0.,7.2,.25)
        self.assertEqual(mode,"SHADOW")
        self.assertGreater(pose[0],0)
        self.assertGreaterEqual(age,0)

    def test_repeated_cached_gps_while_moving_is_not_fresh(self):
        p=GpsShadowPose({"stale_moving_sec":1.0})
        p.update((10.,10.),0.,7.2,0.)
        self.assertEqual(p.update((10.,10.),0.,7.2,.5)[0],"GPS")
        self.assertEqual(p.update((10.,10.),0.,7.2,1.25)[0],"SHADOW")

    def test_three_distinct_plausible_updates_recover(self):
        p=GpsShadowPose({"gps_recovery_samples":3})
        p.update((0.,0.),0.,3.6,0.)
        p.update(None,0.,3.6,.2)
        self.assertEqual(p.update((.30,0.),0.,3.6,.3)[0],"SHADOW")
        self.assertEqual(p.update((.42,0.),0.,3.6,.4)[0],"SHADOW")
        self.assertEqual(p.update((.55,0.),0.,3.6,.5)[0],"GPS")

    def test_stop_after_unsafe_blackout_or_control_gap(self):
        p=GpsShadowPose({"max_shadow_sec":1.0})
        p.update((0.,0.),0.,3.6,0.)
        p.update(None,0.,3.6,.2)
        self.assertEqual(p.update(None,0.,3.6,1.6)[0],"STOP")

    def test_recovery_rejects_large_jump(self):
        p=GpsShadowPose({"gps_recovery_max_error_m":5})
        p.update((0.,0.),0.,3.6,0.)
        p.update(None,0.,3.6,.2)
        self.assertEqual(p.update((100.,100.),0.,3.6,.3)[0],"SHADOW")
        self.assertEqual(p.recovery_count,0)


class LaneProtocolTest(unittest.TestCase):
    def test_valid_lane_packet(self):
        raw=json.dumps({"kind":"LANE_CENTER","valid":True,
            "frame":22,"steering":-0.1,"confidence":0.8,
            "center_error":0.4}).encode()
        p=parse_lane_result(raw)
        self.assertTrue(p["valid"])
        self.assertAlmostEqual(p["steering"],-0.1)

    def test_invalid_or_nonfinite(self):
        for raw in (b"",b"{}",b'{"kind":"LANE_CENTER","valid":true}',
                    json.dumps({"kind":"LANE_CENTER","valid":True,
                        "frame":2,"steering":float("nan"),
                        "confidence":.9,"center_error":0}).encode()):
            with self.subTest(raw=raw):
                self.assertFalse(parse_lane_result(raw)["valid"])


class IntegrationStructureTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.net=json.loads((ROOT/"config/integration_network.json").read_text())
        cls.cfg=json.loads((ROOT/"config/integration_missions.json").read_text())

    def test_localhost_distinct_ports(self):
        self.assertEqual(self.net["gps_shadow_camera_relay"],{"bind_ip":"127.0.0.1","port":9311})
        self.assertEqual(self.net["gps_shadow_lane_result"],{"bind_ip":"127.0.0.1","port":9312})
        ports=[v["port"] for k,v in self.net.items()
               if k not in ("control","gps","imu","competition_status","lidar")]
        self.assertEqual(len(ports),len(set(ports)))

    def test_hsv_canny_bev_ema_not_tensorflow(self):
        source=(ROOT/"src/integration/gps_shadow_lane.py").read_text()
        for term in ("cv2.inRange","cv2.Canny","cv2.warpPerspective",
                     "np.polyfit","ALPHA = 0.25","self.left_prev"):
            self.assertIn(term,source)
        self.assertNotIn("tensorflow",source.lower())

    def test_only_main_sends_control(self):
        for worker in ("gps_shadow_lane_worker.py","dynamic_pedestrian_worker.py",
                       "merge_side_camera_worker.py","lidar/static_avoidance_module.py"):
            src=(ROOT/"src"/worker).read_text()
            ast.parse(src)
            self.assertNotIn("EgoCtrlCmd(",src,worker)
        main=(ROOT/"src/main_integrated_v1.py").read_text()
        ast.parse(main)
        self.assertIn("base_steering = (",main)
        self.assertIn("if shadow_active:",main)
        self.assertIn("steering = float(local_pp[\"steering\"])",main)
        self.assertIn("or pedestrian_stop or shadow_stop",main)

    def test_shadow_enables_both_original_obstacle_workers(self):
        lidar=(ROOT/"src/lidar/static_avoidance_module.py").read_text()
        ped=(ROOT/"src/dynamic_pedestrian_worker.py").read_text()
        self.assertIn("or gps_shadow",lidar)
        self.assertIn("last_gate_shadow or",ped)

    def test_launcher_has_all_worker_processes(self):
        script=(ROOT/"scripts/run_integrated_v25.sh").read_text()
        for term in ("--relay-lane","src/gps_shadow_lane_worker.py",
                     "src/merge_side_camera_worker.py",
                     "src/lidar/static_avoidance_module.py",
                     "src/dynamic_pedestrian_worker.py",
                     "--gps-shadow"):
            self.assertIn(term,script)
        self.assertEqual(script.count("src/main_integrated_v1.py"),1)


if __name__=="__main__":
    unittest.main()
