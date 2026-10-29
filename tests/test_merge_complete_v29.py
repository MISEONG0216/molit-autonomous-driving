"""Regression: do not let a stopped car behind block an already-completed merge."""
import ast
import json
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from integration.merge_side_io import merge_camera_mode, active_merge_cameras, format_result
from integration.mission_io import parse_merge_message
from integration.mission_manager import MissionManager


class MergeCompleteV29Test(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.cfg = json.loads((ROOT/"config/integration_missions.json")
                             .read_text(encoding="utf-8"))["merge"]

    def test_observed_first_merge_release_boundary(self):
        self.assertEqual(self.cfg["rear_camera_release_wp_by_zone"], {"2480":2538})
        self.assertEqual(merge_camera_mode(2537,self.cfg),"highway_merge")
        self.assertEqual(merge_camera_mode(2538,self.cfg),"post_merge")
        self.assertEqual(merge_camera_mode(2560,self.cfg),"post_merge")
        self.assertEqual(merge_camera_mode(2561,self.cfg),"off")
        self.assertEqual(active_merge_cameras("post_merge"),("left",))
        # Next highway merge is NOT assigned a guessed completion waypoint.
        self.assertEqual(merge_camera_mode(3050,self.cfg),"highway_merge")
        self.assertEqual(merge_camera_mode(3190,self.cfg),"highway_merge")
        self.assertEqual(active_merge_cameras("highway_merge"),("left","left_rear"))
        self.assertEqual(merge_camera_mode(1775,self.cfg),"roundabout")

    def test_left_is_still_required_in_post_merge(self):
        # Side risk 0.70 still means REJECT, even after rear was excluded.
        left_danger = format_result(0.70,0.0,"MERGE_REJECT")
        parsed = parse_merge_message(left_danger)
        self.assertTrue(parsed["valid"])
        self.assertAlmostEqual(parsed["max_risk"],0.7)
        self.assertEqual(parsed["decision"],"MERGE_REJECT")

    def apply_merge(self, mgr, wp, risk, decision, stamp=100.0):
        return mgr.apply(
            nearest_index=wp, base_target_speed_kmh=57.0,
            base_steering=0.0, traffic_enabled=False, merge_enabled=True,
            merge_control_enabled=True,
            merge_result=parse_merge_message(format_result(risk,0.0,decision)),
            merge_updated_at=stamp, merge_all=False, now=stamp+0.01,
        )

    def test_yellow_highway_keeps_v7_target(self):
        mgr = MissionManager({}, self.cfg, [])
        result = self.apply_merge(mgr,2500,0.55,"MERGE_ACCEPT")
        self.assertFalse(result["merge_stop"])
        self.assertAlmostEqual(result["target_speed_kmh"],57.0)
        self.assertNotIn("merge_zone_speed",result["reasons"])
        # Roundabout continues keeping its existing 20 km/h zone cap.
        roundabout = self.apply_merge(mgr,1780,0.0,"MERGE_ACCEPT",stamp=101.0)
        self.assertAlmostEqual(roundabout["target_speed_kmh"],20.0)

    def test_red_rear_before_completion_still_stops(self):
        mgr = MissionManager({}, self.cfg, [])
        danger=self.apply_merge(mgr,2537,0.8,"MERGE_REJECT")
        self.assertTrue(danger["merge_stop"])
        self.assertEqual(danger["target_speed_kmh"],0.0)

    def test_existing_three_fresh_accepts_required(self):
        mgr=MissionManager({},self.cfg,[])
        self.apply_merge(mgr,2537,0.8,"MERGE_REJECT",stamp=100.0)
        for stamp in (101.0,102.0):
            step=self.apply_merge(mgr,2538,0.0,"MERGE_ACCEPT",stamp=stamp)
            self.assertTrue(step["merge_stop"])
        final=self.apply_merge(mgr,2538,0.0,"MERGE_ACCEPT",stamp=103.0)
        self.assertFalse(final["merge_stop"])
        self.assertAlmostEqual(final["target_speed_kmh"],57.0)

    def test_source_only_changes_merge(self):
        worker=(ROOT/"src/merge_side_camera_worker.py").read_text(encoding="utf-8")
        manager=(ROOT/"src/integration/mission_manager.py").read_text(encoding="utf-8")
        ast.parse(worker)
        ast.parse(manager)
        self.assertIn('"roundabout_done"',worker)
        self.assertIn('"highway_yellow_no_speed_cap"',manager)
        self.assertNotIn("EgoCtrlCmd(",worker)
        self.assertNotIn('("right", 9295)',worker)


if __name__=="__main__":
    unittest.main()
