"""V30: prewarm, roundabout release and close-car lane ROI regression."""
import ast
import json
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from integration.merge_side_io import merge_camera_mode, active_merge_cameras
from integration.highway_follow_io import (
    LANE_ROI_RATIO, vehicle_overlaps_lane, retained_follow_risk,
)
from integration.mission_manager import MissionManager
from integration.mission_io import parse_merge_message
from integration.merge_side_io import format_result


class V30Test(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.missions = json.loads((ROOT/"config/integration_missions.json")
                                  .read_text(encoding="utf-8"))
        cls.cfg = cls.missions["merge"]

    def apply(self, manager, wp, risk, decision, stamp):
        packet = parse_merge_message(format_result(risk, 0, decision))
        return manager.apply(
            nearest_index=wp, base_target_speed_kmh=57.0,
            base_steering=0.0, merge_enabled=True,
            merge_control_enabled=True, merge_result=packet,
            merge_updated_at=stamp, now=stamp+0.01,
        )

    def test_camera_prearms_and_stays_live_on_entry(self):
        self.assertEqual(merge_camera_mode(1699, self.cfg), "off")
        self.assertEqual(merge_camera_mode(1700, self.cfg), "roundabout_prearm")
        self.assertEqual(merge_camera_mode(1769, self.cfg), "roundabout_prearm")
        self.assertEqual(merge_camera_mode(1770, self.cfg), "roundabout")
        self.assertEqual(active_merge_cameras("roundabout_prearm"), ("left",))
        self.assertEqual(merge_camera_mode(2455, self.cfg), "highway_prearm")
        self.assertEqual(active_merge_cameras("highway_prearm"),
                         ("left", "left_rear"))
        self.assertEqual(merge_camera_mode(2480, self.cfg), "highway_merge")
        self.assertEqual(merge_camera_mode(1770, self.cfg, roundabout_done=True),
                         "roundabout_done")
        self.assertEqual(active_merge_cameras("roundabout_done"), ())

    def test_roundabout_no_automatic_release_without_stop(self):
        m = MissionManager({}, self.cfg, [])
        for wp in (1770, 1780, 1800, 1810):
            result = self.apply(m, wp, 0.0, "MERGE_ACCEPT", 100+wp)
            self.assertFalse(result["roundabout_done"])

    def test_roundabout_release_after_stop_and_three_fresh_accepts(self):
        m = MissionManager({}, self.cfg, [])
        stopped = self.apply(m, 1770, 0.8, "MERGE_REJECT", 100)
        self.assertTrue(stopped["merge_stop"])
        for stamp in (101, 102):
            still = self.apply(m, 1778, 0.0, "MERGE_ACCEPT", stamp)
            self.assertTrue(still["merge_stop"])
        safe = self.apply(m, 1778, 0.0, "MERGE_ACCEPT", 103)
        self.assertFalse(safe["merge_stop"])
        self.assertFalse(safe["roundabout_done"])
        # Too early: still use camera even after safe release.
        early = self.apply(m, 1799, 0.0, "MERGE_ACCEPT", 104)
        self.assertFalse(early["roundabout_done"])
        completed = self.apply(m, 1800, 0.0, "MERGE_ACCEPT", 105)
        self.assertTrue(completed["roundabout_done"])
        self.assertEqual(completed["merge_state"], "ROUNDABOUT_DONE")
        # Rear traffic must not repeatedly stop after a confirmed completion.
        later = self.apply(m, 1801, 0.8, "MERGE_REJECT", 106)
        self.assertFalse(later["merge_stop"])
        # Outside the roundabout reset state for the next highway merge.
        self.apply(m, 1821, 0.0, "MERGE_ACCEPT", 107)
        self.assertFalse(m.roundabout_done)

    def test_new_danger_before_completion_prevents_release(self):
        m = MissionManager({}, self.cfg, [])
        self.apply(m, 1770, 0.8, "MERGE_REJECT", 100)
        for t in (101,102,103):
            self.apply(m, 1778, 0.0, "MERGE_ACCEPT", t)
        danger = self.apply(m, 1799, 0.8, "MERGE_REJECT", 104)
        self.assertTrue(danger["merge_stop"])
        not_done = self.apply(m, 1800, 0.0, "MERGE_ACCEPT", 105)
        self.assertFalse(not_done["roundabout_done"])
        self.assertTrue(not_done["merge_stop"])

    def test_near_vehicle_roi_and_short_risk_memory(self):
        self.assertGreater(LANE_ROI_RATIO[2][0], 0.80)
        self.assertTrue(vehicle_overlaps_lane(150, 450, 1100, 705, 1280, 720))
        self.assertTrue(vehicle_overlaps_lane(400, 300, 1000, 705, 1280, 720))
        self.assertFalse(vehicle_overlaps_lane(0, 350, 180, 550, 1280, 720))
        self.assertAlmostEqual(retained_follow_risk(0, 0.9, 0.4), 0.9)
        self.assertAlmostEqual(retained_follow_risk(0, 0.9, 1.2), 0.54)
        self.assertAlmostEqual(retained_follow_risk(0, 0.9, 2.0), 0.0)

    def test_existing_missions_not_replaced(self):
        main=(ROOT/"src/main_integrated_v1.py").read_text(encoding="utf-8")
        worker=(ROOT/"src/dynamic_pedestrian_worker.py").read_text(encoding="utf-8")
        merge_worker=(ROOT/"src/merge_side_camera_worker.py").read_text(encoding="utf-8")
        for code in (main, worker, merge_worker):
            ast.parse(code)
        self.assertIn('"roundabout_done": bool(mission_manager.roundabout_done)',main)
        self.assertIn("vehicle_overlaps_lane(x1, y1, x2, y2, w, h)",worker)
        self.assertNotIn("EgoCtrlCmd(",worker)
        self.assertNotIn("EgoCtrlCmd(",merge_worker)
        self.assertNotIn('("right", 9295)',merge_worker)
        self.assertEqual(self.missions["dynamic_pedestrian"]["zones_wp"],[[720,1050]])


if __name__ == "__main__":
    unittest.main()
