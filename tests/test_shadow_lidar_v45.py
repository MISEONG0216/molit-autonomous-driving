"""V45 smoke regression for GPS-shadow LiDAR-only return and exact STOP reasons."""
import json
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from integration.static_avoidance_io import parse_static_message


class V45ShadowLiDARTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.config = json.loads((ROOT / "config/integration_missions.json").read_text(encoding="utf-8"))
        cls.main_source = (ROOT / "src/main_integrated_v1.py").read_text(encoding="utf-8")

    def test_timed_steering_removed_from_main(self):
        self.assertNotIn("--shadow-timed-steer", self.main_source)
        self.assertNotIn("timed_shadow_steer", self.main_source)
        self.assertIn("shadow_tail_steering(", self.main_source)
        self.assertIn("shadow_wall_verified", self.main_source)

    def test_lidar_scan_age_bounded_and_speeds_preserved(self):
        age = self.config["gps_shadow_ego_planner"]["max_result_age_sec"]
        verified = self.config["gps_shadow_wall_guard"]["max_verified_scan_age_sec"]
        self.assertEqual(age, 2.4)
        self.assertLessEqual(age, verified)
        self.assertEqual(self.config["gps_shadow_ego_planner"]["avoidance_speed_kmh"], 5)
        self.assertEqual(self.config["dynamic_pedestrian"]["crossing_speed_zones"][-1], [900, 940, 20])

    def test_static_stop_reason_is_passed_to_main(self):
        raw = json.dumps({
            "status": "STOP_REQUIRED", "path": [], "reason": "SIDE_WALL_NOT_OBSERVED",
            "latch_state": "EGO_LIDAR_STOP_SIDE_WALL_NOT_OBSERVED",
            "scan": 14, "zone_active": True, "shadow_wall_verified": False,
            "recommended_speed_kmh": 0,
        }).encode("utf-8")
        p = parse_static_message(raw)
        self.assertTrue(p["valid"])
        self.assertEqual(p["reason"], "SIDE_WALL_NOT_OBSERVED")
        self.assertFalse(p["shadow_wall_verified"])


if __name__ == "__main__":
    unittest.main()
