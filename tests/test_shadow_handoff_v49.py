"""V49: detour handoff stays speed bounded, LiDAR-verified and 8s recovery is gone."""
import json
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
CFG = json.loads((ROOT / "config/integration_missions.json").read_text(
    encoding="utf-8"))
WORKER = (ROOT / "src/lidar/static_avoidance_module.py").read_text(
    encoding="utf-8")
MAIN = (ROOT / "src/main_integrated_v1.py").read_text(
    encoding="utf-8")


class V49DetourHandoffTests(unittest.TestCase):
    def test_recovery_timer_removed(self):
        self.assertNotIn("gps_shadow_recovery", CFG)
        self.assertNotIn("shadow_recovery.update(", MAIN)
        self.assertNotIn("SHADOW RECOVERY:", MAIN)

    def test_bounded_handoff_config(self):
        cfg = CFG["gps_shadow_ego_planner"]
        self.assertEqual(cfg["post_detour_handoff_distance_m"], 8)
        self.assertEqual(cfg["post_detour_handoff_speed_kmh"], 3)
        self.assertEqual(cfg["allowed_points"], 0)

    def test_handoff_requires_fresh_lidar_confirmation(self):
        self.assertIn("if forward_clear_verified and not close_forward_collision(", WORKER)
        self.assertIn('"POST_DETOUR_FORWARD_UNVERIFIED"', WORKER)
        self.assertIn('"EGO_POST_DETOUR_VERIFIED_LANE"', WORKER)
        self.assertIn('"post_detour_handoff": bool(', WORKER)
        self.assertIn('static_plan.get("post_detour_handoff", False)', MAIN)

    def test_dynamic_and_lidar_stop_remain(self):
        self.assertIn('"SHADOW_DYNAMIC_TRACK_STOP"', WORKER)
        self.assertIn('"SHADOW_CLOSE_FORWARD_STOP"', WORKER)
        self.assertIn("or static_state == \"STOP_REQUIRED\"", MAIN)


if __name__ == "__main__":
    unittest.main()
