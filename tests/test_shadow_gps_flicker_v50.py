"""Regression for first-left GPS shadow flicker state continuity (V50).

No MORAI runtime is required; tests statically inspect the worker and check
the observed sequence from the 18:42 CSV.
"""
import ast
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
WORKER = ROOT / "src" / "lidar" / "static_avoidance_module.py"

class GPSFlickerV50(unittest.TestCase):
    def test_worker_syntax(self):
        ast.parse(WORKER.read_text(encoding="utf-8"))

    def test_short_gps_fix_does_not_reset_mission_immediately(self):
        src = WORKER.read_text(encoding="utf-8")
        self.assertNotIn("if not gps_shadow and shadow_was_active:", src)
        self.assertIn("shadow_resume_grace_s = 8.0", src)
        self.assertIn("if gps_shadow:", src)
        self.assertIn("gps_normal_since = None", src)
        self.assertIn("time.monotonic() - gps_normal_since >=", src)

    def test_locked_shadow_path_is_not_cleared_by_each_gps_frame(self):
        src = WORKER.read_text(encoding="utf-8")
        gps_branch = src.split(
            "else:\n                    # GPS valid: original global static planner controls.",
            1)[1].split("raw_plan = planner.plan(", 1)[0]
        self.assertNotIn("shadow_locked_path = None", gps_branch)
        self.assertIn("check_wall_clearance(", src)

    def test_first_path_and_second_survey_still_available(self):
        src = WORKER.read_text(encoding="utf-8")
        self.assertIn("choose_template_detour(", src)
        self.assertIn("template_completed_count += 1", src)
        self.assertIn('template_active_id == "SECOND_RIGHT"', src)
        self.assertIn("plan_second_verified_return(", src)

if __name__ == "__main__":
    unittest.main()
