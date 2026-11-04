"""V51: retain V48 first LEFT; second RIGHT holds until LiDAR-clear release then camera."""
import ast
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
WORKER = ROOT / "src/lidar/static_avoidance_module.py"


class V51Tests(unittest.TestCase):
    def test_worker_syntax(self):
        ast.parse(WORKER.read_text(encoding="utf-8"))

    def test_no_forced_left_return_after_second_right(self):
        s = WORKER.read_text(encoding="utf-8")
        self.assertNotIn("plan_second_verified_return(", s)
        self.assertNotIn("EGO_LIDAR_NEW_SECOND_RETURN", s)
        self.assertIn("handoff=CAMERA (fresh LiDAR ahead clear)", s)
        self.assertIn('if template_active_id is not None:', s)

    def test_handoff_still_requires_end_position_and_fresh_front_clear(self):
        s = WORKER.read_text(encoding="utf-8")
        self.assertIn("ready_for_release = (", s)
        self.assertIn("nearest >= len(locked_xy)-tail_points", s)
        self.assertIn("end_distance <=", s)
        self.assertIn('if len(front) < int(ego_cfg.get(', s)
        self.assertIn('EGO_LIDAR_TAIL_BLOCKED', s)
        self.assertIn("check_wall_clearance(", s)
        self.assertIn("close_forward_collision(", s)

    def test_first_left_and_survey_trigger_intact(self):
        s = WORKER.read_text(encoding="utf-8")
        self.assertIn("choose_template_detour(", s)
        self.assertIn("template_completed_count += 1", s)
        self.assertIn("second_match_streak += 1", s)
        self.assertIn("shadow_resume_grace_s = 8.0", s)

if __name__ == "__main__":
    unittest.main()
