"""V49 second-survey matching and two-stage RIGHT hold/return tests."""
import ast
import json
import sys
import unittest
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src/lidar"))
from shadow_predefined_templates import (
    choose_template_detour, plan_second_verified_return,
)
CFG = json.loads((ROOT / "config/tunnel_predefined_templates.json").read_text(
    encoding="utf-8"))
WALL = json.loads((ROOT / "config/integration_missions.json").read_text(
    encoding="utf-8"))["gps_shadow_wall_guard"]


def survey_cloud(front=17.5, obstacle_left=(0.4, 3.4),
                 left_wall=6.0, right_wall=-6.0,
                 walls_end=27.0, include_object=True):
    xx = np.arange(2.0, walls_end, .25)
    side = [
        np.column_stack((xx, np.full(len(xx), left_wall), np.zeros(len(xx)))),
        np.column_stack((xx, np.full(len(xx), right_wall), np.zeros(len(xx)))),
    ]
    if include_object:
        xs, ys = np.meshgrid(
            np.linspace(front, front+.35, 8),
            np.linspace(obstacle_left[0], obstacle_left[1], 12),
        )
        side.append(np.column_stack((
            xs.ravel(), ys.ravel(), np.zeros(xs.size)
        )))
    return np.vstack(side)


class SecondSurveyV49Tests(unittest.TestCase):
    def test_right_hold_triggered_from_survey_at_17m(self):
        r = choose_template_detour(survey_cloud(), CFG, 1)
        self.assertEqual(r["status"], "AVOIDING", r["reason"])
        self.assertEqual(r["side"], -1)
        self.assertEqual(r["template_id"], "SECOND_RIGHT")
        path = np.asarray(r["path"])
        self.assertAlmostEqual(path[-1, 0], 23.0, places=4)
        self.assertAlmostEqual(path[-1, 1],
            -r["selected_offset_m"], places=4)
        self.assertEqual(r["selected_return_delay_m"], 0.0)

    def test_other_shape_wont_start_right_shift(self):
        r = choose_template_detour(
            survey_cloud(obstacle_left=(-2.0, -.6)), CFG, 1)
        self.assertEqual(r["status"], "CLEAR")
        self.assertNotEqual(r["reason"],
                            "SECOND_SURVEY_RIGHT_HOLD_VERIFIED")

    def test_missing_far_observation_rejected(self):
        r = choose_template_detour(
            survey_cloud(walls_end=18.0), CFG, 1)
        self.assertEqual(r["status"], "STOP_REQUIRED")
        self.assertEqual(r["reason"], "NO_VERIFIED_SECOND_HOLD")

    def test_wall_too_close_to_right_shift_rejected(self):
        r = choose_template_detour(
            survey_cloud(right_wall=-3.0), CFG, 1)
        self.assertEqual(r["status"], "STOP_REQUIRED")

    def test_first_template_config_and_path_code_preserved(self):
        self.assertEqual(CFG["templates"][0]["id"], "FIRST_LEFT")
        self.assertEqual(CFG["templates"][0]["side"], "LEFT")
        s=(ROOT/"src/lidar/shadow_predefined_templates.py").read_text(
            encoding="utf-8")
        self.assertIn("if completed_count == 1", s)
        self.assertIn("return choose_second_survey_hold", s)
        self.assertIn("base_return_start = last_x +", s)

    def test_second_return_is_separate_and_verified(self):
        # Ego at (0,0) after holding at right offset. A clean path heading
        # from current right side back to left is checked independently.
        r = plan_second_verified_return(
            survey_cloud(include_object=False), CFG, (0.,0.,0.), WALL, 2.4)
        self.assertEqual(r["status"], "AVOIDING", r["reason"])
        path = np.asarray(r["path"])
        self.assertAlmostEqual(path[-1,1], 2.4, places=4)

    def test_return_collision_blocks(self):
        pts = survey_cloud(include_object=False)
        blocked = np.column_stack((
            np.linspace(5., 7., 50),
            np.full(50, 1.1),
            np.zeros(50),
        ))
        r = plan_second_verified_return(
            np.vstack((pts,blocked)), CFG, (0.,0.,0.), WALL, 2.4)
        self.assertEqual(r["status"], "STOP_REQUIRED")

    def test_worker_state_machine_present(self):
        p=ROOT/"src/lidar/static_avoidance_module.py"
        source=p.read_text(encoding="utf-8")
        ast.parse(source)
        self.assertIn('template_active_id == "SECOND_RIGHT"', source)
        self.assertNotIn('template_active_id = "SECOND_RETURN"', source)
        self.assertIn("second_match_streak += 1", source)
        self.assertIn("handoff=CAMERA", source)
        self.assertIn("template_completed_count += 1", source)


if __name__ == "__main__":
    unittest.main()
