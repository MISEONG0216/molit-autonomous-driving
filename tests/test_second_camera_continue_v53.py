"""V53: keep CAMERA lane tracking until second LiDAR survey pattern matches."""
import ast
import json
import sys
import unittest
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src/lidar"))
from shadow_predefined_templates import choose_template_detour

CFG = json.loads((ROOT / "config/tunnel_predefined_templates.json").read_text(
    encoding="utf-8"))


def scan(obstacle_forward=17.5, obstacle_left=(-0.5, 2.5),
         left_wall=3.5, right_wall=-7.0, put_obstacle=True):
    xx = np.arange(2.0, 25.5, .25)
    clouds = [
        np.column_stack((xx, np.full(xx.shape, left_wall), np.zeros(xx.shape))),
        np.column_stack((xx, np.full(xx.shape, right_wall), np.zeros(xx.shape))),
    ]
    if put_obstacle:
        xs, ys = np.meshgrid(
            np.linspace(obstacle_forward, obstacle_forward+.35, 8),
            np.linspace(obstacle_left[0], obstacle_left[1], 12))
        clouds.append(np.column_stack(
            (xs.ravel(), ys.ravel(), np.zeros(xs.size))))
    return np.vstack(clouds)


class CameraContinueV53(unittest.TestCase):
    def test_code_syntax_and_no_old_mismatch_stop(self):
        s = (ROOT/"src/lidar/shadow_predefined_templates.py").read_text(
            encoding="utf-8")
        worker = (ROOT/"src/lidar/static_avoidance_module.py").read_text(
            encoding="utf-8")
        ast.parse(s);ast.parse(worker)
        self.assertNotIn('reason="SECOND_SURVEY_UNMATCHED_CLOSE"',s)
        self.assertNotIn('reason="SECOND_MATCH_WINDOW_MISSED"',s)
        self.assertNotIn('reason="SECOND_MATCH_CONFIRMING")',worker)
        self.assertIn('reason="SECOND_MATCH_CONFIRMING_CAMERA"',worker)

    def test_distance_mismatch_keeps_camera_following(self):
        r=choose_template_detour(
            scan(obstacle_forward=13.0), CFG, 1)
        self.assertEqual(r["status"],"CLEAR",r["reason"])
        self.assertEqual(r["path"],[])
        self.assertFalse(r["match"])

    def test_no_second_object_keeps_camera(self):
        r=choose_template_detour(scan(put_obstacle=False), CFG, 1)
        self.assertEqual(r["status"],"CLEAR",r["reason"])

    def test_second_same_wall_relative_pose_can_match(self):
        # The stored survey was taken from a different lateral ego pose.
        # With the ego shifted left, the left wall is 3.5 and object is
        # ~0.5, instead of wall~6/object~2; wall-object gap stays similar.
        r=choose_template_detour(scan(), CFG, 1)
        self.assertEqual(r["status"],"AVOIDING",r["reason"])
        self.assertEqual(r["side"],-1)
        self.assertAlmostEqual(r["path"][-1][1],-r["selected_offset_m"],places=5)

    def test_close_central_object_still_fails_stop(self):
        r=choose_template_detour(scan(
            obstacle_forward=5.0, obstacle_left=(-.4,.4)), CFG, 1)
        self.assertEqual(r["status"],"STOP_REQUIRED")
        self.assertEqual(r["reason"],"SECOND_APPROACH_TOO_CLOSE")

    def test_first_left_hold_unchanged(self):
        self.assertTrue(CFG["first_left_hold"]["enabled"])
        self.assertEqual(CFG["templates"][0]["id"],"FIRST_LEFT")

if __name__ == "__main__":
    unittest.main()
