"""V31 regression: roundabout RIGHT-half-only LEFT9293 ROI scope and source."""
import ast
import json
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from integration.merge_side_io import merge_camera_mode, active_merge_cameras


class RightHalfRoundaboutTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.cfg = json.loads(
            (ROOT / "config/integration_missions.json").read_text(encoding="utf-8")
        )["merge"]

    def test_roi_config(self):
        self.assertAlmostEqual(self.cfg["roundabout_left_camera_min_x_ratio"], 0.5)
        self.assertEqual(merge_camera_mode(1700, self.cfg), "roundabout_prearm")
        self.assertEqual(merge_camera_mode(1770, self.cfg), "roundabout")
        self.assertEqual(active_merge_cameras("roundabout"), ("left",))
        self.assertEqual(active_merge_cameras("roundabout_done"), ())
        self.assertEqual(active_merge_cameras("highway_merge"), ("left", "left_rear"))

    def test_crop_only_left_roundabout(self):
        path = ROOT / "src/merge_side_camera_worker.py"
        source = path.read_text(encoding="utf-8")
        ast.parse(source)
        self.assertIn('side == "left" and mode in', source)
        self.assertIn('("roundabout", "roundabout_prearm")', source)
        self.assertIn('inference_frame = frame[:, crop_x:] if roundabout_roi else frame',source)
        self.assertIn('x1 += crop_x',source)
        self.assertIn('x2 += crop_x',source)
        self.assertIn('LEFT HALF IGNORED',source)
        self.assertNotIn('EgoCtrlCmd(',source)

if __name__ == "__main__":
    unittest.main()
