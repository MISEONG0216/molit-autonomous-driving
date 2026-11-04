"""Stationary tunnel survey geometry regression (no MORAI hardware required)."""
import ast
import sys
import unittest
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/"src/lidar"))
from capture_tunnel_stationary import vehicle_frame, summarize_cloud

class SurveyTests(unittest.TestCase):
    def test_vehicle_frame_axes(self):
        src=np.array([[2.0,5.0,1.0,.3],[-3.0,6.0,0.0,.7]])
        out=vehicle_frame(src)
        np.testing.assert_allclose(out,
            [[5.0,-2.0,1.0,.3],[6.0,3.0,0.0,.7]])

    def test_sector_summary(self):
        pts=np.array([[5.,0.,0.,1.],[6.,2.,0.,1.],
                      [7.,-2.,0.,1.],[9.,0.,2.,1.]])
        s=summarize_cloud(pts)
        self.assertEqual(s["center_points"],1)
        self.assertEqual(s["left_points"],1)
        self.assertEqual(s["right_points"],1)
        self.assertEqual(s["nearest_center_forward_m"],5.)

    def test_sensor_only_no_controls(self):
        p=ROOT/"src/lidar/capture_tunnel_stationary.py"
        source=p.read_text(encoding="utf-8")
        ast.parse(source)
        self.assertNotIn("EgoCtrlCmd", source)
        self.assertNotIn("run_integrated_windows.bat --drive", source)

if __name__=="__main__":
    unittest.main()
