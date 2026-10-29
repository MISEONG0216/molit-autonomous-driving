import ast
import json
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from integration.merge_side_io import (
    JpegAssembler, in_merge_zone, vehicle_risk, format_result,
)
from integration.mission_io import parse_merge_message


class MergeSideV25Test(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.cfg = json.loads(
            (ROOT / "config/integration_missions.json").read_text(encoding="utf-8")
        )["merge"]

    def test_user_mapped_merge_zone_boundaries(self):
        for wp in (1770, 1820, 2480, 2560, 3050, 3190):
            self.assertTrue(in_merge_zone(wp, self.cfg))
        for wp in (1769, 1821, 2479, 2561, 3049, 3191):
            self.assertFalse(in_merge_zone(wp, self.cfg))
        self.assertTrue(in_merge_zone(10, self.cfg, True))
        self.assertFalse(in_merge_zone(None, self.cfg, True))

    def test_risk_increases_for_large_or_growing_vehicle(self):
        small = vehicle_risk(10, 20, 30, 50, 640, 480)
        large = vehicle_risk(10, 20, 250, 310, 640, 480)
        approaching = vehicle_risk(10, 20, 60, 100, 640, 480, 0.0011)
        self.assertLess(small, 0.6)
        self.assertGreater(large, small)
        self.assertGreater(approaching, small)

    def test_result_is_compatible_with_existing_main_receiver(self):
        red = parse_merge_message(format_result(0.80, 0.10, "MERGE_REJECT"))
        self.assertTrue(red["valid"])
        self.assertEqual(red["decision"], "MERGE_REJECT")
        self.assertAlmostEqual(red["max_risk"], 0.8)
        green = parse_merge_message(format_result(0, 0, "MERGE_ACCEPT"))
        self.assertTrue(green["valid"])
        self.assertEqual(green["decision"], "MERGE_ACCEPT")

    def test_fragmented_camera_jpeg(self):
        a = JpegAssembler()
        self.assertIsNone(a.feed(bytes((0, 255, 216)) + b"abcd"))
        self.assertEqual(a.feed(b"ef" + bytes((255, 217))),
                         bytes((255, 216)) + b"abcdef" + bytes((255, 217)))

    def test_worker_never_sends_vehicle_control(self):
        source = (ROOT / "src/merge_side_camera_worker.py").read_text(encoding="utf-8")
        ast.parse(source)
        self.assertNotIn("EgoCtrlCmd(", source)
        self.assertNotIn("send_stop(", source)

    def test_main_has_waypoint_merge_gate(self):
        source = (ROOT / "src/main_integrated_v1.py").read_text(encoding="utf-8")
        ast.parse(source)
        self.assertIn('network["merge_gate"]', source)
        self.assertIn('"merge_all": bool(args.merge_all)', source)


if __name__ == "__main__":
    unittest.main()
