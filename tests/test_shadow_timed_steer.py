"""MORAI-only timed GPS shadow steering regression tests."""
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from integration.shadow_timed_steer import timed_shadow_steer


class TimedShadowSteeringTests(unittest.TestCase):
    def test_left_and_countersteer(self):
        expected = [
            (9.99, "KEEP_BEFORE_LEFT", None),
            (10.00, "FULL_LEFT_5S", 1.0),
            (14.99, "FULL_LEFT_5S", 1.0),
            (15.00, "CORRECT_RIGHT_3S", -1.0),
            (17.99, "CORRECT_RIGHT_3S", -1.0),
            (18.00, "KEEP_AFTER_LEFT", None),
        ]
        for t, phase, steer in expected:
            with self.subTest(t=t):
                self.assertEqual(timed_shadow_steer(t), (phase, steer))

    def test_right_fifteen_seconds_after_left_correction(self):
        expected = [
            (32.99, "KEEP_AFTER_LEFT", None),
            (33.00, "FULL_RIGHT_5S", -1.0),
            (37.99, "FULL_RIGHT_5S", -1.0),
            (38.00, "CORRECT_LEFT_3S", 1.0),
            (40.99, "CORRECT_LEFT_3S", 1.0),
            (41.00, "KEEP_AFTER_RIGHT", None),
        ]
        for t, phase, steer in expected:
            with self.subTest(t=t):
                self.assertEqual(timed_shadow_steer(t), (phase, steer))

    def test_limit_and_invalid(self):
        self.assertEqual(timed_shadow_steer(10, 0.22), ("FULL_LEFT_5S", 0.22))
        self.assertEqual(timed_shadow_steer(15, 0.22), ("CORRECT_RIGHT_3S", -0.22))
        self.assertEqual(timed_shadow_steer(33, 0.22), ("FULL_RIGHT_5S", -0.22))
        self.assertEqual(timed_shadow_steer(38, 0.22), ("CORRECT_LEFT_3S", 0.22))
        self.assertEqual(timed_shadow_steer(-1), ("INVALID", None))
        self.assertEqual(timed_shadow_steer(float("nan")), ("INVALID", None))
        self.assertEqual(timed_shadow_steer(12, 1.5), ("INVALID", None))


if __name__ == "__main__":
    unittest.main()
