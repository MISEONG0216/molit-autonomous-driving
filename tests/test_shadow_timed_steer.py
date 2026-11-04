"""MORAI-only time scheduler regression checks; no vehicle control."""
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from integration.shadow_timed_steer import timed_shadow_steer


class TimedShadowSteeringTests(unittest.TestCase):
    def test_left_starts_ten_seconds_and_runs_five_seconds(self):
        self.assertEqual(timed_shadow_steer(9.99), ("KEEP_BEFORE_LEFT", None))
        self.assertEqual(timed_shadow_steer(10.0), ("FULL_LEFT_5S", 1.0))
        self.assertEqual(timed_shadow_steer(14.99), ("FULL_LEFT_5S", 1.0))
        self.assertEqual(timed_shadow_steer(15.0), ("KEEP_AFTER_LEFT", None))

    def test_right_starts_fifteen_seconds_after_left_finishes(self):
        self.assertEqual(timed_shadow_steer(29.99), ("KEEP_AFTER_LEFT", None))
        self.assertEqual(timed_shadow_steer(30.0), ("FULL_RIGHT_5S", -1.0))
        self.assertEqual(timed_shadow_steer(34.99), ("FULL_RIGHT_5S", -1.0))
        self.assertEqual(timed_shadow_steer(35.0), ("KEEP_AFTER_RIGHT", None))

    def test_tunable_steering_limit_and_invalid_inputs(self):
        self.assertEqual(timed_shadow_steer(10.0, 0.22), ("FULL_LEFT_5S", 0.22))
        self.assertEqual(timed_shadow_steer(30.0, 0.22), ("FULL_RIGHT_5S", -0.22))
        self.assertEqual(timed_shadow_steer(-1), ("INVALID", None))
        self.assertEqual(timed_shadow_steer(float("nan")), ("INVALID", None))
        self.assertEqual(timed_shadow_steer(12, 1.5), ("INVALID", None))


if __name__ == "__main__":
    unittest.main()
