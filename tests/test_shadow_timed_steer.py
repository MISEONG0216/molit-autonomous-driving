"""Pure stage timing checks; no vehicle control."""
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from integration.shadow_timed_steer import timed_shadow_steer


class TimedShadowSteeringTests(unittest.TestCase):
    def test_left_at_four_seconds_for_one_second(self):
        self.assertEqual(timed_shadow_steer(3.99), ("KEEP_BEFORE_LEFT", None))
        self.assertEqual(timed_shadow_steer(4.00), ("FULL_LEFT_1S", 0.22))
        self.assertEqual(timed_shadow_steer(4.99), ("FULL_LEFT_1S", 0.22))
        self.assertEqual(timed_shadow_steer(5.00), ("KEEP_AFTER_LEFT", None))

    def test_right_at_fourteen_seconds_for_one_second(self):
        self.assertEqual(timed_shadow_steer(13.99), ("KEEP_AFTER_LEFT", None))
        self.assertEqual(timed_shadow_steer(14.00), ("FULL_RIGHT_1S", -0.22))
        self.assertEqual(timed_shadow_steer(14.99), ("FULL_RIGHT_1S", -0.22))
        self.assertEqual(timed_shadow_steer(15.00), ("KEEP_AFTER_RIGHT", None))

    def test_invalid_time(self):
        self.assertEqual(timed_shadow_steer(-1), ("INVALID", None))
        self.assertEqual(timed_shadow_steer(float("nan")), ("INVALID", None))


if __name__ == "__main__":
    unittest.main()
