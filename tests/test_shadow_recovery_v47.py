"""V47 shadow recovery: timeout never substitutes for clear LiDAR evidence."""
import json
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from integration.shadow_recovery_gate import ShadowRecoveryGate
from integration.static_avoidance_io import parse_static_message


def msg(reason="RETURN_CORRIDOR_UNOBSERVED", clear=True, scan=1):
    return {"status": "STOP_REQUIRED", "reason": reason,
            "forward_clear_verified": clear, "scan": scan}


class RecoveryGateTests(unittest.TestCase):
    def test_eight_seconds_and_three_fresh_scans(self):
        g = ShadowRecoveryGate(8.0, 3)
        self.assertEqual(g.update(True, True, msg(scan=1), True, now=0)[0], "COUNTDOWN")
        self.assertEqual(g.update(True, True, msg(scan=2), True, now=4)[1], False)
        self.assertEqual(g.update(True, True, msg(scan=3), True, now=7.9)[1], False)
        self.assertEqual(g.update(True, True, msg(scan=3), True, now=8)[0], "RECOVERY_ELIGIBLE")

    def test_stale_lidar_never_authorizes_recovery(self):
        g = ShadowRecoveryGate(8, 1)
        g.update(True, True, msg(), True, now=0)
        self.assertFalse(g.update(True, False, msg(), True, now=9)[1])

    def test_observed_danger_never_authorizes_recovery(self):
        g = ShadowRecoveryGate(8, 1)
        g.update(True, True, msg(), True, now=0)
        self.assertFalse(g.update(True, True, msg("IMMEDIATE_OBSTACLE"), True, now=9)[1])
        self.assertFalse(g.update(True, True, msg("SHADOW_DYNAMIC_TRACK_STOP"), True, now=18)[1])
        self.assertFalse(g.update(True, True, msg("EGO_LOCKED_BLOCKED:WALL_OR_OBSTACLE_IN_SWEPT_PATH"), True, now=28)[1])

    def test_missing_forward_evidence_and_lane_never_authorize(self):
        g = ShadowRecoveryGate(8, 1)
        g.update(True, True, msg(clear=False), True, now=0)
        self.assertFalse(g.update(True, True, msg(clear=False, scan=2), True, now=9)[1])
        self.assertFalse(g.update(True, True, msg(clear=True, scan=3), False, now=10)[1])

    def test_shadow_exit_resets_timer(self):
        g = ShadowRecoveryGate(8, 1)
        g.update(True, True, msg(), True, now=0)
        self.assertFalse(g.update(False, True, msg(), True, now=9)[1])
        self.assertFalse(g.update(True, True, msg(scan=2), True, now=10)[1])

    def test_worker_message_boolean_is_validated(self):
        data = {"status": "STOP_REQUIRED", "path": [],
                "reason": "RETURN_CORRIDOR_UNOBSERVED",
                "zone_active": True, "recommended_speed_kmh": 0,
                "forward_clear_verified": True}
        self.assertTrue(parse_static_message(json.dumps(data).encode())["forward_clear_verified"])
        data["forward_clear_verified"] = "True"
        self.assertFalse(parse_static_message(json.dumps(data).encode())["valid"])


if __name__ == "__main__":
    unittest.main()
