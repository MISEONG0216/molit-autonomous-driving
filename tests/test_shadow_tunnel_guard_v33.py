"""Guard against executing global-XY LiDAR detours into tunnel walls on GPS loss."""
import ast
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]


class TunnelSafetyGuardTest(unittest.TestCase):
    def test_shadow_avoidance_refused_before_local_pp(self):
        source = (ROOT / "src/main_integrated_v1.py").read_text(encoding="utf-8")
        ast.parse(source)
        start = source.index('elif static_state == "AVOIDING":')
        end = source.index('if static_stop:', start)
        block = source[start:end]
        self.assertIn('if shadow_active and not static_plan.get("shadow_wall_verified", False):', block)
        self.assertIn('static_state = "SHADOW_WALL_NOT_VERIFIED"', block)
        self.assertIn("static_stop = True", block)
        self.assertIn("else:", block)
        self.assertIn("local_path_pure_pursuit(", block)
        # The local steering call must be nested inside the non-shadow branch.
        self.assertIn('                    else:\n                        target_speed_kmh', block)
        self.assertIn('                        local_pp = local_path_pure_pursuit(', block)

    def test_full_brake_and_normal_gps_remain(self):
        main = (ROOT / "src/main_integrated_v1.py").read_text(encoding="utf-8")
        self.assertIn('elif static_active:', main)
        self.assertIn('mission["reasons"].append("static_stop_or_stale")', main)
        self.assertIn('or (args.static_control and static_stop) or pedestrian_stop or shadow_stop:', main)
        self.assertEqual(main.count("ctrl_sender.send(cmd)"), 1)


if __name__ == "__main__":
    unittest.main()
