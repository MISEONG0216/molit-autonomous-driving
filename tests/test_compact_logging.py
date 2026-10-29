"""Regression checks for per-run compact logging, without MORAI dependencies."""
import ast
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]


class CompactLoggingTest(unittest.TestCase):
    def test_lidar_csv_is_opt_in(self):
        source = (ROOT / "src/lidar/static_avoidance_module.py").read_text(encoding="utf-8")
        ast.parse(source)
        self.assertIn('os.environ.get("STATIC_CSV_LOG", "0") == "1"', source)
        self.assertIn("if csv_enabled:", source)
        self.assertIn("if logfile is not None:", source)

    def test_main_csv_still_enabled(self):
        source = (ROOT / "src/main_integrated_v1.py").read_text(encoding="utf-8")
        ast.parse(source)
        self.assertIn('integrated_v1_{stamp}.csv', source)
        self.assertIn("log_writer.writerow(", source)

    def test_launcher_has_compact_and_detailed_mode(self):
        source = (ROOT / "scripts/run_integrated_v25.sh").read_text(encoding="utf-8")
        self.assertIn('SESSION_LOG="logs/session_${STAMP}.log"', source)
        self.assertIn('export STATIC_CSV_LOG=0', source)
        self.assertIn('export STATIC_CSV_LOG=1', source)
        self.assertIn('"--detailed-logs"', source)
        self.assertIn('"\${MAIN_ARGS[@]}"'.replace('\\', ''), source)
        self.assertNotIn("src/main_integrated_v1.py --drive", source)


if __name__ == "__main__":
    unittest.main()
