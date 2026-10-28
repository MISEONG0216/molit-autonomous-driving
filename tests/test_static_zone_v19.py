#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Regression tests for V19 designated static-obstacle waypoint gating."""
import ast
import json
import unittest
from pathlib import Path

from src.integration.static_zone import load_static_zones, in_static_zone
from src.integration.static_avoidance_io import parse_static_message

ROOT = Path(__file__).resolve().parents[1]


class ZoneConfigurationTests(unittest.TestCase):
    def setUp(self):
        self.config = json.loads(
            (ROOT / "config" / "integration_missions.json").read_text(encoding="utf-8")
        )["static"]

    def test_configured_static_zone_from_confirmed_run(self):
        zones = load_static_zones(self.config, 4430)
        self.assertTrue(in_static_zone(635, zones))
        self.assertTrue(in_static_zone(697, zones))
        self.assertFalse(in_static_zone(862, zones))
        self.assertFalse(in_static_zone(1270, zones))

    def test_zone_edges_are_inclusive(self):
        zones = load_static_zones({"avoidance_zones_wp": [[590, 760]]}, 4430)
        for index in (590, 650, 760):
            self.assertTrue(in_static_zone(index, zones))
        for index in (589, 761, 862):
            self.assertFalse(in_static_zone(index, zones))

    def test_multi_zone_supported(self):
        zones = load_static_zones(
            {"avoidance_zones_wp": [[2000, 2010], [590, 760]]}, 4430
        )
        self.assertEqual(zones, ((590, 760), (2000, 2010)))
        self.assertTrue(in_static_zone(2005, zones))

    def test_malformed_zones_fail_fast(self):
        invalid = (None, [], [[760, 590]], [[-1, 20]], [[0, 5000]],
                   [[590, "760"]], [[590, 760], [700, 800]],
                   [[590, 760], [760, 900]], [[True, 760]])
        for entry in invalid:
            with self.subTest(entry=entry):
                with self.assertRaises(ValueError):
                    load_static_zones({"avoidance_zones_wp": entry}, 4430)

    def test_default_config_contains_no_global_static_control(self):
        zones = load_static_zones(self.config, 4430)
        self.assertFalse(in_static_zone(0, zones))
        self.assertFalse(in_static_zone(4429, zones))


class WorkerProtocolTests(unittest.TestCase):
    def test_outside_zone_clear_message(self):
        msg = {
            "status": "CLEAR", "recommended_speed_kmh": 0,
            "waypoint_index": 862, "in_static_zone": False,
            "latch_active": False, "zone_active": False,
            "latch_state": "ZONE_OFF",
        }
        parsed = parse_static_message(json.dumps(msg).encode())
        self.assertTrue(parsed["valid"])
        self.assertFalse(parsed["zone_active"])
        self.assertFalse(parsed["latch_active"])
        self.assertEqual(parsed["waypoint_index"], 862)

    def test_active_route_can_extend_past_zone_boundary(self):
        msg = {
            "status": "AVOIDING", "recommended_speed_kmh": 8,
            "waypoint_index": 765, "in_static_zone": False,
            "latch_active": True, "zone_active": True,
            "latch_state": "LOCKED 50/60",
            "path": [[0, 0], [1, 0]],
        }
        parsed = parse_static_message(json.dumps(msg).encode())
        self.assertTrue(parsed["valid"])
        self.assertTrue(parsed["latch_active"])
        self.assertTrue(parsed["zone_active"])

    def test_invalid_zone_metadata_rejected(self):
        for update in (
            {"zone_active": "yes"}, {"latch_active": 1},
            {"in_static_zone": 0}, {"waypoint_index": -1},
            {"waypoint_index": "635"},
        ):
            msg = {"status": "CLEAR", "recommended_speed_kmh": 0, **update}
            self.assertFalse(parse_static_message(json.dumps(msg).encode())["valid"])

    def test_all_three_touched_sources_parse(self):
        for rel in (
            "src/integration/static_zone.py",
            "src/integration/static_avoidance_io.py",
            "src/main_integrated_v1.py",
            "src/lidar/static_avoidance_module.py",
        ):
            with self.subTest(path=rel):
                ast.parse((ROOT / rel).read_text(encoding="utf-8-sig"), filename=rel)


if __name__ == "__main__":
    unittest.main()
