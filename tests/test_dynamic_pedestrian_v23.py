#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""V23 no-MORAI tests: dynamic pedestrian zone and risk command boundary."""
import ast
import json
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from integration.dynamic_pedestrian_io import (
    load_dynamic_zones, in_dynamic_zone, parse_pedestrian_message,
    PedestrianStopLatch,
)
from dynamic_pedestrian_worker import JpegStreamAssembler


def packet(frame=1, status="CLEAR", wp=450):
    return {
        "status": status,
        "decision": "EMERGENCY_STOP" if status == "STOP_REQUIRED" else "DRIVE_FORWARD",
        "risk": .8 if status == "STOP_REQUIRED" else .1,
        "person_count": 1 if status == "STOP_REQUIRED" else 0,
        "frame": frame,
        "waypoint_index": wp,
    }


class DynamicZoneTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.missions = json.loads(
            (ROOT / "config/integration_missions.json").read_text(encoding="utf-8")
        )
        cls.network = json.loads(
            (ROOT / "config/integration_network.json").read_text(encoding="utf-8")
        )

    def test_dynamic_zone_exact_setting(self):
        z = load_dynamic_zones(self.missions["dynamic_pedestrian"], 4430)
        self.assertEqual(z, ((430, 580),))
        self.assertTrue(in_dynamic_zone(430, z))
        self.assertTrue(in_dynamic_zone(500, z))
        self.assertTrue(in_dynamic_zone(580, z))
        for index in (0, 414, 429, 581, 620, 635, 862, 1224):
            self.assertFalse(in_dynamic_zone(index, z))

    def test_dynamic_and_static_nonoverlap(self):
        dynamic = load_dynamic_zones(self.missions["dynamic_pedestrian"], 4430)
        static = self.missions["static"]["avoidance_zones_wp"]
        for idx in range(4430):
            self.assertFalse(
                in_dynamic_zone(idx, dynamic) and
                any(start <= idx <= end for start, end in static)
            )

    def test_invalid_zones_fail_closed(self):
        bad = ([], None, [[-1, 1]], [[3, 2]], [[0, 5000]],
               [[430, "580"]], [[430, 580], [500, 600]],
               [[430, 580], [580, 600]], [[True, 10]])
        for x in bad:
            with self.subTest(value=x):
                with self.assertRaises(ValueError):
                    load_dynamic_zones({"zones_wp": x}, 4430)

    def test_localhost_relay_and_separate_results(self):
        n = self.network
        for key in ("pedestrian_camera_relay", "pedestrian_result", "pedestrian_gate"):
            self.assertEqual(n[key]["bind_ip"], "127.0.0.1")
        ports = (
            n["pedestrian_camera_relay"]["port"],
            n["pedestrian_result"]["port"],
            n["pedestrian_gate"]["port"],
            n["traffic_result"]["port"], n["static_result"]["port"],
            n["control"]["port"],
        )
        self.assertEqual(len(set(ports)), len(ports))


class RiskProtocolTests(unittest.TestCase):
    def parse(self, obj):
        return parse_pedestrian_message(json.dumps(obj).encode("utf-8"))

    def test_stop_packet(self):
        got = self.parse(packet(frame=11, status="STOP_REQUIRED"))
        self.assertTrue(got["valid"])
        self.assertEqual(got["status"], "STOP_REQUIRED")
        self.assertEqual(got["waypoint_index"], 450)

    def test_clear_packet(self):
        got = self.parse(packet(frame=12))
        self.assertTrue(got["valid"])
        self.assertEqual(got["status"], "CLEAR")

    def test_malformed_or_out_of_range_packet(self):
        for change in (
            {"status": "AVOIDING"}, {"risk": -1}, {"risk": 2},
            {"person_count": -1}, {"person_count": 1.1},
            {"frame": "5"}, {"frame": -1},
            {"waypoint_index": -1}, {"waypoint_index": "450"},
            {"waypoint_index": 4430},
            {"status": "STOP_REQUIRED", "decision": "DRIVE_FORWARD"},
        ):
            with self.subTest(change=change):
                bad = packet()
                bad.update(change)
                self.assertFalse(self.parse(bad)["valid"])
        missing = packet()
        missing.pop("waypoint_index")
        self.assertFalse(self.parse(missing)["valid"])


class PedestrianStopLatchTests(unittest.TestCase):
    def setUp(self):
        self.latch = PedestrianStopLatch(clear_confirm_frames=3)

    def p(self, frame, status):
        return parse_pedestrian_message(
            json.dumps(packet(frame, status)).encode()
        )

    def test_hazard_latches_immediately(self):
        self.latch.update(self.p(1, "STOP_REQUIRED"), True, True)
        self.assertTrue(self.latch.latched)

    def test_three_distinct_fresh_clear_frames_needed(self):
        self.latch.update(self.p(1, "STOP_REQUIRED"), True, True)
        self.latch.update(self.p(2, "CLEAR"), True, True)
        self.assertTrue(self.latch.latched)
        self.latch.update(self.p(2, "CLEAR"), False, True)
        self.assertEqual(self.latch.clear_count, 1)
        self.latch.update(self.p(3, "CLEAR"), True, True)
        self.assertTrue(self.latch.latched)
        self.latch.update(self.p(4, "CLEAR"), True, True)
        self.assertFalse(self.latch.latched)

    def test_stale_data_never_releases_stop(self):
        self.latch.update(self.p(1, "STOP_REQUIRED"), True, True)
        self.latch.update(self.p(2, "CLEAR"), True, False)
        self.assertTrue(self.latch.latched)
        self.assertEqual(self.latch.clear_count, 0)

    def test_new_hazard_resets_clear_counter(self):
        self.latch.update(self.p(1, "STOP_REQUIRED"), True, True)
        self.latch.update(self.p(2, "CLEAR"), True, True)
        self.latch.update(self.p(3, "STOP_REQUIRED"), True, True)
        self.assertEqual(self.latch.clear_count, 0)

    def test_zone_exit_reset(self):
        self.latch.update(self.p(1, "STOP_REQUIRED"), True, True)
        self.latch.reset()
        self.assertFalse(self.latch.latched)
        self.assertEqual(self.latch.last_frame, -1)


class CameraAssemblerTests(unittest.TestCase):
    def test_reassembles_jpeg_split_across_packets(self):
        jpeg = bytes((255, 216)) + b"PAYLOAD" + bytes((255, 217))
        assembler = JpegStreamAssembler()
        self.assertIsNone(assembler.feed([jpeg[:4]]))
        self.assertEqual(assembler.feed([jpeg[4:]]), jpeg)

    def test_rejects_unfinished_or_corrupt_jpeg(self):
        assembler = JpegStreamAssembler()
        self.assertIsNone(assembler.feed([b"garbage", bytes((255, 216))]))

    def test_worker_and_main_syntax(self):
        for path in (
            "src/dynamic_pedestrian_worker.py",
            "src/traffic_light_detector.py",
            "src/main_integrated_v1.py",
            "src/integration/dynamic_pedestrian_io.py",
        ):
            ast.parse((ROOT / path).read_text(encoding="utf-8-sig"), filename=path)


if __name__ == "__main__":
    unittest.main()
