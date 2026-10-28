#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Pedestrian risk UDP boundary and fail-safe stop latch (no control sender).

Only the main controller can send EgoCtrlCmd. This module accepts a risk
decision computed by the independent front-camera YOLO+MiDaS worker.
"""
import json
import math


def load_dynamic_zones(config, waypoint_count):
    raw = config.get("zones_wp")
    if not isinstance(raw, list) or not raw:
        raise ValueError("dynamic_pedestrian.zones_wp must be a nonempty list")
    zones = []
    for value in raw:
        if (not isinstance(value, list) or len(value) != 2
                or any(type(x) is not int for x in value)):
            raise ValueError("Dynamic zones must be [start_wp, end_wp]")
        a, b = value
        if not 0 <= a <= b < waypoint_count:
            raise ValueError("Out-of-range dynamic pedestrian waypoint zone")
        zones.append((a, b))
    zones.sort()
    if any(b[0] <= a[1] for a, b in zip(zones, zones[1:])):
        raise ValueError("Overlapping pedestrian zones")
    return tuple(zones)


def in_dynamic_zone(index, zones):
    return any(start <= index <= end for start, end in zones)


def parse_pedestrian_message(raw):
    base = {
        "valid": False, "status": "UNKNOWN", "risk": 0.0,
        "person_count": 0, "frame": -1, "decision": "UNKNOWN",
    }
    if not raw or len(raw) > 2048:
        return base
    try:
        value = json.loads(raw.decode("utf-8"))
        if not isinstance(value, dict):
            return base
        status = value.get("status")
        if status not in ("CLEAR", "STOP_REQUIRED"):
            return base
        risk = float(value["risk"])
        count, frame = value["person_count"], value["frame"]
        if (not math.isfinite(risk) or not 0.0 <= risk <= 1.0
                or type(count) is not int or not 0 <= count <= 1000
                or type(frame) is not int or frame < 0):
            return base
        decision = value.get("decision")
        if (status == "STOP_REQUIRED" and decision != "EMERGENCY_STOP") or (
                status == "CLEAR" and decision != "DRIVE_FORWARD"):
            return base
        return {
            "valid": True, "status": status, "risk": risk,
            "person_count": count, "frame": frame, "decision": decision,
        }
    except (ValueError, TypeError, KeyError, UnicodeError, AttributeError):
        return base


class PedestrianStopLatch:
    """A STOP persists until N distinct fresh CLEAR camera results arrive."""

    def __init__(self, clear_confirm_frames=3):
        if type(clear_confirm_frames) is not int or clear_confirm_frames < 1:
            raise ValueError("clear_confirm_frames must be a positive integer")
        self.required = clear_confirm_frames
        self.latched = False
        self.clear_count = 0
        self.last_frame = -1

    def reset(self):
        self.latched = False
        self.clear_count = 0
        self.last_frame = -1

    def update(self, packet, new_result, fresh):
        if not fresh or not packet["valid"]:
            # Never clear an existing STOP without fresh perception.
            self.clear_count = 0
            return
        frame = packet["frame"]
        if not new_result or frame <= self.last_frame:
            return
        self.last_frame = frame
        if packet["status"] == "STOP_REQUIRED":
            self.latched = True
            self.clear_count = 0
        elif self.latched and packet["status"] == "CLEAR":
            self.clear_count += 1
            if self.clear_count >= self.required:
                self.latched = False
                self.clear_count = 0

