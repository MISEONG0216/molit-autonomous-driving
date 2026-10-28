#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Configurable global-waypoint gates for the static LiDAR mission.

Indices are inclusive and refer to the supplied competition global path.
Only the static mission uses these gates; traffic and merge are unaffected.
"""


def load_static_zones(static_config, waypoint_count):
    if waypoint_count <= 0:
        raise ValueError("Global path has no waypoints")
    raw = static_config.get("avoidance_zones_wp")
    if not isinstance(raw, list) or not raw:
        raise ValueError("static.avoidance_zones_wp must be a nonempty list")
    zones = []
    for pair in raw:
        if (
            not isinstance(pair, list) or len(pair) != 2
            or any(type(n) is not int for n in pair)
        ):
            raise ValueError("Each static zone must be [start_wp, end_wp]")
        start, end = pair
        if not (0 <= start <= end < waypoint_count):
            raise ValueError(
                f"Invalid static zone [{start}, {end}] for {waypoint_count} waypoints"
            )
        zones.append((start, end))
    zones.sort()
    for a, b in zip(zones, zones[1:]):
        if b[0] <= a[1]:
            raise ValueError("Static avoidance zones must not overlap")
    return tuple(zones)


def in_static_zone(waypoint_index, zones):
    return any(start <= waypoint_index <= end for start, end in zones)
