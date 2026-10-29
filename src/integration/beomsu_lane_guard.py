#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Conservative, opt-in waypoint-local lateral path bias for V20.

RIGHT is positive, LEFT is negative, relative to forward path direction.
Only the Pure Pursuit tracking-path copy changes: official global waypoints,
traffic/mission indices, curvature speed profile and LiDAR worker remain intact.
An offset cannot prove wheel-to-marking or wall clearance; MORAI video and
lane/wall measurements are required before accepting a tune.
"""
import math

import numpy as np

MAX_ABS_OFFSET_M = 0.50
FIRST_ZONE = (1480, 1530, 1655, 1715)
EXTRA_RAMP_WP = 35
TEST_SPEED_LIMIT_KMH = 20.0


def _smoothstep(t):
    t = min(1.0, max(0.0, float(t)))
    return t * t * (3.0 - 2.0 * t)


def parse_extra_zone(spec):
    """START:END:SIGNED_METRES; +right, -left, with 35-WP edge ramps."""
    try:
        parts = str(spec).split(":")
        if len(parts) != 3:
            raise ValueError("expected START:END:SIGNED_METRES")
        start, end = int(parts[0]), int(parts[1])
        amount = float(parts[2])
    except (TypeError, ValueError) as exc:
        raise ValueError("Bad --beomsu-extra-zone %r: expected START:END:SIGNED_METRES" % spec) from exc
    if end - start <= 2 * EXTRA_RAMP_WP or start < 3:
        raise ValueError("Extra zone must start at WP>=3 and be longer than 70 WP")
    return start, start + EXTRA_RAMP_WP, end - EXTRA_RAMP_WP, end, amount


def make_zones(path_length, first_right_m=0.0, extra_specs=()):
    if not math.isfinite(float(first_right_m)) or not 0.0 <= first_right_m <= MAX_ABS_OFFSET_M:
        raise ValueError("first-right offset must be finite and within 0..0.50 m")
    zones = []
    if first_right_m:
        zones.append((*FIRST_ZONE, float(first_right_m)))
    for spec in extra_specs:
        zones.append(parse_extra_zone(spec))
    for start, full_begin, full_end, end, amount in zones:
        if not (3 <= start < full_begin <= full_end < end <= path_length - 4):
            raise ValueError("Offset zone waypoint range is outside path")
        if not math.isfinite(amount) or abs(amount) > MAX_ABS_OFFSET_M or amount == 0.0:
            raise ValueError("Offset magnitude must be nonzero and <= 0.50m")
    zones.sort(key=lambda z: z[0])
    for previous, current in zip(zones, zones[1:]):
        if previous[3] >= current[0]:
            raise ValueError("Offset zones must not overlap or touch")
    return tuple(zones)


def zone_weight(wp, zone):
    start, full_begin, full_end, end, _amount = zone
    if wp <= start or wp >= end:
        return 0.0
    if wp < full_begin:
        return _smoothstep((wp - start) / float(full_begin - start))
    if wp <= full_end:
        return 1.0
    return 1.0 - _smoothstep((wp - full_end) / float(end - full_end))


def offset_at_waypoint(wp, zones):
    return sum(zone[4] * zone_weight(wp, zone) for zone in zones)


def make_tracking_path(official_path, zones):
    """Return a separate Nx2 copy with smooth lateral offsets only in zones."""
    original = np.asarray(official_path, dtype=float)
    if original.ndim != 2 or original.shape[1] < 2:
        raise ValueError("Official path shape must be (N, >=2)")
    if not np.all(np.isfinite(original[:, :2])):
        raise ValueError("Official path must have finite XY coordinates")
    # Validate even when a caller bypassed make_zones.
    for z in zones:
        if len(z) != 5 or not (3 <= z[0] < z[1] <= z[2] < z[3] <= len(original) - 4):
            raise ValueError("Invalid waypoint offset zone")
        if not math.isfinite(z[4]) or abs(z[4]) > MAX_ABS_OFFSET_M:
            raise ValueError("Invalid waypoint offset magnitude")
    tracking = original.copy()
    for start, _begin, _hold, end, amount in zones:
        for wp in range(start + 1, end):
            shift = amount * zone_weight(wp, (start, _begin, _hold, end, amount))
            if shift == 0.0:
                continue
            tangent = original[wp + 3, :2] - original[wp - 3, :2]
            norm = math.hypot(float(tangent[0]), float(tangent[1]))
            if norm < 1e-8:
                raise ValueError("Zero-length tangent at offset WP%d" % wp)
            # Right-hand unit normal to the forward tangent.
            tracking[wp, 0] += shift * tangent[1] / norm
            tracking[wp, 1] -= shift * tangent[0] / norm
    return tracking
