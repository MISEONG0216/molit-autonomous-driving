#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Opt-in distance-based speed cap for V22 roundabout exit stability.

Purpose
-------
Preserve the normal validated V7 speed profile everywhere else.
Preview-brake towards the roundabout speed BEFORE the sharp exit turn,
hold it through the exit, and gradually allow normal acceleration after.

Plan, indexed against the UNMODIFIED official global path:
- WP < 1835: anticipatory cap from sqrt(v_hold**2 + 2*a_brake*distance)
- WP 1835..1892: speed cap == requested hold speed (12 km/h first test)
- WP > 1892: release cap from sqrt(v_hold**2 + 2*a_accel*distance)

These speeds are requested TARGET LIMITS; actual speed must be checked from
Competition Vehicle Status. The 0.9 and 0.8 m/s2 numbers are trial
trajectory parameters, not measured MORAI vehicle deceleration.

The official path, V7 reference profile, Pure Pursuit steering, mission
indices, LiDAR worker and signal recognition are not modified by this module.
"""

import math


HOLD_START_WP = 1835
HOLD_END_WP = 1892
PREVIEW_DECEL_MPS2 = 0.9
RELEASE_ACCEL_MPS2 = 0.8

MIN_TEST_HOLD_KMH = 8.0
MAX_TEST_HOLD_KMH = 20.0


def _cumulative_distances_m(path):
    lengths = [0.0] * len(path)
    for i in range(1, len(path)):
        dx = float(path[i][0]) - float(path[i - 1][0])
        dy = float(path[i][1]) - float(path[i - 1][1])
        lengths[i] = lengths[i - 1] + math.hypot(dx, dy)
    return lengths


def build_roundabout_speed_caps(official_path, max_speed_kmh, hold_speed_kmh):
    """Return (speed_caps, zone_labels); never modify official_path.

    hold_speed_kmh=0.0 disables the entire speed cap.
    Speed limits are computed from actual route distances, not WP spacing.
    """
    n = len(official_path)
    top = float(max_speed_kmh)
    hold = float(hold_speed_kmh)
    if not math.isfinite(top) or top <= 0.0:
        raise ValueError("max_speed_kmh must be finite and positive")
    if not math.isfinite(hold):
        raise ValueError("hold_speed_kmh must be finite")
    if hold < 0.0 or (hold > 0.0 and not MIN_TEST_HOLD_KMH <= hold <= MAX_TEST_HOLD_KMH):
        raise ValueError("roundabout hold speed must be 0 or 8..20 km/h")

    caps = [top] * n
    zone = ["OFF"] * n
    if hold == 0.0:
        return caps, zone
    if n <= HOLD_END_WP:
        raise ValueError("official path has too few WPs for roundabout speed zone")

    # A larger max-speed setting cannot override the explicit bend cap.
    # Use the same values as reference profile cap to avoid an accel spike.
    hold = min(hold, top)
    s = _cumulative_distances_m(official_path)
    s_begin = s[HOLD_START_WP]
    s_end = s[HOLD_END_WP]
    hold_ms = hold / 3.6

    for i in range(n):
        if i < HOLD_START_WP:
            d = s_begin - s[i]
            allowed_kmh = 3.6 * math.sqrt(
                hold_ms * hold_ms + 2.0 * PREVIEW_DECEL_MPS2 * d
            )
            mode = "PREBRAKE"
        elif i <= HOLD_END_WP:
            allowed_kmh = hold
            mode = "HOLD"
        else:
            d = s[i] - s_end
            allowed_kmh = 3.6 * math.sqrt(
                hold_ms * hold_ms + 2.0 * RELEASE_ACCEL_MPS2 * d
            )
            mode = "RELEASE"

        cap = min(top, allowed_kmh)
        caps[i] = cap
        if cap < top - 1e-6:
            zone[i] = mode

    return caps, zone
