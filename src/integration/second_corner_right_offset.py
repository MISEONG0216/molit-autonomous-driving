#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Optional delayed RIGHT tracking path at second corner entry, WP231-300.

Evidence: integrated_v1_20261030_034020.csv shows old first-corner exit
RIGHT bias fading from 0.12m at WP231 to 0.00m at WP238; the car then
moves LEFT of original waypoints by ~0.26m at WP245 and ~1.19m at WP265.
Moving the bias later avoids an early right-left-right target sequence.

This function changes ONLY an independent copy of the tracking path.
It does not change the official global waypoint file, V7 speed profile,
nearest official waypoint lookup, mission indices, LiDAR or signal logic.

Set --first-corner-exit-right-offset-m 0 when testing this alternative.
Keep --first-corner-left-offset-m 0.30 as the earlier working first turn.

This is experimental geometry, not a lane-boundary measurement.
"""

import math
import numpy as np

BEGIN_WP = 231
FULL_BEGIN_WP = 250
FULL_END_WP = 276
END_WP = 300
MAX_RIGHT_OFFSET_M = 0.40


def _smoothstep(t):
    t = max(0.0, min(1.0, float(t)))
    return t * t * (3.0 - 2.0 * t)


def weight_at_wp(wp):
    """Zero before WP231/after WP300; smooth ramps around the turn."""
    i = int(wp)
    if i <= BEGIN_WP or i >= END_WP:
        return 0.0
    if i < FULL_BEGIN_WP:
        return _smoothstep((i - BEGIN_WP) / float(FULL_BEGIN_WP - BEGIN_WP))
    if i <= FULL_END_WP:
        return 1.0
    return 1.0 - _smoothstep((i - FULL_END_WP) / float(END_WP - FULL_END_WP))


def offset_at_wp(wp, right_offset_m):
    """Positive metres means RIGHT of the path travel direction."""
    return float(right_offset_m) * weight_at_wp(wp)


def make_second_corner_right_path(input_tracking_path, right_offset_m):
    """Return an independent Nx2/Nx3 tracking path shifted locally RIGHT.

    RIGHT normal of forward tangent (dx,dy) = (dy,-dx).
    Waypoint count, waypoint order and off-zone positions are unchanged.
    """
    original = np.asarray(input_tracking_path, dtype=float)
    if original.ndim != 2 or original.shape[1] < 2:
        raise ValueError("Expected tracking path shape (N, >=2)")

    amount_m = float(right_offset_m)
    if not math.isfinite(amount_m) or not (
        0.0 <= amount_m <= MAX_RIGHT_OFFSET_M
    ):
        raise ValueError(
            "second corner RIGHT offset must be 0..%.2fm" % MAX_RIGHT_OFFSET_M
        )
    tracking = original.copy()
    if amount_m == 0.0:
        return tracking
    if len(original) <= END_WP + 3:
        raise ValueError("Global path is too short for WP231-300 test zone")

    for i in range(BEGIN_WP + 1, END_WP):
        d = offset_at_wp(i, amount_m)
        if d <= 0.0:
            continue

        before = original[i - 3, :2]
        after = original[i + 3, :2]
        dx = float(after[0] - before[0])
        dy = float(after[1] - before[1])
        tangent_size = math.hypot(dx, dy)
        if tangent_size <= 1e-8:
            continue

        tracking[i, 0] += d * dy / tangent_size
        tracking[i, 1] -= d * dx / tangent_size

    return tracking
