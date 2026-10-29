#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Optional smooth RIGHTward tracking-path correction near roundabout exit.

Evidence: V22 CSVs 20261029_222543 and 20261029_225509 both show
a repeatable left-of-global-path error around WP1870..1885.
This is an experimental candidate, NOT verified lane-center geometry.

The existing WP1480..1715 median correction is independent and unchanged.
Official waypoints, mission indices, and the speed profile are unchanged.
"""

import math
import numpy as np

RAMP_IN_BEGIN_WP = 1859
FULL_BEGIN_WP = 1870
FULL_END_WP = 1885
RAMP_OUT_END_WP = 1902
MAX_RIGHT_OFFSET_M = 0.50


def _smoothstep(value):
    value = max(0.0, min(1.0, float(value)))
    return value * value * (3.0 - 2.0 * value)


def weight_at_waypoint(wp):
    i = int(wp)
    if i <= RAMP_IN_BEGIN_WP or i >= RAMP_OUT_END_WP:
        return 0.0
    if i < FULL_BEGIN_WP:
        return _smoothstep(
            (i - RAMP_IN_BEGIN_WP) / float(FULL_BEGIN_WP - RAMP_IN_BEGIN_WP)
        )
    if i <= FULL_END_WP:
        return 1.0
    return 1.0 - _smoothstep(
        (i - FULL_END_WP) / float(RAMP_OUT_END_WP - FULL_END_WP)
    )


def offset_at_waypoint(wp, max_offset_m):
    return float(max_offset_m) * weight_at_waypoint(wp)


def make_roundabout_exit_path(tracking_path, max_right_offset_m):
    """Make a new path with an extra local right shift; does not mutate input.

    The segment is disjoint from the WP1480..1715 lane correction.
    Direction follows official waypoint sequence; RIGHT normal=(dy,-dx).
    """
    original = np.asarray(tracking_path, dtype=float)
    if original.ndim != 2 or original.shape[1] < 2:
        raise ValueError("tracking_path must be Nx2 or Nx3")
    if not math.isfinite(max_right_offset_m) or not (
        0.0 <= max_right_offset_m <= MAX_RIGHT_OFFSET_M
    ):
        raise ValueError(
            "roundabout offset must be between 0 and %.2f m" %
            MAX_RIGHT_OFFSET_M
        )

    shifted = original.copy()
    if max_right_offset_m == 0.0:
        return shifted
    if len(shifted) <= RAMP_OUT_END_WP + 3:
        raise ValueError("Global path is too short for roundabout exit zone")

    for i in range(RAMP_IN_BEGIN_WP + 1, RAMP_OUT_END_WP):
        amount = offset_at_waypoint(i, max_right_offset_m)
        if amount == 0.0:
            continue
        # Symmetric finite difference gives a stable local travel tangent.
        forward = original[i + 3, :2] - original[i - 3, :2]
        length = math.hypot(float(forward[0]), float(forward[1]))
        if length <= 1e-8:
            continue
        shifted[i, 0] += amount * forward[1] / length
        shifted[i, 1] -= amount * forward[0] / length
    return shifted
