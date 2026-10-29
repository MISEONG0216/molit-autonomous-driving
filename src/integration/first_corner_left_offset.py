#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Opt-in LEFT-shifted Pure Pursuit tracking path on the first LEFT corner.

Evidence: three same-condition MORAI drives (~57km/h cap), including
integrated_v1_20261030_030440.csv, show vehicle right of original path by
~1.5m near WP170-179. The earlier isolated +2deg steering pulse had only
limited benefit. This experiment shifts the *tracking target* left instead.

Official global waypoints are NEVER modified. Nearest WP lookup, traffic
missions, LiDAR mission zones, and longitudinal speed profile still use
the unchanged official global path.

Requested offset = 0 by default, identical to original V22 tracking.
Recommended first test = 0.30m, not a measured lane-center displacement.
"""

import math
import numpy as np

RAMP_IN_START_WP = 125
FULL_LEFT_START_WP = 150
FULL_LEFT_END_WP = 182
RAMP_OUT_END_WP = 215
MAX_LEFT_TEST_OFFSET_M = 0.50


def _smoothstep(value):
    value = max(0.0, min(1.0, float(value)))
    return value * value * (3.0 - 2.0 * value)


def weight_at_waypoint(wp):
    """Smoothly taper local path change; exactly zero outside WP125..215."""
    i = int(wp)
    if i <= RAMP_IN_START_WP or i >= RAMP_OUT_END_WP:
        return 0.0
    if i < FULL_LEFT_START_WP:
        return _smoothstep(
            (i - RAMP_IN_START_WP) /
            float(FULL_LEFT_START_WP - RAMP_IN_START_WP)
        )
    if i <= FULL_LEFT_END_WP:
        return 1.0
    return 1.0 - _smoothstep(
        (i - FULL_LEFT_END_WP) /
        float(RAMP_OUT_END_WP - FULL_LEFT_END_WP)
    )


def offset_at_waypoint(wp, max_left_offset_m):
    """Positive offset means LEFT of travel direction."""
    return float(max_left_offset_m) * weight_at_waypoint(wp)


def make_first_corner_left_path(input_path, max_left_offset_m):
    """Build an independent copy with selected first-corner WPs LEFT-shifted.

    LEFT normal of the local forward tangent (dx, dy) = (-dy, dx).
    The input array is not mutated, and WP indices stay exactly the same.
    """
    original = np.asarray(input_path, dtype=float)
    if original.ndim != 2 or original.shape[1] < 2:
        raise ValueError("Input path must have shape (N, >=2)")

    offset = float(max_left_offset_m)
    if not math.isfinite(offset) or not (0.0 <= offset <= MAX_LEFT_TEST_OFFSET_M):
        raise ValueError(
            "first-corner LEFT offset must be between 0 and %.2f m" %
            MAX_LEFT_TEST_OFFSET_M
        )

    tracking = original.copy()
    if offset == 0.0:
        return tracking
    if len(original) <= RAMP_OUT_END_WP + 3:
        raise ValueError("Global path is too short for first-corner offset zone")

    for i in range(RAMP_IN_START_WP + 1, RAMP_OUT_END_WP):
        shift = offset_at_waypoint(i, offset)
        if shift == 0.0:
            continue
        # Symmetric multi-WP tangent avoids abrupt directions caused by
        # very closely-spaced original global waypoints.
        before = original[i - 3, :2]
        after = original[i + 3, :2]
        dx = float(after[0] - before[0])
        dy = float(after[1] - before[1])
        norm = math.hypot(dx, dy)
        if norm <= 1e-8:
            continue
        tracking[i, 0] -= shift * dy / norm
        tracking[i, 1] += shift * dx / norm

    return tracking
