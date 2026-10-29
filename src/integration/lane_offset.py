#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Opt-in rightward tracking-path offset for yellow-median clearance.

This does NOT change the competition's official global waypoint file.
It only builds a temporary, smooth, shifted path for Pure Pursuit steering.
Nearest-waypoint lookup, mission indices, speed profiles, and LiDAR worker
continue to reference the unmodified official path.

Clip verification (2026-10-29): video 9 s -> WP 1533; 13 s -> WP 1578;
19 s -> WP 1655. Yellow median is on the vehicle's left, hence shift RIGHT.
"""

import math

import numpy as np


RAMP_IN_BEGIN_WP = 1480
FULL_RIGHT_BEGIN_WP = 1530
FULL_RIGHT_END_WP = 1655
RAMP_OUT_END_WP = 1715

MAX_TEST_OFFSET_M = 0.50


def _smoothstep(t):
    """C1-continuous transition from 0 to 1."""
    t = min(1.0, max(0.0, float(t)))
    return t * t * (3.0 - 2.0 * t)


def weight_at_waypoint(wp):
    """Zero outside the target zone; fade smoothly at both ends."""
    i = int(wp)
    if i <= RAMP_IN_BEGIN_WP or i >= RAMP_OUT_END_WP:
        return 0.0
    if i < FULL_RIGHT_BEGIN_WP:
        return _smoothstep(
            (i - RAMP_IN_BEGIN_WP) /
            float(FULL_RIGHT_BEGIN_WP - RAMP_IN_BEGIN_WP)
        )
    if i <= FULL_RIGHT_END_WP:
        return 1.0
    return 1.0 - _smoothstep(
        (i - FULL_RIGHT_END_WP) /
        float(RAMP_OUT_END_WP - FULL_RIGHT_END_WP)
    )


def offset_at_waypoint(wp, max_right_offset_m):
    """Positive output means RIGHT of official travel direction."""
    return float(max_right_offset_m) * weight_at_waypoint(wp)


def make_right_offset_path(official_path, max_right_offset_m):
    """Return a separate Nx2 path shifted right in the selected WP interval.

    For local forward tangent (dx, dy), RIGHT normal = (dy, -dx).
    No original global path coordinates or mission waypoint indices are changed.
    """
    original = np.asarray(official_path, dtype=float)
    if original.ndim != 2 or original.shape[1] < 2:
        raise ValueError("Official path must have shape (N, >=2)")
    if not math.isfinite(max_right_offset_m):
        raise ValueError("lane offset must be finite")
    if not 0.0 <= max_right_offset_m <= MAX_TEST_OFFSET_M:
        raise ValueError(
            "lane offset must be between 0 and %.2f m" % MAX_TEST_OFFSET_M
        )

    tracking = original.copy()
    if max_right_offset_m == 0.0:
        return tracking
    if len(original) <= RAMP_OUT_END_WP + 3:
        raise ValueError("Official path is too short for lane-offset WP zone")

    for wp in range(RAMP_IN_BEGIN_WP + 1, RAMP_OUT_END_WP):
        distance_m = offset_at_waypoint(wp, max_right_offset_m)
        if distance_m <= 0.0:
            continue

        # Averaging several waypoints reduces normal-vector discontinuities
        # caused by dense/noisy original path sampling.
        before = original[wp - 3, :2]
        after = original[wp + 3, :2]
        tangent = after - before
        norm = float(math.hypot(tangent[0], tangent[1]))
        if norm <= 1e-8:
            continue

        right_x = tangent[1] / norm
        right_y = -tangent[0] / norm
        tracking[wp, 0] += distance_m * right_x
        tracking[wp, 1] += distance_m * right_y

    return tracking
