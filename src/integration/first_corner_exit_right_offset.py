#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Opt-in RIGHT tracking-path transition just AFTER first LEFT corner.

The first corner has a separately validated-in-video LEFT path shift:
  WP 125 -> 150: smooth LEFT increase
  WP 150 -> 182: maintain LEFT 0.30m
  WP 182 -> 215: smooth decrease to original route

The latest drive CSV (20261030_031705) shows the car returning close to
the official route around WP195-200. This module gives Pure Pursuit a
gentle RIGHT target after that point, preserving the successful first turn.

This modifies a *COPY* of the tracking path, never the official path.
It does not change traffic stop indices, LiDAR zones, speed profile,
nearest official waypoint lookup, or steering gain/LFD.

Only trial-run in MORAI. RIGHT correction is NOT lane-boundary awareness.
"""

import math

import numpy as np


RAMP_IN_START_WP = 195
FULL_RIGHT_START_WP = 215
FULL_RIGHT_END_WP = 222
RAMP_OUT_END_WP = 238

MAX_RIGHT_TEST_OFFSET_M = 0.40


def _smoothstep(t):
    t = max(0.0, min(1.0, float(t)))
    return t * t * (3.0 - 2.0 * t)


def weight_at_waypoint(wp):
    """Smooth right-offset profile, exactly zero outside WP195..238."""
    index = int(wp)
    if index <= RAMP_IN_START_WP or index >= RAMP_OUT_END_WP:
        return 0.0
    if index < FULL_RIGHT_START_WP:
        return _smoothstep(
            (index - RAMP_IN_START_WP) /
            float(FULL_RIGHT_START_WP - RAMP_IN_START_WP)
        )
    if index <= FULL_RIGHT_END_WP:
        return 1.0
    return 1.0 - _smoothstep(
        (index - FULL_RIGHT_END_WP) /
        float(RAMP_OUT_END_WP - FULL_RIGHT_END_WP)
    )


def offset_at_waypoint(wp, max_right_offset_m):
    """Positive output is desired RIGHT displacement in metres."""
    return float(max_right_offset_m) * weight_at_waypoint(wp)


def make_first_corner_exit_right_path(input_tracking_path, max_right_offset_m):
    """Return separate path shifted RIGHT after first corner, without mutation.

    Local forward tangent=(dx,dy); RIGHT normal=(dy,-dx).
    Original waypoint indices, count, and non-zone coordinates are preserved.
    """
    source = np.asarray(input_tracking_path, dtype=float)
    if source.ndim != 2 or source.shape[1] < 2:
        raise ValueError("Expected tracking path of shape (N, >=2)")

    maximum = float(max_right_offset_m)
    if not math.isfinite(maximum) or not (
        0.0 <= maximum <= MAX_RIGHT_TEST_OFFSET_M
    ):
        raise ValueError(
            "first-corner exit RIGHT offset must be 0..%.2f m" %
            MAX_RIGHT_TEST_OFFSET_M
        )
    shifted = source.copy()
    if maximum == 0.0:
        return shifted
    if len(source) <= RAMP_OUT_END_WP + 3:
        raise ValueError("Tracking path is too short for first-corner exit")

    for i in range(RAMP_IN_START_WP + 1, RAMP_OUT_END_WP):
        delta_m = offset_at_waypoint(i, maximum)
        if delta_m <= 0.0:
            continue
        # Use the tracking path tangent to keep the transition continuous
        # with the preceding first-corner LEFT offset that is fading out.
        p_before = source[i - 3, :2]
        p_after = source[i + 3, :2]
        tangent_x = float(p_after[0] - p_before[0])
        tangent_y = float(p_after[1] - p_before[1])
        magnitude = math.hypot(tangent_x, tangent_y)
        if magnitude <= 1e-8:
            continue
        shifted[i, 0] += delta_m * tangent_y / magnitude
        shifted[i, 1] -= delta_m * tangent_x / magnitude

    return shifted
