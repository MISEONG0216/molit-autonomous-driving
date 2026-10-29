#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Opt-in roundabout EXIT recovery steering when vehicle remains left of path.

V22 25km/h CSV, after WP1843-1866 early RIGHT 4 degree assist:
WP1877 center is about 1.34m LEFT of official path; WP1886 about 1.00m LEFT.
By WP1900, the vehicle begins crossing to the RIGHT side of the path.
A right-steering pulse must therefore be local, tapered, and LEFT-error-gated.

This module never modifies waypoints or longitudinal control.
Positive signed cross track error = LEFT of official path travel direction.
MORAI RIGHT steering = negative radians.
"""

import math


START_WP = 1862
FULL_START_WP = 1870
FULL_END_WP = 1881
END_WP = 1891

MIN_LEFT_ERROR_M = 0.35
FULL_LEFT_ERROR_M = 0.90
MAX_EXTRA_RIGHT_DEG = 3.0


def _smoothstep(t):
    t = max(0.0, min(1.0, float(t)))
    return t * t * (3.0 - 2.0 * t)


def waypoint_weight(wp):
    i = int(wp)
    if i <= START_WP or i >= END_WP:
        return 0.0
    if i < FULL_START_WP:
        return _smoothstep((i - START_WP) / float(FULL_START_WP - START_WP))
    if i <= FULL_END_WP:
        return 1.0
    return 1.0 - _smoothstep((i - FULL_END_WP) / float(END_WP - FULL_END_WP))


def signed_left_error_m(official_path, wp, x_m, y_m):
    """Signed lateral error in metres relative to original route WP.

    Positive means vehicle is to LEFT of forward travel direction.
    """
    i = int(wp)
    if i < 3 or i + 3 >= len(official_path):
        return 0.0
    before = official_path[i - 3]
    after = official_path[i + 3]
    nearest = official_path[i]
    dx = float(after[0]) - float(before[0])
    dy = float(after[1]) - float(before[1])
    mag = math.hypot(dx, dy)
    if mag <= 1e-8:
        return 0.0
    delta_x = float(x_m) - float(nearest[0])
    delta_y = float(y_m) - float(nearest[1])
    return (dx * delta_y - dy * delta_x) / mag


def apply_recovery_right_steering(steer_rad, official_path,
                                  nearest_index, x_m, y_m, max_extra_right_deg):
    """Return (steer_rad, applied_right_deg, signed_left_error_m).

    Adding steering is allowed only in WP1862-1891, and only when left of
    original path by more than 0.35 m. Full 3deg (max) only when left>=0.90m.
    Both the waypoint envelope and lateral-error weighting are smooth.
    """
    magnitude_deg = float(max_extra_right_deg)
    if not math.isfinite(magnitude_deg) or not (
        0.0 <= magnitude_deg <= MAX_EXTRA_RIGHT_DEG
    ):
        raise ValueError("recovery-right-deg must be 0..3")

    left_error = signed_left_error_m(
        official_path, nearest_index, x_m, y_m
    )
    wp_weight = waypoint_weight(nearest_index)
    if wp_weight <= 0.0 or magnitude_deg <= 0.0:
        return float(steer_rad), 0.0, left_error

    error_weight = _smoothstep(
        (left_error - MIN_LEFT_ERROR_M) /
        float(FULL_LEFT_ERROR_M - MIN_LEFT_ERROR_M)
    )
    extra_rad = math.radians(magnitude_deg * wp_weight * error_weight)
    base = float(steer_rad)
    adjusted = max(-1.0, min(1.0, base - extra_rad))
    applied_right_deg = max(0.0, math.degrees(base - adjusted))
    return adjusted, applied_right_deg, left_error
