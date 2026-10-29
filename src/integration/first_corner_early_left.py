#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Optional early LEFT steering for the first corner only (WP 140..163).

2026-10-30 drive screenshot: WP144; speed 26.17km/h, target 18.62km/h,
baseline steering +1.23 degrees (LEFT); next WPs steepen to 20+ degrees.
This experiment moves the steering response earlier with a small smooth
positive steering addition; original global WPs and V7 speed profile unchanged.

MORAI convention here: positive steering is LEFT, negative is RIGHT.
Default zero means completely unchanged original controller behavior.
"""

import math

RAMP_BEGIN_WP = 140
FULL_BEGIN_WP = 144
FULL_END_WP = 150
RAMP_END_WP = 163
MAX_EXTRA_LEFT_DEG = 4.0


def _smoothstep(t):
    t = max(0.0, min(1.0, float(t)))
    return t * t * (3.0 - 2.0 * t)


def weight_at_wp(wp):
    """C1-continuous isolated assist, 0 outside WP 140..163."""
    i = int(wp)
    if i <= RAMP_BEGIN_WP or i >= RAMP_END_WP:
        return 0.0
    if i < FULL_BEGIN_WP:
        return _smoothstep(
            (i - RAMP_BEGIN_WP) / float(FULL_BEGIN_WP - RAMP_BEGIN_WP)
        )
    if i <= FULL_END_WP:
        return 1.0
    return 1.0 - _smoothstep(
        (i - FULL_END_WP) / float(RAMP_END_WP - FULL_END_WP)
    )


def add_early_left_steering(steering_rad, wp, extra_left_deg):
    """Return (steer_rad, applied_left_deg), bounded to +/-1.0rad.

    No direct CtrlCmd transmission. Actual steering is applied only in
    the caller when the existing V7 path is valid and no STOP override
    or LiDAR local avoidance is controlling the vehicle.
    """
    magnitude = float(extra_left_deg)
    if not math.isfinite(magnitude) or not (
        0.0 <= magnitude <= MAX_EXTRA_LEFT_DEG
    ):
        raise ValueError("first corner left assist must be in [0, 4] deg")
    before = float(steering_rad)
    extra = math.radians(magnitude * weight_at_wp(wp))
    after = max(-1.0, min(1.0, before + extra))
    actual = max(0.0, math.degrees(after - before))
    return after, actual
