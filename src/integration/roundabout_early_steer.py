#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Opt-in early RIGHT steering assist at roundabout exit.

Problem: V22 25km/h CSV repeatedly shows lane-adjacent overrun near
WP1859-1885, with the steering direction changing around WP1850.
This test advances a small RIGHT steering command locally instead
of moving the global waypoints or changing the V7 LFD/gain.

Control conventions: MORAI CtrlCmd right steering is negative radians.
Positive extra_right_deg is a convenient magnitude, not steering sign.
"""

import math


# Only this segment gets a steering pulse. All indices inclusive.
RAMP_IN_START_WP = 1843
FULL_START_WP = 1851
FULL_END_WP = 1857
RAMP_OUT_END_WP = 1866

MAX_EXTRA_RIGHT_DEG = 6.0


def _smoothstep(t):
    t = max(0.0, min(1.0, float(t)))
    return t * t * (3.0 - 2.0 * t)


def weight_at_wp(wp):
    wp = int(wp)
    if wp <= RAMP_IN_START_WP or wp >= RAMP_OUT_END_WP:
        return 0.0
    if wp < FULL_START_WP:
        return _smoothstep(
            (wp - RAMP_IN_START_WP) /
            float(FULL_START_WP - RAMP_IN_START_WP)
        )
    if wp <= FULL_END_WP:
        return 1.0
    return 1.0 - _smoothstep(
        (wp - FULL_END_WP) /
        float(RAMP_OUT_END_WP - FULL_END_WP)
    )


def extra_right_rad_at_wp(wp, extra_right_deg):
    """Return a NEGATIVE steering addition in radians, or zero outside zone."""
    magnitude = float(extra_right_deg)
    if not math.isfinite(magnitude) or not (
        0.0 <= magnitude <= MAX_EXTRA_RIGHT_DEG
    ):
        raise ValueError("roundabout early-right degrees must be in [0, 6]")
    return -math.radians(magnitude * weight_at_wp(wp))


def add_early_right_steering(steering_rad, wp, extra_right_deg):
    """Return (steering, actual_added_right_degrees).

    Existing steering is not replaced: only a bounded pulse is added,
    then clipped to the controller's existing +/-1.0rad envelope.
    """
    before = float(steering_rad)
    delta = extra_right_rad_at_wp(wp, extra_right_deg)
    after = max(-1.0, min(1.0, before + delta))
    actual_right_deg = max(0.0, math.degrees(before - after))
    return after, actual_right_deg
