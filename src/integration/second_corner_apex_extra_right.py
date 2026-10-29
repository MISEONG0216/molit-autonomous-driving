#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Optional additional RIGHT bias at the APEX of second right-hand corner.

Evidence, same 57km/h-cap MORAI runs:
- +0.30m main WP231-300 offset: WP260-279 max official-path error 0.808m.
- +0.40m main WP231-300 offset: WP260-279 max error 0.669m.
- WP240-249 is already close to the original route: do not shift that area
  further; improve WP260-279 only.

This small additional path bump is applied on TOP of the existing tracking
path and fades continuously, never touching official global waypoints.

Always use with --second-corner-right-offset-m 0.40 and
--first-corner-exit-right-offset-m 0.
"""
import math
import numpy as np

RAMP_IN_START_WP = 248
FULL_START_WP = 260
FULL_END_WP = 276
RAMP_OUT_END_WP = 292

MAX_APEX_EXTRA_M = 0.15


def _smoothstep(t):
    t = max(0.0, min(1.0, float(t)))
    return t * t * (3.0 - 2.0 * t)


def weight_at_wp(wp):
    i = int(wp)
    if i <= RAMP_IN_START_WP or i >= RAMP_OUT_END_WP:
        return 0.0
    if i < FULL_START_WP:
        return _smoothstep(
            (i - RAMP_IN_START_WP) / float(FULL_START_WP - RAMP_IN_START_WP)
        )
    if i <= FULL_END_WP:
        return 1.0
    return 1.0 - _smoothstep(
        (i - FULL_END_WP) / float(RAMP_OUT_END_WP - FULL_END_WP)
    )


def extra_at_wp(wp, max_extra_right_m):
    return float(max_extra_right_m) * weight_at_wp(wp)


def make_apex_extra_right_path(path, max_extra_right_m):
    """Build an Nx2/Nx3 copy with a small local extra RIGHT displacement."""
    src = np.asarray(path, dtype=float)
    if src.ndim != 2 or src.shape[1] < 2:
        raise ValueError("Tracking path must be an Nx2 or Nx3 array")
    value = float(max_extra_right_m)
    if not math.isfinite(value) or not (0.0 <= value <= MAX_APEX_EXTRA_M):
        raise ValueError("Second-corner apex extra RIGHT must be 0..0.15m")
    dst = src.copy()
    if value == 0.0:
        return dst
    if len(dst) <= RAMP_OUT_END_WP + 3:
        raise ValueError("Tracking path too short for apex correction")

    for wp in range(RAMP_IN_START_WP + 1, RAMP_OUT_END_WP):
        d = extra_at_wp(wp, value)
        if d == 0.0:
            continue
        # The pre-existing tracking curve determines its RIGHT normal;
        # avoid discontinuities with the earlier 0.40m second-corner shift.
        before = src[wp - 3, :2]
        after = src[wp + 3, :2]
        dx = float(after[0] - before[0])
        dy = float(after[1] - before[1])
        length = math.hypot(dx, dy)
        if length <= 1e-8:
            continue
        dst[wp, 0] += d * dy / length
        dst[wp, 1] -= d * dx / length
    return dst
