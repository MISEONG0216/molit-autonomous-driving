#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Offline verification of LEFT path offset for first corner.

    python tests/test_first_corner_left_offset.py

This test sends NO control commands and does not start MORAI.
"""

from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from integration.first_corner_left_offset import (
    make_first_corner_left_path,
    offset_at_waypoint,
    RAMP_IN_START_WP,
    FULL_LEFT_START_WP,
    FULL_LEFT_END_WP,
    RAMP_OUT_END_WP,
)
from integration.lane_offset import make_right_offset_path

official = np.loadtxt(
    str(ROOT / "path" / "2026_molit_comp_global_path.txt"),
    usecols=(0, 1),
)
snapshot = official.copy()

# Previous successful guardrail correction can coexist, unchanged.
guardrail = make_right_offset_path(official, 0.40)
baseline = make_first_corner_left_path(guardrail, 0.0)
assert np.array_equal(baseline, guardrail)

left_path = make_first_corner_left_path(guardrail, 0.30)
assert np.array_equal(official, snapshot)
assert not np.shares_memory(left_path, guardrail)

for wp in (0, 3, 100, RAMP_IN_START_WP, RAMP_OUT_END_WP,
           220, 250, 265, 400, 1250, 1533, 1550, 1655,
           1770, 1877, len(official) - 1):
    assert np.array_equal(left_path[wp], guardrail[wp]), wp

for wp in (150, 170, 174, 182):
    tangent = guardrail[wp + 3] - guardrail[wp - 3]
    tangent /= np.linalg.norm(tangent)
    left = np.array([-tangent[1], tangent[0]])
    delta = left_path[wp] - guardrail[wp]
    assert abs(float(np.dot(delta, left)) - 0.30) < 1e-6, wp
    assert abs(float(np.dot(delta, tangent))) < 1e-6, wp

for wp in (125, 130, 140, 144, 150):
    print(
        "WP %d LEFT %.3f m" % (wp, offset_at_waypoint(wp, 0.30))
    )
assert offset_at_waypoint(RAMP_IN_START_WP, 0.30) == 0.0
assert offset_at_waypoint(FULL_LEFT_START_WP, 0.30) == 0.30
assert offset_at_waypoint(FULL_LEFT_END_WP, 0.30) == 0.30
assert offset_at_waypoint(RAMP_OUT_END_WP, 0.30) == 0.0

try:
    make_first_corner_left_path(official, 0.51)
except ValueError:
    pass
else:
    raise AssertionError("Offset greater than 0.50m must fail")

print("[PASS] Global waypoints and previous 0.40m guardrail correction preserved")
print("[PASS] Shift is LEFT of forward path direction")
print("[PASS] First corner WP125-215 only; other WPs unaffected")
print("[PASS] First-corner 0.0m option exactly reproduces baseline")
