#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Offline checks: FIRST LEFT corner stays intact; RIGHT exit starts WP195.

Run from repository root with Python 3.8 virtual environment:
    python tests/test_first_corner_exit_right_offset.py

No MORAI, sensor sockets, or CtrlCmd is involved.
"""
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from integration.first_corner_left_offset import (
    make_first_corner_left_path,
    offset_at_waypoint as left_offset_at_wp,
)
from integration.first_corner_exit_right_offset import (
    make_first_corner_exit_right_path,
    offset_at_waypoint as right_offset_at_wp,
    RAMP_IN_START_WP,
    FULL_RIGHT_START_WP,
    FULL_RIGHT_END_WP,
    RAMP_OUT_END_WP,
)
from integration.lane_offset import make_right_offset_path

official = np.loadtxt(
    str(ROOT / "path" / "2026_molit_comp_global_path.txt"),
    usecols=(0, 1),
)
snapshot = official.copy()

# Preserve earlier successful modifications, including guardrail WP1533.
guardrail = make_right_offset_path(official, 0.40)
first_left = make_first_corner_left_path(guardrail, 0.30)
disabled = make_first_corner_exit_right_path(first_left, 0.0)
assert np.array_equal(disabled, first_left), "0.00m option changed the original"

corrected = make_first_corner_exit_right_path(first_left, 0.20)
assert np.array_equal(official, snapshot), "Official global path mutated"
assert not np.shares_memory(corrected, first_left), "Input mutated"

# Correction only on the post-corner WP195..238 portion.
for wp in (3, 120, 140, 150, 170, 174, 182, 190, 194,
           RAMP_IN_START_WP, RAMP_OUT_END_WP,
           240, 245, 250, 265, 300, 760, 1224, 1533, 1550,
           1655, 1877, len(official) - 1):
    assert np.array_equal(corrected[wp], first_left[wp]), (
        "Unexpected change to WP %d" % wp
    )

# Geometric displacement must be RIGHT of the first-left tracking path.
for wp in (200, 210, FULL_RIGHT_START_WP, 220, FULL_RIGHT_END_WP, 230):
    tangent = first_left[wp + 3] - first_left[wp - 3]
    tangent /= np.linalg.norm(tangent)
    right_unit = np.array([tangent[1], -tangent[0]])
    shift = corrected[wp] - first_left[wp]
    actual = float(np.dot(shift, right_unit))
    expected = right_offset_at_wp(wp, 0.20)
    assert abs(actual - expected) < 1e-6, (wp, actual, expected)
    assert abs(float(np.dot(shift, tangent))) < 1e-6, wp

# Left fade and right ramp overlap smoothly, not a discontinuity.
assert left_offset_at_wp(195, 0.30) > 0.0
assert right_offset_at_wp(195, 0.20) == 0.0
assert right_offset_at_wp(200, 0.20) < right_offset_at_wp(210, 0.20)
assert right_offset_at_wp(FULL_RIGHT_START_WP, 0.20) == 0.20
assert right_offset_at_wp(FULL_RIGHT_END_WP, 0.20) == 0.20
assert right_offset_at_wp(RAMP_OUT_END_WP, 0.20) == 0.0

for wp in (185, 190, 195, 200, 205, 210, 215, 220, 225, 230, 235, 238):
    print(
        "WP %d: LEFT %.3fm / RIGHT %.3fm" %
        (wp, left_offset_at_wp(wp, 0.30),
         right_offset_at_wp(wp, 0.20))
    )

for invalid in (-0.1, 0.41):
    try:
        make_first_corner_exit_right_path(first_left, invalid)
    except ValueError:
        pass
    else:
        raise AssertionError("Invalid right shift accepted: %s" % invalid)

print("[PASS] LEFT 0.30m first corner unchanged")
print("[PASS] RIGHT 0.20m gradual transition WP195-238 only")
print("[PASS] Second corner WP240+ / guardrail WP1533 unaffected")
print("[PASS] Official path unchanged and zero option keeps baseline")
