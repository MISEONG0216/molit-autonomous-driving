#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Offline geometry checks; does not connect to MORAI or send CtrlCmd.

Usage (repo root, Python 3.8 venv):
    python tests/test_lane_offset_wp1533.py
"""
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from integration.lane_offset import (
    RAMP_IN_BEGIN_WP,
    FULL_RIGHT_BEGIN_WP,
    FULL_RIGHT_END_WP,
    RAMP_OUT_END_WP,
    make_right_offset_path,
    offset_at_waypoint,
)

original = np.loadtxt(
    str(ROOT / "path" / "2026_molit_comp_global_path.txt"),
    usecols=(0, 1),
)
baseline = make_right_offset_path(original, 0.0)
assert np.array_equal(original, baseline), "Zero offset must preserve all WPs"

amount_m = 0.25
adjusted = make_right_offset_path(original, amount_m)
assert not np.shares_memory(adjusted, original)
for wp in (0, 1000, RAMP_IN_BEGIN_WP, RAMP_OUT_END_WP, 1770, 1877):
    assert np.array_equal(adjusted[wp], original[wp]), "Outside-zone WP changed: %d" % wp

for wp in (1533, 1578, 1655):
    tangent = original[wp + 3] - original[wp - 3]
    tangent /= np.linalg.norm(tangent)
    right = np.array([tangent[1], -tangent[0]])
    shift = adjusted[wp] - original[wp]
    right_distance = np.dot(shift, right)
    left_distance = tangent[0] * shift[1] - tangent[1] * shift[0]
    assert abs(right_distance - amount_m) < 1e-6, (wp, right_distance)
    assert left_distance < 0, (wp, left_distance)

assert abs(offset_at_waypoint(FULL_RIGHT_BEGIN_WP, amount_m) - amount_m) < 1e-12
assert abs(offset_at_waypoint(FULL_RIGHT_END_WP, amount_m) - amount_m) < 1e-12
assert offset_at_waypoint(RAMP_OUT_END_WP, amount_m) == 0.0

print("[PASS] Original global path remains unchanged")
print("[PASS] RIGHT 0.25m at WP 1533/1578/1655")
print("[PASS] Smooth zone: %d -> %d hold through %d -> %d" % (
    RAMP_IN_BEGIN_WP, FULL_RIGHT_BEGIN_WP,
    FULL_RIGHT_END_WP, RAMP_OUT_END_WP,
))
print("[PASS] WP1770 roundabout and all other zones unaffected")
