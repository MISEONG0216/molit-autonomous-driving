#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Offline check: WP1533 0.40m tune is preserved and exit tune is isolated.

Run at repository root using the same Python 3.8 venv as the controller:
    python tests/test_roundabout_exit_offset.py

No MORAI, GPS, LiDAR or CtrlCmd is used.
"""
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from integration.lane_offset import make_right_offset_path
from integration.roundabout_exit_offset import (
    make_roundabout_exit_path,
    offset_at_waypoint,
    RAMP_IN_BEGIN_WP,
    FULL_BEGIN_WP,
    FULL_END_WP,
    RAMP_OUT_END_WP,
)

original = np.loadtxt(
    str(ROOT / "path" / "2026_molit_comp_global_path.txt"),
    usecols=(0, 1),
)
snapshot = original.copy()
median_tuned = make_right_offset_path(original, 0.40)
assert np.array_equal(original, snapshot), "Official path mutated"

unchanged = make_roundabout_exit_path(median_tuned, 0.0)
assert np.array_equal(median_tuned, unchanged), "Zero exit-offset changed path"

exit_tuned = make_roundabout_exit_path(median_tuned, 0.25)
assert np.array_equal(original, snapshot), "Official path mutated by exit tune"
assert not np.shares_memory(exit_tuned, median_tuned)

# Other missions, and the WP1533 yellow-line correction, remain IDENTICAL.
for wp in (0, 590, 760, 1224, 1480, 1500, 1533, 1578, 1655,
           1715, 1770, 1817, 1849, RAMP_IN_BEGIN_WP,
           RAMP_OUT_END_WP, 1917, 2100):
    assert np.array_equal(exit_tuned[wp], median_tuned[wp]), (
        "Unexpected shift outside exit zone, waypoint %d" % wp
    )

for wp in (1870, 1876, 1885):
    delta = exit_tuned[wp] - median_tuned[wp]
    tangent = median_tuned[wp + 3] - median_tuned[wp - 3]
    tangent /= np.linalg.norm(tangent)
    right = np.array([tangent[1], -tangent[0]])
    assert abs(float(np.dot(delta, right)) - 0.25) < 1e-6, wp
    assert abs(float(np.dot(delta, tangent))) < 1e-6, wp

assert offset_at_waypoint(FULL_BEGIN_WP, 0.25) == 0.25
assert offset_at_waypoint(FULL_END_WP, 0.25) == 0.25
assert offset_at_waypoint(RAMP_IN_BEGIN_WP, 0.25) == 0.0
assert offset_at_waypoint(RAMP_OUT_END_WP, 0.25) == 0.0

print("[PASS] Official global path unchanged")
print("[PASS] Original 0.40m median correction unchanged at WP1533-1655")
print("[PASS] Roundabout entry WP1770-1859 unchanged")
print("[PASS] Roundabout exit RIGHT 0.25m at WP1870/1876/1885")
print("[PASS] Exit offset smoothly zero again by WP1902")
