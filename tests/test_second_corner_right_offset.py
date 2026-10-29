#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Offline geometry regression: delayed right tracking path begins WP231.

Run from repo root:
    python tests/test_second_corner_right_offset.py

No MORAI, no UDP sockets, and no CtrlCmd.
"""
from pathlib import Path
import sys
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from integration.lane_offset import make_right_offset_path
from integration.first_corner_left_offset import make_first_corner_left_path
from integration.second_corner_right_offset import (
    make_second_corner_right_path, offset_at_wp,
    BEGIN_WP, FULL_BEGIN_WP, FULL_END_WP, END_WP,
)

official = np.loadtxt(
    str(ROOT / "path" / "2026_molit_comp_global_path.txt"),
    usecols=(0, 1),
)
snapshot = official.copy()
guardrail = make_right_offset_path(official, 0.40)
first_corner = make_first_corner_left_path(guardrail, 0.30)

# Old WP195-238 exit bias is intentionally disabled by CLI argument 0.0.
unchanged = make_second_corner_right_path(first_corner, 0.0)
assert np.array_equal(unchanged, first_corner)

shifted = make_second_corner_right_path(first_corner, 0.30)
assert np.array_equal(official, snapshot), "Official path was changed"
assert not np.shares_memory(shifted, first_corner)

# The working first LEFT turn, second turn outside the designated zone,
# and later guardrail correction are unaffected by this new bias.
for wp in (0, 3, 100, 125, 144, 150, 170, 182, 195, 205,
           215, 220, 225, 230, BEGIN_WP, END_WP,
           301, 310, 590, 760, 1224, 1533, 1550, 1655,
           1877, len(official) - 1):
    assert np.array_equal(shifted[wp], first_corner[wp]), wp

for wp in (240, 250, 260, 265, 276, 285, 295):
    tangent = first_corner[wp + 3] - first_corner[wp - 3]
    tangent /= np.linalg.norm(tangent)
    right_normal = np.array([tangent[1], -tangent[0]])
    displacement = shifted[wp] - first_corner[wp]
    actual_right = float(np.dot(displacement, right_normal))
    expected_right = offset_at_wp(wp, 0.30)
    assert abs(actual_right - expected_right) < 1e-6, wp
    assert abs(float(np.dot(displacement, tangent))) < 1e-6, wp

assert offset_at_wp(BEGIN_WP, 0.30) == 0.0
assert offset_at_wp(FULL_BEGIN_WP, 0.30) == 0.30
assert offset_at_wp(FULL_END_WP, 0.30) == 0.30
assert offset_at_wp(END_WP, 0.30) == 0.0

for wp in (220, 230, 231, 235, 240, 245, 250,
           260, 265, 276, 285, 295, 300, 305):
    print("WP %d: new RIGHT %.3f m" % (
        wp, offset_at_wp(wp, 0.30),
    ))

for invalid in (-0.01, 0.41, float("inf")):
    try:
        make_second_corner_right_path(first_corner, invalid)
    except ValueError:
        pass
    else:
        raise AssertionError("Invalid offset accepted: %s" % invalid)

print("[PASS] Official global path remains unchanged")
print("[PASS] First LEFT corner 0.30m and guardrail 0.40m preserved")
print("[PASS] Old WP195-238 exit correction absent; new WP231-300 shift RIGHT")
print("[PASS] Zero option exactly restores pre-test tracking path")
