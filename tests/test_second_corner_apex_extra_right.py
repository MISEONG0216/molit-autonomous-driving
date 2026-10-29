#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Offline regression: old 0.40m second-corner correction + apex +0.10m.

    python tests/test_second_corner_apex_extra_right.py

This check does not create sockets, run MORAI or send CtrlCmd.
"""
from pathlib import Path
import sys
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from integration.lane_offset import make_right_offset_path
from integration.first_corner_left_offset import make_first_corner_left_path
from integration.second_corner_right_offset import (
    make_second_corner_right_path,
    offset_at_wp as base_right_at_wp,
)
from integration.second_corner_apex_extra_right import (
    make_apex_extra_right_path,
    extra_at_wp,
    RAMP_IN_START_WP,
    FULL_START_WP,
    FULL_END_WP,
    RAMP_OUT_END_WP,
)

official = np.loadtxt(
    str(ROOT / "path" / "2026_molit_comp_global_path.txt"),
    usecols=(0, 1),
)
original = official.copy()

# Preserve the original working first-left and guardrail corrections.
guardrail = make_right_offset_path(official, 0.40)
first_turn = make_first_corner_left_path(guardrail, 0.30)
second_turn = make_second_corner_right_path(first_turn, 0.40)

# 0.0 option is an EXACT identity, and the original waypoint file is intact.
off = make_apex_extra_right_path(second_turn, 0.0)
assert np.array_equal(off, second_turn)
apex = make_apex_extra_right_path(second_turn, 0.10)
assert not np.shares_memory(apex, second_turn)
assert np.array_equal(official, original)

# Do not touch earlier first corner, WP231-248 entry, recovery WP292+, or
# existing guardrail adjustment WP1533-1655.
for wp in (0, 3, 125, 140, 150, 170, 185, 195, 205, 215,
           231, 235, 240, 245, 248, RAMP_IN_START_WP,
           RAMP_OUT_END_WP, 295, 300, 310, 760,
           1533, 1578, 1655, 1877, len(official) - 1):
    assert np.array_equal(apex[wp], second_turn[wp]), (
        "Unexpected changed WP: %d" % wp
    )

# Additional positive RIGHT shift only in the requested apex zone.
for wp in (250, 255, 260, 265, 270, 276, 280, 285, 290):
    dxdy = second_turn[wp + 3] - second_turn[wp - 3]
    unit = dxdy / np.linalg.norm(dxdy)
    right_normal = np.array([unit[1], -unit[0]])
    added = apex[wp] - second_turn[wp]
    actual = float(np.dot(added, right_normal))
    expected = extra_at_wp(wp, 0.10)
    assert abs(actual - expected) < 1e-6, (wp, actual, expected)
    assert abs(float(np.dot(added, unit))) < 1e-6, wp

for wp in range(FULL_START_WP, FULL_END_WP + 1):
    assert abs(extra_at_wp(wp, 0.10) - 0.10) < 1e-12
assert extra_at_wp(RAMP_IN_START_WP, 0.10) == 0.0
assert extra_at_wp(RAMP_OUT_END_WP, 0.10) == 0.0

for wp in (231, 240, 245, 250, 255, 260, 265, 270,
           276, 280, 285, 290, 292, 300):
    print(
        "WP %d: main RIGHT %.3fm + extra RIGHT %.3fm" %
        (wp, base_right_at_wp(wp, 0.40), extra_at_wp(wp, 0.10))
    )

for invalid in (-0.1, 0.16, float("inf")):
    try:
        make_apex_extra_right_path(second_turn, invalid)
    except ValueError:
        pass
    else:
        raise AssertionError("Invalid extra right accepted: %s" % invalid)

print("[PASS] Official global waypoints unchanged")
print("[PASS] First LEFT corner + guardrail correction fully preserved")
print("[PASS] Apex RIGHT +0.10m WP260-276, smooth WP248-292")
print("[PASS] Outside apex, original 0.40m second corner is unchanged")
print("[PASS] Zero extra offset restores previous tested profile")
