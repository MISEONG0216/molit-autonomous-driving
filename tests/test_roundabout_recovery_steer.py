#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Offline tests for the roundabout exit recovery steering gate.

Run from repository root:
    python tests/test_roundabout_recovery_steer.py
No MORAI simulation, network, or CtrlCmd is used.
"""
from pathlib import Path
import math
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from integration.roundabout_recovery_steer import (
    apply_recovery_right_steering,
    signed_left_error_m,
    waypoint_weight,
    START_WP, FULL_START_WP, FULL_END_WP, END_WP,
)

# Straight northbound path: LEFT is negative world X.
path = [(0.0, float(i)) for i in range(2000)]
before = tuple(path)
baseline_steer = -0.12

def calculate(wp, x, strength=2.0):
    return apply_recovery_right_steering(
        baseline_steer, path, wp, x, float(wp), strength
    )

def close(a, b, tol=1e-9):
    assert abs(a-b) < tol, (a, b)

for wp in (1533, 1655, 1849, 1851, 1862, 1891, 1900, 1920):
    steer, added, left = calculate(wp, -1.2)
    close(steer, baseline_steer)
    close(added, 0.0)
    close(left, 1.2)

# Full recovery only at middle of the requested WP interval.
for wp in (1870, 1877, 1881):
    steer, added, left = calculate(wp, -1.2)
    close(left, 1.2)
    close(added, 2.0)
    close(steer, baseline_steer - math.radians(2.0))

# Do NOT steer further RIGHT if vehicle has already reached or crossed path.
for x in (0.0, 0.3, 1.2, -0.34):
    steer, added, left = calculate(1877, x)
    close(steer, baseline_steer)
    close(added, 0.0)

# Smooth reduction by lateral error and waypoint, and zero-option identity.
_, partial, _ = calculate(1877, -0.65)
assert 0.0 < partial < 2.0, partial
_, ramp, _ = calculate(1866, -1.2)
assert 0.0 < ramp < 2.0, ramp
assert waypoint_weight(1877) == 1.0
assert waypoint_weight(1891) == 0.0
close(calculate(1877, -1.2, 0.0)[0], baseline_steer)
close(signed_left_error_m(path, 1877, -1.2, 1877.0), 1.2)
close(signed_left_error_m(path, 1877, 1.2, 1877.0), -1.2)
assert tuple(path) == before

print("[PASS] 2deg maximum RIGHT steering inside WP1862-1891")
print("[PASS] Left-of-path conditional gating and smooth taper")
print("[PASS] Normal V22 driving and earlier lane tune unchanged elsewhere")
print("[PASS] Official path input not mutated")
