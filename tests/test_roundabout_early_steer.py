#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Offline unit test of bounded local right-turn steering, no MORAI needed.

    python tests/test_roundabout_early_steer.py
"""

from pathlib import Path
import math
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from integration.roundabout_early_steer import (
    add_early_right_steering, weight_at_wp,
    RAMP_IN_START_WP, FULL_START_WP, FULL_END_WP,
    RAMP_OUT_END_WP,
)


def close(a, b, tol=1e-9):
    assert abs(a - b) <= tol, (a, b)


def main():
    for wp in (0, 1533, 1715, 1800, 1842, 1843, 1866, 1870,
               1876, 1882, 1900, 2000):
        output, actual = add_early_right_steering(0.12, wp, 4.0)
        close(output, 0.12)
        close(actual, 0.0)
    assert weight_at_wp(1846) > 0.0
    assert weight_at_wp(1850) > weight_at_wp(1846)
    for wp in range(FULL_START_WP, FULL_END_WP + 1):
        output, actual = add_early_right_steering(0.12, wp, 4.0)
        close(output, 0.12 - math.radians(4.0))
        close(actual, 4.0)
    assert weight_at_wp(1860) < 1.0
    assert weight_at_wp(1865) < weight_at_wp(1860)
    for wp in (1848, 1851, 1858, 1862):
        output, actual = add_early_right_steering(-0.25, wp, 4.0)
        assert output <= -0.25, (wp, output)
        assert 0.0 < actual <= 4.0, (wp, actual)
    assert add_early_right_steering(-0.2, 1853, 0.0) == (-0.2, 0.0)
    print("[PASS] WP1843-1866 only, all other WPs unchanged")
    print("[PASS] RIGHT sign is negative MORAI steering")
    print("[PASS] smooth 4 degree addition with fade-in/out")
    print("[PASS] Zero option keeps original V22 output")


if __name__ == "__main__":
    main()
