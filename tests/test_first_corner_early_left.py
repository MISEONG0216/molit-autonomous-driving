#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Offline test for isolated WP144 early LEFT steering.

    python tests/test_first_corner_early_left.py

No network access, simulator, or CtrlCmd is used.
"""
from pathlib import Path
import math
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from integration.first_corner_early_left import (
    add_early_left_steering,
    weight_at_wp,
    RAMP_BEGIN_WP,
    FULL_BEGIN_WP,
    FULL_END_WP,
    RAMP_END_WP,
)


def close(a, b, tol=1e-9):
    assert abs(a - b) <= tol, (a, b)


def test_all():
    baseline = math.radians(1.23)

    # Original output is untouched outside the local first corner.
    for wp in (3, 120, 139, 140, 163, 164, 200, 250, 265, 1500, 1851):
        output, added = add_early_left_steering(baseline, wp, 2.0)
        close(output, baseline)
        close(added, 0.0)

    # At WP144 image location: V7 +1.23deg becomes about +3.23deg.
    for wp in range(FULL_BEGIN_WP, FULL_END_WP + 1):
        output, added = add_early_left_steering(baseline, wp, 2.0)
        close(output, baseline + math.radians(2.0))
        close(added, 2.0)

    wp142, amount142 = add_early_left_steering(baseline, 142, 2.0)
    assert 0.0 < amount142 < 2.0
    assert wp142 > baseline
    assert 0.0 < weight_at_wp(155) < 1.0
    assert weight_at_wp(160) < weight_at_wp(155)

    # Disabled option reproduces V22 exactly.
    for wp in range(0, 500, 7):
        output, added = add_early_left_steering(baseline, wp, 0.0)
        close(output, baseline)
        close(added, 0.0)

    # Clip at existing actuator bounds, not outside command range.
    output, added = add_early_left_steering(0.999, 144, 4.0)
    assert output <= 1.0
    assert added >= 0.0

    print("[PASS] WP140..163 only; other WPs unchanged")
    print("[PASS] First left turn at WP144: baseline +1.23deg plus 2.0deg")
    print("[PASS] Smooth ramp-in/ramp-out around WP144")
    print("[PASS] Option 0 reproduces original controller steering")


if __name__ == "__main__":
    test_all()
