#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Offline check for opt-in WP1835-1892 roundabout speed cap.

Run from repository root with the existing Python 3.8 virtual environment:
    python tests/test_roundabout_speed_cap.py

No simulator, UDP, or vehicle CtrlCmd is used.
"""

from pathlib import Path
import math
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from integration.roundabout_speed_cap import (
    build_roundabout_speed_caps,
    HOLD_START_WP, HOLD_END_WP,
    PREVIEW_DECEL_MPS2, RELEASE_ACCEL_MPS2,
)

path = np.loadtxt(
    str(ROOT / "path" / "2026_molit_comp_global_path.txt"),
    usecols=(0, 1),
)
original = path.copy()

off_caps, off_zone = build_roundabout_speed_caps(path, 25.0, 0.0)
assert all(x == 25.0 for x in off_caps)
assert all(x == "OFF" for x in off_zone)

caps, zone = build_roundabout_speed_caps(path, 25.0, 12.0)
assert len(caps) == len(path)
assert len(zone) == len(path)
assert np.array_equal(path, original), "Official path was modified"

for i in range(HOLD_START_WP, HOLD_END_WP + 1):
    assert abs(caps[i] - 12.0) < 1e-9, (i, caps[i])
    assert zone[i] == "HOLD", (i, zone[i])

assert caps[1790] > caps[1800] > caps[1810] > caps[1820]
assert caps[1820] > caps[1830] > caps[1835]
assert caps[1892] < caps[1900] < caps[1910] < caps[1920]
assert abs(caps[1700] - 25.0) < 1e-9
assert abs(caps[1950] - 25.0) < 1e-9
assert zone[1700] == "OFF"
assert zone[1825] == "PREBRAKE"
assert zone[1910] == "RELEASE"
assert zone[1950] == "OFF"

assert 0.0 < PREVIEW_DECEL_MPS2 <= 1.0
assert 0.0 < RELEASE_ACCEL_MPS2 <= 1.0

for i in (1780, 1790, 1800, 1810, 1820, 1830, 1835,
          1840, 1850, 1870, 1885, 1892, 1900, 1910, 1920, 1940):
    print(
        "WP %4d: speed cap %5.2f km/h  (%s)" %
        (i, caps[i], zone[i])
    )

for invalid in (-1.0, 5.0, 21.0):
    try:
        build_roundabout_speed_caps(path, 25.0, invalid)
    except ValueError:
        pass
    else:
        raise AssertionError("Invalid cap speed accepted: %s" % invalid)

print("[PASS] Official path unchanged and 0=OFF behaves like original")
print("[PASS] Hold 12km/h at WP1835-1892")
print("[PASS] Gradual prebraking and speed recovery")
print("[PASS] All other WPs retain 25km/h maximum")
