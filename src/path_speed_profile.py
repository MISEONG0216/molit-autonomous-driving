#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Offline path analysis for the 2026 competition global path.

Purpose
-------
1) Compute a robust curvature estimate for all 4,430 waypoints.
2) Classify straight / curve severity from path geometry.
3) Build an INITIAL TEST target-speed profile.
4) Add preview braking so the target speed falls before a sharp curve.
5) Print checkpoint-to-checkpoint summaries for tuning.

Important
---------
- This script does NOT drive MORAI.
- The competition rule limit is handled separately from map metadata.
- Normal sections are capped below 60 km/h. We use 57 km/h as the initial
  straight-line test target to keep margin below the 60 km/h rule threshold.
- The curvature-to-speed bands and preview deceleration below are TEAM TUNING
  PARAMETERS, not values stated in the competition rulebook.
- The high-speed exception section (roughly waypoint 2277~3528) is identified,
  but this first profile intentionally keeps it at the same 57 km/h ceiling.
  Higher speed there should be tested only after the 55~59 km/h baseline is stable.
"""

import argparse
import csv
import math
from pathlib import Path

import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[1]
PATH_FILE = PROJECT_ROOT / "path" / "2026_molit_comp_global_path.txt"

# Official-rule-derived route references already mapped to the official path.
HIGH_SPEED_START_INDEX = 2277
HIGH_SPEED_END_INDEX = 3528

# Checkpoint indices matched exactly to the official global path.
CHECKPOINTS = [
    ("START", 0),
    ("CP1", 146),
    ("CP2", 378),
    ("CP3", 483),
    ("CP4", 704),
    ("CP5", 961),
    ("CP6", 1185),
    ("CP7", 1297),
    ("CP8", 1745),
    ("CP9", 1895),
    ("CP10", 2275),
    ("CP11", 2611),
    ("CP12", 3231),
    ("CP13", 3529),
    ("CP14", 3768),
    ("CP15", 4045),
    ("END", 4429),
]

# Curvature estimation window. Path spacing is about 0.5 m, so 20 waypoints
# is roughly 10 m on either side and suppresses waypoint-scale noise.
CURVATURE_WINDOW = 20

# First high-speed baseline: stay below the competition 60 km/h threshold.
STRAIGHT_TARGET_KMH = 57.0
NORMAL_RULE_TEST_CAP_KMH = 57.0
HIGH_SPEED_TEST_CAP_KMH = 57.0

# Team tuning parameters for the FIRST test profile.
# These are deliberately conservative and should be replaced by measured data.
RADIUS_SPEED_BANDS = [
    (20.0, 15.0),
    (35.0, 22.0),
    (60.0, 30.0),
    (100.0, 40.0),
    (180.0, 48.0),
    (350.0, 53.0),
    (float("inf"), STRAIGHT_TARGET_KMH),
]

# Preview braking design assumption for initial tuning, not from the rulebook.
PREVIEW_DECEL_MPS2 = 2.0


def load_path(path_file):
    points = []
    with open(path_file, "r") as f:
        for line in f:
            values = line.split()
            if len(values) >= 2:
                points.append((float(values[0]), float(values[1])))

    if not points:
        raise RuntimeError("전역경로 파일에 waypoint가 없습니다.")

    return np.asarray(points, dtype=float)


def cumulative_distance(path):
    ds = np.hypot(np.diff(path[:, 0]), np.diff(path[:, 1]))
    s = np.zeros(len(path), dtype=float)
    s[1:] = np.cumsum(ds)
    return s, ds


def three_point_curvature(p1, p2, p3):
    """Return absolute curvature (1/m) of the circle through 3 points."""
    a = math.hypot(*(p2 - p1))
    b = math.hypot(*(p3 - p2))
    c = math.hypot(*(p3 - p1))

    denom = a * b * c
    if denom < 1e-9:
        return 0.0

    cross = abs(
        (p2[0] - p1[0]) * (p3[1] - p1[1])
        - (p2[1] - p1[1]) * (p3[0] - p1[0])
    )

    # curvature = 4*Area/(abc), and 2*Area = |cross|
    return (2.0 * cross) / denom


def compute_curvature(path, window=CURVATURE_WINDOW):
    n = len(path)
    curvature = np.zeros(n, dtype=float)

    # The official path is closed. Wrap around for START/END continuity.
    for i in range(n):
        i1 = (i - window) % n
        i3 = (i + window) % n
        curvature[i] = three_point_curvature(path[i1], path[i], path[i3])

    return curvature


def curvature_to_radius(curvature):
    radius = np.full_like(curvature, np.inf, dtype=float)
    mask = curvature > 1e-8
    radius[mask] = 1.0 / curvature[mask]
    return radius


def radius_to_seed_speed(radius_m):
    for radius_limit, speed_kmh in RADIUS_SPEED_BANDS:
        if radius_m < radius_limit:
            return speed_kmh
    return STRAIGHT_TARGET_KMH


def build_seed_profile(radius):
    speeds = np.array([radius_to_seed_speed(r) for r in radius], dtype=float)

    # For now BOTH normal and high-speed-exception sections use the same
    # sub-60 baseline. The exception is preserved in the data for later tests.
    for i in range(len(speeds)):
        if HIGH_SPEED_START_INDEX <= i <= HIGH_SPEED_END_INDEX:
            speeds[i] = min(speeds[i], HIGH_SPEED_TEST_CAP_KMH)
        else:
            speeds[i] = min(speeds[i], NORMAL_RULE_TEST_CAP_KMH)

    return speeds


def apply_braking_preview(seed_speed_kmh, ds, decel_mps2=PREVIEW_DECEL_MPS2):
    """
    Backward pass: if a slower waypoint is ahead, lower earlier targets so
    deceleration begins before reaching the curve.
    """
    profile = seed_speed_kmh.copy()

    # Use one backward pass over the open representation of the lap.
    for i in range(len(profile) - 2, -1, -1):
        next_v = profile[i + 1] / 3.6
        distance = max(float(ds[i]), 1e-6)
        allowable_v = math.sqrt(max(0.0, next_v * next_v + 2.0 * decel_mps2 * distance))
        allowable_kmh = allowable_v * 3.6
        profile[i] = min(profile[i], allowable_kmh)

    return profile


def severity_from_radius(radius_m):
    if radius_m < 20.0:
        return "VERY_SHARP"
    if radius_m < 35.0:
        return "SHARP"
    if radius_m < 60.0:
        return "CURVE"
    if radius_m < 100.0:
        return "MEDIUM"
    if radius_m < 180.0:
        return "GENTLE"
    if radius_m < 350.0:
        return "VERY_GENTLE"
    return "STRAIGHT"


def segment_summary(name_a, idx_a, name_b, idx_b, s, radius, profile):
    lo = min(idx_a, idx_b)
    hi = max(idx_a, idx_b)
    segment_radius = radius[lo : hi + 1]
    segment_speed = profile[lo : hi + 1]

    finite_r = segment_radius[np.isfinite(segment_radius)]
    if len(finite_r):
        r_min = float(np.min(finite_r))
        r_p05 = float(np.percentile(finite_r, 5))
    else:
        r_min = float("inf")
        r_p05 = float("inf")

    return {
        "name": f"{name_a}->{name_b}",
        "distance_m": float(s[hi] - s[lo]),
        "min_radius_m": r_min,
        "r05_m": r_p05,
        "min_target_kmh": float(np.min(segment_speed)),
        "max_target_kmh": float(np.max(segment_speed)),
    }


def print_summary(path, s, curvature, radius, profile):
    print("============================================================")
    print(" OFFLINE PATH / SPEED PROFILE ANALYSIS")
    print("============================================================")
    print(f"Path file       : {PATH_FILE}")
    print(f"Waypoint count  : {len(path)}")
    print(f"Lap length      : {s[-1]:.1f} m")
    print(f"Curvature window: +/-{CURVATURE_WINDOW} waypoints")
    print(f"Straight target : {STRAIGHT_TARGET_KMH:.1f} km/h")
    print(f"Preview decel   : {PREVIEW_DECEL_MPS2:.1f} m/s^2  [TEAM TUNING PARAMETER]")
    print(
        f"High-speed rule-exception reference: idx "
        f"{HIGH_SPEED_START_INDEX}~{HIGH_SPEED_END_INDEX}"
    )
    print("NOTE: exception section is still capped at 57 km/h in this first test profile.")
    print()

    print("Checkpoint segment summary")
    print("-" * 92)
    print(f"{'SEGMENT':<14} {'DIST(m)':>9} {'Rmin(m)':>10} {'R05(m)':>10} {'Vmin':>8} {'Vmax':>8}")
    print("-" * 92)

    for (name_a, idx_a), (name_b, idx_b) in zip(CHECKPOINTS[:-1], CHECKPOINTS[1:]):
        row = segment_summary(name_a, idx_a, name_b, idx_b, s, radius, profile)
        print(
            f"{row['name']:<14} "
            f"{row['distance_m']:>9.1f} "
            f"{row['min_radius_m']:>10.1f} "
            f"{row['r05_m']:>10.1f} "
            f"{row['min_target_kmh']:>8.1f} "
            f"{row['max_target_kmh']:>8.1f}"
        )

    print("-" * 92)
    print()

    # Show strongest curvature points while suppressing nearby duplicate peaks.
    order = np.argsort(curvature)[::-1]
    selected = []
    min_gap = 30
    for idx in order:
        if all(min(abs(int(idx) - j), len(path) - abs(int(idx) - j)) >= min_gap for j in selected):
            selected.append(int(idx))
        if len(selected) >= 12:
            break

    print("Strongest curvature locations (for MORAI test logging)")
    print("-" * 76)
    print(f"{'WP':>6} {'S(m)':>9} {'R(m)':>10} {'TYPE':>13} {'TARGET':>9}")
    print("-" * 76)
    for idx in selected:
        print(
            f"{idx:>6d} {s[idx]:>9.1f} {radius[idx]:>10.1f} "
            f"{severity_from_radius(radius[idx]):>13} {profile[idx]:>8.1f}"
        )
    print("-" * 76)


def export_csv(output_file, path, s, curvature, radius, seed_speed, profile):
    output_file.parent.mkdir(parents=True, exist_ok=True)

    with open(output_file, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([
            "waypoint_index",
            "x",
            "y",
            "distance_m",
            "curvature_1_per_m",
            "radius_m",
            "severity",
            "high_speed_exception",
            "seed_target_kmh",
            "preview_target_kmh",
        ])

        for i in range(len(path)):
            writer.writerow([
                i,
                f"{path[i, 0]:.6f}",
                f"{path[i, 1]:.6f}",
                f"{s[i]:.3f}",
                f"{curvature[i]:.8f}",
                "inf" if not math.isfinite(radius[i]) else f"{radius[i]:.3f}",
                severity_from_radius(radius[i]),
                int(HIGH_SPEED_START_INDEX <= i <= HIGH_SPEED_END_INDEX),
                f"{seed_speed[i]:.2f}",
                f"{profile[i]:.2f}",
            ])

    print()
    print(f"CSV saved: {output_file}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--csv",
        default=str(PROJECT_ROOT / "logs" / "path_speed_profile.csv"),
        help="분석 CSV 출력 경로",
    )
    args = parser.parse_args()

    path = load_path(PATH_FILE)
    s, ds = cumulative_distance(path)
    curvature = compute_curvature(path)
    radius = curvature_to_radius(curvature)
    seed_speed = build_seed_profile(radius)
    profile = apply_braking_preview(seed_speed, ds)

    print_summary(path, s, curvature, radius, profile)
    export_csv(Path(args.csv), path, s, curvature, radius, seed_speed, profile)


if __name__ == "__main__":
    main()
