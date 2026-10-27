#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""57 km/h corner V2 test.

Changes from the validated 57 km/h baseline:
1) Remove the global 5~57 km/h min/max normalization.
   Use the curvature-based full profile directly (capped by --max-speed).
2) Add a forward acceleration-feasibility pass after the existing backward
   braking preview so corner exits have an explicit, measurable target ramp.

Unchanged:
- Competition Vehicle Status speed source / negative-speed clamp
- curvature-capped dynamic LFD and Pure Pursuit steering
- longitudinal controller / command limiter / brake cap
- ctrl_mode=2 / cmd_type=1
- existing validated baseline files
"""

import csv
import math
from datetime import datetime

import numpy as np

import pure_pursuit_profile_57_test as validated57

profile_test = validated57.profile_test
base = validated57.best55.curvature_test.base
profile_builder = profile_test.profile_builder

# V2 is still limited to the already-tested 57 km/h ceiling.
profile_test.DEFAULT_TEST_MAX_KMH = 57.0
profile_test.ALLOWED_TEST_MAX_KMH = 57.0

# Team tuning parameter for this V2 experiment, not a rulebook value.
# This only shapes the target-speed ramp after slower sections.
EXIT_ACCEL_MPS2 = 3.0


def apply_forward_accel_limit(profile_kmh, ds, accel_mps2=EXIT_ACCEL_MPS2):
    """Forward pass limiting how quickly target speed may rise after a curve."""
    profile = np.asarray(profile_kmh, dtype=float).copy()

    for i in range(1, len(profile)):
        prev_v = profile[i - 1] / 3.6
        distance = max(float(ds[i - 1]), 1e-6)
        allowable_v = math.sqrt(
            max(0.0, prev_v * prev_v + 2.0 * accel_mps2 * distance)
        )
        profile[i] = min(profile[i], allowable_v * 3.6)

    return profile


def build_corner_v2_profile(global_path, max_speed_kmh):
    """Build V2 without the old global 5~57 normalization."""
    _, ds = profile_builder.cumulative_distance(global_path)
    curvature = profile_builder.compute_curvature(global_path)
    radius = profile_builder.curvature_to_radius(curvature)

    seed = profile_builder.build_seed_profile(radius)
    braking_profile = profile_builder.apply_braking_preview(seed, ds)

    # Directly preserve the physical meaning of the curvature speed bands.
    capped_profile = np.minimum(braking_profile, float(max_speed_kmh))

    # Explicit exit ramp for repeatable corner-exit tuning.
    target_profile = apply_forward_accel_limit(capped_profile, ds)

    return target_profile, radius, braking_profile


class V2DiagnosticWriter:
    def __init__(self, writer):
        self.writer = writer

    def writerow(self, row):
        self.writer.writerow(list(row) + [f"{EXIT_ACCEL_MPS2:.2f}"])


def open_run_log_v2(max_speed_kmh, drive_mode):
    log_dir = base.PROJECT_ROOT / "logs"
    log_dir.mkdir(parents=True, exist_ok=True)

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    mode = "drive" if drive_mode else "dryrun"
    speed_tag = f"{max_speed_kmh:g}kph_corner_v2"
    log_path = log_dir / f"run_{timestamp}_{speed_tag}_{mode}.csv"

    log_file = open(log_path, "w", newline="", buffering=1)
    raw_writer = csv.writer(log_file)
    raw_writer.writerow([
        "elapsed_sec",
        "waypoint_index",
        "x_m",
        "y_m",
        "yaw_deg",
        "path_dist_m",
        "curve_radius_m",
        "full_profile_kmh",
        "target_speed_kmh",
        "comp_speed_kmh",
        "speed_error_kmh",
        "ctrl_zone",
        "steering_rad",
        "steering_deg",
        "long_cmd",
        "accel_cmd",
        "brake_cmd",
        "exit_accel_profile_mps2",
    ])
    log_file.flush()
    return log_path, log_file, V2DiagnosticWriter(raw_writer)


# Inject V2 behavior only for this executable. Baseline files remain untouched.
profile_test.build_test_profile = build_corner_v2_profile
profile_test.open_run_log = open_run_log_v2


if __name__ == "__main__":
    print("[STAGE] 57 km/h CORNER V2")
    print("[PROFILE] global 5~57 normalization: OFF")
    print("[BRAKING] existing backward preview: ON")
    print(f"[EXIT] forward acceleration profile: {EXIT_ACCEL_MPS2:.1f} m/s^2")
    print("[BASELINE] Pure Pursuit / longitudinal control / UDP logic unchanged")
    profile_test.main()
