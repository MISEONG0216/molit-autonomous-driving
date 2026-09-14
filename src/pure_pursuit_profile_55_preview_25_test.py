#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""55 km/h staged test: curvature-LFD + brake cap 0.10 + stronger preview.

Purpose
-------
The 55 km/h curvature-LFD test with MAX_BRAKE_CMD=0.10 improved corner
tracking and reduced peak path error. This test keeps that validated setup and
changes ONE additional longitudinal profile parameter:

    PREVIEW_DECEL_MPS2: 2.0 -> 2.5 m/s^2

Important: in path_speed_profile.py, apply_braking_preview() has the decel value
bound as a default argument at function definition time. Merely changing the
module constant would therefore NOT change the generated profile. This staged
wrapper explicitly rebuilds the same profile while passing decel_mps2=2.5.

Everything else remains unchanged:
- max staged speed: 55 km/h
- Competition Vehicle Status speed source
- negative signed_vel clamp
- MAX_BRAKE_CMD=0.10
- brake trigger / brake Kp / accel policy
- longitudinal command slew-rate limit
- speed-based dynamic LFD + curvature cap
- Pure Pursuit steering geometry
- regulation-compliant ctrl_mode=2 / cmd_type=1
"""

import numpy as np

import pure_pursuit_profile_55_brake_cap_test as brake_test


curvature_test = brake_test.curvature_test
profile_test = brake_test.profile_test
profile_builder = profile_test.profile_builder

profile_test.ALLOWED_TEST_MAX_KMH = 55.0

ORIGINAL_PREVIEW_DECEL_MPS2 = profile_builder.PREVIEW_DECEL_MPS2
TEST_PREVIEW_DECEL_MPS2 = 2.5


def build_test_profile_preview_25(global_path, max_speed_kmh):
    """Rebuild the validated staged profile with only preview decel changed."""
    _, ds = profile_builder.cumulative_distance(global_path)
    curvature = profile_builder.compute_curvature(global_path)
    radius = profile_builder.curvature_to_radius(curvature)
    seed = profile_builder.build_seed_profile(radius)

    full_profile = profile_builder.apply_braking_preview(
        seed,
        ds,
        decel_mps2=TEST_PREVIEW_DECEL_MPS2,
    )

    full_min = float(np.min(full_profile))
    full_max = float(np.max(full_profile))

    if full_max - full_min < 1e-6:
        test_profile = np.full_like(full_profile, max_speed_kmh)
    else:
        normalized = (full_profile - full_min) / (full_max - full_min)
        test_profile = profile_test.TEST_MIN_KMH + normalized * (
            max_speed_kmh - profile_test.TEST_MIN_KMH
        )

    # Curvature-LFD controller needs the same radius profile for its LFD caps.
    curvature_test._RADIUS_PROFILE = radius

    return test_profile, radius, full_profile


# Test-only injection. Existing validated files remain unchanged.
profile_test.build_test_profile = build_test_profile_preview_25


if __name__ == "__main__":
    print("[STAGE] 55 km/h curvature-LFD + brake cap 0.10 + preview 2.5")
    print(
        f"[PREVIEW DECEL] {ORIGINAL_PREVIEW_DECEL_MPS2:.1f} -> "
        f"{TEST_PREVIEW_DECEL_MPS2:.1f} m/s^2"
    )
    print(f"[BRAKE CAP] {brake_test.TEST_MAX_BRAKE_CMD:.2f} (unchanged)")
    print("[UNCHANGED] brake trigger / brake Kp / accel policy / lateral control")
    profile_test.main()
