#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""55 km/h staged test: validated curvature-LFD + increased brake ceiling.

Purpose
-------
The previous 55 km/h curvature-capped LFD run improved lateral tracking, but
brake_cmd repeatedly reached the existing 0.06 ceiling. This test changes ONE
longitudinal variable only:

    MAX_BRAKE_CMD: 0.06 -> 0.10

Everything else remains unchanged:
- Competition Vehicle Status speed source
- negative signed_vel clamp
- waypoint speed profile and braking preview
- accel/coast/brake thresholds and gains
- longitudinal command slew-rate limit
- speed-based dynamic LFD + curvature cap
- Pure Pursuit steering geometry
- regulation-compliant ctrl_mode=2 / cmd_type=1

This is a staged validation file. Existing validated files are not modified.
"""

import pure_pursuit_profile_55_curvature_lfd_test as curvature_test


base = curvature_test.base
profile_test = curvature_test.profile_test

# Keep the 55 km/h stage ceiling.
profile_test.ALLOWED_TEST_MAX_KMH = 55.0

# ONE longitudinal change for this test.
ORIGINAL_MAX_BRAKE_CMD = base.MAX_BRAKE_CMD
TEST_MAX_BRAKE_CMD = 0.10
base.MAX_BRAKE_CMD = TEST_MAX_BRAKE_CMD


if __name__ == "__main__":
    print("[STAGE] 55 km/h curvature-LFD + stronger brake-cap validation")
    print(
        f"[BRAKE CAP] {ORIGINAL_MAX_BRAKE_CMD:.2f} -> "
        f"{TEST_MAX_BRAKE_CMD:.2f}"
    )
    print("[UNCHANGED] braking preview / brake trigger / brake Kp / lateral control")
    profile_test.main()
