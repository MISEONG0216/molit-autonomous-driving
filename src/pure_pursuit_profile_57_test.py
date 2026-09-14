#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""57 km/h staged validation using the current best 55 km/h configuration.

Only one test variable changes relative to the validated 55 km/h setup:

    staged maximum speed: 55 -> 57 km/h

Kept unchanged:
- Competition Vehicle Status speed source
- negative signed_vel clamp
- waypoint speed profile
- braking preview = 2.0 m/s^2
- MAX_BRAKE_CMD = 0.10
- accel/coast/brake thresholds and gains
- speed-based dynamic LFD
- curvature-dependent LFD cap
- Pure Pursuit steering geometry
- regulation-compliant ctrl_mode=2 / cmd_type=1

57 km/h is still below the competition's normal-section 60 km/h limit and is
used here as the final normal-section staged target before broader integration.
"""

import pure_pursuit_profile_55_brake_cap_test as best55


profile_test = best55.profile_test
base = best55.base

# ONE staged change for this test.
profile_test.ALLOWED_TEST_MAX_KMH = 57.0


if __name__ == "__main__":
    print("[STAGE] 57 km/h validation from best 55 km/h configuration")
    print("[UNCHANGED] curvature LFD / preview=2.0 m/s^2 / brake cap=0.10")
    print("[CTRL] ctrl_mode=2 / cmd_type=1")
    profile_test.main()
