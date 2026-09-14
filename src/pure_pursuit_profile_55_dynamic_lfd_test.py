#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""55 km/h staged profile test using the validated 50 km/h dynamic-LFD setup.

Only the staged maximum-speed ceiling is raised from 50 to 55 km/h.
The following remain unchanged:
- Competition Vehicle Status speed source
- negative signed_vel clamp
- waypoint speed profile / braking preview
- dynamic LFD: clip(0.65 * speed_mps, 2.5, 9.0)
- Pure Pursuit geometry
- regulation-compliant ctrl_mode=2 / cmd_type=1 control

This is an intermediate validation step before any 57 km/h run.
"""

import pure_pursuit_profile_50_dynamic_lfd_test as dynamic_test


dynamic_test.profile_test.ALLOWED_TEST_MAX_KMH = 55.0


if __name__ == "__main__":
    print("[STAGE] 55 km/h dynamic-LFD validation")
    dynamic_test.profile_test.main()
