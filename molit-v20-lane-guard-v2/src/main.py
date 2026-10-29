#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Team entrypoint for the current competition controller.

Current baseline:
- default staged maximum speed: 57 km/h
- Competition Vehicle Status / signed_vel speed source
- negative signed_vel clamp
- waypoint speed profile
- braking preview = 2.0 m/s^2
- MAX_BRAKE_CMD = 0.10
- speed-based dynamic LFD
- curvature-dependent LFD cap
- Pure Pursuit steering geometry
- regulation-compliant ctrl_mode=2 / cmd_type=1

The validated controller implementation is reused without changing its control
parameters. Use --drive only after MORAI UDP settings are confirmed.
"""

import pure_pursuit_profile_57_test as validated57


# Team main entrypoint defaults to the current 57 km/h baseline.
validated57.profile_test.DEFAULT_TEST_MAX_KMH = 57.0
validated57.profile_test.ALLOWED_TEST_MAX_KMH = 57.0


if __name__ == "__main__":
    print("[MAIN] MOLIT competition controller")
    print("[BASELINE] 57 km/h staged configuration")
    print("[CTRL] ctrl_mode=2 / cmd_type=1")
    validated57.profile_test.main()
