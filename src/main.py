#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Team entrypoint for the current validated competition controller.

Current baseline:
- staged maximum speed: 57 km/h
- Competition Vehicle Status / signed_vel speed source
- negative signed_vel clamp
- waypoint speed profile
- braking preview = 2.0 m/s^2
- MAX_BRAKE_CMD = 0.10
- speed-based dynamic LFD
- curvature-dependent LFD cap
- Pure Pursuit steering geometry
- regulation-compliant ctrl_mode=2 / cmd_type=1

This entrypoint reuses the validated 57 km/h controller without changing any
control parameter. Use --drive only after MORAI UDP settings are confirmed.
"""

import pure_pursuit_profile_57_test as validated57


if __name__ == "__main__":
    print("[MAIN] MOLIT competition controller")
    print("[BASELINE] validated 57 km/h staged configuration")
    print("[CTRL] ctrl_mode=2 / cmd_type=1")
    validated57.profile_test.main()
