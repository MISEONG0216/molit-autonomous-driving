#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""50 km/h staged profile test with validated negative-speed clamp.

Reuses the 45 km/h clamp-validation controller unchanged and raises only the
explicit staged-test ceiling to 50 km/h. The waypoint speed profile, lateral
Pure Pursuit logic, braking preview, Competition Vehicle Status speed source,
and negative signed_vel guard are otherwise unchanged.
"""

import pure_pursuit_profile_45_clamp_test as clamp_test


clamp_test.profile_test.ALLOWED_TEST_MAX_KMH = 50.0


if __name__ == "__main__":
    clamp_test.profile_test.main()
