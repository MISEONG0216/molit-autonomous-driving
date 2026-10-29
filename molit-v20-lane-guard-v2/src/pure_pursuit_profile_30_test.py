#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""30 km/h staged profile test wrapper.

Reuses the validated pure_pursuit_profile_test controller unchanged, but raises
only the explicit test ceiling from 20 to 30 km/h. The default remains 10 km/h,
so 30 km/h is enabled only when --max-speed 30 is supplied.
"""

import pure_pursuit_profile_test as profile_test


profile_test.ALLOWED_TEST_MAX_KMH = 30.0


if __name__ == "__main__":
    profile_test.main()
