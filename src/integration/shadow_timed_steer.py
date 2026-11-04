"""MORAI-only timed GPS-shadow steering experiment.

All timing is relative to first SHADOW loop. These commands do not establish
lane-change completion or correct camera selection of the desired lane.
No UDP or direct control commands originate in this module.
"""
import math


def timed_shadow_steer(elapsed_sec, steering_limit_rad=1.0):
    """Return (phase, override_rad); None requests camera lane-centering.

    0-10s: camera keep, 10-15s: left full, 15-18s: right full correction,
    18-33s: camera keep, 33-38s: right full, 38-41s: left full correction,
    >=41s: camera keep. +rad means steering left in current controller.
    """
    elapsed = float(elapsed_sec)
    limit = float(steering_limit_rad)
    if not (math.isfinite(elapsed) and math.isfinite(limit)
            and elapsed >= 0 and 0 < limit <= 1.0):
        return "INVALID", None
    if elapsed < 10.0:
        return "KEEP_BEFORE_LEFT", None
    if elapsed < 15.0:
        return "FULL_LEFT_5S", limit
    if elapsed < 18.0:
        return "CORRECT_RIGHT_3S", -limit
    if elapsed < 33.0:
        return "KEEP_AFTER_LEFT", None
    if elapsed < 38.0:
        return "FULL_RIGHT_5S", -limit
    if elapsed < 41.0:
        return "CORRECT_LEFT_3S", limit
    return "KEEP_AFTER_RIGHT", None
