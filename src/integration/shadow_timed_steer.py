"""MORAI simulator-only timed shadow steering phases.

Timing starts at the first SHADOW controller loop. These are fixed-duration
steering requests, NOT verified lane changes. No control packets are sent here.
The integrated controller retains camera-lane validation and all STOP gates.
"""
import math


def timed_shadow_steer(elapsed_sec, steering_limit_rad=1.0):
    """Return (phase, override_rad); None means camera lane-center control.

    Timeline: 0-10 keep, 10-15 left, 15-30 keep, 30-35 right,
    >=35 keep. A positive control steer represents a left turn.
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
    if elapsed < 30.0:
        return "KEEP_AFTER_LEFT", None
    if elapsed < 35.0:
        return "FULL_RIGHT_5S", -limit
    return "KEEP_AFTER_RIGHT", None
