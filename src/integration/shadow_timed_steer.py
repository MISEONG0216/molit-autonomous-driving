"""MORAI-only timed shadow steering request; no socket or vehicle control.

Times are measured from the first SHADOW-mode controller loop.
A fixed one-second maximum steer does NOT guarantee a lane change.
All obstacle and perception STOP gates must be handled by main.
"""
import math


def timed_shadow_steer(elapsed_sec, steering_limit_rad=0.22):
    """Return (phase, override_rad) with None for camera lane keeping.

    Positive steering is left per MORAI control convention used here.
    """
    elapsed = float(elapsed_sec)
    limit = float(steering_limit_rad)
    if not (math.isfinite(elapsed) and math.isfinite(limit)
            and elapsed >= 0 and 0 < limit <= 1.0):
        return "INVALID", None
    if elapsed < 4.0:
        return "KEEP_BEFORE_LEFT", None
    if elapsed < 5.0:
        return "FULL_LEFT_1S", limit
    if elapsed < 14.0:
        return "KEEP_AFTER_LEFT", None
    if elapsed < 15.0:
        return "FULL_RIGHT_1S", -limit
    return "KEEP_AFTER_RIGHT", None
