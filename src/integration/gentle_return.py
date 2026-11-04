"""Gentle return steering for an already LiDAR-verified, locked local path.

Does NOT generate or approve paths. Near endpoint, caller may fall back to
the existing short lookahead tail target. Safety stop arbitration stays in main.
"""
import math
import numpy as np


def gentle_return_pure_pursuit(path, ego_x, ego_y, yaw, vehicle_length,
                               lookahead_m=3.8):
    if not (3.0 <= lookahead_m <= 6.0):
        return None
    xy = np.asarray(path, dtype=float)
    if xy.ndim != 2 or xy.shape[1] < 2 or len(xy) < 2:
        return None
    xy = xy[:, :2]
    if not np.isfinite(xy).all():
        return None
    delta = xy - np.array([ego_x, ego_y], dtype=float)
    nearest = int(np.argmin(np.einsum("ij,ij->i", delta, delta)))
    c, s = math.cos(yaw), math.sin(yaw)
    for index in range(nearest, len(xy)):
        dx = c * delta[index, 0] + s * delta[index, 1]
        dy = -s * delta[index, 0] + c * delta[index, 1]
        d2 = dx * dx + dy * dy
        if dx <= 0.0 or d2 < lookahead_m * lookahead_m:
            continue
        # Use the ACTUAL target distance, not a shorter fixed denominator.
        steering = math.atan2(2.0 * vehicle_length * dy, d2)
        if not math.isfinite(steering):
            return None
        return {"nearest_index": nearest, "target_index": index,
                "steering": float(steering),
                "lookahead_m": float(math.sqrt(d2))}
    return None


def limit_return_steering(desired, previous, dt, max_abs_rad=0.22,
                           max_rate_rad_s=0.35):
    """Clamp return steer magnitude and slope; independent emergency STOP wins."""
    if not all(math.isfinite(x) for x in (desired, dt, max_abs_rad, max_rate_rad_s)):
        raise ValueError("non-finite return steering")
    if not (0.05 <= max_abs_rad <= 0.5 and 0.05 <= max_rate_rad_s <= 1.5):
        raise ValueError("invalid return steering limits")
    cmd = max(-max_abs_rad, min(max_abs_rad, float(desired)))
    if previous is None or not math.isfinite(previous):
        return cmd
    max_step = max_rate_rad_s * max(0., min(float(dt), 0.2))
    return max(float(previous) - max_step,
               min(float(previous) + max_step, cmd))
