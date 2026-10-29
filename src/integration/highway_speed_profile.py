"""Experimental highway target profile; preserve validated V7 outside zones.

This is a *simulator-only* speed-target extension. Lateral tracking and
braking at 120 km/h are NOT validated. The last normal-road section and
unverified toll exit remain at the existing V7 target (<=57).
"""
import math
import numpy as np


def build_highway_profile(v7_target, radii, path, fast_zones, request_max_kmh,
                          a_lat_mps2=2.0, preview_decel_mps2=2.0,
                          accel_mps2=3.0):
    speed = np.asarray(v7_target, dtype=float).copy()
    radii = np.asarray(radii, dtype=float)
    if speed.ndim != 1 or len(speed) != len(radii) or len(speed) != len(path):
        raise ValueError("Speed/radius/path shape mismatch")
    if request_max_kmh <= 57.0:
        return speed
    if not all(math.isfinite(x) and x > 0 for x in (
            a_lat_mps2, preview_decel_mps2, accel_mps2)):
        raise ValueError("Positive finite acceleration parameters required")

    enabled = np.zeros(len(speed), dtype=bool)
    for start, end in fast_zones:
        if not (type(start) is int and type(end) is int
                and 0 <= start <= end < len(speed)):
            raise ValueError("Invalid highway waypoint zone")
        enabled[start:end+1] = True

    # Curvature-limited seed, not a blind 120 km/h constant.
    for i in range(len(speed)):
        if enabled[i]:
            radius = float(radii[i])
            lateral_cap = (math.sqrt(a_lat_mps2 * max(0.0, radius)) * 3.6
                           if math.isfinite(radius) else float(request_max_kmh))
            speed[i] = min(float(request_max_kmh), lateral_cap)
            if not math.isfinite(speed[i]):
                speed[i] = float(v7_target[i])
            # Keep V7 speed on comparatively tight turns.
            if radius < 150.0:
                speed[i] = min(speed[i], float(v7_target[i]))

    # Braking preview at every upcoming turn and speed-zone exit. Backward
    # feasibility can lower speeds BEFORE the zone; never raise normal V7 cap.
    ds = np.linalg.norm(np.diff(np.asarray(path)[:, :2], axis=0), axis=1)
    for i in range(len(speed)-2, -1, -1):
        step = max(0.0, float(ds[i]))
        next_v = speed[i+1] / 3.6
        max_v = math.sqrt(next_v * next_v + 2 * preview_decel_mps2 * step) * 3.6
        speed[i] = min(speed[i], max_v)
    for i in range(1, len(speed)):
        step = max(0.0, float(ds[i-1]))
        prev_v = speed[i-1] / 3.6
        max_v = math.sqrt(prev_v * prev_v + 2 * accel_mps2 * step) * 3.6
        speed[i] = min(speed[i], max_v)
    return np.maximum(0.0, speed)
