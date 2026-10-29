#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Read-only path-tracking diagnostics for V20 video/CSV correlation.

The sign is relative to waypoint travel direction:
+ signed_cte_m = ego is LEFT of the reference path.
- signed_cte_m = ego is RIGHT of the reference path.
This is distance to the OFFICIAL GLOBAL PATH, not to a camera lane line.
"""
import math


def _wrap_rad(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def path_tracking_metrics(path, nearest_index, ego_x, ego_y, ego_yaw):
    """Project ego onto nearby path segments (not just nearest waypoint).

    Scanning +/- 3 segments handles sparse waypoint spacing and reduces
    nearest-waypoint distance quantization. No alteration of control signals.
    """
    count = len(path)
    if count < 2:
        raise ValueError("path requires at least two waypoints")

    index = max(0, min(count - 1, int(nearest_index)))
    start = max(0, index - 3)
    stop = min(count - 2, index + 3)
    best = None

    for i in range(start, stop + 1):
        ax, ay = float(path[i][0]), float(path[i][1])
        bx, by = float(path[i + 1][0]), float(path[i + 1][1])
        vx, vy = bx - ax, by - ay
        length2 = vx * vx + vy * vy
        if length2 < 1e-12:
            continue

        fraction = max(
            0.0,
            min(1.0, ((ego_x - ax) * vx + (ego_y - ay) * vy) / length2),
        )
        px, py = ax + fraction * vx, ay + fraction * vy
        ex, ey = ego_x - px, ego_y - py
        distance2 = ex * ex + ey * ey
        if best is None or distance2 < best[0]:
            heading = math.atan2(vy, vx)
            signed = (vx * ey - vy * ex) / math.sqrt(length2)
            best = (distance2, signed, heading, i)

    if best is None:
        raise ValueError("path has no nonzero-length segment near the ego")

    distance2, signed, heading, seg_index = best
    return {
        "signed_cte_m": float(signed),
        "abs_cte_m": math.sqrt(distance2),
        "path_heading_deg": math.degrees(heading),
        "heading_error_deg": math.degrees(_wrap_rad(ego_yaw - heading)),
        "nearest_segment_index": seg_index,
    }
