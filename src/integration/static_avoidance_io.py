#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Validated UDP boundary for the independent LiDAR static-avoidance worker.

Only the main integrated drive loop may send EgoCtrlCmd. This module supplies
a validated plan and the same 2.5-m local Pure Pursuit used by the source
5-km/h avoidance test; it does not import or operate a network sender.
"""
import json
import math

import numpy as np

STATUSES = frozenset(("CLEAR", "AVOIDING", "STOP_REQUIRED"))
MAX_PATH_POINTS = 400
LOCAL_PATH_LOOKAHEAD_M = 2.5
LOCAL_PATH_SEARCH_SIZE = 80


def parse_static_message(raw):
    result = {
        "valid": False,
        "status": "UNKNOWN",
        "path": [],
        "obstacle_id": None,
        "avoidance_side": None,
        "recommended_speed_kmh": 0.0,
        "latch_state": "",
        "scan": 0,
        "detected": 0,
    }
    if not raw or len(raw) > 64000:
        return result
    try:
        value = json.loads(raw.decode("utf-8"))
        if not isinstance(value, dict):
            return result
        status = value.get("status")
        if status not in STATUSES:
            return result
        path = value.get("path", [])
        if status == "AVOIDING":
            if not isinstance(path, list) or not 2 <= len(path) <= MAX_PATH_POINTS:
                return result
            normalized = []
            for point in path:
                if not isinstance(point, list) or len(point) < 2:
                    return result
                x, y = float(point[0]), float(point[1])
                if not math.isfinite(x) or not math.isfinite(y):
                    return result
                if abs(x) > 1e6 or abs(y) > 1e6:
                    return result
                normalized.append([x, y])
            path = normalized
        else:
            path = []
        speed = float(value.get("recommended_speed_kmh", 0.0))
        if not math.isfinite(speed) or speed < 0:
            return result
        result.update(
            valid=True,
            status=status,
            path=path,
            obstacle_id=value.get("obstacle_id"),
            avoidance_side=value.get("avoidance_side"),
            recommended_speed_kmh=speed,
            latch_state=str(value.get("latch_state", ""))[:100],
            scan=int(value.get("scan", 0)),
            detected=int(value.get("detected", 0)),
        )
    except (TypeError, ValueError, UnicodeError, AttributeError, OverflowError):
        return result
    return result


def local_path_pure_pursuit(path, ego_x, ego_y, ego_yaw, vehicle_length):
    """Original local path steering from lidar_static_avoidance_drive_5kph.py.

    Returns None if there is no valid forward target. Never send a control
    command or use the global waypoint index as a local path index.
    """
    xy = np.asarray(path, dtype=float)
    if xy.ndim != 2 or len(xy) < 2 or xy.shape[1] < 2:
        return None
    xy = xy[:, :2]
    distances2 = np.sum((xy - [ego_x, ego_y]) ** 2, axis=1)
    nearest = int(np.argmin(distances2))
    limit = min(nearest + LOCAL_PATH_SEARCH_SIZE, len(xy))
    cos_yaw = math.cos(ego_yaw)
    sin_yaw = math.sin(ego_yaw)

    for i in range(nearest, limit):
        dx, dy = xy[i, 0] - ego_x, xy[i, 1] - ego_y
        local_x = cos_yaw * dx + sin_yaw * dy
        local_y = -sin_yaw * dx + cos_yaw * dy
        if local_x <= 0.0:
            continue
        if math.hypot(local_x, local_y) < LOCAL_PATH_LOOKAHEAD_M:
            continue
        theta = math.atan2(local_y, local_x)
        steering = math.atan2(
            2.0 * vehicle_length * math.sin(theta),
            LOCAL_PATH_LOOKAHEAD_M,
        )
        return {
            "nearest_index": nearest,
            "target_index": i,
            "steering": float(steering),
            "local_x": float(local_x),
            "local_y": float(local_y),
        }
    return None
