#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Pure Pursuit lateral controller for sensor integration.

Responsibilities:
- load the official global path
- convert GPS latitude/longitude to the existing local coordinates
- convert IMU quaternion to yaw
- find the nearest waypoint
- calculate Pure Pursuit steering

This module intentionally does NOT:
- receive UDP packets
- send EgoCtrlCmd
- decide accel/brake
- decide mission behavior
- own the final driving loop

The steering geometry and coordinate conversion are kept from the previously
validated Pure Pursuit baseline.
"""

import math
from pathlib import Path

import numpy as np
from pyproj import Proj


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_PATH_FILE = PROJECT_ROOT / "path" / "2026_molit_comp_global_path.txt"

# Existing validated coordinate conversion.
PROJ_UTM = Proj(
    proj="utm",
    zone=52,
    ellps="WGS84",
    preserve_units=False,
)
EAST_OFFSET = 302595.0
NORTH_OFFSET = 4124145.0

# Existing validated lateral-control baseline.
VEHICLE_LENGTH = 1.0
LOOKAHEAD_DISTANCE = 2.5
LOCAL_PATH_SIZE = 50


def load_path(path_file=DEFAULT_PATH_FILE):
    """Load x/y waypoints from the competition global-path file."""
    points = []

    with open(path_file, "r") as f:
        for line in f:
            values = line.split()
            if len(values) >= 2:
                points.append((float(values[0]), float(values[1])))

    if not points:
        raise RuntimeError("전역경로 파일에 waypoint가 없습니다.")

    return np.asarray(points, dtype=float)


def gps_to_local(lat, lon):
    """Convert WGS84 GPS coordinates to the local map coordinates."""
    if lat == 0.0 and lon == 0.0:
        return None

    utm_x, utm_y = PROJ_UTM(lon, lat)
    return utm_x - EAST_OFFSET, utm_y - NORTH_OFFSET


def quaternion_to_yaw(x, y, z, w):
    """Convert an IMU quaternion to yaw in radians."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class PurePursuit:
    """Small lateral-control module intended to be called by the final main."""

    def __init__(
        self,
        path=None,
        path_file=DEFAULT_PATH_FILE,
        vehicle_length=VEHICLE_LENGTH,
        lookahead_distance=LOOKAHEAD_DISTANCE,
        local_path_size=LOCAL_PATH_SIZE,
    ):
        self.path = load_path(path_file) if path is None else np.asarray(path, dtype=float)
        self.vehicle_length = float(vehicle_length)
        self.lookahead_distance = float(lookahead_distance)
        self.local_path_size = int(local_path_size)

        if len(self.path) == 0:
            raise ValueError("Pure Pursuit path가 비어 있습니다.")

    def find_nearest_waypoint(self, x, y):
        """Return the index of the waypoint nearest to the current position."""
        dx = self.path[:, 0] - x
        dy = self.path[:, 1] - y
        return int(np.argmin(dx * dx + dy * dy))

    def calculate(self, x, y, yaw):
        """Calculate steering from the current local position and yaw.

        Returns None when no valid forward lookahead point can be found.
        Otherwise returns a compact dictionary that the integration main can use.
        """
        nearest_index = self.find_nearest_waypoint(x, y)
        end_index = min(nearest_index + self.local_path_size, len(self.path))

        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)

        for target_index in range(nearest_index, end_index):
            dx = self.path[target_index, 0] - x
            dy = self.path[target_index, 1] - y

            local_x = cos_yaw * dx + sin_yaw * dy
            local_y = -sin_yaw * dx + cos_yaw * dy

            if local_x <= 0.0:
                continue

            if math.hypot(local_x, local_y) < self.lookahead_distance:
                continue

            theta = math.atan2(local_y, local_x)
            steering = math.atan2(
                2.0 * self.vehicle_length * math.sin(theta),
                self.lookahead_distance,
            )

            nearest_dx = self.path[nearest_index, 0] - x
            nearest_dy = self.path[nearest_index, 1] - y

            return {
                "steering": steering,
                "nearest_index": nearest_index,
                "target_index": target_index,
                "path_distance": math.hypot(nearest_dx, nearest_dy),
            }

        return None
