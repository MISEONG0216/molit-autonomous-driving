#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Pure Pursuit lateral controller for final sensor integration.

Keeps the validated 57 km/h lateral behavior: dynamic speed-based LFD,
curvature-dependent LFD cap, and the proven Pure Pursuit steering geometry.
UDP, longitudinal control, mission logic, and the final loop are intentionally
outside this module.
"""
import math
from pathlib import Path
import numpy as np
from pyproj import Proj
import path_speed_profile as profile_builder

PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_PATH_FILE = PROJECT_ROOT / "path" / "2026_molit_comp_global_path.txt"
PROJ_UTM = Proj(proj="utm", zone=52, ellps="WGS84", preserve_units=False)
EAST_OFFSET = 302595.0
NORTH_OFFSET = 4124145.0

VEHICLE_LENGTH = 1.0
LOCAL_PATH_SIZE = 50
LFD_GAIN = 0.65
MIN_LFD_M = 2.5
MAX_LFD_M = 9.0

def load_path(path_file=DEFAULT_PATH_FILE):
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
    if lat == 0.0 and lon == 0.0:
        return None
    utm_x, utm_y = PROJ_UTM(lon, lat)
    return utm_x - EAST_OFFSET, utm_y - NORTH_OFFSET

def quaternion_to_yaw(x, y, z, w):
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)

def dynamic_lfd(speed_kmh):
    speed_mps = max(0.0, float(speed_kmh)) / 3.6
    return max(MIN_LFD_M, min(MAX_LFD_M, LFD_GAIN * speed_mps))

def curvature_lfd_cap(radius_m):
    if not math.isfinite(radius_m):
        return MAX_LFD_M
    if radius_m < 20.0: return 3.0
    if radius_m < 40.0: return 4.0
    if radius_m < 80.0: return 5.0
    if radius_m < 150.0: return 6.5
    return MAX_LFD_M

class PurePursuit:
    def __init__(self, path=None, path_file=DEFAULT_PATH_FILE,
                 vehicle_length=VEHICLE_LENGTH, local_path_size=LOCAL_PATH_SIZE):
        self.path = load_path(path_file) if path is None else np.asarray(path, dtype=float)
        if len(self.path) == 0:
            raise ValueError("Pure Pursuit path가 비어 있습니다.")
        self.vehicle_length = float(vehicle_length)
        self.local_path_size = int(local_path_size)
        curvature = profile_builder.compute_curvature(self.path)
        self.radius_profile = profile_builder.curvature_to_radius(curvature)

    def find_nearest_waypoint(self, x, y):
        dx = self.path[:, 0] - x
        dy = self.path[:, 1] - y
        return int(np.argmin(dx * dx + dy * dy))

    def calculate(self, x, y, yaw, speed_kmh):
        nearest_index = self.find_nearest_waypoint(x, y)
        speed_lfd = dynamic_lfd(speed_kmh)
        radius_m = float(self.radius_profile[nearest_index])
        lfd = max(MIN_LFD_M, min(speed_lfd, curvature_lfd_cap(radius_m)))

        end_index = min(nearest_index + self.local_path_size, len(self.path))
        cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)

        for target_index in range(nearest_index, end_index):
            dx = self.path[target_index, 0] - x
            dy = self.path[target_index, 1] - y
            local_x = cos_yaw * dx + sin_yaw * dy
            local_y = -sin_yaw * dx + cos_yaw * dy
            if local_x <= 0.0 or math.hypot(local_x, local_y) < lfd:
                continue

            theta = math.atan2(local_y, local_x)
            steering = math.atan2(
                2.0 * self.vehicle_length * math.sin(theta), lfd
            )
            nearest_dx = self.path[nearest_index, 0] - x
            nearest_dy = self.path[nearest_index, 1] - y
            return {
                "steering": steering,
                "nearest_index": nearest_index,
                "target_index": target_index,
                "path_distance": math.hypot(nearest_dx, nearest_dy),
                "curve_radius": radius_m,
                "lookahead_distance": lfd,
            }
        return None
