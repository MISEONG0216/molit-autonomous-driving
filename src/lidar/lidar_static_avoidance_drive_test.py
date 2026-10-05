#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Static-obstacle path-selector DRY-RUN integration test.

Pipeline:
GPS/IMU + VLP16 -> detection -> tracking with ego-motion compensation
-> global obstacle adapter -> StaticObstaclePlanner
-> path selector -> Pure Pursuit calculation.

Safety:
- NO EgoCtrlCmd is created or transmitted in this stage.
- Existing validated controller files are not modified.
- CLEAR follows the planner's global-path slice.
- AVOIDING follows the generated local avoidance path.
- STOP_REQUIRED requests zero target speed and zero steering for diagnostics.
"""

import math
import sys
import time
from pathlib import Path

import numpy as np

LIDAR_DIR = Path(__file__).resolve().parent
SRC_DIR = LIDAR_DIR.parent
PROJECT_ROOT = SRC_DIR.parent

for path in (SRC_DIR, LIDAR_DIR):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from morai_common import ensure_morai_import_path, load_network_config

ensure_morai_import_path()

from lib.network.UDP import Receiver
from lib.define.GPS import GPS
from lib.define.IMU import IMU

import pure_pursuit_regulation as pp_base
from pure_pursuit_competition import CompetitionSpeedReceiver

from lidar_receiver import LidarReceiver
from object_detection.object_detector import ObjectDetector
from object_detection.tracker import ObjectTracker
from obstacle_adapter import tracked_list_to_planner_obstacles
from static_obstacle_planner import StaticObstaclePlanner

# Reuse the already validated sensor/coordinate helpers instead of duplicating
# their formulas in this first integration stage.
from lidar_ego_motion_test import (
    read_ego_pose,
    calculate_ego_motion,
    convert_lidar_to_vehicle,
)


GLOBAL_PATH_FILE = PROJECT_ROOT / "path" / "2026_molit_comp_global_path.txt"
STATIC_CONFIG_FILE = LIDAR_DIR / "static_obstacle_config.json"

DRY_RUN_SPEED_LIMIT_KPH = 5.0
LOCAL_PATH_LOOKAHEAD_M = 2.5
LOCAL_PATH_SEARCH_SIZE = 80


def prepare_path(path):
    """Return an Nx2 float path accepted by the Pure Pursuit calculation."""
    array = np.asarray(path, dtype=float)
    if array.ndim != 2 or array.shape[1] < 2:
        return np.empty((0, 2), dtype=float)
    return array[:, :2].copy()


def calculate_selected_path_pure_pursuit(path, ego_x, ego_y, ego_yaw):
    """Pure Pursuit on a short selected path using a local path index.

    This deliberately does NOT reuse the global waypoint index.  The #4 path
    is a short slice beginning near the ego vehicle, so its own nearest index
    must be calculated.
    """
    selected_path = prepare_path(path)
    if len(selected_path) < 2:
        return None, None

    nearest_index = pp_base.find_nearest_waypoint(
        selected_path,
        ego_x,
        ego_y,
    )

    end_index = min(
        nearest_index + LOCAL_PATH_SEARCH_SIZE,
        len(selected_path),
    )

    cos_yaw = math.cos(ego_yaw)
    sin_yaw = math.sin(ego_yaw)

    for i in range(nearest_index, end_index):
        dx = selected_path[i, 0] - ego_x
        dy = selected_path[i, 1] - ego_y

        local_x = cos_yaw * dx + sin_yaw * dy
        local_y = -sin_yaw * dx + cos_yaw * dy

        if local_x <= 0.0:
            continue

        distance = math.hypot(local_x, local_y)
        if distance < LOCAL_PATH_LOOKAHEAD_M:
            continue

        theta = math.atan2(local_y, local_x)
        steering = math.atan2(
            2.0 * pp_base.VEHICLE_LENGTH * math.sin(theta),
            LOCAL_PATH_LOOKAHEAD_M,
        )

        return {
            "nearest_index": nearest_index,
            "target_index": i,
            "target_x": float(selected_path[i, 0]),
            "target_y": float(selected_path[i, 1]),
            "local_x": float(local_x),
            "local_y": float(local_y),
            "theta": float(theta),
            "steering": float(steering),
        }, selected_path

    return None, selected_path


def main():
    print("=" * 76)
    print(" STATIC OBSTACLE PATH SELECTOR + PURE PURSUIT - DRY RUN")
    print("=" * 76)
    print("[SAFE] This file never sends EgoCtrlCmd.")
    print("[LIMIT] Diagnostic target speed <= 5.0 km/h")
    print("[PATH] CLEAR -> global slice / AVOIDING -> #4 local path")
    print()

    if not GLOBAL_PATH_FILE.exists():
        raise FileNotFoundError(f"Global path not found: {GLOBAL_PATH_FILE}")
    if not STATIC_CONFIG_FILE.exists():
        raise FileNotFoundError(f"Planner config not found: {STATIC_CONFIG_FILE}")

    global_path = np.loadtxt(GLOBAL_PATH_FILE, dtype=float)
    if global_path.ndim != 2 or global_path.shape[1] < 2:
        raise RuntimeError("Global path requires x/y columns.")
    if global_path.shape[1] == 2:
        global_path = np.column_stack((global_path, np.zeros(len(global_path))))
    global_path = global_path[:, :3]

    planner = StaticObstaclePlanner(global_path, str(STATIC_CONFIG_FILE))

    network_config = load_network_config()
    gps_cfg = network_config["gps"]
    imu_cfg = network_config["imu"]

    gps_receiver = Receiver(gps_cfg["ip"], int(gps_cfg["port"]), GPS())
    imu_receiver = Receiver(imu_cfg["ip"], int(imu_cfg["port"]), IMU())
    speed_receiver = CompetitionSpeedReceiver()
    lidar_receiver = LidarReceiver()

    detector = ObjectDetector(
        ground_z_threshold=-1.70,
        dbscan_eps=0.5,
        dbscan_min_samples=5,
        min_cluster_points=20,
        min_width=0.15,
        max_width=3.00,
        min_length=0.05,
        max_length=5.00,
    )

    tracker = ObjectTracker(
        max_match_distance=1.0,
        velocity_window_size=5,
        dynamic_speed_threshold=0.5,
        dynamic_confirm_frames=3,
        max_missed_frames=3,
    )

    previous_scan_timestamp = None
    last_processed_timestamp = None
    previous_ego_pose = None
    scan_number = 0

    try:
        lidar_receiver.start()
        print("[READY] GPS / IMU / Competition Status / LiDAR")
        print("[WAIT] Waiting for first fresh LiDAR scan...")

        while True:
            ego_pose = read_ego_pose(gps_receiver, imu_receiver)
            if ego_pose is None:
                time.sleep(0.03)
                continue
            ego_x, ego_y, ego_yaw = ego_pose

            current_speed_kmh, speed_valid = speed_receiver.read()

            latest = lidar_receiver.get_latest()
            if latest is None:
                time.sleep(0.01)
                continue

            points, stats = latest
            if points is None or not stats.get("fresh", False):
                time.sleep(0.01)
                continue

            scan_timestamp = stats.get("timestamp")
            if scan_timestamp is None or scan_timestamp == last_processed_timestamp:
                time.sleep(0.005)
                continue

            last_processed_timestamp = scan_timestamp
            scan_number += 1

            if previous_scan_timestamp is None:
                dt = 0.1
            else:
                dt = scan_timestamp - previous_scan_timestamp
                if dt <= 0.0 or dt > 1.0:
                    dt = 0.1
            previous_scan_timestamp = scan_timestamp

            if previous_ego_pose is None:
                delta_forward = 0.0
                delta_left = 0.0
                delta_yaw = 0.0
            else:
                _, _, delta_forward, delta_left, delta_yaw = calculate_ego_motion(
                    previous_ego_pose[0],
                    previous_ego_pose[1],
                    previous_ego_pose[2],
                    ego_x,
                    ego_y,
                    ego_yaw,
                )
            previous_ego_pose = (ego_x, ego_y, ego_yaw)

            vehicle_points = convert_lidar_to_vehicle(points)
            obstacles, non_ground_points, _ = detector.detect(vehicle_points)

            tracked_obstacles = tracker.update(
                obstacles,
                dt,
                delta_x=delta_forward,
                delta_y=delta_left,
                delta_yaw=delta_yaw,
            )

            planner_obstacles = tracked_list_to_planner_obstacles(
                tracked_obstacles,
                ego_x=ego_x,
                ego_y=ego_y,
                ego_yaw=ego_yaw,
                confidence=1.0,
            )

            plan_result = planner.plan(
                ego_x=ego_x,
                ego_y=ego_y,
                obstacles=planner_obstacles,
            )

            if plan_result.status == "STOP_REQUIRED":
                path_source = "STOP_REQUIRED"
                selected_path = plan_result.local_path
                pp_result = None
                target_speed_kmh = 0.0
                steering = 0.0
            else:
                path_source = (
                    "LOCAL_AVOIDANCE"
                    if plan_result.status == "AVOIDING"
                    else "GLOBAL_CLEAR"
                )
                selected_path = plan_result.local_path
                pp_result, _ = calculate_selected_path_pure_pursuit(
                    selected_path,
                    ego_x,
                    ego_y,
                    ego_yaw,
                )
                target_speed_kmh = min(
                    DRY_RUN_SPEED_LIMIT_KPH,
                    max(0.0, float(plan_result.recommended_speed_kph)),
                )
                steering = 0.0 if pp_result is None else pp_result["steering"]

            print("\033[2J\033[H", end="")
            print("=" * 76)
            print(" STATIC OBSTACLE PATH SELECTOR + PURE PURSUIT - DRY RUN")
            print("=" * 76)
            print(f"SCAN          : {scan_number}")
            print(
                f"EGO           : x={ego_x:.3f}, y={ego_y:.3f}, "
                f"yaw={math.degrees(ego_yaw):.2f} deg"
            )
            print(
                f"EGO MOTION    : forward={delta_forward:+.3f} m, "
                f"left={delta_left:+.3f} m, "
                f"dyaw={math.degrees(delta_yaw):+.3f} deg"
            )
            print(
                f"LIDAR         : points={len(vehicle_points)}, "
                f"non_ground={len(non_ground_points)}, detected={len(obstacles)}"
            )
            print(
                f"TRACK/GLOBAL  : {len(tracked_obstacles)} / "
                f"{len(planner_obstacles)} objects"
            )
            print()
            print(f"PLANNER       : {plan_result.status}")
            print(f"OBSTACLE ID   : {plan_result.obstacle_id}")
            print(f"AVOID SIDE    : {plan_result.avoidance_side}")
            print(f"PATH SOURCE   : {path_source}")
            print(f"PATH POINTS   : {len(selected_path)}")
            print(f"PLAN SPEED    : {plan_result.recommended_speed_kph:.1f} km/h")
            print(f"TEST TARGET   : {target_speed_kmh:.1f} km/h")
            print(
                "COMP SPEED    : "
                + (
                    f"{current_speed_kmh:.2f} km/h"
                    if speed_valid
                    else "WAIT / INVALID"
                )
            )
            print()

            if plan_result.status == "STOP_REQUIRED":
                print("PURE PURSUIT  : BYPASSED - STOP REQUEST")
                print("STEERING      : 0.00000 rad")
            elif pp_result is None:
                print("PURE PURSUIT  : LOOKAHEAD NOT FOUND")
                print("STEERING      : 0.00000 rad")
            else:
                print(
                    f"PURE PURSUIT  : nearest={pp_result['nearest_index']}, "
                    f"target={pp_result['target_index']}"
                )
                print(
                    f"TARGET LOCAL  : x={pp_result['local_x']:.2f} m, "
                    f"y={pp_result['local_y']:.2f} m"
                )
                print(
                    f"STEERING      : {steering:.5f} rad "
                    f"({math.degrees(steering):.2f} deg)"
                )

            print()
            print("[DRY-RUN] NO EgoCtrlCmd transmitted.")

    except KeyboardInterrupt:
        print()
        print("[STOP] Ctrl+C received.")
    finally:
        lidar_receiver.stop()
        print("[STOP] LiDAR receiver stopped.")
        print("[SAFE] No vehicle control command was transmitted.")


if __name__ == "__main__":
    main()
