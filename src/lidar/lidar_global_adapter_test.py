#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Real GPS / IMU + LiDAR Adapter Integration Test

Pipeline
--------
MORAI VLP16
    -> #1 LiDAR Receiver / Preprocess
    -> #2 Object Detection
    -> #3 Object Tracking
    -> tracked obstacle (vehicle coordinates)

MORAI GPS
    -> latitude / longitude
    -> UTM Zone 52
    -> competition local coordinates
    -> ego_x / ego_y

MORAI IMU
    -> quaternion
    -> yaw
    -> ego_yaw

Tracked obstacle + Ego pose
    -> obstacle_adapter
    -> planner-compatible GLOBAL obstacle

IMPORTANT
---------
This test does NOT connect #4 StaticObstaclePlanner yet.

ObjectTracker ego-motion compensation is still OFF.
KEEP THE VEHICLE STATIONARY during this test.
"""

import math
import sys
import time
from pathlib import Path

import numpy as np
from pyproj import Proj


# ============================================================
# Import path setup
# ============================================================

LIDAR_DIR = Path(__file__).resolve().parent
SRC_DIR = LIDAR_DIR.parent

if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))


# ============================================================
# MORAI Network Module setup
# ============================================================

from morai_common import (
    ensure_morai_import_path,
    load_network_config,
)

# Add third_party/MORAI-NetworkModule to Python import path.
ensure_morai_import_path()

from lib.network.UDP import Receiver
from lib.define.GPS import GPS
from lib.define.IMU import IMU


# ============================================================
# LiDAR modules
# ============================================================

from lidar_receiver import LidarReceiver
from object_detection.object_detector import ObjectDetector
from object_detection.tracker import ObjectTracker
from obstacle_adapter import tracked_list_to_planner_obstacles


# ============================================================
# Competition coordinate system
#
# Keep exactly the same projection / offsets used by the
# existing Pure Pursuit driving code.
# ============================================================

PROJ_UTM = Proj(
    proj="utm",
    zone=52,
    ellps="WGS84",
    preserve_units=False,
)

EAST_OFFSET = 302595.0
NORTH_OFFSET = 4124145.0


def gps_to_local(lat, lon):
    """
    Convert GPS latitude / longitude to the same local coordinate
    system used by the competition global path.
    """

    if lat == 0.0 and lon == 0.0:
        return None

    utm_x, utm_y = PROJ_UTM(lon, lat)

    return (
        utm_x - EAST_OFFSET,
        utm_y - NORTH_OFFSET,
    )


def quaternion_to_yaw(x, y, z, w):
    """
    Convert quaternion orientation to yaw [rad].

    Same equation used by the existing Pure Pursuit controller.
    """

    siny_cosp = 2.0 * (w * z + x * y)

    cosy_cosp = (
        1.0
        - 2.0 * (y * y + z * z)
    )

    return math.atan2(
        siny_cosp,
        cosy_cosp,
    )


# ============================================================
# LiDAR coordinate conversion
# ============================================================

def convert_lidar_to_vehicle(points):
    """
    Convert #1 LiDAR coordinates to #2/#3 vehicle coordinates.

    #1 LiDAR:
        +X = Right
        +Y = Forward
        +Z = Up

    #2/#3:
        +X = Forward
        +Y = Left
        +Z = Up

    Conversion:
        vehicle_x = lidar_y
        vehicle_y = -lidar_x
        vehicle_z = lidar_z
    """

    if points is None or len(points) == 0:
        return np.empty(
            (0, 3),
            dtype=np.float32,
        )

    converted = np.column_stack(
        (
            points[:, 1],
            -points[:, 0],
            points[:, 2],
        )
    )

    return converted.astype(np.float32)


# ============================================================
# Ego pose
# ============================================================

def read_ego_pose(gps_receiver, imu_receiver):
    """
    Read GPS + IMU and return:

        ego_x
        ego_y
        ego_yaw

    Returns None while GPS or IMU data are not ready.
    """

    # --------------------------------------------------------
    # GPS
    # --------------------------------------------------------

    gps_data = gps_receiver.get_data()

    try:
        gps_data.parsing()

        lat = float(gps_data.gprmc.lat)
        lon = float(gps_data.gprmc.lon)

    except Exception:
        return None

    position = gps_to_local(
        lat,
        lon,
    )

    if position is None:
        return None

    ego_x, ego_y = position

    # --------------------------------------------------------
    # IMU
    # --------------------------------------------------------

    imu_data = imu_receiver.get_data()

    try:
        ori_w = float(imu_data.ori_w)
        ori_x = float(imu_data.ori_x)
        ori_y = float(imu_data.ori_y)
        ori_z = float(imu_data.ori_z)

    except Exception:
        return None

    quaternion_is_zero = (
        ori_w == 0.0
        and ori_x == 0.0
        and ori_y == 0.0
        and ori_z == 0.0
    )

    if quaternion_is_zero:
        return None

    ego_yaw = quaternion_to_yaw(
        ori_x,
        ori_y,
        ori_z,
        ori_w,
    )

    return (
        float(ego_x),
        float(ego_y),
        float(ego_yaw),
    )


# ============================================================
# Main
# ============================================================

def main():

    print("=" * 70)
    print(" LiDAR + Real GPS/IMU -> Global Obstacle Test")
    print("=" * 70)

    print()
    print("[PIPELINE]")
    print("LiDAR -> #1 -> #2 -> #3")
    print("GPS   -> UTM -> ego_x / ego_y")
    print("IMU   -> quaternion -> ego_yaw")
    print("#3 + Ego Pose -> Adapter -> Global Obstacle")

    print()
    print("[WARNING]")
    print("StaticObstaclePlanner (#4) is NOT connected yet.")
    print("ObjectTracker ego-motion compensation is still OFF.")
    print("KEEP THE VEHICLE STATIONARY during this test.")
    print()

    # ========================================================
    # Network configuration
    # ========================================================

    network_config = load_network_config()

    gps_config = network_config["gps"]
    imu_config = network_config["imu"]

    gps_ip = gps_config["ip"]
    gps_port = int(gps_config["port"])

    imu_ip = imu_config["ip"]
    imu_port = int(imu_config["port"])

    # ========================================================
    # GPS / IMU receivers
    # ========================================================

    gps_receiver = Receiver(
        gps_ip,
        gps_port,
        GPS(),
    )

    imu_receiver = Receiver(
        imu_ip,
        imu_port,
        IMU(),
    )

    print(
        f"[GPS] {gps_ip}:{gps_port}"
    )

    print(
        f"[IMU] {imu_ip}:{imu_port}"
    )

    # ========================================================
    # #1 LiDAR
    # ========================================================

    lidar_receiver = LidarReceiver()

    # ========================================================
    # #2 Object Detector
    # ========================================================

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

    # ========================================================
    # #3 Object Tracker
    # ========================================================

    tracker = ObjectTracker(
        max_match_distance=1.0,
        velocity_window_size=5,
        dynamic_speed_threshold=0.5,
        dynamic_confirm_frames=3,
        max_missed_frames=3,
    )

    previous_scan_timestamp = None
    last_processed_timestamp = None

    scan_number = 0

    try:

        lidar_receiver.start()

        print()
        print("[READY] LiDAR receiver started.")
        print("[WAIT] Waiting for LiDAR + GPS + IMU...")
        print()

        while True:

            # =================================================
            # Real GPS / IMU ego pose
            # =================================================

            ego_pose = read_ego_pose(
                gps_receiver,
                imu_receiver,
            )

            if ego_pose is None:

                print(
                    "\r[WAIT] GPS / IMU data not ready...",
                    end="",
                    flush=True,
                )

                time.sleep(0.05)
                continue

            ego_x, ego_y, ego_yaw = ego_pose

            # =================================================
            # LiDAR
            # =================================================

            latest = lidar_receiver.get_latest()

            if latest is None:
                time.sleep(0.01)
                continue

            points, stats = latest

            if points is None:
                time.sleep(0.01)
                continue

            if not stats.get("fresh", False):
                time.sleep(0.01)
                continue

            scan_timestamp = stats.get("timestamp")

            if scan_timestamp is None:
                time.sleep(0.01)
                continue

            # Do not process the same scan twice.
            if scan_timestamp == last_processed_timestamp:
                time.sleep(0.005)
                continue

            last_processed_timestamp = scan_timestamp
            scan_number += 1

            # =================================================
            # dt
            # =================================================

            if previous_scan_timestamp is None:

                dt = 0.1

            else:

                dt = (
                    scan_timestamp
                    - previous_scan_timestamp
                )

                if dt <= 0.0 or dt > 1.0:
                    dt = 0.1

            previous_scan_timestamp = scan_timestamp

            # =================================================
            # #1 -> #2 coordinate conversion
            # =================================================

            vehicle_points = convert_lidar_to_vehicle(
                points
            )

            # =================================================
            # #2 Object Detection
            # =================================================

            (
                obstacles,
                non_ground_points,
                labels,
            ) = detector.detect(
                vehicle_points
            )

            # =================================================
            # #3 Object Tracking
            #
            # Ego-motion compensation is intentionally OFF
            # during this stationary integration test.
            # =================================================

            tracked_obstacles = tracker.update(
                obstacles,
                dt,
                delta_x=0.0,
                delta_y=0.0,
                delta_yaw=0.0,
            )

            # =================================================
            # Adapter
            # =================================================

            planner_obstacles = (
                tracked_list_to_planner_obstacles(
                    tracked_obstacles,
                    ego_x=ego_x,
                    ego_y=ego_y,
                    ego_yaw=ego_yaw,
                    confidence=1.0,
                )
            )

            # =================================================
            # Diagnostics
            # =================================================

            print()
            print("=" * 70)
            print(f"SCAN {scan_number}")
            print("=" * 70)

            print(
                f"[EGO] "
                f"x={ego_x:.3f}, "
                f"y={ego_y:.3f}, "
                f"yaw={math.degrees(ego_yaw):.2f} deg"
            )

            print(
                f"[LIDAR] "
                f"points={len(vehicle_points)}, "
                f"non_ground={len(non_ground_points)}, "
                f"detected={len(obstacles)}, "
                f"dt={dt:.3f} s"
            )

            print(
                f"[TRACK] objects="
                f"{len(tracked_obstacles)}"
            )

            if len(tracked_obstacles) == 0:

                print(
                    "  No tracked obstacle"
                )

            for obstacle in tracked_obstacles:

                speed = float(
                    np.hypot(
                        obstacle.velocity_x,
                        obstacle.velocity_y,
                    )
                )

                print(
                    f"  Track {obstacle.track_id}: "
                    f"local_x={obstacle.x:.2f} m, "
                    f"local_y={obstacle.y:.2f} m, "
                    f"distance={obstacle.distance:.2f} m, "
                    f"speed={speed:.2f} m/s, "
                    f"dynamic={obstacle.is_dynamic}"
                )

            print(
                f"[GLOBAL ADAPTER] objects="
                f"{len(planner_obstacles)}"
            )

            if len(planner_obstacles) == 0:

                print(
                    "  No global obstacle"
                )

            for obstacle in planner_obstacles:

                print(
                    f"  PlannerObstacle "
                    f"id={obstacle.id}: "
                    f"global_x={obstacle.x:.3f}, "
                    f"global_y={obstacle.y:.3f}, "
                    f"width={obstacle.width:.2f}, "
                    f"length={obstacle.length:.2f}, "
                    f"distance={obstacle.distance:.2f}, "
                    f"dynamic={obstacle.is_dynamic}, "
                    f"confidence={obstacle.confidence:.2f}"
                )

    except KeyboardInterrupt:

        print()
        print("[STOP] Ctrl+C received.")

    finally:

        lidar_receiver.stop()

        print("[STOP] LiDAR receiver stopped.")


if __name__ == "__main__":
    main()