#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
LiDAR #1 -> #2 -> #3 -> Global Adapter -> #4 Static Planner Integration Test

Purpose
-------
This test connects the real MORAI sensor pipeline to StaticObstaclePlanner.

Pipeline
--------
MORAI GPS
    -> competition local x / y

MORAI IMU
    -> yaw

MORAI VLP16
    -> #1 LiDAR Receiver / Preprocess
    -> coordinate conversion
    -> #2 Object Detection
    -> #3 Object Tracking
    -> Global Obstacle Adapter
    -> #4 StaticObstaclePlanner
    -> CLEAR / AVOIDING / STOP_REQUIRED
    -> Local Path

IMPORTANT
---------
- This file DOES NOT send EgoCtrlCmd.
- The vehicle must remain stationary during this test.
- Tracker ego-motion compensation is still OFF.
"""

import math
import sys
import time
from pathlib import Path

import numpy as np
from pyproj import Proj


# ============================================================
# Path setup
# ============================================================

LIDAR_DIR = Path(__file__).resolve().parent
SRC_DIR = LIDAR_DIR.parent
PROJECT_ROOT = SRC_DIR.parent

if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

if str(LIDAR_DIR) not in sys.path:
    sys.path.insert(0, str(LIDAR_DIR))


# ============================================================
# MORAI Network Module
# ============================================================

from morai_common import (
    ensure_morai_import_path,
    load_network_config,
)

ensure_morai_import_path()

from lib.network.UDP import Receiver
from lib.define.GPS import GPS
from lib.define.IMU import IMU


# ============================================================
# LiDAR #1 / #2 / #3
# ============================================================

from lidar_receiver import LidarReceiver

from object_detection.object_detector import ObjectDetector
from object_detection.tracker import ObjectTracker

from obstacle_adapter import tracked_list_to_planner_obstacles


# ============================================================
# #4 Static Obstacle Planner
# ============================================================

from static_obstacle_planner import StaticObstaclePlanner


# ============================================================
# Competition coordinate system
# ============================================================

PROJ_UTM = Proj(
    proj="utm",
    zone=52,
    ellps="WGS84",
    preserve_units=False,
)

EAST_OFFSET = 302595.0
NORTH_OFFSET = 4124145.0


# ============================================================
# Files
# ============================================================

GLOBAL_PATH_FILE = (
    PROJECT_ROOT
    / "path"
    / "2026_molit_comp_global_path.txt"
)

STATIC_CONFIG_FILE = (
    LIDAR_DIR
    / "static_obstacle_config.json"
)


# ============================================================
# GPS
# ============================================================

def gps_to_local(lat, lon):
    """
    GPS latitude / longitude
    -> competition local x / y
    """

    if lat == 0.0 and lon == 0.0:
        return None

    utm_x, utm_y = PROJ_UTM(
        lon,
        lat,
    )

    return (
        utm_x - EAST_OFFSET,
        utm_y - NORTH_OFFSET,
    )


# ============================================================
# IMU
# ============================================================

def quaternion_to_yaw(x, y, z, w):
    """
    Quaternion -> yaw [rad]
    """

    siny_cosp = 2.0 * (
        w * z
        + x * y
    )

    cosy_cosp = (
        1.0
        - 2.0 * (
            y * y
            + z * z
        )
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
    #1 LiDAR coordinates:

        +X = Right
        +Y = Forward
        +Z = Up

    #2 / #3 vehicle coordinates:

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

    return converted.astype(
        np.float32
    )


# ============================================================
# Ego pose
# ============================================================

def read_ego_pose(
    gps_receiver,
    imu_receiver,
):
    """
    Read real MORAI GPS + IMU.

    Returns:

        ego_x
        ego_y
        ego_yaw

    Returns None while sensor data are not ready.
    """

    # --------------------------------------------------------
    # GPS
    # --------------------------------------------------------

    gps_data = gps_receiver.get_data()

    try:

        gps_data.parsing()

        lat = float(
            gps_data.gprmc.lat
        )

        lon = float(
            gps_data.gprmc.lon
        )

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

        ori_w = float(
            imu_data.ori_w
        )

        ori_x = float(
            imu_data.ori_x
        )

        ori_y = float(
            imu_data.ori_y
        )

        ori_z = float(
            imu_data.ori_z
        )

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

    print("=" * 72)
    print(" LiDAR #1 -> #2 -> #3 -> Adapter -> #4 Static Planner")
    print("=" * 72)

    print()
    print("[MODE]")
    print("STATIC PLANNER INTEGRATION TEST")
    print("NO EgoCtrlCmd will be transmitted.")

    print()
    print("[GLOBAL PATH]")
    print(GLOBAL_PATH_FILE)

    print()
    print("[PLANNER CONFIG]")
    print(STATIC_CONFIG_FILE)

    # ========================================================
    # Check required files
    # ========================================================

    if not GLOBAL_PATH_FILE.exists():

        raise FileNotFoundError(
            f"Global path not found: "
            f"{GLOBAL_PATH_FILE}"
        )

    if not STATIC_CONFIG_FILE.exists():

        raise FileNotFoundError(
            f"Planner config not found: "
            f"{STATIC_CONFIG_FILE}"
        )

    # ========================================================
    # Network configuration
    # ========================================================

    network_config = load_network_config()

    gps_config = network_config["gps"]
    imu_config = network_config["imu"]

    gps_ip = gps_config["ip"]
    gps_port = int(
        gps_config["port"]
    )

    imu_ip = imu_config["ip"]
    imu_port = int(
        imu_config["port"]
    )

    print()
    print(
        f"[GPS] {gps_ip}:{gps_port}"
    )

    print(
        f"[IMU] {imu_ip}:{imu_port}"
    )

    # ========================================================
    # GPS / IMU
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

    # ========================================================
    # #1 LiDAR Receiver
    # ========================================================

    lidar_receiver = LidarReceiver()

    # ========================================================
    # #2 Object Detector
    # ========================================================

    detector = ObjectDetector(

        # Validated for the current flat MORAI test scene.
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

    # ========================================================
    # #4 Static Obstacle Planner
    # ========================================================

    global_path = np.loadtxt(
        GLOBAL_PATH_FILE,
        dtype=float,
    )

    if global_path.ndim != 2:

        raise RuntimeError(
            "Invalid global path."
        )

    if global_path.shape[1] < 2:

        raise RuntimeError(
            "Global path requires x/y columns."
        )

    if global_path.shape[1] == 2:

        global_path = np.column_stack(
            (
                global_path,
                np.zeros(
                    len(global_path)
                ),
            )
        )

    global_path = global_path[:, :3]

    planner = StaticObstaclePlanner(
        global_path,
        str(STATIC_CONFIG_FILE),
    )

    print()
    print(
        f"[PATH] waypoints={len(global_path)}"
    )

    print("[#4] StaticObstaclePlanner READY")

    # ========================================================
    # LiDAR frame state
    # ========================================================

    previous_scan_timestamp = None
    last_processed_timestamp = None

    scan_number = 0

    try:

        # ====================================================
        # Start LiDAR
        # ====================================================

        lidar_receiver.start()

        print()
        print("[READY] LiDAR receiver started.")
        print("[WAIT] Waiting for GPS + IMU + LiDAR...")
        print()
        print("[IMPORTANT] Keep the ego vehicle stationary.")
        print()

        while True:

            # =================================================
            # GPS + IMU
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
            # #1 LiDAR
            # =================================================

            latest = lidar_receiver.get_latest()

            if latest is None:

                time.sleep(0.01)
                continue

            points, stats = latest

            if points is None:

                time.sleep(0.01)
                continue

            if not stats.get(
                "fresh",
                False,
            ):

                time.sleep(0.01)
                continue

            scan_timestamp = stats.get(
                "timestamp"
            )

            if scan_timestamp is None:

                time.sleep(0.01)
                continue

            # -------------------------------------------------
            # Prevent duplicate processing
            # -------------------------------------------------

            if (
                scan_timestamp
                == last_processed_timestamp
            ):

                time.sleep(0.005)
                continue

            last_processed_timestamp = (
                scan_timestamp
            )

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

                if (
                    dt <= 0.0
                    or dt > 1.0
                ):

                    dt = 0.1

            previous_scan_timestamp = (
                scan_timestamp
            )

            # =================================================
            # #1 -> #2 coordinate conversion
            # =================================================

            vehicle_points = (
                convert_lidar_to_vehicle(
                    points
                )
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
            # Ego-motion compensation is still OFF.
            # =================================================

            tracked_obstacles = (
                tracker.update(
                    obstacles,
                    dt,
                    delta_x=0.0,
                    delta_y=0.0,
                    delta_yaw=0.0,
                )
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

                    # Temporary integration confidence.
                    confidence=1.0,
                )
            )

            # =================================================
            # #4 Static Obstacle Planner
            # =================================================

            plan_result = planner.plan(
                ego_x=ego_x,
                ego_y=ego_y,
                obstacles=planner_obstacles,
            )

            # =================================================
            # Diagnostics
            # =================================================

            print()
            print("=" * 72)
            print(
                f"SCAN {scan_number}"
            )
            print("=" * 72)

            # -------------------------------------------------
            # Ego
            # -------------------------------------------------

            print(
                f"[EGO] "
                f"x={ego_x:.3f}, "
                f"y={ego_y:.3f}, "
                f"yaw="
                f"{math.degrees(ego_yaw):.2f} deg"
            )

            # -------------------------------------------------
            # #1 / #2
            # -------------------------------------------------

            print(
                f"[LIDAR] "
                f"points={len(vehicle_points)}, "
                f"non_ground="
                f"{len(non_ground_points)}, "
                f"detected={len(obstacles)}, "
                f"dt={dt:.3f} s"
            )

            # -------------------------------------------------
            # #3
            # -------------------------------------------------

            print(
                f"[TRACK] "
                f"objects="
                f"{len(tracked_obstacles)}"
            )

            if len(
                tracked_obstacles
            ) == 0:

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
                    f"  Track "
                    f"{obstacle.track_id}: "
                    f"local_x="
                    f"{obstacle.x:.2f} m, "
                    f"local_y="
                    f"{obstacle.y:.2f} m, "
                    f"distance="
                    f"{obstacle.distance:.2f} m, "
                    f"speed="
                    f"{speed:.2f} m/s, "
                    f"dynamic="
                    f"{obstacle.is_dynamic}"
                )

            # -------------------------------------------------
            # Adapter
            # -------------------------------------------------

            print(
                f"[GLOBAL ADAPTER] "
                f"objects="
                f"{len(planner_obstacles)}"
            )

            for obstacle in planner_obstacles:

                print(
                    f"  PlannerObstacle "
                    f"id={obstacle.id}: "
                    f"global_x="
                    f"{obstacle.x:.3f}, "
                    f"global_y="
                    f"{obstacle.y:.3f}, "
                    f"width="
                    f"{obstacle.width:.2f}, "
                    f"length="
                    f"{obstacle.length:.2f}, "
                    f"distance="
                    f"{obstacle.distance:.2f}, "
                    f"dynamic="
                    f"{obstacle.is_dynamic}, "
                    f"confidence="
                    f"{obstacle.confidence:.2f}"
                )

            # -------------------------------------------------
            # #4
            # -------------------------------------------------

            print()
            print(
                "[#4 STATIC PLANNER]"
            )

            print(
                f"STATUS          : "
                f"{plan_result.status}"
            )

            print(
                f"OBSTACLE ID     : "
                f"{plan_result.obstacle_id}"
            )

            print(
                f"AVOIDANCE SIDE  : "
                f"{plan_result.avoidance_side}"
            )

            print(
                f"RECOMMENDED SPD : "
                f"{plan_result.recommended_speed_kph:.1f} "
                f"km/h"
            )

            print(
                f"LOCAL PATH      : "
                f"{len(plan_result.local_path)} "
                f"points"
            )

            print(
                f"REASON          : "
                f"{plan_result.reason}"
            )

            # -------------------------------------------------
            # First / last local path point
            # -------------------------------------------------

            if len(
                plan_result.local_path
            ) > 0:

                first_point = (
                    plan_result.local_path[0]
                )

                last_point = (
                    plan_result.local_path[-1]
                )

                print(
                    f"PATH START      : "
                    f"x={first_point[0]:.3f}, "
                    f"y={first_point[1]:.3f}"
                )

                print(
                    f"PATH END        : "
                    f"x={last_point[0]:.3f}, "
                    f"y={last_point[1]:.3f}"
                )

    except KeyboardInterrupt:

        print()
        print("[STOP] Ctrl+C received.")

    finally:

        lidar_receiver.stop()

        print(
            "[STOP] LiDAR receiver stopped."
        )

        print(
            "[SAFE] No vehicle control "
            "command was transmitted."
        )


if __name__ == "__main__":
    main()