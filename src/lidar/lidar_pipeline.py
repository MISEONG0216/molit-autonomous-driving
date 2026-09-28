#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
LiDAR 1 -> 2 -> 3 -> Adapter Integration Pipeline

#1 LiDAR Receiver / Parser / Preprocess
    MORAI VLP16 UDP
    -> ROI-filtered point cloud

#2 Object Detection
    Ground removal
    -> DBSCAN clustering
    -> Obstacle creation

#3 Object Tracking
    Track ID assignment
    -> velocity estimation
    -> static / dynamic classification

Adapter
    #3 vehicle-relative obstacle
    -> #4 planner-compatible global obstacle format

IMPORTANT
---------
This test does NOT use real GPS / IMU yet.

For adapter integration testing only:

    ego_x   = 100.0
    ego_y   = 200.0
    ego_yaw = 0 deg

Therefore the resulting global obstacle coordinates are TEST coordinates,
not actual MORAI global coordinates.

Ego-motion compensation for tracking is also NOT connected yet.
Keep the ego vehicle stationary during this test.
"""

import math
import time

import numpy as np

from lidar_receiver import LidarReceiver
from object_detection.object_detector import ObjectDetector
from object_detection.tracker import ObjectTracker
from obstacle_adapter import tracked_list_to_planner_obstacles


# ============================================================
# TEMPORARY EGO POSE
# ============================================================
#
# These values are used ONLY to verify that the real #3 Track
# object can pass through obstacle_adapter.py.
#
# Real GPS / IMU will replace these values in the next stage.
#

TEST_EGO_X = 100.0
TEST_EGO_Y = 200.0
TEST_EGO_YAW = math.radians(0.0)


def convert_lidar_to_vehicle(points):
    """
    Convert #1 LiDAR coordinates to #2/#3 vehicle coordinates.

    #1 LiDAR:
        +X = Right
        +Y = Forward
        +Z = Up

    #2 / #3:
        +X = Forward
        +Y = Left
        +Z = Up

    Conversion:
        vehicle_x = lidar_y
        vehicle_y = -lidar_x
        vehicle_z = lidar_z
    """

    if points is None or len(points) == 0:
        return np.empty((0, 3), dtype=np.float32)

    converted = np.column_stack(
        (
            points[:, 1],
            -points[:, 0],
            points[:, 2],
        )
    )

    return converted.astype(np.float32)


def print_cluster_diagnostics(non_ground_points, labels):
    """
    Print DBSCAN cluster information.
    """

    if len(non_ground_points) == 0 or len(labels) == 0:
        return

    cluster_ids = sorted(set(labels))
    cluster_ids = [
        cluster_id
        for cluster_id in cluster_ids
        if cluster_id != -1
    ]

    noise_count = int(np.sum(labels == -1))

    print(
        f"[DBSCAN] clusters={len(cluster_ids)}, "
        f"noise={noise_count}"
    )

    for cluster_id in cluster_ids:

        cluster_points = non_ground_points[
            labels == cluster_id
        ]

        if len(cluster_points) == 0:
            continue

        x_min = float(np.min(cluster_points[:, 0]))
        x_max = float(np.max(cluster_points[:, 0]))

        y_min = float(np.min(cluster_points[:, 1]))
        y_max = float(np.max(cluster_points[:, 1]))

        z_min = float(np.min(cluster_points[:, 2]))
        z_max = float(np.max(cluster_points[:, 2]))

        center_x = (x_min + x_max) / 2.0
        center_y = (y_min + y_max) / 2.0

        length = x_max - x_min
        width = y_max - y_min
        height = z_max - z_min

        distance = float(
            np.hypot(center_x, center_y)
        )

        print(
            f"  Cluster {cluster_id}: "
            f"points={len(cluster_points)}, "
            f"x={center_x:.2f} m, "
            f"y={center_y:.2f} m, "
            f"dist={distance:.2f} m, "
            f"length={length:.2f} m, "
            f"width={width:.2f} m, "
            f"height={height:.2f} m, "
            f"z={z_min:.2f}~{z_max:.2f}"
        )


def main():

    print("============================================================")
    print(" LiDAR 1 -> 2 -> 3 -> Adapter Integration")
    print("============================================================")
    print()

    print("[#1] MORAI VLP16 UDP Receiver / Preprocess")
    print("[#2] Ground Removal + DBSCAN Object Detection")
    print("[#3] Object Tracking")
    print("[ADAPTER] Vehicle obstacle -> Planner obstacle")
    print()

    print("[TEST EGO POSE]")
    print(
        f"x={TEST_EGO_X:.2f}, "
        f"y={TEST_EGO_Y:.2f}, "
        f"yaw={math.degrees(TEST_EGO_YAW):.1f} deg"
    )

    print()
    print("[WARNING]")
    print("GPS / IMU are NOT connected yet.")
    print("Adapter global coordinates are TEST coordinates.")
    print("Keep the ego vehicle stationary.")
    print()

    # ---------------------------------------------------------
    # #1 LiDAR Receiver
    # ---------------------------------------------------------

    receiver = LidarReceiver()

    # ---------------------------------------------------------
    # #2 Object Detector
    # ---------------------------------------------------------

    detector = ObjectDetector(
        ground_z_threshold=0.08,
        dbscan_eps=0.5,
        dbscan_min_samples=5,
        min_cluster_points=20,
        min_width=0.15,
        max_width=3.00,
        min_length=0.05,
        max_length=5.00,
    )

    # ---------------------------------------------------------
    # #3 Object Tracker
    # ---------------------------------------------------------

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

        receiver.start()

        print("[READY] LiDAR receiver started.")
        print("[WAIT] Waiting for MORAI LiDAR data...")
        print()

        while True:

            latest = receiver.get_latest()

            if latest is None:
                time.sleep(0.01)
                continue

            points, stats = latest

            if points is None:
                time.sleep(0.01)
                continue

            # -------------------------------------------------
            # Reject stale LiDAR data
            # -------------------------------------------------

            if not stats.get("fresh", False):
                time.sleep(0.01)
                continue

            scan_timestamp = stats.get("timestamp")

            if scan_timestamp is None:
                time.sleep(0.01)
                continue

            # -------------------------------------------------
            # Prevent duplicate processing
            # -------------------------------------------------

            if scan_timestamp == last_processed_timestamp:
                time.sleep(0.005)
                continue

            last_processed_timestamp = scan_timestamp
            scan_number += 1

            # -------------------------------------------------
            # Tracker dt
            # -------------------------------------------------

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

            # -------------------------------------------------
            # #1 -> #2 coordinate conversion
            # -------------------------------------------------

            vehicle_points = convert_lidar_to_vehicle(
                points
            )

            # -------------------------------------------------
            # #2 Object Detection
            # -------------------------------------------------

            (
                obstacles,
                non_ground_points,
                labels,
            ) = detector.detect(
                vehicle_points
            )

            # -------------------------------------------------
            # #3 Object Tracking
            #
            # Ego-motion compensation is still OFF.
            # -------------------------------------------------

            tracked_obstacles = tracker.update(
                obstacles,
                dt,
                delta_x=0.0,
                delta_y=0.0,
                delta_yaw=0.0,
            )

            # -------------------------------------------------
            # #3 -> Adapter
            #
            # Real Track objects are passed to the adapter.
            #
            # GPS / IMU are intentionally replaced with a
            # fixed test pose at this stage.
            # -------------------------------------------------

            planner_obstacles = (
                tracked_list_to_planner_obstacles(
                    tracked_obstacles,
                    ego_x=TEST_EGO_X,
                    ego_y=TEST_EGO_Y,
                    ego_yaw=TEST_EGO_YAW,
                    confidence=1.0,
                )
            )

            # -------------------------------------------------
            # Diagnostics
            # -------------------------------------------------

            print()
            print(
                "============================================================"
            )
            print(f"SCAN {scan_number}")
            print(
                "============================================================"
            )

            print(
                f"[INPUT] points={len(vehicle_points)}, "
                f"non_ground={len(non_ground_points)}, "
                f"objects={len(obstacles)}, "
                f"dt={dt:.3f} s"
            )

            print_cluster_diagnostics(
                non_ground_points,
                labels,
            )

            # -------------------------------------------------
            # #3 output
            # -------------------------------------------------

            print(
                f"[TRACK] objects="
                f"{len(tracked_obstacles)}"
            )

            if len(tracked_obstacles) == 0:
                print("  No tracked obstacle")

            for obstacle in tracked_obstacles:

                speed = float(
                    np.hypot(
                        obstacle.velocity_x,
                        obstacle.velocity_y,
                    )
                )

                print(
                    f"  Track {obstacle.track_id}: "
                    f"cluster={obstacle.cluster_id}, "
                    f"x={obstacle.x:.2f} m, "
                    f"y={obstacle.y:.2f} m, "
                    f"dist={obstacle.distance:.2f} m, "
                    f"width={obstacle.width:.2f} m, "
                    f"length={obstacle.length:.2f} m, "
                    f"speed={speed:.2f} m/s, "
                    f"dynamic={obstacle.is_dynamic}"
                )

            # -------------------------------------------------
            # Adapter output
            # -------------------------------------------------

            print(
                f"[ADAPTER] planner_objects="
                f"{len(planner_obstacles)}"
            )

            if len(planner_obstacles) == 0:
                print("  No planner obstacle")

            for obstacle in planner_obstacles:

                print(
                    f"  PlannerObstacle "
                    f"id={obstacle.id}: "
                    f"global_x={obstacle.x:.2f}, "
                    f"global_y={obstacle.y:.2f}, "
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

        receiver.stop()

        print("[STOP] LiDAR receiver stopped.")


if __name__ == "__main__":
    main()