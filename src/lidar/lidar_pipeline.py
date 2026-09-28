#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
LiDAR 1 -> 2 -> 3 Integration Pipeline

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

Coordinate conversion
---------------------
#1 LiDAR coordinates:
    +X = Right
    +Y = Forward
    +Z = Up

#2 / #3 coordinates:
    +X = Forward
    +Y = Left
    +Z = Up

Important:
    Ego-motion compensation is NOT connected yet.
    Therefore velocity / dynamic classification is only reliable
    while the ego vehicle is stationary.
"""

import time

import numpy as np

from lidar_receiver import LidarReceiver
from object_detection.object_detector import ObjectDetector
from object_detection.tracker import ObjectTracker


def convert_lidar_to_vehicle(points):
    """
    Convert #1 LiDAR coordinate system to the coordinate system
    used by #2 ObjectDetector and #3 ObjectTracker.

    Input:
        points[:, 0] = LiDAR X = Right
        points[:, 1] = LiDAR Y = Forward
        points[:, 2] = LiDAR Z = Up
        points[:, 3] = Intensity

    Output:
        converted[:, 0] = Forward
        converted[:, 1] = Left
        converted[:, 2] = Up

    Coordinate conversion:
        vehicle_x = lidar_y
        vehicle_y = -lidar_x
        vehicle_z = lidar_z
    """

    if points is None or len(points) == 0:
        return np.empty((0, 3), dtype=np.float32)

    return np.column_stack(
        (
            points[:, 1],
            -points[:, 0],
            points[:, 2],
        )
    ).astype(np.float32)


def print_cluster_diagnostics(non_ground_points, labels):
    """
    Print diagnostic information for every DBSCAN cluster.

    This is useful while tuning:
        - min_cluster_points
        - min_width
        - min_length
        - DBSCAN eps / min_samples
    """

    if len(non_ground_points) == 0 or len(labels) == 0:
        return

    cluster_ids = sorted(set(labels))
    cluster_ids = [cluster_id for cluster_id in cluster_ids if cluster_id != -1]

    noise_count = int(np.sum(labels == -1))

    print(
        f"[DBSCAN] clusters={len(cluster_ids)}, "
        f"noise={noise_count}"
    )

    for cluster_id in cluster_ids:
        cluster_points = non_ground_points[labels == cluster_id]

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

        distance = float(np.hypot(center_x, center_y))

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
    print(" LiDAR 1 -> 2 -> 3 Integration Pipeline")
    print("============================================================")
    print()
    print("[#1] MORAI VLP16 UDP Receiver / Preprocess")
    print("[#2] Ground Removal + DBSCAN Object Detection")
    print("[#3] Object Tracking")
    print()
    print("[WARNING]")
    print("Ego-motion compensation is NOT connected yet.")
    print("Dynamic classification is reliable only while ego is stationary.")
    print()

    # ---------------------------------------------------------
    # #1 LiDAR receiver
    # ---------------------------------------------------------

    receiver = LidarReceiver()

    # ---------------------------------------------------------
    # #2 Object detector
    # ---------------------------------------------------------
    #
    # These are the latest values used during the previous
    # real MORAI integration test.
    #
    # NOTE:
    # min_cluster_points=20 can reject distant obstacles because
    # a far object may contain only a small number of LiDAR points.
    # Do NOT lower it yet until false-positive tests are performed.
    #

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
    # #3 Object tracker
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
            # Ignore stale LiDAR data.
            # -------------------------------------------------

            if not stats.get("fresh", False):
                time.sleep(0.01)
                continue

            scan_timestamp = stats.get("timestamp")

            if scan_timestamp is None:
                time.sleep(0.01)
                continue

            # -------------------------------------------------
            # Do not process the same LiDAR scan repeatedly.
            # -------------------------------------------------

            if scan_timestamp == last_processed_timestamp:
                time.sleep(0.005)
                continue

            last_processed_timestamp = scan_timestamp
            scan_number += 1

            # -------------------------------------------------
            # Calculate dt for tracker.
            # -------------------------------------------------

            if previous_scan_timestamp is None:
                dt = 0.1
            else:
                dt = scan_timestamp - previous_scan_timestamp

                if dt <= 0.0 or dt > 1.0:
                    dt = 0.1

            previous_scan_timestamp = scan_timestamp

            # -------------------------------------------------
            # Coordinate conversion
            #
            # #1:
            #   X = Right
            #   Y = Forward
            #
            # #2/#3:
            #   X = Forward
            #   Y = Left
            # -------------------------------------------------

            vehicle_points = convert_lidar_to_vehicle(points)

            # -------------------------------------------------
            # #2 Object Detection
            # -------------------------------------------------

            obstacles, non_ground_points, labels = detector.detect(
                vehicle_points
            )

            # -------------------------------------------------
            # #3 Object Tracking
            #
            # Ego-motion compensation is currently OFF.
            #
            # This means:
            #     delta_x   = 0
            #     delta_y   = 0
            #     delta_yaw = 0
            #
            # We will connect GPS / IMU later.
            # -------------------------------------------------

            tracked_obstacles = tracker.update(
                obstacles,
                dt,
                delta_x=0.0,
                delta_y=0.0,
                delta_yaw=0.0,
            )

            # -------------------------------------------------
            # Diagnostics
            # -------------------------------------------------

            print()
            print("============================================================")
            print(f"SCAN {scan_number}")
            print("============================================================")

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

            print(f"[TRACK] objects={len(tracked_obstacles)}")

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
                    f"vx={obstacle.velocity_x:.2f} m/s, "
                    f"vy={obstacle.velocity_y:.2f} m/s, "
                    f"speed={speed:.2f} m/s, "
                    f"dynamic={obstacle.is_dynamic}"
                )

    except KeyboardInterrupt:

        print()
        print("[STOP] Ctrl+C received.")

    finally:

        receiver.stop()

        print("[STOP] LiDAR receiver stopped.")


if __name__ == "__main__":
    main()