#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""LiDAR dynamic-obstacle planner DRY-RUN integration test.

Pipeline:
GPS/IMU + VLP16 -> #2 detection -> #3 tracking/ego compensation
-> #5 DynamicObstaclePlanner -> console + CSV diagnostics.

This script NEVER sends EgoCtrlCmd.
"""

import csv
import math
import sys
import time
from pathlib import Path

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

from pure_pursuit_competition import CompetitionSpeedReceiver

from lidar_receiver import LidarReceiver
from object_detection.object_detector import ObjectDetector
from object_detection.tracker import ObjectTracker
from dynamic_obstacle_planner import DynamicObstaclePlanner
from lidar_ego_motion_test import (
    read_ego_pose,
    calculate_ego_motion,
    convert_lidar_to_vehicle,
)

LOG_DIR = PROJECT_ROOT / "logs" / "lidar_dynamic_planner"
RISK_CONFIRM_SCANS = 3
MIN_EGO_SPEED_FOR_TTC_MPS = 0.3


def fmt(value, digits=2):
    if value is None:
        return "-"
    return f"{float(value):.{digits}f}"


def main():
    print("=" * 76)
    print(" LiDAR #1 -> #2 -> #3 -> #5 DYNAMIC PLANNER TEST")
    print("=" * 76)
    print("[MODE] DRY-RUN ONLY - EgoCtrlCmd is NOT transmitted")
    print("[RISK] dynamic + IN_PATH, or predicted path entry within 5.0 s")
    print(f"[CONFIRM] RISK must persist for {RISK_CONFIRM_SCANS} consecutive scans")
    print("[TTC] Logging object path-entry time vs ego arrival time (diagnostic only)")
    print()

    network_config = load_network_config()
    gps_cfg = network_config["gps"]
    imu_cfg = network_config["imu"]

    gps_receiver = Receiver(gps_cfg["ip"], int(gps_cfg["port"]), GPS())
    imu_receiver = Receiver(imu_cfg["ip"], int(imu_cfg["port"]), IMU())
    lidar_receiver = LidarReceiver()
    speed_receiver = CompetitionSpeedReceiver()

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

    dynamic_planner = DynamicObstaclePlanner(
        path_y_min=-1.5,
        path_y_max=1.5,
        prediction_time_s=5.0,
        min_forward_m=0.0,
        max_forward_m=20.0,
        min_lateral_speed_mps=0.01,
    )

    previous_scan_timestamp = None
    last_processed_timestamp = None
    previous_ego_pose = None
    scan_number = 0
    previous_raw_status = None
    previous_confirmed_status = None
    risk_count_by_track = {}

    LOG_DIR.mkdir(parents=True, exist_ok=True)
    run_stamp = time.strftime("%Y%m%d_%H%M%S")
    log_file = LOG_DIR / f"dynamic_planner_{run_stamp}.csv"
    log_handle = log_file.open("w", newline="", encoding="utf-8-sig")
    log_writer = csv.writer(log_handle)
    log_writer.writerow([
        "scan",
        "ego_x", "ego_y", "ego_yaw_deg", "ego_speed_kmh",
        "cluster_id", "track_id",
        "object_x", "object_y", "distance",
        "velocity_x_mps", "velocity_y_mps", "track_speed_mps",
        "is_dynamic", "dynamic_count",
        "path_state", "prediction_state", "time_to_path_s",
        "predicted_cross_x_m", "ego_time_to_cross_s", "time_gap_s",
        "object_result",
        "raw_final_status", "confirmed_status", "risk_confirm_count",
        "final_obstacle_id", "final_reason",
    ])
    log_handle.flush()

    print(f"[LOG] {log_file}")

    try:
        lidar_receiver.start()
        print("[READY] GPS / IMU / LiDAR")
        print("[WAIT] Waiting for first fresh LiDAR scan...")
        print()

        while True:
            ego_pose = read_ego_pose(gps_receiver, imu_receiver)
            if ego_pose is None:
                time.sleep(0.03)
                continue

            ego_x, ego_y, ego_yaw = ego_pose

            ego_speed_kmh, speed_valid = speed_receiver.read()
            if not speed_valid:
                ego_speed_kmh = 0.0
            ego_speed_mps = max(0.0, float(ego_speed_kmh) / 3.6)

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
            obstacles, _, _ = detector.detect(vehicle_points)

            tracked_obstacles = tracker.update(
                obstacles,
                dt,
                delta_x=delta_forward,
                delta_y=delta_left,
                delta_yaw=delta_yaw,
            )

            final_result = dynamic_planner.plan(tracked_obstacles)
            raw_status = final_result.status

            # Confirm risk per persistent track ID.  A risk from one object
            # must not continue the counter of another object.
            current_risk_track = (
                final_result.obstacle_id
                if raw_status == "RISK"
                else None
            )

            if current_risk_track is None:
                risk_count_by_track.clear()
                current_risk_count = 0
            else:
                previous_count = risk_count_by_track.get(current_risk_track, 0)
                risk_count_by_track = {
                    current_risk_track: previous_count + 1
                }
                current_risk_count = risk_count_by_track[current_risk_track]

            confirmed_status = (
                "RISK_CONFIRMED"
                if (
                    current_risk_track is not None
                    and current_risk_count >= RISK_CONFIRM_SCANS
                )
                else "CLEAR"
            )

            if raw_status != previous_raw_status:
                print(
                    f"[RAW EVENT] SCAN {scan_number}: "
                    f"{previous_raw_status} -> {raw_status}"
                )
                previous_raw_status = raw_status

            if confirmed_status != previous_confirmed_status:
                print(
                    f"[CONFIRMED EVENT] SCAN {scan_number}: "
                    f"{previous_confirmed_status} -> {confirmed_status}"
                )
                previous_confirmed_status = confirmed_status

            print(
                f"[SCAN {scan_number:05d}] "
                f"detected={len(obstacles)} "
                f"tracked={len(tracked_obstacles)} "
                f"=> RAW={raw_status} "
                f"confirm={current_risk_count}/{RISK_CONFIRM_SCANS} "
                f"=> {confirmed_status} "
                f"track={final_result.obstacle_id} "
                f"time_to_path={fmt(final_result.time_to_path_s)} s"
            )

            diagnostic_rows = []

            for obstacle in tracked_obstacles:
                track_id = getattr(obstacle, "track_id", None)
                cluster_id = getattr(obstacle, "cluster_id", getattr(obstacle, "id", None))
                x = float(getattr(obstacle, "x", 0.0))
                y = float(getattr(obstacle, "y", 0.0))
                distance = float(getattr(obstacle, "distance", math.hypot(x, y)))
                velocity_x = float(getattr(obstacle, "velocity_x", 0.0))
                velocity_y = float(getattr(obstacle, "velocity_y", 0.0))
                speed = math.hypot(velocity_x, velocity_y)
                is_dynamic = bool(getattr(obstacle, "is_dynamic", False))
                dynamic_count = (
                    tracker.dynamic_count.get(track_id, 0)
                    if track_id is not None
                    else 0
                )

                object_result = dynamic_planner.evaluate_obstacle(obstacle)

                # Diagnostic only: predict the object's longitudinal X
                # position at the moment it reaches the ego-path corridor.
                # Then compare that event time with the ego's arrival time at
                # the same predicted X.  Do NOT use this to brake yet.
                if object_result.time_to_path_s is not None:
                    predicted_cross_x = (
                        x
                        + velocity_x * object_result.time_to_path_s
                    )
                else:
                    predicted_cross_x = None

                if (
                    predicted_cross_x is not None
                    and predicted_cross_x > 0.0
                    and ego_speed_mps >= MIN_EGO_SPEED_FOR_TTC_MPS
                ):
                    ego_time_to_cross = predicted_cross_x / ego_speed_mps
                else:
                    ego_time_to_cross = None

                if (
                    object_result.time_to_path_s is not None
                    and ego_time_to_cross is not None
                ):
                    time_gap = abs(
                        ego_time_to_cross - object_result.time_to_path_s
                    )
                else:
                    time_gap = None

                if is_dynamic:
                    print(
                        "  [DYNAMIC] "
                        f"track={track_id} "
                        f"x={x:.2f} y={y:.2f} "
                        f"vx={velocity_x:+.2f} vy={velocity_y:+.2f} "
                        f"speed={speed:.2f} "
                        f"confirm={dynamic_count}/{tracker.dynamic_confirm_frames}"
                    )
                    print(
                        "            "
                        f"path={object_result.path_state} "
                        f"prediction={object_result.prediction_state} "
                        f"time_to_path={fmt(object_result.time_to_path_s)} s "
                        f"cross_x={fmt(predicted_cross_x)} m "
                        f"ego_to_cross={fmt(ego_time_to_cross)} s "
                        f"gap={fmt(time_gap)} s "
                        f"=> {object_result.status}"
                    )

                diagnostic_rows.append([
                    scan_number,
                    ego_x, ego_y, math.degrees(ego_yaw), ego_speed_kmh,
                    cluster_id, track_id,
                    x, y, distance,
                    velocity_x, velocity_y, speed,
                    is_dynamic, dynamic_count,
                    object_result.path_state,
                    object_result.prediction_state,
                    object_result.time_to_path_s,
                    predicted_cross_x,
                    ego_time_to_cross,
                    time_gap,
                    object_result.status,
                    raw_status,
                    confirmed_status,
                    current_risk_count,
                    final_result.obstacle_id,
                    final_result.reason,
                ])

            if diagnostic_rows:
                log_writer.writerows(diagnostic_rows)
            else:
                log_writer.writerow([
                    scan_number,
                    ego_x, ego_y, math.degrees(ego_yaw), ego_speed_kmh,
                    "", "", "", "", "", "", "", "",
                    False, 0,
                    "", "", "", "", "", "",
                    raw_status,
                    confirmed_status,
                    consecutive_risk_scans,
                    final_result.obstacle_id,
                    final_result.reason,
                ])
            log_handle.flush()

            time.sleep(0.005)

    except KeyboardInterrupt:
        print()
        print("[STOP] Dynamic planner DRY-RUN finished.")

    finally:
        try:
            lidar_receiver.stop()
        except Exception:
            pass
        log_handle.close()
        print(f"[LOG SAVED] {log_file}")


if __name__ == "__main__":
    main()
