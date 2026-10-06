#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Integrated LiDAR mission-manager diagnostic DRY-RUN.

Adds tracker/ego-motion diagnostics so false dynamic classifications can be
explained before any real vehicle braking is connected.

This script NEVER sends EgoCtrlCmd.
"""

import csv
import math
import sys
import time
from datetime import datetime
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

from lidar_receiver import LidarReceiver
from object_detection.object_detector import ObjectDetector
from object_detection.tracker import ObjectTracker
from obstacle_adapter import tracked_list_to_planner_obstacles
from static_obstacle_planner import StaticObstaclePlanner
from dynamic_obstacle_planner import DynamicObstaclePlanner
from mission_manager import MissionManager
from lidar_static_avoidance_drive_5kph import AvoidancePathLatch
from lidar_ego_motion_test import (
    read_ego_pose,
    calculate_ego_motion,
    convert_lidar_to_vehicle,
)

GLOBAL_PATH_FILE = PROJECT_ROOT / "path" / "2026_molit_comp_global_path.txt"
STATIC_CONFIG_FILE = LIDAR_DIR / "static_obstacle_config.json"
LOG_DIR = PROJECT_ROOT / "logs" / "lidar_mission_manager"


def object_id(obj):
    tid = getattr(obj, "track_id", None)
    if tid is not None:
        return tid
    return getattr(obj, "id", None)


def main():
    print("=" * 78)
    print(" LiDAR MISSION MANAGER - TRACK / EGO-MOTION DIAGNOSTIC DRY-RUN")
    print("=" * 78)
    print("[SAFE] EgoCtrlCmd is NOT transmitted.")
    print("[GOAL] Diagnose intermittent detection and false dynamic classification.")
    print()

    if not GLOBAL_PATH_FILE.exists():
        raise FileNotFoundError(f"Global path not found: {GLOBAL_PATH_FILE}")
    if not STATIC_CONFIG_FILE.exists():
        raise FileNotFoundError(f"Static config not found: {STATIC_CONFIG_FILE}")

    global_path = np.loadtxt(GLOBAL_PATH_FILE, dtype=float)
    if global_path.ndim != 2 or global_path.shape[1] < 2:
        raise RuntimeError("Global path requires x/y columns.")
    if global_path.shape[1] == 2:
        global_path = np.column_stack((global_path, np.zeros(len(global_path))))
    global_path = global_path[:, :3]

    static_planner = StaticObstaclePlanner(global_path, str(STATIC_CONFIG_FILE))
    static_latch = AvoidancePathLatch(global_path)
    dynamic_planner = DynamicObstaclePlanner(
        path_y_min=-1.5, path_y_max=1.5, prediction_time_s=5.0,
        min_forward_m=0.0, max_forward_m=20.0,
        min_lateral_speed_mps=0.01,
        risk_confirm_scans=3, release_clear_scans=3,
    )
    mission_manager = MissionManager()

    detector = ObjectDetector(
        ground_z_threshold=-1.70, dbscan_eps=0.5, dbscan_min_samples=5,
        min_cluster_points=20, min_width=0.15, max_width=3.00,
        min_length=0.05, max_length=5.00,
    )
    tracker = ObjectTracker(
        max_match_distance=1.0, velocity_window_size=5,
        dynamic_speed_threshold=0.5, dynamic_confirm_frames=3,
        max_missed_frames=3,
    )

    cfg = load_network_config()
    gps_receiver = Receiver(cfg["gps"]["ip"], int(cfg["gps"]["port"]), GPS())
    imu_receiver = Receiver(cfg["imu"]["ip"], int(cfg["imu"]["port"]), IMU())
    lidar_receiver = LidarReceiver()

    LOG_DIR.mkdir(parents=True, exist_ok=True)
    log_path = LOG_DIR / (
        "mission_diagnostic_" + datetime.now().strftime("%Y%m%d_%H%M%S") + ".csv"
    )
    log_file = open(log_path, "w", newline="", encoding="utf-8-sig")
    writer = csv.writer(log_file)
    writer.writerow([
        "scan", "dt_s", "ego_x", "ego_y", "ego_yaw_deg",
        "ego_delta_forward_m", "ego_delta_left_m", "ego_delta_yaw_deg",
        "det_count", "track_count",
        "track_id", "object_x_m", "object_y_m", "distance_m",
        "velocity_x_mps", "velocity_y_mps", "track_speed_mps",
        "is_dynamic", "dynamic_count",
        "static_raw", "static_final", "static_obstacle_id", "latch_state",
        "dynamic_raw", "dynamic_final", "dynamic_obstacle_id",
        "risk_confirm_count", "release_clear_count", "mission",
    ])
    print(f"[LOG] {log_path}")

    previous_scan_timestamp = None
    last_processed_timestamp = None
    previous_ego_pose = None
    previous_mode = None
    scan_number = 0

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
                delta_forward = delta_left = delta_yaw = 0.0
            else:
                _, _, delta_forward, delta_left, delta_yaw = calculate_ego_motion(
                    previous_ego_pose[0], previous_ego_pose[1], previous_ego_pose[2],
                    ego_x, ego_y, ego_yaw,
                )
            previous_ego_pose = (ego_x, ego_y, ego_yaw)

            vehicle_points = convert_lidar_to_vehicle(points)
            obstacles, _, _ = detector.detect(vehicle_points)
            tracked = tracker.update(
                obstacles, dt,
                delta_x=delta_forward,
                delta_y=delta_left,
                delta_yaw=delta_yaw,
            )

            planner_obstacles = tracked_list_to_planner_obstacles(
                tracked, ego_x, ego_y, ego_yaw
            )
            raw_static = static_planner.plan(ego_x, ego_y, planner_obstacles)
            static_result, latch_state = static_latch.apply(
                raw_static, ego_x, ego_y
            )

            dynamic_state = dynamic_planner.plan_confirmed(tracked)
            decision = mission_manager.decide(static_result, dynamic_state)

            if decision.mode != previous_mode:
                print(
                    f"[MISSION EVENT] SCAN {scan_number}: "
                    f"{previous_mode} -> {decision.mode}"
                )
                previous_mode = decision.mode

            print(
                f"[SCAN {scan_number:05d}] dt={dt:.3f}s "
                f"ego_d=({delta_forward:+.3f},{delta_left:+.3f})m "
                f"dyaw={math.degrees(delta_yaw):+.2f}deg | "
                f"det={len(obstacles)} track={len(tracked)} | "
                f"STATIC {raw_static.status}->{static_result.status} | "
                f"DYNAMIC {dynamic_state.raw_result.status}->{dynamic_state.status} | "
                f"MISSION={decision.mode}"
            )

            if not tracked:
                writer.writerow([
                    scan_number, dt, ego_x, ego_y, math.degrees(ego_yaw),
                    delta_forward, delta_left, math.degrees(delta_yaw),
                    len(obstacles), 0,
                    "", "", "", "", "", "", "", "", "",
                    raw_static.status, static_result.status,
                    static_result.obstacle_id, latch_state,
                    dynamic_state.raw_result.status, dynamic_state.status,
                    dynamic_state.confirmed_obstacle_id,
                    dynamic_state.risk_confirm_count,
                    dynamic_state.release_clear_count, decision.mode,
                ])
            else:
                for obj in tracked:
                    tid = object_id(obj)
                    vx = float(getattr(obj, "velocity_x", 0.0))
                    vy = float(getattr(obj, "velocity_y", 0.0))
                    speed = math.hypot(vx, vy)
                    dyn_count = tracker.dynamic_count.get(tid, 0)
                    is_dynamic = bool(getattr(obj, "is_dynamic", False))
                    x = float(getattr(obj, "x", 0.0))
                    y = float(getattr(obj, "y", 0.0))
                    distance = float(getattr(obj, "distance", math.hypot(x, y)))

                    print(
                        f"  [TRACK {tid}] x={x:+.2f} y={y:+.2f} "
                        f"vx={vx:+.2f} vy={vy:+.2f} "
                        f"speed={speed:.2f}m/s "
                        f"dynamic={is_dynamic} "
                        f"count={dyn_count}/{tracker.dynamic_confirm_frames}"
                    )

                    writer.writerow([
                        scan_number, dt, ego_x, ego_y, math.degrees(ego_yaw),
                        delta_forward, delta_left, math.degrees(delta_yaw),
                        len(obstacles), len(tracked),
                        tid, x, y, distance, vx, vy, speed,
                        is_dynamic, dyn_count,
                        raw_static.status, static_result.status,
                        static_result.obstacle_id, latch_state,
                        dynamic_state.raw_result.status, dynamic_state.status,
                        dynamic_state.confirmed_obstacle_id,
                        dynamic_state.risk_confirm_count,
                        dynamic_state.release_clear_count, decision.mode,
                    ])

            if decision.mode == "STOP":
                print(
                    "  [DECISION] Confirmed dynamic risk -> STOP "
                    "(DRY-RUN: no brake command sent)"
                )
            elif decision.mode == "STATIC_STOP":
                print(
                    "  [DECISION] Static planner requires stop "
                    "(DRY-RUN: no brake command sent)"
                )
            elif decision.mode == "STATIC_AVOID":
                print(
                    f"  [DECISION] Use static local path "
                    f"side={getattr(static_result, 'avoidance_side', None)}"
                )

            log_file.flush()
            time.sleep(0.005)

    except KeyboardInterrupt:
        print()
        print("[STOP] Diagnostic DRY-RUN finished.")
        print(f"[LOG SAVED] {log_path}")
    finally:
        try:
            lidar_receiver.stop()
        except Exception:
            pass
        log_file.close()


if __name__ == "__main__":
    main()
