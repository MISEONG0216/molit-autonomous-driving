#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""#4 baseline + #5 dynamic judgment isolation test.

Purpose:
- Keep the same GPS/IMU/LiDAR Receiver initialization and ego-motion/tracker
  chain used by the validated static-obstacle (#4) integration.
- Add ONLY DynamicObstaclePlanner judgment.
- Do not create MissionManager and do not send EgoCtrlCmd.

This isolates whether adding #5 itself affects the previously working #4
sensor/tracking chain.
"""

import math
import sys
import time
from pathlib import Path

LIDAR_DIR = Path(__file__).resolve().parent
SRC_DIR = LIDAR_DIR.parent

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
from dynamic_obstacle_planner import DynamicObstaclePlanner
from lidar_ego_motion_test import (
    read_ego_pose,
    calculate_ego_motion,
    convert_lidar_to_vehicle,
)


def main():
    print("=" * 78)
    print(" #4 BASELINE + #5 DYNAMIC JUDGMENT ISOLATION TEST")
    print("=" * 78)
    print("[BASE] GPS / IMU / LiDAR / detector / tracker = #4 structure")
    print("[ADD ] DynamicObstaclePlanner only")
    print("[NO  ] MissionManager")
    print("[SAFE] No EgoCtrlCmd is transmitted")
    print("[TEST] Keep still briefly, then move the vehicle slowly.")
    print()

    network_config = load_network_config()
    gps_cfg = network_config["gps"]
    imu_cfg = network_config["imu"]

    # Keep the same receiver construction order/style as the working #4 code.
    gps_receiver = Receiver(gps_cfg["ip"], int(gps_cfg["port"]), GPS())
    imu_receiver = Receiver(imu_cfg["ip"], int(imu_cfg["port"]), IMU())
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

    dynamic_planner = DynamicObstaclePlanner(
        path_y_min=-1.5,
        path_y_max=1.5,
        prediction_time_s=5.0,
        min_forward_m=0.0,
        max_forward_m=20.0,
        min_lateral_speed_mps=0.01,
        risk_confirm_scans=3,
        release_clear_scans=3,
    )

    previous_scan_timestamp = None
    last_processed_timestamp = None
    previous_ego_pose = None
    previous_print_pose = None
    scan_number = 0

    try:
        lidar_receiver.start()
        print(
            f"[READY] GPS {gps_cfg['ip']}:{gps_cfg['port']} / "
            f"IMU {imu_cfg['ip']}:{imu_cfg['port']} / LiDAR"
        )
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

            if previous_print_pose is None:
                global_step = 0.0
            else:
                global_step = math.hypot(
                    ego_x - previous_print_pose[0],
                    ego_y - previous_print_pose[1],
                )
            previous_print_pose = (ego_x, ego_y)

            vehicle_points = convert_lidar_to_vehicle(points)
            obstacles, _, _ = detector.detect(vehicle_points)

            tracked = tracker.update(
                obstacles,
                dt,
                delta_x=delta_forward,
                delta_y=delta_left,
                delta_yaw=delta_yaw,
            )

            dynamic_state = dynamic_planner.plan_confirmed(tracked)

            print(
                f"[SCAN {scan_number:05d}] "
                f"GPS=({ego_x:+.3f},{ego_y:+.3f}) "
                f"step={global_step:.3f}m | "
                f"ego_d=({delta_forward:+.3f},{delta_left:+.3f})m "
                f"dyaw={math.degrees(delta_yaw):+.2f}deg | "
                f"det={len(obstacles)} track={len(tracked)} | "
                f"DYNAMIC {dynamic_state.raw_result.status}"
                f"->{dynamic_state.status}"
            )

            for obj in tracked:
                track_id = getattr(obj, "track_id", None)
                vx = float(getattr(obj, "velocity_x", 0.0))
                vy = float(getattr(obj, "velocity_y", 0.0))
                speed = math.hypot(vx, vy)
                print(
                    f"  [TRACK {track_id}] "
                    f"x={float(getattr(obj, 'x', 0.0)):+.2f} "
                    f"y={float(getattr(obj, 'y', 0.0)):+.2f} "
                    f"v=({vx:+.2f},{vy:+.2f}) "
                    f"speed={speed:.2f}m/s "
                    f"dynamic={bool(getattr(obj, 'is_dynamic', False))}"
                )

            time.sleep(0.005)

    except KeyboardInterrupt:
        print()
        print("[STOP] #4 + #5 isolation test finished.")
    finally:
        try:
            lidar_receiver.stop()
        except Exception:
            pass
        print("[SAFE] No vehicle-control command was transmitted.")


if __name__ == "__main__":
    main()
