#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Experimental V7 57 km/h global + LiDAR static avoidance 25 km/h.
DRY-RUN by default. Not yet validated in MORAI.
"""
import argparse
import math
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
SRC = HERE.parent
for p in (SRC, HERE):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

from morai_common import ensure_morai_import_path, load_network_config
ensure_morai_import_path()
from lib.network.UDP import Receiver, Sender
from lib.define.GPS import GPS
from lib.define.IMU import IMU
from pure_pursuit_competition import CompetitionSpeedReceiver
import pure_pursuit_regulation as control
import pure_pursuit_profile_57_corner_v7_early_brake as v7
from lidar_receiver import LidarReceiver
from object_detection.object_detector import ObjectDetector
from object_detection.tracker import ObjectTracker
from obstacle_adapter import tracked_list_to_planner_obstacles
from static_obstacle_planner import StaticObstaclePlanner
from lidar_ego_motion_test import read_ego_pose, calculate_ego_motion, convert_lidar_to_vehicle
from lidar_static_avoidance_drive_5kph import AvoidancePathLatch, calculate_selected_path_pure_pursuit, send_control

GLOBAL_MAX = 57.0
AVOID_MAX = 25.0
DECEL_MPS2 = 2.0
BUFFER_M = 5.0
GPS_STALE_SEC = 1.0


def speed_cap(distance):
    if distance is None:
        return GLOBAL_MAX
    remaining = max(0.0, distance - BUFFER_M)
    return min(GLOBAL_MAX, 3.6 * math.sqrt((AVOID_MAX / 3.6)**2 + 2.0 * DECEL_MPS2 * remaining))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--drive", action="store_true", help="Send real MORAI control commands")
    args = parser.parse_args()
    global_path = np.loadtxt(SRC.parent / "path" / "2026_molit_comp_global_path.txt", dtype=float)
    if global_path.shape[1] == 2:
        global_path = np.column_stack((global_path, np.zeros(len(global_path))))
    global_path = global_path[:, :3]
    planner = StaticObstaclePlanner(global_path, str(HERE / "static_obstacle_config.json"))
    latch = AvoidancePathLatch(global_path)
    profile, _, _ = v7.build_v7_profile_capture(global_path, GLOBAL_MAX)
    cfg = load_network_config()
    gps = Receiver(cfg["gps"]["ip"], int(cfg["gps"]["port"]), GPS())
    imu = Receiver(cfg["imu"]["ip"], int(cfg["imu"]["port"]), IMU())
    speed = CompetitionSpeedReceiver()
    lidar = LidarReceiver()
    sender = Sender("127.0.0.1", 9093) if args.drive else None
    detector = ObjectDetector(ground_z_threshold=-1.70, dbscan_eps=0.5,
                              dbscan_min_samples=5, min_cluster_points=20,
                              min_width=0.15, max_width=3.0,
                              min_length=0.05, max_length=5.0)
    tracker = ObjectTracker(max_match_distance=1.0, velocity_window_size=5,
                            dynamic_speed_threshold=0.5, dynamic_confirm_frames=3,
                            max_missed_frames=3)
    limiter = control.LongitudinalCommandLimiter(control.LONGITUDINAL_CMD_RATE_PER_SEC)
    prev_pose = None
    prev_scan = None
    last_scan = None
    last_gps = None
    last_gps_change = time.monotonic()
    last_control = time.time()
    lidar.start()
    print("[MODE]", "DRIVE" if args.drive else "DRY-RUN")
    print("[CONTROL] V7 global 57 / static avoidance 25 / full-brake safety")
    print("[WARNING] Experimental. Not validated at 57->25 km/h.")
    try:
        while True:
            pose = read_ego_pose(gps, imu)
            if pose is None:
                time.sleep(0.02)
                continue
            x, y, yaw = pose
            if last_gps is None or math.hypot(x-last_gps[0], y-last_gps[1]) > 0.02:
                last_gps = (x, y)
                last_gps_change = time.monotonic()
            actual, valid = speed.read()
            if not valid:
                if args.drive:
                    send_control(sender, 0.0, 1.0, 0.0)
                time.sleep(0.05)
                continue
            latest = lidar.get_latest()
            if latest is None:
                if args.drive:
                    send_control(sender, 0.0, 1.0, 0.0)
                time.sleep(0.01)
                continue
            points, stats = latest
            stamp = stats.get("timestamp")
            if points is None or not stats.get("fresh", False) or stamp is None:
                if args.drive:
                    send_control(sender, 0.0, 1.0, 0.0)
                time.sleep(0.01)
                continue
            if stamp == last_scan:
                time.sleep(0.005)
                continue
            last_scan = stamp
            dt = 0.1 if prev_scan is None else stamp - prev_scan
            if dt <= 0.0 or dt > 1.0:
                dt = 0.1
            prev_scan = stamp
            if prev_pose is None:
                dx = dy = dyaw = 0.0
            else:
                _, _, dx, dy, dyaw = calculate_ego_motion(*prev_pose, x, y, yaw)
            prev_pose = (x, y, yaw)
            detected, _, _ = detector.detect(convert_lidar_to_vehicle(points))
            tracked = tracker.update(detected, dt, delta_x=dx, delta_y=dy, delta_yaw=dyaw)
            obstacles = tracked_list_to_planner_obstacles(
                tracked, ego_x=x, ego_y=y, ego_yaw=yaw, confidence=1.0)
            raw = planner.plan(x, y, obstacles)
            plan, latch_state = latch.apply(raw, x, y)
            gi = control.find_nearest_waypoint(global_path, x, y)
            horizon = set(int(i) for i in planner._forward_indices(
                gi, planner.cfg["detect_ahead_m"]))
            distances = []
            for obs in obstacles:
                if obs.is_dynamic or obs.confidence < 0.5:
                    continue
                if obs.width < planner.cfg.get("min_static_obstacle_width_m", 0.0):
                    continue
                if obs.height < planner.cfg.get("min_static_obstacle_height_m", 0.0):
                    continue
                oi = control.find_nearest_waypoint(global_path, obs.x, obs.y)
                if oi not in horizon:
                    continue
                if np.linalg.norm(global_path[oi,:2]-[obs.x,obs.y]) > (
                    planner.cfg["drivable_width_m"]/2 + obs.width/2):
                    continue
                forward = math.cos(yaw)*(obs.x-x) + math.sin(yaw)*(obs.y-y)
                if forward > 0:
                    distances.append(forward)
            distance = min(distances) if distances else None
            cap = speed_cap(distance)
            gps_stale = actual > 3.0 and time.monotonic()-last_gps_change > GPS_STALE_SEC
            too_fast = actual > AVOID_MAX + 1.0
            near = distance is not None and distance <= planner.cfg["avoid_trigger_distance_m"]
            now = time.time()
            dt_control = max(0.001, now-last_control)
            last_control = now
            stop = (gps_stale or plan.status == "STOP_REQUIRED"
                    or (near and too_fast))
            if stop:
                mode = "GPS_STALE" if gps_stale else "EMERGENCY_BRAKE"
                target, steer, accel, brake = 0.0, 0.0, 0.0, 1.0
                limiter.reset()
            elif plan.status == "AVOIDING":
                mode = "LOCAL_AVOID"
                # Reuse the validated #4 local-path steering only below 25 km/h.
                pp, _ = calculate_selected_path_pure_pursuit(plan.local_path, x, y, yaw)
                target = min(AVOID_MAX, cap)
                if pp is None:
                    mode, target, steer, accel, brake = "PATH_LOST", 0.0, 0.0, 0.0, 1.0
                    limiter.reset()
                else:
                    steer = pp["steering"]
                    a, b, _, _ = control.longitudinal_policy(target, actual)
                    accel, brake, _ = limiter.update(a, b, dt_control)
            else:
                mode = "V7_GLOBAL"
                v7.dynamic_test._STATE["control_speed_kmh"] = actual
                pp = v7.calculate_pure_pursuit_v5(global_path, gi, x, y, yaw)
                target = min(float(profile[gi]), cap)
                if pp is None:
                    mode, target, steer, accel, brake = "PATH_LOST", 0.0, 0.0, 0.0, 1.0
                    limiter.reset()
                else:
                    steer = pp["steering"]
                    a, b, _, _ = control.longitudinal_policy(target, actual)
                    accel, brake, _ = limiter.update(a, b, dt_control)
            if args.drive:
                send_control(sender, accel, brake, steer)
            print(f"MODE={mode:<17} GPS_STALE={gps_stale} "
                  f"DIST={distance!s:<8} CAP={cap:5.1f} "
                  f"TARGET={target:5.1f} ACTUAL={actual:5.1f} "
                  f"RAW={raw.status} PLAN={plan.status} LATCH={latch_state} "
                  f"STEER={steer:+.3f} BRAKE={brake:.2f}")
    except KeyboardInterrupt:
        print("\n[STOP] Ctrl+C")
    finally:
        if args.drive:
            for _ in range(10):
                send_control(sender, 0.0, 1.0, 0.0)
                time.sleep(0.05)
        lidar.stop()


if __name__ == "__main__":
    main()
