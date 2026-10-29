#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Non-driving LiDAR static obstacle perception + local path planner.

Refactored from lidar_object_detection/src/lidar/lidar_static_avoidance_drive_5kph.py.
Receives MORAI LiDAR only and a localhost pose feed from main_integrated_v1.py.
Outputs CLEAR / AVOIDING(local global-XY path) / STOP_REQUIRED to UDP localhost.
NEVER creates or transmits EgoCtrlCmd; GPS/IMU are owned by main only.
"""
import argparse
import csv
import os
import json
import math
import socket
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
SRC_DIR = HERE.parent
ROOT = SRC_DIR.parent
for p in (HERE, SRC_DIR):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

from lidar_receiver import LidarReceiver
from object_detection.object_detector import ObjectDetector
from object_detection.tracker import ObjectTracker
from obstacle_adapter import tracked_list_to_planner_obstacles
from static_obstacle_planner import StaticObstaclePlanner, PlanResult, nearest_index
from integration.static_zone import load_static_zones, in_static_zone
from integration.static_avoidance_io import configured_avoidance_speed_kmh
from static_avoidance_path_latch import AvoidancePathLatch
from shadow_wall_guard import pick_shadow_detour, check_wall_clearance, close_forward_collision

NETWORK_FILE = ROOT / "config" / "integration_network.json"
GLOBAL_PATH_FILE = ROOT / "path" / "2026_molit_comp_global_path.txt"
PLANNER_FILE = HERE / "static_obstacle_config.json"
LOG_DIR = ROOT / "logs"
POSE_TIMEOUT_SEC = 1.0


def lidar_to_vehicle(points):
    """Source coordinate transform: (LiDAR right, forward, up) -> (forward, left, up)."""
    if points is None or len(points) == 0:
        return np.empty((0, 3), dtype=np.float32)
    return np.column_stack((points[:, 1], -points[:, 0], points[:, 2])).astype(np.float32)


def delta_ego(previous, current):
    if previous is None:
        return 0.0, 0.0, 0.0
    px, py, pyaw = previous
    x, y, yaw = current
    dx, dy = x - px, y - py
    delta_forward = math.cos(pyaw) * dx + math.sin(pyaw) * dy
    delta_left = -math.sin(pyaw) * dx + math.cos(pyaw) * dy
    delta_yaw = math.atan2(math.sin(yaw - pyaw), math.cos(yaw - pyaw))
    return delta_forward, delta_left, delta_yaw


def parse_pose(raw):
    try:
        p = json.loads(raw.decode("ascii"))
        x, y, yaw = float(p["x"]), float(p["y"]), float(p["yaw"])
        if not all(math.isfinite(v) for v in (x, y, yaw)):
            return None
        return x, y, yaw
    except (ValueError, TypeError, KeyError, UnicodeError, AttributeError):
        return None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--lidar-bind", default=None, help="override LiDAR UDP bind address")
    parser.add_argument("--lidar-port", type=int, default=None, help="override LiDAR UDP port")
    args = parser.parse_args()

    with NETWORK_FILE.open("r", encoding="utf-8") as f:
        network = json.load(f)
    with (ROOT / "config" / "integration_missions.json").open("r", encoding="utf-8") as f:
        missions = json.load(f)
    lidar_cfg = network.get("lidar", {"bind_ip": "0.0.0.0", "port": 1116})
    pose_cfg = network.get("static_pose", {"bind_ip": "127.0.0.1", "port": 9302})
    result_cfg = network.get("static_result", {"bind_ip": "127.0.0.1", "port": 9303})

    lidar_ip = args.lidar_bind or lidar_cfg["bind_ip"]
    lidar_port = args.lidar_port or int(lidar_cfg["port"])
    path = np.loadtxt(GLOBAL_PATH_FILE, dtype=float)
    if path.ndim != 2 or path.shape[1] < 2:
        raise RuntimeError("Invalid competition global path")
    if path.shape[1] == 2:
        path = np.column_stack((path, np.zeros(len(path))))
    path = path[:, :3]
    static_cfg = missions.get("static", {})
    wall_cfg = missions.get("gps_shadow_wall_guard", {})
    static_zones = load_static_zones(static_cfg, len(path))
    avoidance_speed_kmh = configured_avoidance_speed_kmh(static_cfg)

    planner = StaticObstaclePlanner(
        path, str(PLANNER_FILE), avoidance_speed_kph=avoidance_speed_kmh
    )
    latch = AvoidancePathLatch(
        path,
        vehicle_width_m=planner.cfg["vehicle_width_m"],
        safety_margin_m=planner.cfg["safety_margin_m"],
        min_obstacle_width_m=planner.cfg.get("min_static_obstacle_width_m", 0.0),
        min_obstacle_height_m=planner.cfg.get("min_static_obstacle_height_m", 0.0),
        avoidance_speed_kmh=avoidance_speed_kmh,
    )
    detector = ObjectDetector(
        ground_z_threshold=-1.70,
        dbscan_eps=0.5, dbscan_min_samples=5,
        min_cluster_points=20,
        min_width=0.15, max_width=3.00,
        min_length=0.05, max_length=5.00,
    )
    tracker = ObjectTracker(
        max_match_distance=1.0, velocity_window_size=5,
        dynamic_speed_threshold=0.5, dynamic_confirm_frames=3,
        max_missed_frames=3,
    )

    pose_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    pose_sock.bind((pose_cfg["bind_ip"], int(pose_cfg["port"])))
    pose_sock.setblocking(False)
    result_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    result_target = (result_cfg["bind_ip"], int(result_cfg["port"]))
    receiver = LidarReceiver(ip=lidar_ip, port=lidar_port)

    # Main integrated CSV already contains static state; don't duplicate it.
    # Opt in to per-scan CSV only when debugging LiDAR.
    csv_enabled = os.environ.get("STATIC_CSV_LOG", "0") == "1"
    logfile = None
    writer = None
    log_path = None
    if csv_enabled:
        LOG_DIR.mkdir(parents=True, exist_ok=True)
        log_path = LOG_DIR / ("static_worker_" + time.strftime("%Y%m%d_%H%M%S") + ".csv")
        logfile = log_path.open("w", newline="", encoding="utf-8")
        writer = csv.writer(logfile)
        writer.writerow([
            "scan", "ego_x", "ego_y", "raw_status", "status", "latch_state",
            "objects", "tracked", "obstacle_id", "avoid_side", "path_points",
            "recommended_kmh", "waypoint_index", "in_static_zone",
            "latch_active", "static_zone_active",
        ])

    pose = None
    pose_time = 0.0
    gps_shadow = False
    previous_pose = None
    previous_scan_timestamp = None
    last_scan_timestamp = None
    scan_count = 0
    previous_status = None
    planning_was_enabled = False
    shadow_verified = False

    print("[STATIC ZONES] " + str(static_zones))
    print(f"[STATIC AVOID SPEED] {avoidance_speed_kmh:.1f} km/h")
    print("[STATIC WORKER] LiDAR -> detect -> track -> local path -> UDP result")
    print("[CONTROL] No EgoCtrlCmd sender in this process.")
    print(f"[LIDAR] {lidar_ip}:{lidar_port}")
    print(f"[POSE] {pose_cfg['bind_ip']}:{pose_cfg['port']} from main")
    print(f"[RESULT] {result_target} to main")
    if csv_enabled:
        print(f"[LOG] {log_path}")
    else:
        print("[LOG] per-scan LiDAR CSV OFF; integrated main CSV stays ON")

    try:
        receiver.start()
        while True:
            # Main sends current position at its own controller rate. Keep only
            # latest; never bind GPS or IMU ports a second time.
            while True:
                try:
                    raw, _ = pose_sock.recvfrom(512)
                except BlockingIOError:
                    break
                decoded = parse_pose(raw)
                if decoded is not None:
                    pose = decoded
                    pose_time = time.monotonic()
                    # Shadow pose is bounded dead-reckoning from main.
                    # Keep original obstacle detection/tracking/local planner.
                    try:
                        gps_shadow = (json.loads(raw.decode("ascii"))
                                      .get("gps_shadow") is True)
                    except (ValueError, UnicodeError, AttributeError):
                        gps_shadow = False

            if pose is None or time.monotonic() - pose_time > POSE_TIMEOUT_SEC:
                time.sleep(0.02)
                continue
            points, stats = receiver.get_latest(copy=True)
            if points is None or not stats or not stats.get("fresh", False):
                time.sleep(0.02)
                continue
            stamp = stats.get("timestamp")
            if stamp is None or stamp == last_scan_timestamp:
                time.sleep(0.01)
                continue
            last_scan_timestamp = stamp
            scan_count += 1
            if previous_scan_timestamp is None:
                dt = 0.1
            else:
                dt = stamp - previous_scan_timestamp
                if dt <= 0 or dt > 1:
                    dt = 0.1
            previous_scan_timestamp = stamp

            ego_x, ego_y, ego_yaw = pose
            waypoint_index = nearest_index(path, ego_x, ego_y)
            inside_zone = (in_static_zone(waypoint_index, static_zones)
                           or gps_shadow)
            # During a GPS blackout, any road obstacle may need the original
            # static planner. The main still imposes a strict shadow speed cap
            # and a short dead-reckoning expiry fail-stop.
            # Keep a route accepted inside the zone until geometric release.
            planner_enabled = bool(inside_zone or latch.active)
            shadow_verified = False
            if not planner_enabled:
                # Suppress detection, tracking and path generation elsewhere.
                obstacles, tracked = [], []
                raw_plan = PlanResult(
                    "CLEAR", np.empty((0, 3)), None, None, 0.0,
                    "Outside static mission zone",
                )
                plan, latch_state = raw_plan, "ZONE_OFF"
                previous_pose = None
            else:
                if not planning_was_enabled:
                    tracker = ObjectTracker(
                        max_match_distance=1.0, velocity_window_size=5,
                        dynamic_speed_threshold=0.5, dynamic_confirm_frames=3,
                        max_missed_frames=3,
                    )
                    previous_pose = None
                forward, left, delta_yaw = delta_ego(previous_pose, pose)
                previous_pose = pose
                vehicle_points = lidar_to_vehicle(points)
                obstacles, _, _ = detector.detect(vehicle_points)
                tracked = tracker.update(
                    obstacles, dt, delta_x=forward, delta_y=left, delta_yaw=delta_yaw
                )
                planner_obstacles = tracked_list_to_planner_obstacles(
                    tracked, ego_x=ego_x, ego_y=ego_y, ego_yaw=ego_yaw, confidence=1.0
                )
                raw_plan = planner.plan(
                    ego_x=ego_x, ego_y=ego_y, obstacles=planner_obstacles
                )
                if gps_shadow and not latch.active and raw_plan.status == "AVOIDING":
                    # Generate the SAME original planner candidates, but
                    # validate both sides against the full current LiDAR scan.
                    raw_plan, shadow_verified, wall_reason = pick_shadow_detour(
                        planner, (ego_x, ego_y, ego_yaw),
                        planner_obstacles, vehicle_points, wall_cfg
                    )
                plan, latch_state = latch.apply(
                    raw_plan, ego_x, ego_y, obstacles=planner_obstacles
                )
                if gps_shadow and close_forward_collision(vehicle_points, wall_cfg):
                    # Independent near-field wall/object guard. This also
                    # protects against a wall missed by width-limited DBSCAN.
                    plan = PlanResult(
                        "STOP_REQUIRED", np.empty((0,3)), None, None, 0.0,
                        "SHADOW_FORWARD_LIDAR_OBSTACLE"
                    )
                    latch_state = "SHADOW_FORWARD_LIDAR_OBSTACLE"
                # Preserve dynamic obstacle STOP in the GPS shadow using
                # the existing LiDAR tracker's dynamic classification.
                # No unverified passing maneuver around moving traffic.
                if gps_shadow:
                    for track in tracked:
                        if (bool(getattr(track, "is_dynamic", False))
                                and 0.0 < float(track.x) < float(
                                    wall_cfg.get("dynamic_stop_ahead_m", 12.0))
                                and abs(float(track.y)) < float(
                                    wall_cfg.get("dynamic_stop_half_width_m", 1.8))):
                            plan = PlanResult(
                                "STOP_REQUIRED", np.empty((0,3)),
                                int(getattr(track, "track_id", -1)),
                                None, 0.0, "SHADOW_DYNAMIC_TRACK_STOP"
                            )
                            latch_state = "SHADOW_DYNAMIC_TRACK_STOP"
                            break
                if gps_shadow and plan.status == "AVOIDING":
                    # Re-check an ALREADY LATCHED detour with every new scan.
                    shadow_verified, wall_reason = check_wall_clearance(
                        plan.local_path, (ego_x, ego_y, ego_yaw),
                        vehicle_points, wall_cfg
                    )
                    if not shadow_verified:
                        plan = PlanResult(
                            "STOP_REQUIRED", np.empty((0,3)), plan.obstacle_id,
                            plan.avoidance_side, 0.0,
                            "SHADOW_WALL_UNVERIFIED:"+wall_reason
                        )
                        latch_state = "SHADOW_WALL_UNVERIFIED:"+wall_reason
                # Never accept a brand-new avoidance path after exiting zone.
                if not inside_zone and not latch.active:
                    raw_plan = PlanResult(
                        "CLEAR", np.empty((0, 3)), None, None, 0.0,
                        "Outside static mission zone",
                    )
                    plan, latch_state = raw_plan, "ZONE_OFF"
            planning_was_enabled = planner_enabled
            zone_active = bool(inside_zone or latch.active)
            status = plan.status
            selected_path = []
            if status == "AVOIDING":
                selected_path = np.asarray(plan.local_path, dtype=float)[:, :2].round(3).tolist()
                if not 2 <= len(selected_path) <= 400:
                    status = "STOP_REQUIRED"
                    selected_path = []
                    latch_state = "INVALID_LOCAL_PATH"

            packet = {
                "status": status,
                "path": selected_path,
                "obstacle_id": plan.obstacle_id,
                "avoidance_side": plan.avoidance_side,
                "recommended_speed_kmh": max(0.0, float(plan.recommended_speed_kph)),
                "latch_state": latch_state,
                "scan": scan_count,
                "detected": len(obstacles),
                "waypoint_index": waypoint_index,
                "in_static_zone": bool(inside_zone),
                "latch_active": bool(latch.active),
                "zone_active": zone_active,
                "shadow_wall_verified": bool(gps_shadow and shadow_verified and status == "AVOIDING"),
            }
            data = json.dumps(packet, separators=(",", ":"), allow_nan=False).encode("utf-8")
            if len(data) > 60000:
                packet.update(status="STOP_REQUIRED", path=[], latch_state="UDP_TOO_LARGE")
                data = json.dumps(packet, separators=(",", ":")).encode("utf-8")
            result_sock.sendto(data, result_target)
            if csv_enabled:
                writer.writerow([
                    scan_count, f"{ego_x:.3f}", f"{ego_y:.3f}",
                    raw_plan.status, status, latch_state,
                    len(obstacles), len(tracked),
                    plan.obstacle_id, plan.avoidance_side,
                    len(selected_path), plan.recommended_speed_kph,
                    waypoint_index, int(inside_zone), int(latch.active),
                    int(zone_active),
                ])
                logfile.flush()
            if (status, latch_state) != previous_status:
                print(
                    f"[STATIC] scan={scan_count} {raw_plan.status} -> {status} "
                    f"latch={latch_state} objects={len(obstacles)} "
                    f"track={len(tracked)} wp={waypoint_index} "
                    f"zone={inside_zone} side={plan.avoidance_side} "
                    f"path={len(selected_path)}"
                )
                previous_status = (status, latch_state)
    except KeyboardInterrupt:
        print("\n[STATIC] stopped by user.")
    finally:
        receiver.stop()
        pose_sock.close()
        result_sock.close()
        if logfile is not None:
            logfile.close()
            print(f"[LOG SAVED] {log_path}")


if __name__ == "__main__":
    main()
