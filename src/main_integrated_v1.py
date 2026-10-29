#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Stage-1 integration entrypoint.

Base controller: the exact V7 import chain already validated on MORAI.
Missions:
- traffic approaches at <=30 km/h; RED triggers full brake, confirmed GO releases it
- highway merge risk optionally caps speed; it never changes the global path or steering

Only this process sends EgoCtrlCmd.
"""

import argparse
import csv
import json
import math
import socket
import time
from datetime import datetime
from pathlib import Path

import pure_pursuit_profile_57_corner_v7_early_brake as v7
import pure_pursuit_competition as competition

from lib.network.UDP import Receiver, Sender
from lib.define.GPS import GPS
from lib.define.IMU import IMU
from lib.define.EgoCtrlCmd import EgoCtrlCmd

from integration.mission_io import (
    LatestUdpTextReceiver,
    parse_merge_message,
    parse_traffic_message,
)
from integration.mission_manager import MissionManager
from integration.highway_follow_io import (
    in_follow_zone, highway_speed_zones, grade_speed_cap, parse_follow_message,
)
from integration.highway_speed_profile import build_highway_profile
from integration.static_avoidance_io import (
    parse_static_message, local_path_pure_pursuit,
    configured_avoidance_speed_kmh,
)
from integration.static_zone import load_static_zones, in_static_zone
from integration.dynamic_pedestrian_io import (
    load_dynamic_zones, in_dynamic_zone,
    parse_pedestrian_message, PedestrianStopLatch,
)
from integration.gps_shadow_io import GpsShadowPose
from integration.gps_shadow_lane import parse_lane_result


PROJECT_ROOT = Path(__file__).resolve().parents[1]
NETWORK_FILE = PROJECT_ROOT / "config" / "integration_network.json"
MISSION_FILE = PROJECT_ROOT / "config" / "integration_missions.json"
STOP_INDEX_FILE = PROJECT_ROOT / "path" / "traffic_stop_indices.txt"

base = v7.base
profile_test = v7.profile_test


def load_json(path):
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def load_stop_indices(path):
    values = []
    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            text = line.strip()
            if text:
                values.append(int(text))
    if not values:
        raise RuntimeError("traffic stop index file is empty")
    return values


def open_log():
    log_dir = PROJECT_ROOT / "logs"
    log_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    path = log_dir / f"integrated_v1_{stamp}.csv"
    f = open(path, "w", newline="", buffering=1)
    w = csv.writer(f)
    w.writerow([
        "elapsed_sec", "waypoint_index", "x_m", "y_m", "yaw_deg",
        "path_dist_m", "base_target_kmh", "final_target_kmh",
        "comp_speed_kmh", "steering_rad", "accel_cmd", "brake_cmd",
        "traffic_raw", "traffic_age_sec", "traffic_state",
        "red_latched", "go_release_count",
        "merge_state", "merge_risk", "mission_reasons",
        "traffic_stop_index", "traffic_stop_distance_m",
        "traffic_full_brake", "traffic_approach", "traffic_left_override",
        "merge_zone", "merge_stop", "merge_accept_count",
        "static_state", "static_age_sec", "static_obstacle_id",
        "static_avoid_side", "static_path_points", "static_active",
        "static_stop", "static_detected", "static_scan",
        "static_in_zone", "static_route_locked", "static_worker_wp",
        "static_latch_state", "traffic_green_bypass_620",
        "traffic_green_count_620",
        "pedestrian_zone", "pedestrian_state", "pedestrian_risk",
        "pedestrian_count", "pedestrian_stop", "pedestrian_age_sec",
        "pedestrian_frame", "pedestrian_clear_count",
        "follow_zone", "follow_state", "follow_risk", "follow_grade",
        "follow_speed_cap_kmh", "follow_age_sec", "follow_vehicles",
        "gps_mode", "gps_shadow_sec", "lane_valid", "lane_confidence",
        "lane_steering_rad", "shadow_stop",
    ])
    return path, f, w


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--drive", action="store_true")
    parser.add_argument("--gps-shadow", action="store_true",
                        help="on GPS loss use lane-center steering + existing LiDAR/person safety")
    parser.add_argument(
        "--traffic",
        action="store_true",
        help="apply traffic-light mission",
    )
    parser.add_argument(
        "--merge",
        action="store_true",
        help="receive/log merge-risk result",
    )
    parser.add_argument(
        "--merge-control",
        action="store_true",
        help="apply configured merge-risk speed caps; requires configured zones or --merge-all",
    )
    parser.add_argument(
        "--merge-all",
        action="store_true",
        help="test only: treat the whole path as merge zone",
    )
    parser.add_argument("--static", action="store_true",
                        help="observe/log independent LiDAR static plans")
    parser.add_argument("--static-control", action="store_true",
                        help="static avoidance in configured WP zones; fail-stop while active")
    parser.add_argument("--pedestrian", action="store_true",
                        help="observe camera pedestrian risk only in configured dynamic WP zone")
    parser.add_argument("--pedestrian-control", action="store_true",
                        help="stop on pedestrian risk or stale results only in dynamic WP zone")
    parser.add_argument("--follow", action="store_true",
                        help="observe highway in-lane front camera vehicle risk")
    parser.add_argument("--follow-control", action="store_true",
                        help="apply graded highway-follow target speed caps")
    parser.add_argument("--max-speed", type=float, default=57.0,
                        help="normal <=57; explicitly request up to 120 on verified highway WPs")
    args = parser.parse_args()

    if args.gps_shadow and not args.traffic:
        parser.error("--gps-shadow requires --traffic for shared 9291 camera relay")
    if args.gps_shadow and not (args.static_control and args.pedestrian_control):
        parser.error("--gps-shadow requires --static --static-control --pedestrian --pedestrian-control")
    if args.merge_control and not args.merge:
        parser.error("--merge-control requires --merge")
    if args.static_control and not args.static:
        parser.error("--static-control requires --static")
    if args.pedestrian_control and not args.pedestrian:
        parser.error("--pedestrian-control requires --pedestrian")
    if args.pedestrian and not args.traffic:
        parser.error("--pedestrian requires --traffic and the 9291 camera relay")
    if args.follow_control and not args.follow:
        parser.error("--follow-control requires --follow")
    if args.follow and not args.traffic:
        parser.error("--follow requires --traffic for camera relay 9291->9305")
    if not 0 < args.max_speed <= 120.0:
        parser.error("--max-speed must be >0 and <=120")
    if args.max_speed > 57.0 and not args.follow_control:
        parser.error("speed >57 requires active --follow --follow-control; not V7 validated")

    network = load_json(NETWORK_FILE)
    missions = load_json(MISSION_FILE)
    shadow_cfg = missions.get("gps_shadow", {})
    stop_indices = load_stop_indices(STOP_INDEX_FILE)

    # Importing V7 already applies the validated V7 monkey-patches.
    global_path = base.load_path(base.PATH_FILE)
    # Preserve the V7 <=57 baseline; >57 only in explicitly verified WPs
    # and ONLY when following perception/control is enabled.
    target_profile, radius_profile, full_profile = (
        profile_test.build_test_profile(global_path, min(args.max_speed, 57.0))
    )
    follow_cfg = missions.get("highway_follow", {})
    fast_zones = highway_speed_zones(follow_cfg, len(global_path))
    if args.max_speed > 57.0:
        target_profile = build_highway_profile(
            target_profile, radius_profile, global_path, fast_zones,
            args.max_speed,
            a_lat_mps2=float(follow_cfg.get("assumed_max_lateral_accel_mps2", 2.0)),
            preview_decel_mps2=float(follow_cfg.get("assumed_preview_decel_mps2", 2.0)),
            accel_mps2=float(follow_cfg.get("assumed_profile_accel_mps2", 3.0)),
        )

    gps_receiver = Receiver(
        network["gps"]["bind_ip"],
        int(network["gps"]["port"]),
        GPS(),
    )
    imu_receiver = Receiver(
        network["imu"]["bind_ip"],
        int(network["imu"]["port"]),
        IMU(),
    )

    competition.COMP_STATUS_IP = network["competition_status"]["bind_ip"]
    competition.COMP_STATUS_PORT = int(
        network["competition_status"]["port"]
    )
    speed_receiver = profile_test.CompetitionSpeedReceiver()

    ctrl_sender = None
    if args.drive:
        ctrl_sender = Sender(
            network["control"]["target_ip"],
            int(network["control"]["port"]),
        )

    traffic_rx = None
    if args.traffic:
        traffic_rx = LatestUdpTextReceiver(
            network["traffic_result"]["bind_ip"],
            int(network["traffic_result"]["port"]),
        )

    merge_rx = None
    merge_gate_tx = None
    merge_gate_target = None
    if args.merge:
        merge_rx = LatestUdpTextReceiver(
            network["merge_result"]["bind_ip"],
            int(network["merge_result"]["port"]),
        )
        merge_gate_cfg = network["merge_gate"]
        if merge_gate_cfg["bind_ip"] != "127.0.0.1":
            raise RuntimeError("Merge waypoint gate must use localhost")
        merge_gate_target = ("127.0.0.1", int(merge_gate_cfg["port"]))
        merge_gate_tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # Only this main loop transmits EgoCtrlCmd. The LiDAR worker receives a
    # localhost pose feed and returns a plan/status; it never owns GPS/IMU.
    static_rx = None
    pose_tx = None
    pose_target = None
    if args.static:
        static_rx = LatestUdpTextReceiver(
            network["static_result"]["bind_ip"],
            int(network["static_result"]["port"]), max_size=64000,
        )
        pose_tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        pose_target = (
            network["static_pose"]["bind_ip"],
            int(network["static_pose"]["port"]),
        )

    # Camera 9291 remains owned by traffic detector; pedestrian/follow
    # share its UDP relay and the SAME YOLO/MiDaS process (no duplicate bind).
    follow_rx = None
    follow_msg = parse_follow_message(None)
    follow_updated_at = 0.0
    follow_was_in_zone = False
    follow_zone_entered_at = 0.0
    follow_timeout_sec = float(follow_cfg.get("risk_result_timeout_sec", 1.5))
    if not 0.2 <= follow_timeout_sec <= 5.0:
        raise ValueError("Invalid highway_follow.risk_result_timeout_sec")
    if args.follow:
        follow_net = network["highway_follow_result"]
        if follow_net["bind_ip"] != "127.0.0.1":
            raise RuntimeError("Follow result must remain on localhost")
        follow_rx = LatestUdpTextReceiver(
            "127.0.0.1", int(follow_net["port"]), max_size=2048,
        )
    pedestrian_rx = None
    pedestrian_gate_tx = None
    pedestrian_gate_target = None
    pedestrian_cfg = missions.get("dynamic_pedestrian", {})
    pedestrian_zones = load_dynamic_zones(pedestrian_cfg, len(global_path))
    pedestrian_timeout_sec = float(pedestrian_cfg.get("result_timeout_sec", 3.5))
    if not 0 < pedestrian_timeout_sec <= 30:
        raise ValueError("Invalid dynamic_pedestrian.result_timeout_sec")
    pedestrian_latch = PedestrianStopLatch(
        int(pedestrian_cfg.get("clear_confirm_frames", 3))
    )
    if args.pedestrian:
        pedestrian_rx = LatestUdpTextReceiver(
            network["pedestrian_result"]["bind_ip"],
            int(network["pedestrian_result"]["port"]), max_size=2048,
        )
    if args.pedestrian or args.follow:
        gate_cfg = network["pedestrian_gate"]
        if gate_cfg["bind_ip"] != "127.0.0.1":
            raise RuntimeError("Pedestrian gate must be localhost")
        pedestrian_gate_target = ("127.0.0.1", int(gate_cfg["port"]))
        pedestrian_gate_tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    lane_rx = None
    lane_result = parse_lane_result(None)
    lane_updated_at = 0.0
    gps_shadow_pose = GpsShadowPose(shadow_cfg)
    if args.gps_shadow:
        lane_net = network["gps_shadow_lane_result"]
        if lane_net["bind_ip"] != "127.0.0.1":
            raise RuntimeError("GPS shadow lane result must be localhost")
        lane_rx = LatestUdpTextReceiver(
            "127.0.0.1", int(lane_net["port"]), max_size=2048,
        )

    mission_manager = MissionManager(
        missions.get("traffic", {}),
        missions.get("merge", {}),
        stop_indices,
    )
    mission_manager.set_global_path(global_path)

    command_limiter = base.LongitudinalCommandLimiter(
        base.LONGITUDINAL_CMD_RATE_PER_SEC
    )

    log_path, log_file, log_writer = open_log()
    run_start = time.time()
    period = 1.0 / base.CONTROL_HZ
    prev_control_time = time.time()

    traffic = parse_traffic_message(None)
    traffic_updated_at = 0.0
    merge = parse_merge_message(None)
    merge_updated_at = 0.0
    static_plan = parse_static_message(None)
    static_updated_at = 0.0
    static_cfg = missions.get("static", {})
    static_zones = load_static_zones(static_cfg, len(global_path))
    static_route_engaged = False  # Hold fail-stop beyond zone until latch releases.
    static_timeout_sec = float(static_cfg.get("result_timeout_sec", 3.5))
    # Same speed request as the LiDAR worker/planner/path latch.
    # CLEAR continues using the existing V7 speed profile unchanged.
    static_speed_limit = configured_avoidance_speed_kmh(static_cfg)
    pedestrian_result = parse_pedestrian_message(None)
    pedestrian_updated_at = 0.0
    pedestrian_was_in_zone = False
    pedestrian_zone_entered_at = 0.0
    merge_was_in_zone = False
    merge_zone_entered_at = 0.0

    print("====================================================")
    print(" V7 + TRAFFIC + MERGE INTEGRATION V1")
    print("====================================================")
    print("[BASE] validated V7 import chain")
    print("[CTRL] single EgoCtrlCmd sender / ctrl_mode=2 / cmd_type=1")
    print(f"[MODE] {'DRIVE' if args.drive else 'DRY-RUN'}")
    if args.gps_shadow:
        print("[GPS SHADOW] ON: lane steering replaces global PP on stale/missing GPS")
        print("[GPS SHADOW] existing pedestrian STOP and LiDAR local path retain priority")
        print("[GPS SHADOW] never launch a second UDP9291 subscriber")
    print(f"[TRAFFIC] {'ON' if args.traffic else 'OFF'} / stops={stop_indices}")
    if args.traffic:
        print("[TRAFFIC] 60m approach: <=30km/h (regardless of colour)")
        print("[TRAFFIC BRAKE] RED only: accel=0 / brake=1")
        print("[TRAFFIC GO] 8 fresh consecutive GO packets required to release")
    print(
        f"[MERGE] {'ON' if args.merge else 'OFF'} / "
        f"control={'ON' if args.merge_control else 'OBSERVE'} / "
        f"test_all={args.merge_all} / "
        f"zones={missions.get('merge', {}).get('zones', [])} / "
        f"roundabout_zones={missions.get('merge', {}).get('roundabout_zones', [])}"
    )
    print(
        f"[MORAI CTRL] {network['control']['target_ip']}:"
        f"{network['control']['port']}"
    )
    print(
        f"[STATIC] {'ON' if args.static else 'OFF'} / "
        f"control={'ON' if args.static_control else 'OBSERVE'} / "
        f"avoid_cap={static_speed_limit:.1f}km/h / CLEAR=V7 speed profile / timeout={static_timeout_sec:.1f}s"
    )
    print(
        f"[PEDESTRIAN] {'ON' if args.pedestrian else 'OFF'} "
        f"CTRL={'ON' if args.pedestrian_control else 'OBSERVE'} "
        f"zones={pedestrian_zones} timeout={pedestrian_timeout_sec:.1f}s"
    )
    if args.static:
        print(f"[STATIC ZONES WP] {static_zones}; outside=V7 only")
        print(f"[STATIC POSE] {pose_target}")
        print(f"[STATIC RESULT] {network['static_result']}")
    print(f"[HIGHWAY FOLLOW] {'ON' if args.follow else 'OFF'} "
          f"control={'ON' if args.follow_control else 'OBSERVE'} "
          f"zones={follow_cfg.get('zones_wp')} fast_zones={fast_zones} "
          f"speed_request={args.max_speed:.1f}")
    if args.max_speed > 57.0:
        print("[HIGHWAY WARNING] 120 km/h profile is EXPERIMENTAL; V7 steering not validated at this speed.")
        if follow_cfg.get("tollgate_exit_wp") is None:
            print("[HIGHWAY WARNING] tollgate WP unknown: SECOND section stays <=57 km/h.")
    print(f"[LOG] {log_path}")

    try:
        while True:
            loop_start = time.time()
            now = loop_start

            if traffic_rx is not None:
                raw, updated_at, _ = traffic_rx.poll()
                if raw is not None:
                    traffic = parse_traffic_message(raw)
                    traffic_updated_at = updated_at
                    mission_manager.update_traffic_input(
                        traffic,
                        traffic_updated_at,
                    )

            if merge_rx is not None:
                raw, updated_at, _ = merge_rx.poll()
                if raw is not None:
                    merge = parse_merge_message(raw)
                    merge_updated_at = updated_at

            if follow_rx is not None:
                raw, updated_at, is_new = follow_rx.poll()
                if raw is not None and is_new:
                    follow_msg = parse_follow_message(raw)
                    follow_updated_at = updated_at

            if static_rx is not None:
                raw, updated_at, is_new = static_rx.poll()
                if raw is not None and is_new:
                    static_plan = parse_static_message(raw)
                    static_updated_at = updated_at

            if lane_rx is not None:
                lane_raw, lane_ts, lane_new = lane_rx.poll()
                if lane_raw is not None and lane_new:
                    lane_result = parse_lane_result(lane_raw)
                    lane_updated_at = lane_ts

            pedestrian_new_result = False
            if pedestrian_rx is not None:
                ped_raw, ped_updated_at, pedestrian_new_result = pedestrian_rx.poll()
                if ped_raw is not None and pedestrian_new_result:
                    pedestrian_result = parse_pedestrian_message(ped_raw)
                    pedestrian_updated_at = ped_updated_at

            gps_data = gps_receiver.get_data()
            gps_data.parsing()
            lat = gps_data.gprmc.lat
            lon = gps_data.gprmc.lon
            position = base.gps_to_local(lat, lon)
            if position is None and not args.gps_shadow:
                print("[WAIT] GPS")
                time.sleep(0.1)
                continue

            imu_data = imu_receiver.get_data()
            if (
                imu_data.ori_w == 0.0
                and imu_data.ori_x == 0.0
                and imu_data.ori_y == 0.0
                and imu_data.ori_z == 0.0
            ):
                if args.drive and ctrl_sender is not None:
                    base.send_stop(ctrl_sender)
                print("[WAIT] IMU => FULL STOP")
                time.sleep(0.1)
                continue

            yaw = base.quaternion_to_yaw(
                imu_data.ori_x,
                imu_data.ori_y,
                imu_data.ori_z,
                imu_data.ori_w,
            )

            current_speed_kmh, speed_valid = speed_receiver.read()
            if not speed_valid:
                if args.drive and ctrl_sender is not None:
                    base.send_stop(ctrl_sender)
                print("[WAIT] Competition Vehicle Status")
                time.sleep(0.05)
                continue

            gps_mode = "GPS"
            shadow_sec = 0.0
            if args.gps_shadow:
                gps_mode, position, shadow_sec = gps_shadow_pose.update(
                    position, yaw, current_speed_kmh, time.time()
                )
            if position is None:
                # Never guess an absolute pose before the first valid GPS fix.
                if args.drive and ctrl_sender is not None:
                    base.send_stop(ctrl_sender)
                print("[GPS SHADOW] NO_FIX => FULL STOP (waiting for first GPS position)")
                time.sleep(0.05)
                continue
            current_x, current_y = position
            shadow_active = args.gps_shadow and gps_mode in ("SHADOW", "STOP")
            lane_age_sec = (max(0.0, time.time()-lane_updated_at)
                            if lane_updated_at > 0.0 else float("inf"))
            lane_fresh = bool(
                args.gps_shadow and lane_result["valid"]
                and lane_age_sec <= float(shadow_cfg.get("lane_result_timeout_sec", 0.8))
                and lane_result["confidence"] >= float(
                    shadow_cfg.get("min_confidence", 0.35))
            )
            shadow_stop = bool(
                shadow_active and (gps_mode == "STOP" or not lane_fresh)
            )
            if pose_tx is not None:
                pose_tx.sendto(
                    json.dumps({
                        "x":current_x, "y":current_y, "yaw":yaw,
                        "gps_shadow":bool(shadow_active),
                    }).encode("ascii"),
                    pose_target,
                )

            nearest_index = base.find_nearest_waypoint(
                global_path,
                current_x,
                current_y,
            )
            if merge_gate_tx is not None:
                merge_gate_tx.sendto(
                    json.dumps({
                        "waypoint_index": nearest_index,
                        "merge_all": bool(args.merge_all),
                        "roundabout_done": bool(mission_manager.roundabout_done),
                    }).encode("ascii"),
                    merge_gate_target,
                )
            merge_cfg = missions.get("merge", {})
            merge_ranges = merge_cfg.get("zones", []) + merge_cfg.get("roundabout_zones", [])
            merge_in_zone = bool(args.merge_all) or any(
                start <= nearest_index <= end for start, end in merge_ranges
            )
            if merge_in_zone and not merge_was_in_zone:
                # Permit only recent prewarmed camera frames (not stale
                # observations from the previous intersection/mission).
                merge_zone_entered_at = time.time() - min(
                    1.5, max(0.0, float(merge_cfg.get("prewarm_result_age_sec", 1.0)))
                )
            merge_was_in_zone = merge_in_zone
            pedestrian_in_zone = (
                in_dynamic_zone(nearest_index, pedestrian_zones) or shadow_active
            )
            if pedestrian_gate_tx is not None:
                pedestrian_gate_tx.sendto(
                    json.dumps({
                        "waypoint_index":nearest_index,
                        "gps_shadow":bool(shadow_active),
                    }).encode("ascii"),
                    pedestrian_gate_target,
                )
            if pedestrian_in_zone and not pedestrian_was_in_zone:
                pedestrian_zone_entered_at = time.time() - min(
                    1.5, max(0.0, float(pedestrian_cfg.get("prewarm_result_age_sec", 1.0)))
                )
                pedestrian_latch.reset()
                # A prewarmed STOP_REQUIRED may have been received just
                # BEFORE crossing WP720. Re-evaluate that fresh packet on
                # entry so resetting the latch cannot erase a real hazard.
                if (pedestrian_result["valid"]
                        and pedestrian_updated_at >= pedestrian_zone_entered_at):
                    pedestrian_new_result = True
            elif not pedestrian_in_zone and pedestrian_was_in_zone:
                pedestrian_latch.reset()
            pedestrian_was_in_zone = pedestrian_in_zone
            nearest_x = global_path[nearest_index, 0]
            nearest_y = global_path[nearest_index, 1]
            nearest_distance = math.hypot(
                nearest_x - current_x,
                nearest_y - current_y,
            )
            # The dead-reckoned global position is NOT a shadow navigation
            # reference: no global-path-offset stop in ego-LiDAR mode.
            # Camera lane + LiDAR freshness and local clearance replace it.

            # GPS mode uses the unchanged V7 Pure Pursuit.
            # Shadow mode uses camera lane-center as PRIMARY guidance.
            # The dead-reckoned pose is only for LiDAR path alignment and
            # rough mission gates, never for base steering.
            result = None if shadow_active else base.calculate_pure_pursuit(
                global_path, nearest_index, current_x, current_y, yaw,
            )
            if shadow_active:
                base_steering = (
                    float(lane_result["steering"]) if lane_fresh else 0.0
                )
                # A fresh LiDAR local avoidance path may temporarily
                # replace a lane view occluded by the obstacle itself.
                # Do not set zero here; the final safety arbiter still
                # commands FULL STOP unless lane or valid LiDAR is available.
                base_target = (
                    min(float(shadow_cfg.get("shadow_speed_kmh", 12.0)),
                        float(args.max_speed)) if gps_mode == "SHADOW" else 0.0
                )
            elif result is None:
                base_steering = 0.0
                base_target = 0.0
            else:
                base_steering = float(result["steering"])
                base_target = float(target_profile[nearest_index])

            mission = mission_manager.apply(
                nearest_index=nearest_index,
                base_target_speed_kmh=base_target,
                base_steering=base_steering,
                traffic_enabled=bool(args.traffic and not shadow_active),
                merge_enabled=bool(args.merge and not shadow_active),
                merge_control_enabled=bool(args.merge_control and not shadow_active),
                # Accept only recent prewarmed observations from this approach.
                merge_result=(merge if merge_updated_at >= merge_zone_entered_at else parse_merge_message(None)),
                merge_updated_at=(merge_updated_at if merge_updated_at >= merge_zone_entered_at else 0.0),
                merge_all=args.merge_all,
                now=now,
            )

            target_speed_kmh = mission["target_speed_kmh"]
            steering = mission["steering"]

            # GRADED highway following, no arbitrary full brake on an image
            # score. Other mission STOPs retain precedence. Stale data means
            # a low-speed cap, never permission to drive 120.
            follow_in_zone = (not shadow_active and in_follow_zone(
                nearest_index, follow_cfg, len(global_path)))
            if follow_in_zone and not follow_was_in_zone:
                follow_zone_entered_at = time.time() - min(
                    1.5, max(0.0, float(follow_cfg.get("prewarm_result_age_sec", 1.0)))
                )
            follow_was_in_zone = follow_in_zone
            follow_age = (max(0.0, time.time() - follow_updated_at)
                          if follow_updated_at > 0.0 else float("inf"))
            follow_fresh = bool(
                follow_msg["valid"]
                and follow_updated_at >= follow_zone_entered_at
                and follow_age <= follow_timeout_sec
                and follow_msg["waypoint_index"] is not None
                and abs(int(follow_msg["waypoint_index"]) - nearest_index) <= 60
            )
            follow_zone_active = bool(args.follow and follow_in_zone)
            follow_state = ("OFF" if not args.follow else
                            "ZONE_OFF" if not follow_in_zone else
                            "VISION_WAIT" if not follow_fresh else
                            f"GRADE_{follow_msg['grade']}")
            follow_grade = (follow_msg["grade"] if follow_fresh else 4)
            follow_risk = (follow_msg["risk"] if follow_fresh else 1.0)
            follow_cap = (grade_speed_cap(follow_risk)[1] if follow_fresh else
                          float(follow_cfg.get("missing_camera_cap_kmh", 15.0)))
            if args.follow_control and follow_in_zone:
                target_speed_kmh = min(target_speed_kmh, follow_cap)
                mission["reasons"].append("highway_follow_" + follow_state.lower())

            pedestrian_age = (
                max(0.0, time.time() - pedestrian_updated_at)
                if pedestrian_updated_at > 0 else float("inf")
            )
            pedestrian_fresh = bool(
                pedestrian_result["valid"]
                and pedestrian_updated_at >= pedestrian_zone_entered_at
                and pedestrian_age <= pedestrian_timeout_sec
                and pedestrian_result.get("waypoint_index") is not None
                and (
                    (shadow_active and
                     pedestrian_updated_at >= pedestrian_zone_entered_at)
                    or in_dynamic_zone(
                        pedestrian_result["waypoint_index"], pedestrian_zones
                    ) or any(
                        start - max(0, int(pedestrian_cfg.get("prewarm_wp", 0)))
                        <= pedestrian_result["waypoint_index"] < start
                        for start, end in pedestrian_zones
                    )
                )
                and abs(pedestrian_result["waypoint_index"] - nearest_index) <= 30
            )
            if args.pedestrian and pedestrian_in_zone:
                pedestrian_latch.update(
                    pedestrian_result, pedestrian_new_result, pedestrian_fresh,
                )
            pedestrian_state = (
                "OFF" if not args.pedestrian else
                "ZONE_OFF" if not pedestrian_in_zone else
                "VISION_WAIT" if not pedestrian_fresh else
                "STOP_REQUIRED" if pedestrian_latch.latched else
                pedestrian_result["status"]
            )
            pedestrian_stop = bool(
                args.pedestrian_control and pedestrian_in_zone
                and (not pedestrian_fresh or pedestrian_latch.latched)
            )
            if pedestrian_stop:
                target_speed_kmh = 0.0
                mission["reasons"].append("pedestrian_stop_or_stale")

            # UDP poll() can timestamp a new packet a few milliseconds
            # after loop_start ("now"). Do not flag a fresh plan as stale
            # merely because its calculated age is slightly negative.
            static_age = (
                max(0.0, time.time() - static_updated_at)
                if static_updated_at > 0.0 else float("inf")
            )
            static_fresh = bool(
                static_plan["valid"] and static_age <= (
                    min(static_timeout_sec, float(
                        missions.get("gps_shadow_ego_planner", {}).get(
                            "max_result_age_sec", 1.6
                        )
                    )) if shadow_active else static_timeout_sec
                )
            )
            static_in_zone = (
                in_static_zone(nearest_index, static_zones) or shadow_active
            )
            # A route accepted in-zone keeps LiDAR fail-stop protection until
            # the worker reports geometric release, even past the zone end.
            if args.static and static_fresh:
                static_route_engaged = bool(static_plan["latch_active"])
            static_required = bool(static_in_zone or static_route_engaged)
            static_state = (
                "OFF" if not args.static else
                "ZONE_OFF" if not static_required else
                static_plan["status"] if static_fresh else "VISION_WAIT"
            )
            static_active = False
            static_stop = False
            if args.static_control and static_required:
                # CLEAR: V7 or shadow-lane speed. AVOIDING: original
                # LiDAR locked local route temporarily overrides lane steer.
                # STOP, stale result, or mismatched zone: full brake.
                if (
                    not static_fresh
                    or not static_plan["zone_active"]
                    or static_state == "STOP_REQUIRED"
                ):
                    static_stop = True
                    if static_fresh and not static_plan["zone_active"]:
                        static_state = "ZONE_MISMATCH"
                elif static_state == "AVOIDING":
                    if shadow_active and not static_plan.get("shadow_wall_verified", False):
                        # Mandatory positive verification by current LiDAR scan.
                        static_stop = True
                        static_state = "SHADOW_WALL_NOT_VERIFIED"
                    else:
                        target_speed_kmh = min(target_speed_kmh, static_speed_limit)
                        local_pp = local_path_pure_pursuit(
                            static_plan["path"], current_x, current_y, yaw,
                            base.VEHICLE_LENGTH,
                        )
                        if local_pp is None:
                            static_stop = True
                            static_state = "LOCAL_PATH_LOST"
                        else:
                            static_active = True
                            steering = float(local_pp["steering"])
                            target_speed_kmh = min(
                                target_speed_kmh,
                                float(static_plan["recommended_speed_kmh"]),
                            )
                if static_stop:
                    target_speed_kmh = 0.0
                    mission["reasons"].append("static_stop_or_stale")
                elif static_active:
                    mission["reasons"].append("static_local_avoidance")

            if (shadow_active and static_active and gps_mode == "SHADOW"):
                # Obstacle may occlude the lane. Permit ONLY a fresh valid
                # original LiDAR local route to override lane-vision dropout;
                # GPS/DR expiry, too much path drift, and LiDAR STOP still win.
                shadow_stop = False

            if shadow_active:
                # GPS-shadow requires independent, recent 3D LiDAR evidence
                # even when no clustered static obstacle is detected.
                if not static_fresh:
                    shadow_stop = True
                    target_speed_kmh = 0.0
                    mission["reasons"].append("gps_shadow_lidar_stale")
                target_speed_kmh = min(
                    target_speed_kmh,
                    float(shadow_cfg.get("shadow_speed_kmh", 12.0)),
                )
                mission["reasons"].append(
                    "gps_shadow_lane" if not static_active else
                    "gps_shadow_lidar_avoid"
                )
                if shadow_stop:
                    target_speed_kmh = 0.0
                    mission["reasons"].append("gps_shadow_safe_stop")

            (
                desired_accel,
                desired_brake,
                speed_error,
                control_zone,
            ) = base.longitudinal_policy(
                target_speed_kmh,
                current_speed_kmh,
            )

            if result is None and not static_active and not (
                    shadow_active and lane_fresh and not shadow_stop):
                desired_accel = 0.0
                desired_brake = base.PATH_LOST_BRAKE_CMD
                control_zone = "PATH_LOST"

            dt_control = max(1e-3, now - prev_control_time)
            prev_control_time = now

            if ((args.traffic and not shadow_active and mission["traffic_full_brake"])
                or (args.merge_control and not shadow_active and mission["merge_stop"])
                or (args.static_control and static_stop) or pedestrian_stop or shadow_stop):
                # Any mission STOP commands full brake from main.
                command_limiter.reset()
                accel_cmd, brake_cmd, signed_cmd = 0.0, 1.0, -1.0
                control_zone = (
                    "TRAFFIC_FULL_BRAKE"
                    if args.traffic and mission["traffic_full_brake"]
                    else "MERGE_FULL_BRAKE" if args.merge_control and mission["merge_stop"]
                    else "STATIC_FULL_BRAKE" if args.static_control and static_stop
                    else "PEDESTRIAN_FULL_BRAKE" if pedestrian_stop
                    else "GPS_SHADOW_SAFE_STOP"
                )
            elif (
                (
                    (args.traffic and mission["traffic_approach"])
                    or (args.merge_control and mission["merge_zone"])
                    or args.static_control or shadow_active
                )
                and current_speed_kmh > target_speed_kmh + 0.8
            ):
                # V7 cruise braking is capped at 0.10. Approach-specific
                # stronger brake helps track the 30/12/4 km/h speed caps.
                command_limiter.reset()
                err_kmh = current_speed_kmh - target_speed_kmh
                cfg = missions.get("traffic", {})
                approach_brake = min(
                    float(cfg.get("approach_brake_max_cmd", 0.5)),
                    max(
                        float(cfg.get("approach_brake_min_cmd", 0.14)),
                        float(cfg.get("approach_brake_gain", 0.028)) * err_kmh,
                    ),
                )
                accel_cmd, brake_cmd = 0.0, approach_brake
                signed_cmd = -brake_cmd
                control_zone = "TRAFFIC_APPROACH_BRAKE"
            else:
                # Resume regular V7 longitudinal limiting after GO.
                accel_cmd, brake_cmd, signed_cmd = command_limiter.update(
                    desired_accel,
                    desired_brake,
                    dt_control,
                )

            elapsed = now - run_start
            log_writer.writerow([
                f"{elapsed:.3f}",
                nearest_index,
                f"{current_x:.3f}",
                f"{current_y:.3f}",
                f"{math.degrees(yaw):.3f}",
                f"{nearest_distance:.3f}",
                f"{base_target:.3f}",
                f"{target_speed_kmh:.3f}",
                f"{current_speed_kmh:.3f}",
                f"{steering:.6f}",
                f"{accel_cmd:.4f}",
                f"{brake_cmd:.4f}",
                traffic.get("signal", "UNKNOWN"),
                (
                    f"{max(0.0, now - traffic_updated_at):.3f}"
                    if traffic_updated_at > 0.0 else "inf"
                ),
                mission["traffic_state"],
                int(mission["red_latched"]),
                mission["go_release_count"],
                mission["merge_state"],
                f"{mission['merge_risk']:.3f}",
                ";".join(mission["reasons"]),
                (mission["active_stop_index"]
                 if mission["active_stop_index"] is not None else ""),
                (f'{mission["stop_distance_m"]:.3f}'
                 if mission["stop_distance_m"] is not None else ""),
                int(mission["traffic_full_brake"]),
                int(mission["traffic_approach"]),
                int(mission["left_arrow_override"]),
                int(mission["merge_zone"]),
                int(mission["merge_stop"]),
                mission["merge_accept_count"],
                static_state,
                f"{static_age:.3f}" if math.isfinite(static_age) else "inf",
                static_plan["obstacle_id"] if static_fresh else "",
                static_plan["avoidance_side"] if static_fresh else "",
                len(static_plan["path"]) if static_fresh else 0,
                int(static_active),
                int(static_stop),
                static_plan["detected"] if static_fresh else 0,
                static_plan["scan"] if static_fresh else 0,
                int(static_in_zone), int(static_route_engaged),
                static_plan["waypoint_index"] if static_fresh else "",
                static_plan["latch_state"] if static_fresh else "",
                int(mission["second_green_bypass"]),
                mission["second_green_count"],
                int(pedestrian_in_zone), pedestrian_state,
                (f"{pedestrian_result['risk']:.3f}" if pedestrian_fresh else ""),
                pedestrian_result["person_count"] if pedestrian_fresh else "",
                int(pedestrian_stop),
                f"{pedestrian_age:.3f}" if math.isfinite(pedestrian_age) else "inf",
                pedestrian_result["frame"] if pedestrian_fresh else "",
                pedestrian_latch.clear_count,
                int(follow_zone_active), follow_state,
                f"{follow_risk:.3f}" if follow_zone_active else "",
                follow_grade if follow_zone_active else "",
                f"{follow_cap:.2f}" if follow_zone_active else "",
                f"{follow_age:.3f}" if math.isfinite(follow_age) else "inf",
                follow_msg["vehicles"] if follow_fresh else "",
                gps_mode, f"{shadow_sec:.3f}",
                int(lane_fresh), f"{lane_result['confidence']:.3f}",
                f"{lane_result['steering']:.4f}", int(shadow_stop),
            ])

            print("\033[2J\033[H", end="")
            print("====================================================")
            print(" V7 + TRAFFIC + MERGE INTEGRATION V1")
            print("====================================================")
            print(f"GPS         : {lat:.6f}, {lon:.6f} / MODE={gps_mode}"
                  f" / outage={shadow_sec:.1f}s")
            print(f"LANE        : fresh={lane_fresh} conf={lane_result['confidence']:.2f}"
                  f" steer={lane_result['steering']:.3f} / SHADOW_STOP={shadow_stop}")
            print(
                f"POSITION    : x={current_x:.3f}, y={current_y:.3f}"
            )
            print(f"YAW         : {math.degrees(yaw):.2f} deg")
            print(f"NEAREST     : {nearest_index}")
            print(f"PATH DIST   : {nearest_distance:.3f} m")
            print(f"BASE TARGET : {base_target:.2f} km/h")
            print(f"FINAL TARGET: {target_speed_kmh:.2f} km/h")
            print(f"COMP SPEED  : {current_speed_kmh:.2f} km/h")
            print(
                f"STEERING    : {steering:.5f} rad "
                f"({math.degrees(steering):.2f} deg)"
            )
            print(
                f"ACCEL/BRAKE : {accel_cmd:.3f} / {brake_cmd:.3f}"
            )
            print(
                f"CTRL ZONE   : {control_zone} / LONG={signed_cmd:+.3f}"
            )
            print()
            traffic_age = (
                now - traffic_updated_at
                if traffic_updated_at > 0.0 else float("inf")
            )
            traffic_age_text = (
                f"{traffic_age:.2f}s"
                if math.isfinite(traffic_age) else "WAIT"
            )
            print(
                f"TRAFFIC RAW : {traffic.get('signal', 'UNKNOWN')} / "
                f"age={traffic_age_text} / "
                f"line={traffic.get('stopline_y')}"
            )
            print(
                f"TRAFFIC CTRL: {mission['traffic_state']} / "
                f"RED={mission['red_latched']} / "
                f"GO={mission['go_release_count']} / "
                f"LEFT_OVERRIDE={mission['left_arrow_override']} / "
                f"GREEN_BYPASS_620={mission['second_green_bypass']} / "
                f"GREEN_COUNT_620={mission['second_green_count']}"
            )
            stop_dist = mission["stop_distance_m"]
            print(
                f"STOP TARGET : {mission['active_stop_index']} / "
                f"distance={stop_dist:.1f}m"
                if stop_dist is not None else
                "STOP TARGET : NONE"
            )
            print(
                f"MERGE       : {mission['merge_state']} / "
                f"RISK={mission['merge_risk']:.2f} / "
                f"CTRL={'ON' if args.merge_control else 'OBSERVE'} / "
                f"ZONE={mission['merge_zone']} / STOP={mission['merge_stop']} / "
                f"SAFE={mission['merge_accept_count']}"
            )
            print(
                f"HIGHWAY     : {follow_state} / RISK={follow_risk:.2f}"
                f" GRADE={follow_grade} CAP={follow_cap:.1f}km/h"
                f" / FAST={'YES' if any(a <= nearest_index <= b for a,b in fast_zones) else 'NO'}"
                f" / AGE={follow_age:.2f}s"
            )
            print(
                f"STATIC      : {static_state} / "
                f"age={static_age:.2f}s / "
                f"CTRL={'ON' if args.static_control else 'OBSERVE'} / "
                f"PATH={len(static_plan['path']) if static_fresh else 0} / "
                f"ZONE={static_in_zone} / LOCK={static_route_engaged} / "
                f"AVOID={static_active} / STOP={static_stop} / "
                f"OBJ={static_plan['obstacle_id'] if static_fresh else None}"
            )
            print(
                f"PEDESTRIAN  : {pedestrian_state} / ZONE={pedestrian_in_zone} "
                f"/ RISK={pedestrian_result['risk'] if pedestrian_fresh else None} "
                f"/ PERSON={pedestrian_result['person_count'] if pedestrian_fresh else 0} "
                f"/ STOP={pedestrian_stop} / age={pedestrian_age:.2f}s"
            )
            print(
                f"MISSION     : {mission['reasons'] or ['base_v7']}"
            )
            print("CMD MODE    : ctrl_mode=2 / cmd_type=1")
            print(f"LOG FILE    : {log_path.name}")

            if args.drive:
                cmd = EgoCtrlCmd()
                cmd.ctrl_mode = 2
                cmd.gear = 4
                cmd.cmd_type = 1
                cmd.accel = max(0.0, min(1.0, accel_cmd))
                cmd.brake = max(0.0, min(1.0, brake_cmd))
                cmd.steer = max(-1.0, min(1.0, steering))
                ctrl_sender.send(cmd)
                print("[DRIVE] CtrlCmd sending")
            else:
                print("[DRY-RUN] CtrlCmd not sent")

            used = time.time() - loop_start
            if used < period:
                time.sleep(period - used)

    except KeyboardInterrupt:
        print("\nStopping...")
        if args.drive and ctrl_sender is not None:
            base.send_stop(ctrl_sender)

    finally:
        if traffic_rx is not None:
            traffic_rx.close()
        if merge_rx is not None:
            merge_rx.close()
        if merge_gate_tx is not None:
            merge_gate_tx.close()
        if static_rx is not None:
            static_rx.close()
        if pose_tx is not None:
            pose_tx.close()
        if pedestrian_rx is not None:
            pedestrian_rx.close()
        if follow_rx is not None:
            follow_rx.close()
        if pedestrian_gate_tx is not None:
            pedestrian_gate_tx.close()
        if lane_rx is not None:
            lane_rx.close()
        log_file.flush()
        log_file.close()
        print(f"[LOG SAVED] {log_path}")


if __name__ == "__main__":
    main()
