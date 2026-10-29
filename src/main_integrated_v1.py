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
from integration.static_avoidance_io import (
    parse_static_message, local_path_pure_pursuit,
    configured_avoidance_speed_kmh,
)
from integration.static_zone import load_static_zones, in_static_zone
from integration.lane_diagnostics import path_tracking_metrics
from integration.beomsu_lane_guard import make_zones, make_tracking_path, offset_at_waypoint, TEST_SPEED_LIMIT_KMH
from integration.roundabout_exit_offset import (
    make_roundabout_exit_path,
    offset_at_waypoint as roundabout_offset_at_waypoint,
)
from integration.roundabout_speed_cap import build_roundabout_speed_caps


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
        "static_latch_state",
        # Video-to-CSV synchronization and read-only lateral diagnostics.
        "wall_time_local", "wall_time_epoch_s",
        "signed_cte_m", "abs_cte_m", "path_heading_deg",
        "heading_error_deg", "nearest_segment_wp", "curve_radius_m",
        "v7_effective_lfd_m", "v7_steering_gain", "v7_target_wp",
        "v7_switch_bias", "tracking_path_source",
        "beomsu_offset_m", "beomsu_tracking_dist_m", "beomsu_zone_active",
        "beomsu_roundabout_right_m", "beomsu_roundabout_speed_cap_kmh",
        "beomsu_roundabout_speed_zone",
    ])
    return path, f, w


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--drive", action="store_true")
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
    parser.add_argument("--max-speed", type=float, default=57.0)
    parser.add_argument("--beomsu-first-right-m", type=float, default=0.0,
                        help="Opt-in WP1480-1715 right offset; low-speed test 0.25m")
    parser.add_argument("--beomsu-extra-zone", action="append", default=[],
                        metavar="START:END:SIGNED_METRES",
                        help="Experimental zone; +right/-left; use only after waypoint verification")
    parser.add_argument("--beomsu-roundabout-right-m", type=float, default=0.0,
                        help="Opt-in smooth RIGHT offset WP1859-1902; initial low-speed test 0.20m")
    parser.add_argument("--beomsu-roundabout-kmh", type=float, default=0.0,
                        help="Opt-in distance-preview slow roundabout speed hold WP1770-1902; test 8km/h")
    args = parser.parse_args()

    if args.merge_control and not args.merge:
        parser.error("--merge-control requires --merge")
    if args.static_control and not args.static:
        parser.error("--static-control requires --static")
    if args.max_speed <= 0.0 or args.max_speed > 57.0:
        parser.error("--max-speed must be >0 and <=57")

    network = load_json(NETWORK_FILE)
    missions = load_json(MISSION_FILE)
    stop_indices = load_stop_indices(STOP_INDEX_FILE)

    # Importing V7 already applies the validated V7 monkey-patches.
    global_path = base.load_path(base.PATH_FILE)
    try:
        lane_zones = make_zones(len(global_path), args.beomsu_first_right_m, args.beomsu_extra_zone)
        tracking_path = make_tracking_path(global_path, lane_zones)
        tracking_path = make_roundabout_exit_path(
            tracking_path, args.beomsu_roundabout_right_m
        )
        roundabout_speed_caps, roundabout_speed_zones = build_roundabout_speed_caps(
            global_path, args.max_speed, args.beomsu_roundabout_kmh
        )
    except ValueError as exc:
        parser.error(str(exc))
    target_profile, radius_profile, full_profile = (
        profile_test.build_test_profile(global_path, args.max_speed)
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
    if args.merge:
        merge_rx = LatestUdpTextReceiver(
            network["merge_result"]["bind_ip"],
            int(network["merge_result"]["port"]),
        )

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

    print("====================================================")
    print(" V7 + TRAFFIC + MERGE INTEGRATION V1")
    print("====================================================")
    print("[BASE] validated V7 import chain")
    print("[CTRL] single EgoCtrlCmd sender / ctrl_mode=2 / cmd_type=1")
    print(f"[MODE] {'DRIVE' if args.drive else 'DRY-RUN'}")
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
    if args.static:
        print(f"[STATIC ZONES WP] {static_zones}; outside=V7 only")
        print(f"[STATIC POSE] {pose_target}")
        print(f"[STATIC RESULT] {network['static_result']}")
    print(f"[BEOMSU LANE] zones={lane_zones} / default OFF / cap={TEST_SPEED_LIMIT_KMH:.1f}km/h in offset zones")
    print(f"[ROUNDABOUT BEOMSU] right_offset={args.beomsu_roundabout_right_m:+.2f}m "
          f"WP1859-1902 / full-circle WP1770-1902 speed_hold={args.beomsu_roundabout_kmh:.1f}km/h "
          f"WP1835-1892 / 0=OFF")
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

            if static_rx is not None:
                raw, updated_at, is_new = static_rx.poll()
                if raw is not None and is_new:
                    static_plan = parse_static_message(raw)
                    static_updated_at = updated_at

            gps_data = gps_receiver.get_data()
            gps_data.parsing()
            lat = gps_data.gprmc.lat
            lon = gps_data.gprmc.lon
            position = base.gps_to_local(lat, lon)
            if position is None:
                print("[WAIT] GPS")
                time.sleep(0.1)
                continue

            current_x, current_y = position

            imu_data = imu_receiver.get_data()
            if (
                imu_data.ori_w == 0.0
                and imu_data.ori_x == 0.0
                and imu_data.ori_y == 0.0
                and imu_data.ori_z == 0.0
            ):
                print("[WAIT] IMU")
                time.sleep(0.1)
                continue

            yaw = base.quaternion_to_yaw(
                imu_data.ori_x,
                imu_data.ori_y,
                imu_data.ori_z,
                imu_data.ori_w,
            )

            if pose_tx is not None:
                pose_tx.sendto(
                    json.dumps({"x": current_x, "y": current_y, "yaw": yaw}).encode("ascii"),
                    pose_target,
                )

            current_speed_kmh, speed_valid = speed_receiver.read()
            if not speed_valid:
                print("[WAIT] Competition Vehicle Status")
                time.sleep(0.05)
                continue

            nearest_index = base.find_nearest_waypoint(
                global_path,
                current_x,
                current_y,
            )
            nearest_x = global_path[nearest_index, 0]
            nearest_y = global_path[nearest_index, 1]
            nearest_distance = math.hypot(
                nearest_x - current_x,
                nearest_y - current_y,
            )

            # Measurement only: project onto nearby global-path segments.
            # It does not change steering, speed, or the reference route.
            try:
                lane_metrics = path_tracking_metrics(
                    global_path, nearest_index, current_x, current_y, yaw,
                )
            except ValueError:
                lane_metrics = {
                    "signed_cte_m": float("nan"),
                    "abs_cte_m": float("nan"),
                    "path_heading_deg": float("nan"),
                    "heading_error_deg": float("nan"),
                    "nearest_segment_index": -1,
                }
            curve_radius_m = (
                float(radius_profile[nearest_index])
                if len(radius_profile) > nearest_index else float("nan")
            )

            result = base.calculate_pure_pursuit(
                tracking_path,
                nearest_index,
                current_x,
                current_y,
                yaw,
            )

            if result is None:
                base_steering = 0.0
                base_target = 0.0
            else:
                base_steering = float(result["steering"])
                base_target = float(target_profile[nearest_index])

            lane_offset_m = offset_at_waypoint(nearest_index, lane_zones)
            tracking_distance_m = math.hypot(tracking_path[nearest_index, 0] - current_x,
                                             tracking_path[nearest_index, 1] - current_y)
            roundabout_right_m = roundabout_offset_at_waypoint(
                nearest_index, args.beomsu_roundabout_right_m
            )
            roundabout_cap_kmh = float(roundabout_speed_caps[nearest_index])
            roundabout_speed_zone = roundabout_speed_zones[nearest_index]

            mission = mission_manager.apply(
                nearest_index=nearest_index,
                base_target_speed_kmh=base_target,
                base_steering=base_steering,
                traffic_enabled=args.traffic,
                merge_enabled=args.merge,
                merge_control_enabled=args.merge_control,
                merge_result=merge,
                merge_updated_at=merge_updated_at,
                merge_all=args.merge_all,
                now=now,
            )

            target_speed_kmh = mission["target_speed_kmh"]
            steering = mission["steering"]
            if lane_offset_m != 0.0:
                target_speed_kmh = min(target_speed_kmh, TEST_SPEED_LIMIT_KMH)
                mission["reasons"].append("beomsu_lane_offset_speed_cap")
            if roundabout_speed_zone != "OFF":
                target_speed_kmh = min(target_speed_kmh, roundabout_cap_kmh)
                mission["reasons"].append("beomsu_roundabout_speed_cap")

            # UDP poll() can timestamp a new packet a few milliseconds
            # after loop_start ("now"). Do not flag a fresh plan as stale
            # merely because its calculated age is slightly negative.
            static_age = (
                max(0.0, time.time() - static_updated_at)
                if static_updated_at > 0.0 else float("inf")
            )
            static_fresh = bool(
                static_plan["valid"] and static_age <= static_timeout_sec
            )
            static_in_zone = in_static_zone(nearest_index, static_zones)
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
                # CLEAR: V7 speed. AVOIDING: low-speed locked local route.
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

            (
                desired_accel,
                desired_brake,
                speed_error,
                control_zone,
            ) = base.longitudinal_policy(
                target_speed_kmh,
                current_speed_kmh,
            )

            if result is None and not static_active:
                desired_accel = 0.0
                desired_brake = base.PATH_LOST_BRAKE_CMD
                control_zone = "PATH_LOST"

            dt_control = max(1e-3, now - prev_control_time)
            prev_control_time = now

            if (args.traffic and mission["traffic_full_brake"]) or (
                args.merge_control and mission["merge_stop"]
            ) or (args.static_control and static_stop):
                # Traffic or merge STOP: zero throttle and immediate full brake.
                command_limiter.reset()
                accel_cmd, brake_cmd, signed_cmd = 0.0, 1.0, -1.0
                control_zone = (
                    "TRAFFIC_FULL_BRAKE"
                    if args.traffic and mission["traffic_full_brake"]
                    else "MERGE_FULL_BRAKE" if args.merge_control and mission["merge_stop"]
                    else "STATIC_FULL_BRAKE"
                )
            elif (
                (
                    (args.traffic and mission["traffic_approach"])
                    or (args.merge_control and mission["merge_zone"])
                    or args.static_control
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

            # Use one timestamp for both the visible video sync cue and CSV.
            elapsed = now - run_start
            wall_time_local = datetime.fromtimestamp(now).astimezone().isoformat(
                timespec="milliseconds"
            )
            v7_lfd_m = float(v7.v3._DIAG.get("effective_lfd_m", float("nan")))
            v7_steer_gain = (
                float(v7._DIAG.get("steering_gain", float("nan")))
                if result is not None else float("nan")
            )
            v7_target_wp = (
                int(v7._DIAG.get("selected_target_index", -1))
                if result is not None else -1
            )
            v7_switch_bias = (
                int(v7._DIAG.get("target_bias_active", 0))
                if result is not None else 0
            )
            tracking_path_source = (
                "LOCAL_AVOIDANCE" if static_active
                else "STOP_REQUIRED" if static_stop
                else "PATH_LOST" if result is None
                else "GLOBAL_V7"
            )
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
                wall_time_local, f"{now:.3f}",
                f"{lane_metrics['signed_cte_m']:+.4f}",
                f"{lane_metrics['abs_cte_m']:.4f}",
                f"{lane_metrics['path_heading_deg']:.3f}",
                f"{lane_metrics['heading_error_deg']:+.3f}",
                lane_metrics["nearest_segment_index"],
                f"{curve_radius_m:.3f}", f"{v7_lfd_m:.3f}",
                f"{v7_steer_gain:.3f}", v7_target_wp, v7_switch_bias,
                tracking_path_source,
                f"{lane_offset_m:+.3f}", f"{tracking_distance_m:.3f}",
                int(lane_offset_m != 0.0),
                f"{roundabout_right_m:+.3f}", f"{roundabout_cap_kmh:.3f}",
                roundabout_speed_zone,
            ])

            print("\033[2J\033[H", end="")
            print("====================================================")
            print(" V7 + TRAFFIC + MERGE INTEGRATION V1")
            print("====================================================")
            print(f"GPS         : {lat:.6f}, {lon:.6f}")
            print(
                f"POSITION    : x={current_x:.3f}, y={current_y:.3f}"
            )
            print(f"YAW         : {math.degrees(yaw):.2f} deg")
            print(f"NEAREST     : {nearest_index}")
            print(f"VIDEO SYNC  : {wall_time_local} / elapsed={elapsed:.2f}s")
            print(f"PATH DIST   : {nearest_distance:.3f} m")
            print(
                f"GLOBAL CTE  : {lane_metrics['signed_cte_m']:+.3f} m "
                f"(+LEFT/-RIGHT) / heading_err={lane_metrics['heading_error_deg']:+.2f}deg"
            )
            print(
                f"V7 LFD/GAIN : {v7_lfd_m:.2f} m / {v7_steer_gain:.2f} "
                f"/ targetWP={v7_target_wp} / switch={v7_switch_bias}"
            )
            print(f"TRACK PATH  : {tracking_path_source} (CTE always vs global path)")
            print(f"BEOMSU PATH : offset={lane_offset_m:+.3f}m / dist={tracking_distance_m:.3f}m")
            if args.beomsu_roundabout_right_m > 0.0 or args.beomsu_roundabout_kmh > 0.0:
                print(f"ROUNDABOUT  : right={roundabout_right_m:+.3f}m / "
                      f"cap={roundabout_cap_kmh:.2f}km/h / {roundabout_speed_zone}")

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
                f"LEFT_OVERRIDE={mission['left_arrow_override']}"
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
                f"STATIC      : {static_state} / "
                f"age={static_age:.2f}s / "
                f"CTRL={'ON' if args.static_control else 'OBSERVE'} / "
                f"PATH={len(static_plan['path']) if static_fresh else 0} / "
                f"ZONE={static_in_zone} / LOCK={static_route_engaged} / "
                f"AVOID={static_active} / STOP={static_stop} / "
                f"OBJ={static_plan['obstacle_id'] if static_fresh else None}"
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
        if static_rx is not None:
            static_rx.close()
        if pose_tx is not None:
            pose_tx.close()
        log_file.flush()
        log_file.close()
        print(f"[LOG SAVED] {log_path}")


if __name__ == "__main__":
    main()
