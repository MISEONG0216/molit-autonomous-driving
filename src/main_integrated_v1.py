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
        "traffic_full_brake", "traffic_approach",
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
    parser.add_argument("--max-speed", type=float, default=57.0)
    args = parser.parse_args()

    if args.merge_control and not args.merge:
        parser.error("--merge-control requires --merge")
    if args.max_speed <= 0.0 or args.max_speed > 57.0:
        parser.error("--max-speed must be >0 and <=57")

    network = load_json(NETWORK_FILE)
    missions = load_json(MISSION_FILE)
    stop_indices = load_stop_indices(STOP_INDEX_FILE)

    # Importing V7 already applies the validated V7 monkey-patches.
    global_path = base.load_path(base.PATH_FILE)
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
        f"control={'ON' if args.merge_control else 'OBSERVE'}"
    )
    print(
        f"[MORAI CTRL] {network['control']['target_ip']}:"
        f"{network['control']['port']}"
    )
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

            result = base.calculate_pure_pursuit(
                global_path,
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

            (
                desired_accel,
                desired_brake,
                speed_error,
                control_zone,
            ) = base.longitudinal_policy(
                target_speed_kmh,
                current_speed_kmh,
            )

            if result is None:
                desired_accel = 0.0
                desired_brake = base.PATH_LOST_BRAKE_CMD
                control_zone = "PATH_LOST"

            dt_control = max(1e-3, now - prev_control_time)
            prev_control_time = now

            if args.traffic and mission["traffic_full_brake"]:
                # STOP latch only: cut throttle and hold full brake.
                command_limiter.reset()
                accel_cmd, brake_cmd, signed_cmd = 0.0, 1.0, -1.0
                control_zone = "TRAFFIC_FULL_BRAKE"
            elif (
                args.traffic
                and mission["traffic_approach"]
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
                f"GO={mission['go_release_count']}"
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
                f"CTRL={'ON' if args.merge_control else 'OBSERVE'}"
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
        log_file.flush()
        log_file.close()
        print(f"[LOG SAVED] {log_path}")


if __name__ == "__main__":
    main()
