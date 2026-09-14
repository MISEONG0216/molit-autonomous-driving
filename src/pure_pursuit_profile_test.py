#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Waypoint-speed-profile integration test.

Safety strategy:
- Reuse the already validated Competition Vehicle Status speed source.
- Reuse the proven Pure Pursuit lateral controller and cmd_type=1 longitudinal controller.
- Reuse path_speed_profile.py for curvature + preview profile generation.
- DO NOT jump directly to the 57 km/h profile. Scale the profile while preserving
  the relative slow-down pattern before curves.

Validated stages:
- 5~10 km/h profile: completed.

Current next stage:
- allow an explicit 5~20 km/h profile test with --max-speed 20.
- default remains 10 km/h so an accidental run does not increase test speed.
"""

import argparse
import math
import time

import numpy as np

import pure_pursuit_regulation as base
import path_speed_profile as profile_builder
from pure_pursuit_competition import CompetitionSpeedReceiver

from lib.network.UDP import Receiver, Sender
from lib.define.GPS import GPS
from lib.define.IMU import IMU
from lib.define.EgoCtrlCmd import EgoCtrlCmd


DEFAULT_TEST_MAX_KMH = 10.0
ALLOWED_TEST_MAX_KMH = 20.0
TEST_MIN_KMH = 5.0


def build_test_profile(global_path, max_speed_kmh):
    """Build the existing preview profile, then scale it for a staged-speed test."""
    _, ds = profile_builder.cumulative_distance(global_path)
    curvature = profile_builder.compute_curvature(global_path)
    radius = profile_builder.curvature_to_radius(curvature)
    seed = profile_builder.build_seed_profile(radius)
    full_profile = profile_builder.apply_braking_preview(seed, ds)

    full_min = float(np.min(full_profile))
    full_max = float(np.max(full_profile))

    # Preserve the shape of the 15~57 km/h profile but map it to 5~max km/h.
    if full_max - full_min < 1e-6:
        test_profile = np.full_like(full_profile, max_speed_kmh)
    else:
        normalized = (full_profile - full_min) / (full_max - full_min)
        test_profile = TEST_MIN_KMH + normalized * (
            max_speed_kmh - TEST_MIN_KMH
        )

    return test_profile, radius, full_profile


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--drive",
        action="store_true",
        help="규정 허용 cmd_type=1 CtrlCmd를 실제 MORAI로 송신",
    )
    parser.add_argument(
        "--max-speed",
        type=float,
        default=DEFAULT_TEST_MAX_KMH,
        help="프로파일 테스트의 직선 최고 목표속도(km/h), 기본 10 / 현재 허용 최대 20",
    )
    args = parser.parse_args()

    if args.max_speed < TEST_MIN_KMH:
        raise ValueError("--max-speed는 5 km/h 이상이어야 합니다.")
    if args.max_speed > ALLOWED_TEST_MAX_KMH:
        raise ValueError(
            "현재 단계에서는 20 km/h를 넘기지 않습니다. "
            "20 km/h 감속/조향 검증 후 단계적으로 상향하세요."
        )

    global_path = base.load_path(base.PATH_FILE)
    target_profile, radius_profile, full_profile = build_test_profile(
        global_path,
        args.max_speed,
    )

    print("====================================================")
    print(" WAYPOINT SPEED PROFILE - STAGED TEST")
    print("====================================================")
    print("[CTRL] ctrl_mode=2 / cmd_type=1")
    print("[SPEED] Competition Vehicle Status / signed_vel")
    print(f"[PROFILE] {TEST_MIN_KMH:.1f} ~ {args.max_speed:.1f} km/h")
    print("[NOTE] 57 km/h full profile is NOT enabled in this test.")
    print("[MODE] DRIVE" if args.drive else "[MODE] DRY-RUN")
    print()

    gps_receiver = Receiver(base.GPS_IP, base.GPS_PORT, GPS())
    imu_receiver = Receiver(base.IMU_IP, base.IMU_PORT, IMU())
    speed_receiver = CompetitionSpeedReceiver()
    ctrl_sender = Sender(base.CTRL_IP, base.CTRL_PORT) if args.drive else None

    command_limiter = base.LongitudinalCommandLimiter(
        base.LONGITUDINAL_CMD_RATE_PER_SEC
    )

    period = 1.0 / base.CONTROL_HZ
    prev_control_time = time.time()

    try:
        while True:
            loop_start = time.time()

            gps_data = gps_receiver.get_data()
            gps_data.parsing()
            lat = gps_data.gprmc.lat
            lon = gps_data.gprmc.lon
            position = base.gps_to_local(lat, lon)
            if position is None:
                print("[WAIT] GPS 데이터 대기 중...")
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
                print("[WAIT] IMU 데이터 대기 중...")
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
                print("[WAIT] Competition Vehicle Status 데이터 대기 중...")
                time.sleep(0.05)
                continue

            nearest_index = base.find_nearest_waypoint(
                global_path, current_x, current_y
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
                steering = 0.0
                target_speed_kmh = 0.0
            else:
                steering = result["steering"]
                target_speed_kmh = float(target_profile[nearest_index])

            now = time.time()
            dt_control = max(1e-3, now - prev_control_time)
            prev_control_time = now

            desired_accel, desired_brake, speed_error, control_zone = (
                base.longitudinal_policy(target_speed_kmh, current_speed_kmh)
            )

            if result is None:
                desired_accel = 0.0
                desired_brake = base.PATH_LOST_BRAKE_CMD
                control_zone = "PATH_LOST"

            accel_cmd, brake_cmd, signed_cmd = command_limiter.update(
                desired_accel,
                desired_brake,
                dt_control,
            )

            radius_m = float(radius_profile[nearest_index])
            full_target = float(full_profile[nearest_index])

            print("\033[2J\033[H", end="")
            print("====================================================")
            print(" WAYPOINT SPEED PROFILE TEST")
            print("====================================================")
            print(f"GPS        : {lat:.6f}, {lon:.6f}")
            print(f"POSITION   : x={current_x:.3f}, y={current_y:.3f}")
            print(f"YAW        : {math.degrees(yaw):.2f} deg")
            print()
            print(f"NEAREST    : index={nearest_index}")
            print(f"PATH DIST  : {nearest_distance:.3f} m")
            print(
                "CURVE R    : "
                + ("inf m" if not math.isfinite(radius_m) else f"{radius_m:.1f} m")
            )
            print(f"FULL PROF  : {full_target:.2f} km/h  [reference only]")
            print(f"TEST TARGET: {target_speed_kmh:.2f} km/h")
            print(f"COMP SPEED : {current_speed_kmh:.2f} km/h")
            print("SPEED SRC  : Competition Vehicle Status / signed_vel")
            print(f"CTRL ZONE  : {control_zone}")
            print(f"SPD ERROR  : {speed_error:+.2f} km/h")
            print(f"STEERING   : {steering:.5f} rad ({math.degrees(steering):.2f} deg)")
            print(f"LONG CMD   : {signed_cmd:+.3f}")
            print(f"ACCEL      : {accel_cmd:.3f}")
            print(f"BRAKE      : {brake_cmd:.3f}")
            print("CMD MODE   : ctrl_mode=2 / cmd_type=1")
            print()

            if args.drive:
                cmd = EgoCtrlCmd()
                cmd.ctrl_mode = 2
                cmd.gear = 4
                cmd.cmd_type = 1
                cmd.accel = max(0.0, min(1.0, accel_cmd))
                cmd.brake = max(0.0, min(1.0, brake_cmd))
                cmd.steer = max(-1.0, min(1.0, steering))
                ctrl_sender.send(cmd)
                print("[DRIVE] CtrlCmd 송신 중")
            else:
                print("[DRY-RUN] CtrlCmd 송신 안 함")

            elapsed = time.time() - loop_start
            if elapsed < period:
                time.sleep(period - elapsed)

    except KeyboardInterrupt:
        print()
        print("종료 중...")
        if args.drive and ctrl_sender is not None:
            print("정지 명령 송신 (cmd_type=1 / brake=1.0)")
            base.send_stop(ctrl_sender)


if __name__ == "__main__":
    main()
