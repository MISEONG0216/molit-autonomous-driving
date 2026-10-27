#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Regulation-compliant Pure Pursuit baseline for the 2026 competition.

This file intentionally preserves the proven lateral-control baseline while
using the competition-allowed EgoCtrlCmd mode:
    ctrl_mode = 2
    cmd_type  = 1  (accel / brake / steer)

Longitudinal control is deliberately conservative during the 5 km/h validation
stage because temporary GPS-displacement speed is noisy. Instead of braking
whenever speed slightly exceeds the target, the controller uses three zones:

    ACCEL : below target - 0.2 km/h
    COAST : from target - 0.2 to target + 2.0 km/h
    BRAKE : above target + 2.0 km/h only

For the default 5 km/h target this means approximately:
    < 4.8 km/h  -> accel
    4.8~7.0     -> coast (accel=0, brake=0)
    > 7.0       -> gentle brake

Once the official Competition Vehicle Status packet format/port is verified,
its x-direction speed should replace GPS-derived speed for final control.
"""

import argparse
import math
import sys
import time
from collections import deque
from pathlib import Path

import numpy as np
from pyproj import Proj


# ============================================================
# Project / MORAI module
# ============================================================

PROJECT_ROOT = Path(__file__).resolve().parents[1]
MORAI_MODULE = PROJECT_ROOT / "third_party" / "MORAI-NetworkModule"
sys.path.insert(0, str(MORAI_MODULE))

from lib.network.UDP import Receiver, Sender
from lib.define.GPS import GPS
from lib.define.IMU import IMU
from lib.define.EgoCtrlCmd import EgoCtrlCmd


# ============================================================
# MORAI UDP settings
# ============================================================

GPS_IP = "127.0.0.1"
GPS_PORT = 9201

IMU_IP = "127.0.0.1"
IMU_PORT = 1111

CTRL_IP = "192.168.0.1"
CTRL_PORT = 9093


# ============================================================
# Coordinate system
# ============================================================

PROJ_UTM = Proj(
    proj="utm",
    zone=52,
    ellps="WGS84",
    preserve_units=False,
)

EAST_OFFSET = 302595.0
NORTH_OFFSET = 4124145.0


# ============================================================
# Proven lateral-control baseline - DO NOT TUNE IN THIS TEST
# ============================================================

VEHICLE_LENGTH = 1.0
LOOKAHEAD_DISTANCE = 2.5
LOCAL_PATH_SIZE = 50
CONTROL_HZ = 15.0

PATH_FILE = PROJECT_ROOT / "path" / "2026_molit_comp_global_path.txt"


# ============================================================
# Regulation-compliant longitudinal validation settings
# ============================================================

TARGET_SPEED_KMH = 5.0

# Acceleration remains proportional to target-speed error.
ACCEL_KP = 0.10
MAX_ACCEL_CMD = 0.30

# Wide coast band prevents GPS-speed noise near 5 km/h from repeatedly
# switching between throttle and brake.
ACCEL_CUTOFF_BELOW_TARGET_KMH = 0.20
BRAKE_TRIGGER_ABOVE_TARGET_KMH = 2.00

# Braking is intentionally weak in this low-speed validation stage.
BRAKE_KP = 0.04
MAX_BRAKE_CMD = 0.06
PATH_LOST_BRAKE_CMD = 0.12

# GPS speed estimator.
GPS_POSITION_CHANGE_EPS_M = 0.01
GPS_SPEED_WINDOW = 5
GPS_SPEED_MAX_VALID_KMH = 120.0
GPS_STATIONARY_TIMEOUT_SEC = 0.50

# Signed command slew-rate limiter (+ accel / - brake).
LONGITUDINAL_CMD_RATE_PER_SEC = 0.80


# ============================================================
# Utility functions
# ============================================================

def load_path(path_file):
    points = []
    with open(path_file, "r") as f:
        for line in f:
            values = line.split()
            if len(values) >= 2:
                points.append((float(values[0]), float(values[1])))

    if not points:
        raise RuntimeError("전역경로 파일에 waypoint가 없습니다.")

    return np.array(points, dtype=float)


def quaternion_to_yaw(x, y, z, w):
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def gps_to_local(lat, lon):
    if lat == 0.0 and lon == 0.0:
        return None

    utm_x, utm_y = PROJ_UTM(lon, lat)
    return utm_x - EAST_OFFSET, utm_y - NORTH_OFFSET


def find_nearest_waypoint(path, x, y):
    dx = path[:, 0] - x
    dy = path[:, 1] - y
    return int(np.argmin(dx * dx + dy * dy))


# ============================================================
# Pure Pursuit - unchanged from proven 5 km/h baseline
# ============================================================

def calculate_pure_pursuit(path, current_index, x, y, yaw):
    end_index = min(current_index + LOCAL_PATH_SIZE, len(path))

    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)

    target = None
    target_local_x = None
    target_local_y = None

    for i in range(current_index, end_index):
        dx = path[i, 0] - x
        dy = path[i, 1] - y

        local_x = cos_yaw * dx + sin_yaw * dy
        local_y = -sin_yaw * dx + cos_yaw * dy

        if local_x <= 0.0:
            continue

        if math.hypot(local_x, local_y) >= LOOKAHEAD_DISTANCE:
            target = i
            target_local_x = local_x
            target_local_y = local_y
            break

    if target is None:
        return None

    theta = math.atan2(target_local_y, target_local_x)
    steering = math.atan2(
        2.0 * VEHICLE_LENGTH * math.sin(theta),
        LOOKAHEAD_DISTANCE,
    )

    return {
        "target_index": target,
        "target_x": path[target, 0],
        "target_y": path[target, 1],
        "local_x": target_local_x,
        "local_y": target_local_y,
        "theta": theta,
        "steering": steering,
    }


# ============================================================
# GPS speed estimator
# ============================================================

class GpsSpeedEstimator:
    def __init__(self):
        self.prev_x = None
        self.prev_y = None
        self.prev_time = None
        self.speed_samples = deque(maxlen=GPS_SPEED_WINDOW)
        self.filtered_speed_kmh = 0.0
        self.raw_speed_kmh = 0.0

    def update(self, x, y, now):
        if self.prev_x is None:
            self.prev_x = x
            self.prev_y = y
            self.prev_time = now
            self.speed_samples.clear()
            self.speed_samples.append(0.0)
            self.filtered_speed_kmh = 0.0
            self.raw_speed_kmh = 0.0
            return 0.0, 0.0, True, "INIT"

        distance = math.hypot(x - self.prev_x, y - self.prev_y)

        if distance < GPS_POSITION_CHANGE_EPS_M:
            stale_time = now - self.prev_time
            if stale_time >= GPS_STATIONARY_TIMEOUT_SEC:
                self.prev_time = now
                self.raw_speed_kmh = 0.0
                self.speed_samples.clear()
                self.speed_samples.append(0.0)
                self.filtered_speed_kmh = 0.0
                return 0.0, 0.0, True, "STOP"

            return (
                self.filtered_speed_kmh,
                self.raw_speed_kmh,
                False,
                "HOLD",
            )

        dt = now - self.prev_time
        if dt <= 1e-4:
            return (
                self.filtered_speed_kmh,
                self.raw_speed_kmh,
                False,
                "HOLD",
            )

        raw_speed_kmh = (distance / dt) * 3.6

        self.prev_x = x
        self.prev_y = y
        self.prev_time = now

        if raw_speed_kmh > GPS_SPEED_MAX_VALID_KMH:
            return (
                self.filtered_speed_kmh,
                self.raw_speed_kmh,
                False,
                "REJECT",
            )

        self.raw_speed_kmh = raw_speed_kmh
        self.speed_samples.append(raw_speed_kmh)
        self.filtered_speed_kmh = sum(self.speed_samples) / len(self.speed_samples)

        return (
            self.filtered_speed_kmh,
            self.raw_speed_kmh,
            True,
            "MOVE",
        )


# ============================================================
# Low-speed accel / coast / brake policy
# ============================================================

def longitudinal_policy(target_kmh, current_kmh):
    """Return accel, brake, error, zone for temporary GPS-speed control."""
    error = target_kmh - current_kmh

    accel_cutoff = max(0.0, target_kmh - ACCEL_CUTOFF_BELOW_TARGET_KMH)
    brake_trigger = target_kmh + BRAKE_TRIGGER_ABOVE_TARGET_KMH

    if current_kmh < accel_cutoff:
        accel = min(MAX_ACCEL_CMD, max(0.0, ACCEL_KP * error))
        return accel, 0.0, error, "ACCEL"

    if current_kmh <= brake_trigger:
        return 0.0, 0.0, error, "COAST"

    overspeed = current_kmh - brake_trigger
    brake = min(MAX_BRAKE_CMD, max(0.0, BRAKE_KP * overspeed))
    return 0.0, brake, error, "BRAKE"


# ============================================================
# Longitudinal command rate limiter
# ============================================================

class LongitudinalCommandLimiter:
    def __init__(self, rate_per_sec):
        self.rate_per_sec = rate_per_sec
        self.signed_command = 0.0

    def reset(self):
        self.signed_command = 0.0

    def update(self, accel, brake, dt):
        desired = accel if accel > 0.0 else -brake

        max_delta = self.rate_per_sec * max(dt, 1e-3)
        delta = desired - self.signed_command
        delta = max(-max_delta, min(max_delta, delta))
        self.signed_command += delta

        self.signed_command = max(
            -PATH_LOST_BRAKE_CMD,
            min(MAX_ACCEL_CMD, self.signed_command),
        )

        if self.signed_command >= 0.0:
            return self.signed_command, 0.0, self.signed_command

        return 0.0, -self.signed_command, self.signed_command


# ============================================================
# Regulation-compliant stop command
# ============================================================

def send_stop(sender):
    cmd = EgoCtrlCmd()
    cmd.ctrl_mode = 2
    cmd.gear = 4
    cmd.cmd_type = 1
    cmd.accel = 0.0
    cmd.brake = 1.0
    cmd.steer = 0.0

    for _ in range(10):
        sender.send(cmd)
        time.sleep(0.05)


# ============================================================
# Main
# ============================================================

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--drive",
        action="store_true",
        help="규정 허용 cmd_type=1 CtrlCmd를 실제 MORAI로 송신",
    )
    parser.add_argument(
        "--target-speed",
        type=float,
        default=TARGET_SPEED_KMH,
        help="첫 검증용 목표속도(km/h), 기본값 5.0",
    )
    args = parser.parse_args()

    if args.target_speed < 0.0:
        raise ValueError("target speed는 0 이상이어야 합니다.")

    if args.target_speed > 10.0:
        raise ValueError(
            "초기 규정 준수 baseline은 10 km/h 이하만 허용합니다. "
            "5 km/h 완주 검증 후 속도 프로파일 단계에서 상향하세요."
        )

    accel_cutoff = max(
        0.0,
        args.target_speed - ACCEL_CUTOFF_BELOW_TARGET_KMH,
    )
    brake_trigger = args.target_speed + BRAKE_TRIGGER_ABOVE_TARGET_KMH

    print("==============================================")
    print(" MORAI REGULATION-COMPLIANT PURE PURSUIT")
    print("==============================================")
    print("[CTRL] ctrl_mode=2 / cmd_type=1 (accel, brake, steer)")
    print(f"[TARGET] {args.target_speed:.1f} km/h")
    print(
        f"[SPEED] GPS fresh-sample moving average "
        f"(window={GPS_SPEED_WINDOW})"
    )
    print(
        f"[POLICY] ACCEL < {accel_cutoff:.1f} / "
        f"COAST {accel_cutoff:.1f}~{brake_trigger:.1f} / "
        f"BRAKE > {brake_trigger:.1f} km/h"
    )
    print(
        f"[BRAKE] gentle cap={MAX_BRAKE_CMD:.2f} / "
        f"path-lost={PATH_LOST_BRAKE_CMD:.2f}"
    )
    print(
        f"[CMD] longitudinal rate limit = "
        f"{LONGITUDINAL_CMD_RATE_PER_SEC:.2f}/s"
    )
    print()

    print("[MODE] DRIVE" if args.drive else "[MODE] DRY-RUN")

    global_path = load_path(PATH_FILE)
    print(f"[PATH] {PATH_FILE}")
    print(f"[PATH] waypoint count = {len(global_path)}")
    print()

    gps_receiver = Receiver(GPS_IP, GPS_PORT, GPS())
    imu_receiver = Receiver(IMU_IP, IMU_PORT, IMU())
    ctrl_sender = Sender(CTRL_IP, CTRL_PORT) if args.drive else None

    speed_estimator = GpsSpeedEstimator()
    command_limiter = LongitudinalCommandLimiter(
        LONGITUDINAL_CMD_RATE_PER_SEC
    )

    period = 1.0 / CONTROL_HZ
    prev_control_time = time.time()

    desired_accel = 0.0
    desired_brake = 0.0
    speed_error = args.target_speed
    control_zone = "INIT"

    try:
        while True:
            loop_start = time.time()

            # GPS
            gps_data = gps_receiver.get_data()
            gps_data.parsing()
            lat = gps_data.gprmc.lat
            lon = gps_data.gprmc.lon

            position = gps_to_local(lat, lon)
            if position is None:
                print("[WAIT] GPS 데이터 대기 중...")
                time.sleep(0.1)
                continue

            current_x, current_y = position

            # IMU
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

            yaw = quaternion_to_yaw(
                imu_data.ori_x,
                imu_data.ori_y,
                imu_data.ori_z,
                imu_data.ori_w,
            )

            now = time.time()
            (
                current_speed_kmh,
                raw_speed_kmh,
                fresh_gps_speed,
                speed_state,
            ) = speed_estimator.update(current_x, current_y, now)

            nearest_index = find_nearest_waypoint(
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

            result = calculate_pure_pursuit(
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
                target_speed_kmh = args.target_speed

            dt_control = max(1e-3, now - prev_control_time)
            prev_control_time = now

            # Update the longitudinal decision only when a new speed sample is
            # available. Between GPS updates, hold the previous decision.
            if fresh_gps_speed:
                (
                    desired_accel,
                    desired_brake,
                    speed_error,
                    control_zone,
                ) = longitudinal_policy(
                    target_speed_kmh,
                    current_speed_kmh,
                )
            else:
                speed_error = target_speed_kmh - current_speed_kmh

            if result is None:
                desired_accel = 0.0
                desired_brake = PATH_LOST_BRAKE_CMD
                control_zone = "PATH_LOST"

            accel_cmd, brake_cmd, signed_cmd = command_limiter.update(
                desired_accel,
                desired_brake,
                dt_control,
            )

            print("\033[2J\033[H", end="")
            print("==============================================")
            print(" REGULATION-COMPLIANT PURE PURSUIT")
            print("==============================================")
            print(f"GPS       : {lat:.6f}, {lon:.6f}")
            print(f"POSITION  : x={current_x:.3f}, y={current_y:.3f}")
            print(f"YAW       : {math.degrees(yaw):.2f} deg")
            print()
            print(f"NEAREST   : index={nearest_index}")
            print(f"PATH POS  : x={nearest_x:.3f}, y={nearest_y:.3f}")
            print(f"PATH DIST : {nearest_distance:.3f} m")
            print()

            if result is None:
                print("LOOKAHEAD : NOT FOUND")
                print("STEERING  : 0.00000 rad")
            else:
                print(f"LOOKAHEAD : index={result['target_index']}")
                print(
                    f"TARGET    : x={result['target_x']:.3f}, "
                    f"y={result['target_y']:.3f}"
                )
                print(
                    f"LOCAL     : x={result['local_x']:.3f}, "
                    f"y={result['local_y']:.3f}"
                )
                print(f"THETA     : {math.degrees(result['theta']):.2f} deg")
                print(
                    f"STEERING  : {steering:.5f} rad "
                    f"({math.degrees(steering):.2f} deg)"
                )

            print()
            print(f"TARGET SPD: {target_speed_kmh:.2f} km/h")
            print(f"RAW GPS SPD: {raw_speed_kmh:.2f} km/h")
            print(f"GPS SPEED : {current_speed_kmh:.2f} km/h")
            print(f"GPS FRESH : {'YES' if fresh_gps_speed else 'NO'}")
            print(f"SPD STATE : {speed_state}")
            print(f"CTRL ZONE : {control_zone}")
            print(f"SPD ERROR : {speed_error:+.2f} km/h")
            print(
                f"DESIRED   : accel={desired_accel:.3f}, "
                f"brake={desired_brake:.3f}"
            )
            print(f"LONG CMD  : {signed_cmd:+.3f}")
            print(f"ACCEL     : {accel_cmd:.3f}")
            print(f"BRAKE     : {brake_cmd:.3f}")
            print("CMD MODE  : ctrl_mode=2 / cmd_type=1")
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
            print("규정 모드 정지 명령 송신 (cmd_type=1 / brake=1.0)")
            send_stop(ctrl_sender)

        print("프로그램 종료")


if __name__ == "__main__":
    main()
