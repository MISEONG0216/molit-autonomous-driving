#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Regulation-compliant Pure Pursuit baseline for the 2026 competition.

Goals of this file:
- Preserve the proven src/pure_pursuit_udp.py baseline unchanged.
- Keep the existing GPS + IMU + Pure Pursuit lateral-control logic.
- Change longitudinal control to the competition-allowed EgoCtrlCmd mode:
    ctrl_mode = 2
    cmd_type  = 1  (accel / brake / steer)
- Start again at 5 km/h before any speed optimization.

Important:
- Current speed feedback is estimated from GPS displacement so that this first
  compliant baseline does not depend on an undocumented Competition Vehicle
  Status binary parser.
- Once the official Competition Vehicle Status packet format/port is verified,
  its x-direction velocity should replace GPS-derived speed for final control.
- The lateral model intentionally keeps VEHICLE_LENGTH = 1.0 for this first
  migration test because that value already completed a full lap. The official
  vehicle wheelbase (3.0 m) will be tested separately after this version can
  complete a lap with cmd_type = 1.
"""

import argparse
import math
import sys
import time
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

CTRL_IP = "127.0.0.1"
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
# Proven lateral-control baseline
# ============================================================

# Keep the proven 5 km/h baseline geometry unchanged for the first test.
VEHICLE_LENGTH = 1.0
LOOKAHEAD_DISTANCE = 2.5
LOCAL_PATH_SIZE = 50
CONTROL_HZ = 15.0

PATH_FILE = PROJECT_ROOT / "path" / "2026_molit_comp_global_path.txt"


# ============================================================
# Regulation-compliant longitudinal baseline
# ============================================================

TARGET_SPEED_KMH = 5.0

# These are deliberately conservative starting values.
# Tune only after the first cmd_type=1 driving test.
KP = 0.10
KI = 0.025
KD = 0.00

SPEED_DEADBAND_KMH = 0.20
INTEGRAL_LIMIT = 20.0

MAX_ACCEL_CMD = 0.30
MAX_BRAKE_CMD = 0.45

# GPS speed estimator filtering.
SPEED_FILTER_ALPHA = 0.25
GPS_SPEED_MAX_VALID_KMH = 120.0


# ============================================================
# Utility functions
# ============================================================

def load_path(path_file):
    points = []

    with open(path_file, "r") as f:
        for line in f:
            values = line.split()
            if len(values) < 2:
                continue

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
# Pure Pursuit
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

        distance = math.hypot(local_x, local_y)

        if distance >= LOOKAHEAD_DISTANCE:
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
        self.filtered_speed_kmh = 0.0

    def update(self, x, y, now):
        if self.prev_x is None:
            self.prev_x = x
            self.prev_y = y
            self.prev_time = now
            return self.filtered_speed_kmh

        dt = now - self.prev_time
        if dt <= 1e-4:
            return self.filtered_speed_kmh

        distance = math.hypot(x - self.prev_x, y - self.prev_y)
        raw_speed_kmh = (distance / dt) * 3.6

        self.prev_x = x
        self.prev_y = y
        self.prev_time = now

        # Reject clearly invalid GPS spikes.
        if raw_speed_kmh > GPS_SPEED_MAX_VALID_KMH:
            return self.filtered_speed_kmh

        self.filtered_speed_kmh = (
            SPEED_FILTER_ALPHA * raw_speed_kmh
            + (1.0 - SPEED_FILTER_ALPHA) * self.filtered_speed_kmh
        )

        return self.filtered_speed_kmh


# ============================================================
# PID longitudinal controller
# ============================================================

class SpeedPID:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0.0
        self.prev_error = None

    def reset(self):
        self.integral = 0.0
        self.prev_error = None

    def update(self, target_kmh, current_kmh, dt):
        error = target_kmh - current_kmh

        if abs(error) <= SPEED_DEADBAND_KMH:
            # Reduce integral wind-up close to target speed.
            self.integral *= 0.95
            self.prev_error = error
            return 0.0, 0.0, error, 0.0

        self.integral += error * dt
        self.integral = max(-INTEGRAL_LIMIT, min(INTEGRAL_LIMIT, self.integral))

        derivative = 0.0
        if self.prev_error is not None and dt > 1e-4:
            derivative = (error - self.prev_error) / dt

        self.prev_error = error

        output = (
            self.kp * error
            + self.ki * self.integral
            + self.kd * derivative
        )

        if output >= 0.0:
            accel = min(MAX_ACCEL_CMD, output)
            brake = 0.0
        else:
            accel = 0.0
            brake = min(MAX_BRAKE_CMD, -output)

        return accel, brake, error, output


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

    # The first compliant test is intentionally capped at 10 km/h.
    # Remove/replace this guard only after the 5 km/h cmd_type=1 lap is proven.
    if args.target_speed > 10.0:
        raise ValueError(
            "초기 규정 준수 baseline은 10 km/h 이하만 허용합니다. "
            "5 km/h 완주 검증 후 속도 프로파일 단계에서 상향하세요."
        )

    print("==============================================")
    print(" MORAI REGULATION-COMPLIANT PURE PURSUIT")
    print("==============================================")
    print("[CTRL] ctrl_mode=2 / cmd_type=1 (accel, brake, steer)")
    print(f"[TARGET] {args.target_speed:.1f} km/h")
    print("[SPEED] GPS displacement estimate (temporary baseline)")
    print()

    if args.drive:
        print("[MODE] DRIVE")
    else:
        print("[MODE] DRY-RUN")

    global_path = load_path(PATH_FILE)
    print(f"[PATH] {PATH_FILE}")
    print(f"[PATH] waypoint count = {len(global_path)}")
    print()

    gps_receiver = Receiver(GPS_IP, GPS_PORT, GPS())
    imu_receiver = Receiver(IMU_IP, IMU_PORT, IMU())

    ctrl_sender = Sender(CTRL_IP, CTRL_PORT) if args.drive else None

    speed_estimator = GpsSpeedEstimator()
    speed_pid = SpeedPID(KP, KI, KD)

    period = 1.0 / CONTROL_HZ
    prev_control_time = time.time()

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
            current_speed_kmh = speed_estimator.update(current_x, current_y, now)

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

            accel_cmd, brake_cmd, speed_error, pid_output = speed_pid.update(
                target_speed_kmh,
                current_speed_kmh,
                dt_control,
            )

            if result is None:
                accel_cmd = 0.0
                brake_cmd = MAX_BRAKE_CMD

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
            print(f"GPS SPEED : {current_speed_kmh:.2f} km/h")
            print(f"SPD ERROR : {speed_error:+.2f} km/h")
            print(f"PID OUT   : {pid_output:+.3f}")
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
