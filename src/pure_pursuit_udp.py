#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import time
import math
import argparse
import socket
from pathlib import Path

import numpy as np
from pyproj import Proj


# ============================================================
# 경로 설정
# ============================================================

PROJECT_ROOT = Path(__file__).resolve().parents[1]

MORAI_MODULE = PROJECT_ROOT / "third_party" / "MORAI-NetworkModule"
sys.path.insert(0, str(MORAI_MODULE))

from lib.network.UDP import Receiver, Sender
from lib.define.GPS import GPS
from lib.define.IMU import IMU
from lib.define.EgoCtrlCmd import EgoCtrlCmd


# ============================================================
# MORAI UDP 설정
# ============================================================

GPS_IP = "127.0.0.1"
GPS_PORT = 9201

IMU_IP = "127.0.0.1"
IMU_PORT = 1111

CTRL_IP = "127.0.0.1"
CTRL_PORT = 9093

# traffic_light_detector.py가 보내는 로컬 판단값 수신
SIGNAL_IP = "127.0.0.1"
SIGNAL_PORT = 9300
SIGNAL_TIMEOUT_SEC = 5.0


# ============================================================
# 좌표계 설정
# 기존 ROS gpsimu_parser.py 방식
# ============================================================

# WGS84 -> UTM Zone 52
PROJ_UTM = Proj(
    proj="utm",
    zone=52,
    ellps="WGS84",
    preserve_units=False
)

# 대회 K-City MORAI Map Offset
EAST_OFFSET = 302595.0
NORTH_OFFSET = 4124145.0


# ============================================================
# Pure Pursuit 설정
# 기존 성공 ROS 코드 기준
# ============================================================

VEHICLE_LENGTH = 1.0
LOOKAHEAD_DISTANCE = 2.5
LOCAL_PATH_SIZE = 50

# 처음 실제 주행 테스트는 저속으로 수행
TARGET_SPEED_KMH = 5.0

CONTROL_HZ = 15.0

PATH_FILE = (
    PROJECT_ROOT
    / "path"
    / "2026_molit_comp_global_path.txt"
)


# ============================================================
# 전역경로 로드
# ============================================================

def load_path(path_file):
    points = []

    with open(path_file, "r") as f:
        for line in f:
            values = line.split()

            if len(values) < 2:
                continue

            x = float(values[0])
            y = float(values[1])

            points.append((x, y))

    if not points:
        raise RuntimeError("전역경로 파일에 waypoint가 없습니다.")

    return np.array(points, dtype=float)


# ============================================================
# IMU quaternion -> yaw
# ============================================================

def quaternion_to_yaw(x, y, z, w):
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)

    return math.atan2(siny_cosp, cosy_cosp)


# ============================================================
# GPS -> MORAI Local X,Y
# ============================================================

def gps_to_local(lat, lon):

    if lat == 0.0 and lon == 0.0:
        return None

    utm_x, utm_y = PROJ_UTM(lon, lat)

    local_x = utm_x - EAST_OFFSET
    local_y = utm_y - NORTH_OFFSET

    return local_x, local_y


# ============================================================
# 가장 가까운 waypoint
# ============================================================

def find_nearest_waypoint(path, x, y):
    dx = path[:, 0] - x
    dy = path[:, 1] - y

    distance_squared = dx * dx + dy * dy

    return int(np.argmin(distance_squared))


# ============================================================
# Pure Pursuit
# ============================================================

def calculate_pure_pursuit(path, current_index, x, y, yaw):

    end_index = min(
        current_index + LOCAL_PATH_SIZE,
        len(path)
    )

    # global -> vehicle local 변환
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)

    target = None
    target_local_x = None
    target_local_y = None

    for i in range(current_index, end_index):

        path_x = path[i, 0]
        path_y = path[i, 1]

        dx = path_x - x
        dy = path_y - y

        # 기존 ROS det_t와 동일한 global -> local 변환
        local_x = cos_yaw * dx + sin_yaw * dy
        local_y = -sin_yaw * dx + cos_yaw * dy

        # 차량 앞쪽 점만 사용
        if local_x <= 0.0:
            continue

        distance = math.sqrt(
            local_x * local_x +
            local_y * local_y
        )

        if distance >= LOOKAHEAD_DISTANCE:
            target = i
            target_local_x = local_x
            target_local_y = local_y
            break

    if target is None:
        return None

    theta = math.atan2(
        target_local_y,
        target_local_x
    )

    steering = math.atan2(
        2.0 * VEHICLE_LENGTH * math.sin(theta),
        LOOKAHEAD_DISTANCE
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
# 정지 명령
# ============================================================

def send_stop(sender):

    cmd = EgoCtrlCmd()

    cmd.ctrl_mode = 2
    cmd.gear = 4
    cmd.cmd_type = 2

    cmd.velocity = 0.0
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
        help="실제 CtrlCmd를 MORAI로 송신"
    )

    parser.add_argument(
        "--traffic-light",
        action="store_true",
        help="차량 신호등 판단(STOP/GO)을 속도 제어에 적용"
    )

    args = parser.parse_args()

    print("==============================================")
    print(" MORAI UDP Pure Pursuit")
    print("==============================================")

    if args.drive:
        print("[MODE] DRIVE")
        print("실제 차량 제어 명령을 송신합니다.")
    else:
        print("[MODE] DRY-RUN")
        print("제어 명령을 송신하지 않습니다.")

    if args.traffic_light:
        print(f"[SIGNAL] traffic-light decision {SIGNAL_IP}:{SIGNAL_PORT}")

    print()

    print("[PATH]", PATH_FILE)

    global_path = load_path(PATH_FILE)

    print(
        f"[PATH] waypoint 개수 = {len(global_path)}"
    )

    print(
        f"[PATH] 시작점 = "
        f"({global_path[0,0]:.3f}, "
        f"{global_path[0,1]:.3f})"
    )

    print()

    gps_receiver = Receiver(
        GPS_IP,
        GPS_PORT,
        GPS()
    )

    imu_receiver = Receiver(
        IMU_IP,
        IMU_PORT,
        IMU()
    )

    ctrl_sender = None

    signal_receiver = None
    signal_state = "UNKNOWN"
    signal_stop_latched = False
    signal_last_update = 0.0

    if args.traffic_light:
        signal_receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        signal_receiver.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        signal_receiver.bind((SIGNAL_IP, SIGNAL_PORT))
        signal_receiver.setblocking(False)

    if args.drive:
        ctrl_sender = Sender(
            CTRL_IP,
            CTRL_PORT
        )

    period = 1.0 / CONTROL_HZ

    try:

        while True:

            loop_start = time.time()

            # ------------------------------------------------
            # 차량 신호등 판단
            # ------------------------------------------------

            if signal_receiver is not None:
                while True:
                    try:
                        raw_signal, _ = signal_receiver.recvfrom(64)
                        received_signal = raw_signal.decode("ascii", errors="ignore").strip().upper()
                        if received_signal in ("STOP", "GO", "GO_LEFT", "UNKNOWN"):
                            signal_state = received_signal
                            signal_last_update = time.time()
                            if signal_state == "STOP":
                                signal_stop_latched = True
                            elif signal_state in ("GO", "GO_LEFT"):
                                signal_stop_latched = False
                    except BlockingIOError:
                        break

            signal_link_ok = (
                not args.traffic_light
                or (
                    signal_last_update > 0.0
                    and time.time() - signal_last_update <= SIGNAL_TIMEOUT_SEC
                )
            )

            # ------------------------------------------------
            # GPS
            # ------------------------------------------------

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


            # ------------------------------------------------
            # IMU
            # ------------------------------------------------

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
                imu_data.ori_w
            )


            # ------------------------------------------------
            # 현재 waypoint
            # ------------------------------------------------

            nearest_index = find_nearest_waypoint(
                global_path,
                current_x,
                current_y
            )

            nearest_x = global_path[nearest_index, 0]
            nearest_y = global_path[nearest_index, 1]

            nearest_distance = math.hypot(
                nearest_x - current_x,
                nearest_y - current_y
            )


            # ------------------------------------------------
            # Pure Pursuit 계산
            # ------------------------------------------------

            result = calculate_pure_pursuit(
                global_path,
                nearest_index,
                current_x,
                current_y,
                yaw
            )


            # ------------------------------------------------
            # 출력
            # ------------------------------------------------

            print("\033[2J\033[H", end="")

            print("==============================================")
            print(" MORAI UDP PURE PURSUIT")
            print("==============================================")

            print(
                f"GPS       : "
                f"{lat:.6f}, {lon:.6f}"
            )

            print(
                f"POSITION  : "
                f"x={current_x:.3f}, "
                f"y={current_y:.3f}"
            )

            print(
                f"YAW       : "
                f"{math.degrees(yaw):.2f} deg"
            )

            print()

            print(
                f"NEAREST   : "
                f"index={nearest_index}"
            )

            print(
                f"PATH POS  : "
                f"x={nearest_x:.3f}, "
                f"y={nearest_y:.3f}"
            )

            print(
                f"PATH DIST : "
                f"{nearest_distance:.3f} m"
            )

            print()

            if result is None:

                print("LOOKAHEAD : NOT FOUND")
                print("STEERING  : 0.0000")

                steering = 0.0
                velocity = 0.0

            else:

                steering = result["steering"]
                velocity = TARGET_SPEED_KMH

                # 적색/황색 정지는 확실한 녹색이 올 때까지 유지한다.
                if args.traffic_light and (signal_stop_latched or not signal_link_ok):
                    velocity = 0.0

                print(
                    f"LOOKAHEAD : "
                    f"index={result['target_index']}"
                )

                print(
                    f"TARGET    : "
                    f"x={result['target_x']:.3f}, "
                    f"y={result['target_y']:.3f}"
                )

                print(
                    f"LOCAL     : "
                    f"x={result['local_x']:.3f}, "
                    f"y={result['local_y']:.3f}"
                )

                print(
                    f"THETA     : "
                    f"{math.degrees(result['theta']):.2f} deg"
                )

                print(
                    f"STEERING  : "
                    f"{steering:.5f} rad "
                    f"({math.degrees(steering):.2f} deg)"
                )

            print()

            if args.traffic_light:
                signal_age = time.time() - signal_last_update if signal_last_update else float("inf")
                age_text = "WAIT" if not math.isfinite(signal_age) else f"{signal_age:.1f}s"
                print(f"SIGNAL    : {signal_state} (age={age_text})")
                print(f"SIGNAL LINK: {'OK' if signal_link_ok else 'WAIT/STALE'}")
                print(f"STOP LATCH: {'ON' if signal_stop_latched else 'OFF'}")
                print()

            if args.drive:
                print(
                    f"[DRIVE] velocity="
                    f"{velocity:.1f} km/h"
                )
            else:
                print("[DRY-RUN] CtrlCmd 송신 안 함")


            # ------------------------------------------------
            # 실제 제어
            # ------------------------------------------------

            if args.drive:

                cmd = EgoCtrlCmd()

                cmd.ctrl_mode = 2
                cmd.gear = 4
                cmd.cmd_type = 2

                cmd.velocity = velocity

                # UDP EgoCtrlCmd는 -1 ~ 1 범위
                cmd.steer = max(
                    -1.0,
                    min(1.0, steering)
                )

                ctrl_sender.send(cmd)


            # ------------------------------------------------
            # 15 Hz 유지
            # ------------------------------------------------

            elapsed = time.time() - loop_start

            if elapsed < period:
                time.sleep(period - elapsed)


    except KeyboardInterrupt:

        print()
        print("종료 중...")

        if args.drive and ctrl_sender is not None:
            print("정지 명령 송신")
            send_stop(ctrl_sender)

        if signal_receiver is not None:
            signal_receiver.close()

        print("프로그램 종료")


if __name__ == "__main__":
    main()
