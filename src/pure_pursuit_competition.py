#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Competition-ready Pure Pursuit entrypoint.

This keeps the proven lateral and longitudinal controller implementation in
pure_pursuit_regulation.py, but uses the competition-allowed Competition
Vehicle Status as the longitudinal speed source instead of GPS-displacement
speed estimation.

Network settings:
    GPS                         -> 127.0.0.1:9201
    IMU                         -> 127.0.0.1:1111
    Competition Vehicle Status  -> 127.0.0.1:1909
    Ego Ctrl Cmd                -> 127.0.0.1:9093

Competition Vehicle Status simulator ports:
    Host Port        1908
    Destination Port 1909
"""

import argparse
import math
import time

import pure_pursuit_regulation as base

from lib.network.UDP import Receiver, Sender
from lib.define.GPS import GPS
from lib.define.IMU import IMU
from lib.define.EgoCtrlCmd import EgoCtrlCmd
from lib.define.EgoVehicleStatus import EgoVehicleStatus


COMP_STATUS_IP = "127.0.0.1"
COMP_STATUS_PORT = 1909
COMP_STATUS_HEADER = b"#MoraiInfo$"
COMP_STATUS_DATA_LENGTH = 152


class CompetitionSpeedReceiver:
    """Read only the validated speed field from Competition Vehicle Status."""

    def __init__(self):
        self.receiver = Receiver(
            COMP_STATUS_IP,
            COMP_STATUS_PORT,
            EgoVehicleStatus(),
        )
        self.last_speed_kmh = 0.0
        self.valid = False

    def read(self):
        data = self.receiver.get_data()

        try:
            header = bytes(data.header).split(b"\x00", 1)[0]
            data_length = int(data.data_lenght)
        except Exception:
            return self.last_speed_kmh, False

        if header != COMP_STATUS_HEADER:
            return self.last_speed_kmh, False

        if data_length != COMP_STATUS_DATA_LENGTH:
            return self.last_speed_kmh, False

        speed_kmh = float(data.signed_vel)

        # MORAI can report a tiny negative floating-point value while stopped.
        if -0.05 < speed_kmh < 0.0:
            speed_kmh = 0.0

        self.last_speed_kmh = speed_kmh
        self.valid = True
        return speed_kmh, True


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
        default=base.TARGET_SPEED_KMH,
        help="현재 저속 검증 단계 목표속도(km/h), 기본값 5.0",
    )
    args = parser.parse_args()

    if args.target_speed < 0.0:
        raise ValueError("target speed는 0 이상이어야 합니다.")

    if args.target_speed > 10.0:
        raise ValueError(
            "현재 저속 검증 단계에서는 10 km/h 이하만 허용합니다. "
            "속도 프로파일 통합 후 상향하세요."
        )

    accel_cutoff = max(
        0.0,
        args.target_speed - base.ACCEL_CUTOFF_BELOW_TARGET_KMH,
    )
    brake_trigger = args.target_speed + base.BRAKE_TRIGGER_ABOVE_TARGET_KMH

    print("====================================================")
    print(" COMPETITION PURE PURSUIT - COMP STATUS SPEED")
    print("====================================================")
    print("[CTRL] ctrl_mode=2 / cmd_type=1 (accel, brake, steer)")
    print("[SPEED SOURCE] Competition Vehicle Status / signed_vel")
    print(f"[COMP UDP] {COMP_STATUS_IP}:{COMP_STATUS_PORT}")
    print(f"[TARGET] {args.target_speed:.1f} km/h")
    print(
        f"[POLICY] ACCEL < {accel_cutoff:.1f} / "
        f"COAST {accel_cutoff:.1f}~{brake_trigger:.1f} / "
        f"BRAKE > {brake_trigger:.1f} km/h"
    )
    print("[MODE] DRIVE" if args.drive else "[MODE] DRY-RUN")
    print()

    global_path = base.load_path(base.PATH_FILE)
    print(f"[PATH] {base.PATH_FILE}")
    print(f"[PATH] waypoint count = {len(global_path)}")
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

    desired_accel = 0.0
    desired_brake = 0.0
    speed_error = args.target_speed
    control_zone = "INIT"

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

            now = time.time()

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
                steering = 0.0
                target_speed_kmh = 0.0
            else:
                steering = result["steering"]
                target_speed_kmh = args.target_speed

            dt_control = max(1e-3, now - prev_control_time)
            prev_control_time = now

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

            accel_cmd, brake_cmd, signed_cmd = command_limiter.update(
                desired_accel,
                desired_brake,
                dt_control,
            )

            print("\033[2J\033[H", end="")
            print("====================================================")
            print(" COMPETITION PURE PURSUIT")
            print("====================================================")
            print(f"GPS        : {lat:.6f}, {lon:.6f}")
            print(f"POSITION   : x={current_x:.3f}, y={current_y:.3f}")
            print(f"YAW        : {math.degrees(yaw):.2f} deg")
            print()
            print(f"NEAREST    : index={nearest_index}")
            print(f"PATH POS   : x={nearest_x:.3f}, y={nearest_y:.3f}")
            print(f"PATH DIST  : {nearest_distance:.3f} m")
            print()

            if result is None:
                print("LOOKAHEAD  : NOT FOUND")
                print("STEERING   : 0.00000 rad")
            else:
                print(f"LOOKAHEAD  : index={result['target_index']}")
                print(
                    f"TARGET     : x={result['target_x']:.3f}, "
                    f"y={result['target_y']:.3f}"
                )
                print(
                    f"LOCAL      : x={result['local_x']:.3f}, "
                    f"y={result['local_y']:.3f}"
                )
                print(f"THETA      : {math.degrees(result['theta']):.2f} deg")
                print(
                    f"STEERING   : {steering:.5f} rad "
                    f"({math.degrees(steering):.2f} deg)"
                )

            print()
            print(f"TARGET SPD : {target_speed_kmh:.2f} km/h")
            print(f"COMP SPEED : {current_speed_kmh:.2f} km/h")
            print("SPEED SRC  : Competition Vehicle Status / signed_vel")
            print(f"CTRL ZONE  : {control_zone}")
            print(f"SPD ERROR  : {speed_error:+.2f} km/h")
            print(
                f"DESIRED    : accel={desired_accel:.3f}, "
                f"brake={desired_brake:.3f}"
            )
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
