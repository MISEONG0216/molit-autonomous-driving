#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import math
import sys
import time
from pathlib import Path

import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[1]
SRC_DIR = PROJECT_ROOT / "src"
MORAI_MODULE = PROJECT_ROOT / "third_party" / "MORAI-NetworkModule"

sys.path.insert(0, str(SRC_DIR))
sys.path.insert(0, str(MORAI_MODULE))


from lib.network.UDP import Receiver, Sender
from lib.define.IMU import IMU
from lib.define.EgoCtrlCmd import EgoCtrlCmd
from lib.define.EgoVehicleStatus import EgoVehicleStatus

from lidar_receiver import MoraiLidarReceiver
import pure_pursuit_regulation as speed_control

from tunnel_avoidance import (
    TunnelAvoidancePlanner,
    filter_obstacle_points,
    front_obstacle_distance,
    TUNNEL_START_INDEX,
    TUNNEL_END_INDEX,
)


IMU_IP = "127.0.0.1"
IMU_PORT = 1111

STATUS_IP = "127.0.0.1"
STATUS_PORT = 1909

CTRL_IP = "127.0.0.1"
CTRL_PORT = 9093


PATH_FILE_1 = PROJECT_ROOT / "path" / "2026_molit_comp_global_path.txt"
PATH_FILE_2 = PROJECT_ROOT / "2026_molit_comp_global_path.txt"


CONTROL_HZ = 10.0

VEHICLE_LENGTH_M = 1.0
LOOKAHEAD_DISTANCE_M = 2.5
LOCAL_PATH_POINTS = 60

MAX_STEER_DEG = 35.0


NORMAL_SPEED_KMH = 5.0
AVOID_MAX_SPEED_KMH = 3.0
CRAWL_SPEED_KMH = 1.5

CLOSE_LATCH_DISTANCE_M = 4.0

REPLAN_CLEARANCE_M = 1.40
TRACKING_ERROR_SLOW_M = 1.00

MAX_TRACKING_YAW_ERROR_DEG = 35.0

MIN_VALID_SPEED_KMH = 0.03
STATUS_WAIT_TIMEOUT_S = 10.0

LIDAR_SCAN_DURATION_S = 0.08


def normalize_angle_deg(angle):
    while angle > 180.0:
        angle -= 360.0

    while angle < -180.0:
        angle += 360.0

    return angle


def yaw_difference_deg(a, b):
    return normalize_angle_deg(a - b)


def quaternion_to_yaw_deg(w, x, y, z):
    siny_cosp = 2.0 * (w * z + x * y)

    cosy_cosp = 1.0 - 2.0 * (
        y * y + z * z
    )

    yaw_rad = math.atan2(
        siny_cosp,
        cosy_cosp,
    )

    return math.degrees(yaw_rad)


def load_global_path():
    if PATH_FILE_1.exists():
        path_file = PATH_FILE_1

    elif PATH_FILE_2.exists():
        path_file = PATH_FILE_2

    else:
        raise FileNotFoundError(
            "global path 파일을 찾지 못했습니다."
        )

    points = []

    with open(
        path_file,
        "r",
        encoding="utf-8",
    ) as f:

        for line in f:
            line = line.strip()

            if not line:
                continue

            values = line.split()

            if len(values) < 2:
                continue

            x = float(values[0])
            y = float(values[1])

            points.append([x, y])

    path = np.asarray(
        points,
        dtype=np.float64,
    )

    if len(path) == 0:
        raise RuntimeError(
            "global path가 비어 있습니다."
        )

    print(f"[PATH] {path_file}")
    print(f"[PATH] points={len(path)}")

    return path


def nearest_path_index(
    path,
    x,
    y,
    hint_index=None,
    window=80,
):
    if len(path) == 0:
        return 0

    if hint_index is None:
        start = 0
        end = len(path)

    else:
        start = max(
            0,
            int(hint_index) - window,
        )

        end = min(
            len(path),
            int(hint_index) + window + 1,
        )

    section = path[start:end]

    dx = section[:, 0] - x
    dy = section[:, 1] - y

    dist2 = dx * dx + dy * dy

    return start + int(
        np.argmin(dist2)
    )


def distance_to_path(path, x, y):
    if path is None:
        return float("inf")

    if len(path) == 0:
        return float("inf")

    path = np.asarray(
        path,
        dtype=np.float64,
    )

    dx = path[:, 0] - x
    dy = path[:, 1] - y

    return float(
        np.min(
            np.hypot(dx, dy)
        )
    )


def path_heading_deg(path, x, y):
    if path is None:
        return None

    if len(path) < 2:
        return None

    path = np.asarray(
        path,
        dtype=np.float64,
    )

    idx = nearest_path_index(
        path,
        x,
        y,
    )

    if idx >= len(path) - 1:
        i0 = max(0, idx - 1)
        i1 = idx

    else:
        i0 = idx
        i1 = idx + 1

    dx = path[i1, 0] - path[i0, 0]
    dy = path[i1, 1] - path[i0, 1]

    if (
        abs(dx) < 1e-9
        and abs(dy) < 1e-9
    ):
        return None

    return math.degrees(
        math.atan2(dy, dx)
    )


def calculate_pure_pursuit(
    path,
    current_x,
    current_y,
    yaw_deg,
):
    if path is None or len(path) < 2:
        return 0.0

    path = np.asarray(
        path,
        dtype=np.float64,
    )

    nearest_idx = nearest_path_index(
        path,
        current_x,
        current_y,
    )

    target_idx = len(path) - 1

    for i in range(
        nearest_idx,
        len(path),
    ):
        dx = path[i, 0] - current_x
        dy = path[i, 1] - current_y

        distance = math.hypot(
            dx,
            dy,
        )

        if distance >= LOOKAHEAD_DISTANCE_M:
            target_idx = i
            break

    target_x = float(
        path[target_idx, 0]
    )

    target_y = float(
        path[target_idx, 1]
    )

    yaw_rad = math.radians(
        yaw_deg
    )

    dx = target_x - current_x
    dy = target_y - current_y

    local_x = (
        math.cos(yaw_rad) * dx
        + math.sin(yaw_rad) * dy
    )

    local_y = (
        -math.sin(yaw_rad) * dx
        + math.cos(yaw_rad) * dy
    )

    ld = math.hypot(
        local_x,
        local_y,
    )

    if ld < 0.01:
        return 0.0

    theta = math.atan2(
        local_y,
        local_x,
    )

    steering_rad = math.atan2(
        2.0
        * VEHICLE_LENGTH_M
        * math.sin(theta),
        ld,
    )

    return math.degrees(
        steering_rad
    )


def send_drive_command(
    sender,
    accel_cmd,
    brake_cmd,
    steer_deg,
):
    # cmd_type=1: explicit throttle/brake/steering inputs.
    # Keep the steering representation used by the validated main controller.
    steer_deg = max(
        -MAX_STEER_DEG,
        min(MAX_STEER_DEG, steer_deg),
    )

    cmd = EgoCtrlCmd()
    cmd.ctrl_mode = 2
    cmd.gear = 4
    cmd.cmd_type = 1

    cmd.accel = max(0.0, min(1.0, float(accel_cmd)))
    cmd.brake = max(0.0, min(1.0, float(brake_cmd)))
    cmd.steer = max(-1.0, min(1.0, math.radians(steer_deg)))

    sender.send(cmd)


def send_stop(sender):
    # Always stop through cmd_type=1, including on Ctrl+C or exceptions.
    cmd = EgoCtrlCmd()
    cmd.ctrl_mode = 2
    cmd.gear = 4
    cmd.cmd_type = 1
    cmd.accel = 0.0
    cmd.brake = 1.0
    cmd.steer = 0.0

    print("[STOP] cmd_type=1 / accel=0.0 / brake=1.0")

    for _ in range(15):
        sender.send(cmd)
        time.sleep(0.05)


def vehicle_status_ready(status):
    try:
        link_raw = bytes(
            status.link_id
        )

        link_id = link_raw.split(
            b"\x00"
        )[0]

    except Exception:
        link_id = b""

    if len(link_id) > 0:
        return True

    if int(status.ctrl_mode) != 0:
        return True

    if abs(float(status.yaw)) > 0.01:
        return True

    return False


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--index",
        type=int,
        default=3900,
    )

    parser.add_argument(
        "--drive",
        action="store_true",
    )

    args = parser.parse_args()

    start_index = int(
        args.index
    )

    drive_enabled = bool(
        args.drive
    )

    global_path = load_global_path()

    if (
        start_index < 0
        or start_index >= len(global_path)
    ):
        raise ValueError(
            f"잘못된 index: {start_index}"
        )

    print("=" * 95)
    print(" MORAI Tunnel Avoidance DRIVE Test V6.2")
    print("=" * 95)

    print(
        f"[MODE] "
        f"{'DRIVE' if drive_enabled else 'DRY-RUN'}"
    )

    print(
        f"[START INDEX] {start_index}"
    )

    print(
        f"[TUNNEL] "
        f"{TUNNEL_START_INDEX} ~ "
        f"{TUNNEL_END_INDEX}"
    )

    print(
        f"[VEHICLE STATUS] "
        f"{STATUS_IP}:{STATUS_PORT}"
    )

    print(
        "[LOCALIZATION] signed_vel + IMU yaw"
    )

    print(
        f"[NORMAL SPEED] "
        f"{NORMAL_SPEED_KMH:.1f} km/h"
    )

    print(
        f"[AVOID SPEED] "
        f"<= {AVOID_MAX_SPEED_KMH:.1f} km/h"
    )

    print(
        f"[CLOSE LATCH] "
        f"< {CLOSE_LATCH_DISTANCE_M:.1f} m "
        f"-> {CRAWL_SPEED_KMH:.1f} km/h"
    )

    print(
        f"[REPLAN] clearance < "
        f"{REPLAN_CLEARANCE_M:.2f} m"
    )

    print(
        "[SIDE LOCK] SAME SIDE ONLY"
    )

    print(
        "[YAW SAFETY] GLOBAL path only"
    )

    print("=" * 95)

    imu_receiver = Receiver(
        IMU_IP,
        IMU_PORT,
        IMU(),
    )

    status_receiver = Receiver(
        STATUS_IP,
        STATUS_PORT,
        EgoVehicleStatus(),
    )

    lidar_receiver = (
        MoraiLidarReceiver()
    )

    sender = (
        Sender(CTRL_IP, CTRL_PORT)
        if drive_enabled
        else None
    )

    planner = (
        TunnelAvoidancePlanner()
    )

    # Reuse the validated main-controller throttle/brake policy and
    # longitudinal command slew-rate limiter.
    command_limiter = speed_control.LongitudinalCommandLimiter(
        speed_control.LONGITUDINAL_CMD_RATE_PER_SEC
    )

    wait_start = time.monotonic()

    while True:
        imu = imu_receiver.get_data()

        status = (
            status_receiver.get_data()
        )

        imu_norm = math.sqrt(
            float(imu.ori_w) ** 2
            + float(imu.ori_x) ** 2
            + float(imu.ori_y) ** 2
            + float(imu.ori_z) ** 2
        )

        imu_ok = imu_norm > 0.5

        status_ok = vehicle_status_ready(
            status
        )

        if imu_ok and status_ok:
            yaw_deg = quaternion_to_yaw_deg(
                float(imu.ori_w),
                float(imu.ori_x),
                float(imu.ori_y),
                float(imu.ori_z),
            )

            print(
                f"[READY] IMU yaw={yaw_deg:.2f}"
            )

            print(
                "[READY] Vehicle Status received"
            )

            break

        if not imu_ok:
            print("[WAIT] IMU")

        if not status_ok:
            print(
                "[WAIT] Vehicle Status"
            )

        if (
            time.monotonic()
            - wait_start
            > STATUS_WAIT_TIMEOUT_S
        ):
            raise RuntimeError(
                "IMU 또는 Vehicle Status 수신 실패"
            )

        time.sleep(0.1)

    current_x = float(
        global_path[
            start_index,
            0
        ]
    )

    current_y = float(
        global_path[
            start_index,
            1
        ]
    )

    current_index = start_index

    total_travel_m = 0.0

    previous_time = (
        time.monotonic()
    )

    close_speed_latched = False

    previous_path_source = (
        "GLOBAL"
    )

    try:
        while True:
            loop_start = (
                time.monotonic()
            )

            imu = imu_receiver.get_data()

            yaw_deg = quaternion_to_yaw_deg(
                float(imu.ori_w),
                float(imu.ori_x),
                float(imu.ori_y),
                float(imu.ori_z),
            )

            yaw_rad = math.radians(
                yaw_deg
            )

            status = (
                status_receiver.get_data()
            )

            actual_speed_kmh = float(
                status.signed_vel
            )

            if (
                abs(actual_speed_kmh)
                < MIN_VALID_SPEED_KMH
            ):
                actual_speed_kmh = 0.0

            actual_speed_mps = (
                actual_speed_kmh
                / 3.6
            )

            now = time.monotonic()

            dt = (
                now - previous_time
            )

            previous_time = now

            dt = max(
                0.0,
                min(dt, 0.5),
            )

            ds = (
                actual_speed_mps
                * dt
            )

            current_x += (
                ds
                * math.cos(yaw_rad)
            )

            current_y += (
                ds
                * math.sin(yaw_rad)
            )

            total_travel_m += abs(ds)

            current_index = nearest_path_index(
                global_path,
                current_x,
                current_y,
                hint_index=current_index,
                window=80,
            )

            lidar_points = (
                lidar_receiver.receive_scan(
                    duration_s=LIDAR_SCAN_DURATION_S
                )
            )

            if lidar_points is None:
                lidar_points = np.empty(
                    (0, 3),
                    dtype=np.float32,
                )

            raw_count = len(
                lidar_points
            )

            try:
                obstacle_points = (
                    filter_obstacle_points(
                        lidar_points
                    )
                )

            except Exception as e:
                print(
                    f"[WARN] filter: {e}"
                )

                obstacle_points = np.empty(
                    (0, 2),
                    dtype=np.float32,
                )

            filtered_count = len(
                obstacle_points
            )

            try:
                front_distance = (
                    front_obstacle_distance(
                        obstacle_points
                    )
                )

            except Exception as e:
                print(
                    f"[WARN] front detection: {e}"
                )

                front_distance = None

            plan = planner.update(
                global_path,
                current_index,
                current_x,
                current_y,
                yaw_rad,
                lidar_points,
            )

            if not isinstance(
                plan,
                dict,
            ):
                raise RuntimeError(
                    f"planner.update() returned {type(plan)}"
                )

            active_path = plan.get(
                "path"
            )

            if (
                active_path is not None
                and len(active_path) >= 2
            ):
                tracking_path = np.asarray(
                    active_path,
                    dtype=np.float64,
                )

                path_source = (
                    "AVOIDANCE"
                )

            else:
                end_idx = min(
                    len(global_path),
                    current_index
                    + LOCAL_PATH_POINTS,
                )

                tracking_path = (
                    global_path[
                        current_index:
                        end_idx
                    ]
                )

                path_source = (
                    "GLOBAL"
                )

            tracking_error = (
                distance_to_path(
                    tracking_path,
                    current_x,
                    current_y,
                )
            )

            clearance = plan.get(
                "clearance"
            )

            replan = False
            replan_reason = None

            if (
                path_source
                == "AVOIDANCE"
                and clearance is not None
                and clearance
                < REPLAN_CLEARANCE_M
            ):
                try:
                    changed = (
                        planner.replan_same_side(
                            global_path,
                            current_index,
                            current_x,
                            current_y,
                            yaw_rad,
                            lidar_points,
                        )
                    )

                    if bool(changed):
                        replan = True

                        replan_reason = (
                            "LOW_CLEARANCE_SAME_SIDE"
                        )

                        plan = planner.update(
                            global_path,
                            current_index,
                            current_x,
                            current_y,
                            yaw_rad,
                            lidar_points,
                        )

                        active_path = (
                            plan.get(
                                "path"
                            )
                        )

                        if (
                            active_path is not None
                            and len(active_path) >= 2
                        ):
                            tracking_path = np.asarray(
                                active_path,
                                dtype=np.float64,
                            )

                            path_source = (
                                "AVOIDANCE"
                            )

                        clearance = plan.get(
                            "clearance"
                        )

                        tracking_error = (
                            distance_to_path(
                                tracking_path,
                                current_x,
                                current_y,
                            )
                        )

                except Exception as e:
                    print(
                        f"[WARN] replan_same_side: {e}"
                    )

            if (
                path_source
                == "AVOIDANCE"
                and front_distance
                is not None
                and front_distance
                < CLOSE_LATCH_DISTANCE_M
            ):
                close_speed_latched = (
                    True
                )

            if (
                previous_path_source
                == "AVOIDANCE"
                and path_source
                == "GLOBAL"
            ):
                close_speed_latched = (
                    False
                )

                print(
                    "[RECOVER] GLOBAL path 복귀"
                )

            previous_path_source = (
                path_source
            )

            tracking_heading = (
                path_heading_deg(
                    tracking_path,
                    current_x,
                    current_y,
                )
            )

            if tracking_heading is None:
                yaw_error = 0.0

            else:
                yaw_error = abs(
                    yaw_difference_deg(
                        yaw_deg,
                        tracking_heading,
                    )
                )

            raw_steer_deg = (
                calculate_pure_pursuit(
                    tracking_path,
                    current_x,
                    current_y,
                    yaw_deg,
                )
            )

            steer_deg = max(
                -MAX_STEER_DEG,
                min(
                    MAX_STEER_DEG,
                    raw_steer_deg,
                ),
            )

            limited = (
                abs(
                    raw_steer_deg
                    - steer_deg
                )
                > 1e-6
            )

            target_speed_kmh = (
                NORMAL_SPEED_KMH
            )

            if (
                path_source
                == "AVOIDANCE"
            ):
                target_speed_kmh = min(
                    target_speed_kmh,
                    AVOID_MAX_SPEED_KMH,
                )

                planner_cap = plan.get(
                    "speed_cap"
                )

                if planner_cap is not None:
                    target_speed_kmh = min(
                        target_speed_kmh,
                        float(planner_cap),
                    )

            if close_speed_latched:
                target_speed_kmh = min(
                    target_speed_kmh,
                    CRAWL_SPEED_KMH,
                )

            if (
                tracking_error
                > TRACKING_ERROR_SLOW_M
            ):
                target_speed_kmh = min(
                    target_speed_kmh,
                    CRAWL_SPEED_KMH,
                )

            # Convert km/h target + Competition Status speed to
            # cmd_type=1 accelerator and brake pedal commands.
            (
                desired_accel,
                desired_brake,
                speed_error,
                speed_zone,
            ) = speed_control.longitudinal_policy(
                target_speed_kmh,
                actual_speed_kmh,
            )

            # The baseline controller coasts up to 2 km/h above target.
            # In the 1.5-3 km/h tunnel avoidance stage, brake earlier.
            if speed_error < -0.25:
                desired_accel = 0.0
                desired_brake = max(
                    desired_brake,
                    min(0.12, 0.08 * (-speed_error)),
                )
                speed_zone = "TUNNEL_BRAKE"

            accel_cmd, brake_cmd, _ = command_limiter.update(
                desired_accel,
                desired_brake,
                max(dt, 1e-3),
            )

            control_state = "READY"
            send_allowed = True

            # Never continue blind if no LiDAR scan is received.
            if raw_count == 0:
                control_state = "LIDAR_SAFETY"
                send_allowed = False

            # 핵심 수정:
            # 회피 중에는 yaw 차이가 커질 수 있으므로
            # GLOBAL 경로 주행 중일 때만 yaw safety 적용
            if (
                path_source == "GLOBAL"
                and yaw_error
                > MAX_TRACKING_YAW_ERROR_DEG
            ):
                control_state = (
                    "YAW_SAFETY"
                )

                send_allowed = False

            print()

            print(
                f"[POSE] "
                f"index={current_index}"
                f" | x={current_x:.2f}"
                f" | y={current_y:.2f}"
                f" | travel={total_travel_m:.2f} m"
            )

            print(
                f"[SPEED] "
                f"actual={actual_speed_kmh:.3f} km/h"
                f" | dt={dt:.3f}"
                f" | ds={ds:.3f} m"
            )

            print(
                f"[YAW] "
                f"imu={yaw_deg:.2f}"
                f" | path={tracking_heading}"
                f" | error={yaw_error:.2f}"
            )

            print(
                f"[LIDAR] "
                f"raw={raw_count}"
                f" | filtered={filtered_count}"
                f" | front={front_distance}"
            )

            print(
                f"[PLAN] "
                f"state={plan.get('state')}"
                f" | path={path_source}"
                f" | offset={plan.get('offset')}"
                f" | clearance={clearance}"
                f" | tracking_error={tracking_error}"
                f" | side_locked={plan.get('side_locked')}"
                f" | replan={replan}"
                f" | reason={replan_reason}"
            )

            print(
                f"[CTRL] "
                f"state={control_state}"
                f" | raw_steer={raw_steer_deg:.2f} deg"
                f" | steer={steer_deg:.2f} deg"
                f" | limited={limited}"
                f" | target_speed={target_speed_kmh:.2f} km/h"
                f" | close_latch={close_speed_latched}"
                f" | send={drive_enabled and send_allowed}"
            )

            print(
                f"[LONGITUDINAL] "
                f"zone={speed_zone}"
                f" | error={speed_error:+.2f} km/h"
                f" | accel={accel_cmd:.3f}"
                f" | brake={brake_cmd:.3f}"
                f" | cmd_type=1"
            )

            print("-" * 95)

            if not send_allowed:
                command_limiter.reset()
                if drive_enabled:
                    send_drive_command(
                        sender,
                        0.0,
                        1.0,
                        0.0,
                    )
            elif drive_enabled:
                send_drive_command(
                    sender,
                    accel_cmd,
                    brake_cmd,
                    steer_deg,
                )

            elapsed = (
                time.monotonic()
                - loop_start
            )

            sleep_time = (
                1.0 / CONTROL_HZ
                - elapsed
            )

            if sleep_time > 0:
                time.sleep(
                    sleep_time
                )

    except KeyboardInterrupt:
        print()
        print("[STOP] Ctrl+C")

    finally:
        # Send braking commands on both normal Ctrl+C and errors.
        if drive_enabled and sender is not None:
            send_stop(sender)
        lidar_receiver.close()


if __name__ == "__main__":
    main()