#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import time
import math
import argparse
import socket
import json
from pathlib import Path

import numpy as np
from pyproj import Proj


# ============================================================
# 寃쎈줈 ?ㅼ젙
# ============================================================

PROJECT_ROOT = Path(__file__).resolve().parents[1]

MORAI_MODULE = PROJECT_ROOT / "third_party" / "MORAI-NetworkModule"
sys.path.insert(0, str(MORAI_MODULE))

from lib.network.UDP import Receiver, Sender
from lib.define.GPS import GPS
from lib.define.IMU import IMU
from lib.define.EgoCtrlCmd import EgoCtrlCmd


# ============================================================
# MORAI UDP ?ㅼ젙
# ============================================================

GPS_IP = "127.0.0.1"
GPS_PORT = 9201

IMU_IP = "127.0.0.1"
IMU_PORT = 1111

CTRL_IP = "127.0.0.1"
CTRL_PORT = 9093

# traffic_light_detector.py媛 蹂대궡??濡쒖뺄 ?먮떒媛??섏떊
SIGNAL_IP = "127.0.0.1"
SIGNAL_PORT = 9300
SIGNAL_TIMEOUT_SEC = 1.2
STOPLINE_TIMEOUT_SEC = 0.45

# Signal hysteresis. A transient false GREEN must not release a RED stop.
GO_RELEASE_CONFIRM_FRAMES = 8


# ============================================================
# 醫뚰몴怨??ㅼ젙
# ============================================================

PROJ_UTM = Proj(
    proj="utm",
    zone=52,
    ellps="WGS84",
    preserve_units=False
)

EAST_OFFSET = 302595.0
NORTH_OFFSET = 4124145.0


# ============================================================
# Pure Pursuit / stop-line control ?ㅼ젙
# ============================================================

VEHICLE_LENGTH = 1.0
LOOKAHEAD_DISTANCE = 2.5
LOCAL_PATH_SIZE = 50

TARGET_SPEED_KMH = 10.0
APPROACH_SPEED_KMH = 3.0
CONTROL_HZ = 15.0

# Camera stop-line y-ratio control.
# A one-time calibration file is created at the verified stop position.
STOPLINE_CALIBRATION_FILE = PROJECT_ROOT / "config" / "stopline_calibration.json"
DEFAULT_STOPLINE_TARGET_RATIO = 0.82

# Start slowing before the calibrated stop position. This is image-space,
# so it is reused automatically at every intersection with the same camera.
SLOWDOWN_MARGIN_RATIO = 0.13

# Trigger zero-speed slightly BEFORE the calibrated static stop position to
# compensate for command/vehicle response delay. Tune this single global value
# if the vehicle consistently stops too early/late at all intersections.
STOP_TRIGGER_EARLY_RATIO = 0.015

PATH_FILE = (
    PROJECT_ROOT
    / "path"
    / "2026_molit_comp_global_path.txt"
)


# ============================================================
# Utility
# ============================================================

def load_stopline_target_ratio():
    try:
        with open(STOPLINE_CALIBRATION_FILE, "r", encoding="utf-8") as f:
            data = json.load(f)
        ratio = float(data["target_y_ratio"])
        if 0.55 <= ratio <= 0.95:
            return ratio, True
    except Exception:
        pass
    return DEFAULT_STOPLINE_TARGET_RATIO, False


def load_path(path_file):
    points = []
    with open(path_file, "r") as f:
        for line in f:
            values = line.split()
            if len(values) < 2:
                continue
            points.append((float(values[0]), float(values[1])))
    if not points:
        raise RuntimeError("?꾩뿭寃쎈줈 ?뚯씪??waypoint媛 ?놁뒿?덈떎.")
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


def calculate_pure_pursuit(path, current_index, x, y, yaw):
    end_index = min(current_index + LOCAL_PATH_SIZE, len(path))
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)

    for i in range(current_index, end_index):
        path_x, path_y = path[i, 0], path[i, 1]
        dx, dy = path_x - x, path_y - y
        local_x = cos_yaw * dx + sin_yaw * dy
        local_y = -sin_yaw * dx + cos_yaw * dy
        if local_x <= 0.0:
            continue
        distance = math.hypot(local_x, local_y)
        if distance >= LOOKAHEAD_DISTANCE:
            theta = math.atan2(local_y, local_x)
            steering = math.atan2(
                2.0 * VEHICLE_LENGTH * math.sin(theta),
                LOOKAHEAD_DISTANCE
            )
            return {
                "target_index": i,
                "target_x": path_x,
                "target_y": path_y,
                "local_x": local_x,
                "local_y": local_y,
                "theta": theta,
                "steering": steering,
            }
    return None


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


def parse_vision_message(raw):
    """Backward compatible with both 'STOP' and 'STOP|0.81|0.9'."""
    text = raw.decode("ascii", errors="ignore").strip().upper()
    parts = text.split("|")
    signal = parts[0] if parts else "UNKNOWN"
    if signal not in ("STOP", "GO", "GO_LEFT", "UNKNOWN"):
        signal = "UNKNOWN"

    stopline_y = None
    stopline_conf = 0.0
    if len(parts) >= 2:
        try:
            value = float(parts[1])
            if 0.0 <= value <= 1.0:
                stopline_y = value
        except Exception:
            pass
    if len(parts) >= 3:
        try:
            stopline_conf = max(0.0, min(1.0, float(parts[2])))
        except Exception:
            pass
    return signal, stopline_y, stopline_conf


# ============================================================
# Main
# ============================================================

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--drive", action="store_true", help="?ㅼ젣 CtrlCmd瑜?MORAI濡??≪떊")
    parser.add_argument(
        "--traffic-light",
        action="store_true",
        help="李⑤웾 ?좏샇 + 移대찓???뺤????먮룞 ?뺤감瑜??곸슜"
    )
    args = parser.parse_args()

    stop_target_ratio, stop_calibrated = load_stopline_target_ratio()
    slow_trigger_ratio = max(0.45, stop_target_ratio - SLOWDOWN_MARGIN_RATIO)
    stop_trigger_ratio = max(slow_trigger_ratio + 0.02,
                             stop_target_ratio - STOP_TRIGGER_EARLY_RATIO)

    print("==============================================")
    print(" MORAI UDP Pure Pursuit + Auto Stop Line")
    print("==============================================")
    print("[MODE]", "DRIVE" if args.drive else "DRY-RUN")
    if args.traffic_light:
        print(f"[VISION] {SIGNAL_IP}:{SIGNAL_PORT}")
        print(
            f"[STOPLINE] target={stop_target_ratio:.4f} "
            f"slow>={slow_trigger_ratio:.4f} stop>={stop_trigger_ratio:.4f} "
            f"({'CALIBRATED' if stop_calibrated else 'DEFAULT'})"
        )
        if not stop_calibrated:
            print("[WARN] Run 'run_windows.bat stopline-calibrate' once at the verified stop position.")
    print("[PATH]", PATH_FILE)

    global_path = load_path(PATH_FILE)
    print(f"[PATH] waypoint 媛쒖닔 = {len(global_path)}")

    gps_receiver = Receiver(GPS_IP, GPS_PORT, GPS())
    imu_receiver = Receiver(IMU_IP, IMU_PORT, IMU())

    ctrl_sender = Sender(CTRL_IP, CTRL_PORT) if args.drive else None

    signal_receiver = None
    signal_state = "UNKNOWN"
    signal_last_update = 0.0
    stopline_y = None
    stopline_conf = 0.0
    stopline_last_update = 0.0

    # State machine: no per-intersection waypoint list is used.
    stopline_armed = False
    intersection_stop_hold = False

    # Once RED is confirmed, latch it so brief UNKNOWN/GREEN misclassifications
    # cannot make the vehicle pass the red light. Release only after several
    # consecutive GO/GO_LEFT decisions.
    red_latched = False
    go_release_count = 0

    if args.traffic_light:
        signal_receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        signal_receiver.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        signal_receiver.bind((SIGNAL_IP, SIGNAL_PORT))
        signal_receiver.setblocking(False)

    period = 1.0 / CONTROL_HZ

    try:
        while True:
            loop_start = time.time()

            # ------------------------------------------------
            # Vision decision receiver
            # ------------------------------------------------
            if signal_receiver is not None:
                while True:
                    try:
                        raw, _ = signal_receiver.recvfrom(128)
                        new_signal, new_stopline_y, new_stopline_conf = parse_vision_message(raw)
                        signal_state = new_signal
                        signal_last_update = time.time()

                        if new_stopline_y is not None:
                            stopline_y = new_stopline_y
                            stopline_conf = new_stopline_conf
                            stopline_last_update = time.time()
                    except BlockingIOError:
                        break

            now = time.time()
            signal_link_ok = (
                not args.traffic_light
                or (signal_last_update > 0.0 and now - signal_last_update <= SIGNAL_TIMEOUT_SEC)
            )
            stopline_fresh = (
                stopline_last_update > 0.0
                and now - stopline_last_update <= STOPLINE_TIMEOUT_SEC
            )

            # ------------------------------------------------
            # Signal hysteresis / RED latch
            # ------------------------------------------------
            if args.traffic_light and signal_link_ok:
                if signal_state == "STOP":
                    red_latched = True
                    go_release_count = 0

                elif signal_state in ("GO", "GO_LEFT"):
                    if red_latched:
                        go_release_count += 1
                        if go_release_count >= GO_RELEASE_CONFIRM_FRAMES:
                            red_latched = False
                            intersection_stop_hold = False
                            stopline_armed = False
                            go_release_count = 0
                    else:
                        go_release_count = 0

                else:
                    # UNKNOWN does not cancel a latched RED.
                    go_release_count = 0

            # ------------------------------------------------
            # GPS
            # ------------------------------------------------
            gps_data = gps_receiver.get_data()
            gps_data.parsing()
            lat = gps_data.gprmc.lat
            lon = gps_data.gprmc.lon
            position = gps_to_local(lat, lon)
            if position is None:
                print("[WAIT] GPS ?곗씠???湲?以?..")
                time.sleep(0.1)
                continue
            current_x, current_y = position

            # ------------------------------------------------
            # IMU
            # ------------------------------------------------
            imu_data = imu_receiver.get_data()
            if (imu_data.ori_w == 0.0 and imu_data.ori_x == 0.0
                    and imu_data.ori_y == 0.0 and imu_data.ori_z == 0.0):
                print("[WAIT] IMU ?곗씠???湲?以?..")
                time.sleep(0.1)
                continue

            yaw = quaternion_to_yaw(
                imu_data.ori_x, imu_data.ori_y,
                imu_data.ori_z, imu_data.ori_w
            )

            # ------------------------------------------------
            # Pure Pursuit
            # ------------------------------------------------
            nearest_index = find_nearest_waypoint(global_path, current_x, current_y)
            nearest_x, nearest_y = global_path[nearest_index]
            nearest_distance = math.hypot(nearest_x - current_x, nearest_y - current_y)
            result = calculate_pure_pursuit(
                global_path, nearest_index, current_x, current_y, yaw
            )

            if result is None:
                steering = 0.0
                base_velocity = 0.0
            else:
                steering = result["steering"]
                base_velocity = TARGET_SPEED_KMH

            velocity = base_velocity
            control_state = "CRUISE"

            # ------------------------------------------------
            # Multi-intersection GPS traffic-light stop logic
            # Verified stop indices:
            # 414, 630, 879, 1114, 1234
            #
            # Camera -> STOP / GO / GO_LEFT / UNKNOWN
            # GPS    -> exact stopping position

            if args.traffic_light:
                STOP_INDICES = [414, 630, 879, 1114, 1234]

                SLOWDOWN_OFFSET = 20
                MISSION_START_OFFSET = 25
                MISSION_END_OFFSET = 15

                active_stop_index = None

                for stop_idx in STOP_INDICES:
                    if (
                        stop_idx - MISSION_START_OFFSET
                        <= nearest_index
                        <= stop_idx + MISSION_END_OFFSET
                    ):
                        active_stop_index = stop_idx
                        break

                if active_stop_index is not None:

                    slowdown_index = (
                        active_stop_index - SLOWDOWN_OFFSET
                    )

                    # Already stopped: hold until GO confirmation releases it.
                    if intersection_stop_hold:
                        velocity = 0.0
                        steering = 0.0

                        if red_latched:
                            control_state = (
                                f"HOLD_RED_{active_stop_index}"
                            )
                        else:
                            control_state = "HOLD_RELEASE_WAIT"

                    # RED / STOP confirmed.
                    elif red_latched:

                        # Stop at the registered GPS waypoint.
                        if nearest_index >= active_stop_index:
                            intersection_stop_hold = True
                            velocity = 0.0
                            steering = 0.0
                            control_state = (
                                f"STOP_GPS_{active_stop_index}"
                            )

                        # Slow approach before the stop point.
                        elif nearest_index >= slowdown_index:
                            velocity = min(
                                base_velocity,
                                APPROACH_SPEED_KMH
                            )
                            control_state = (
                                f"APPROACH_RED_{active_stop_index}"
                            )

                        else:
                            velocity = base_velocity
                            control_state = (
                                f"RED_FAR_{active_stop_index}"
                            )

                    elif not signal_link_ok:
                        velocity = base_velocity
                        control_state = "VISION_WAIT"

                    elif signal_state == "UNKNOWN":
                        velocity = base_velocity
                        control_state = "UNKNOWN"

                    # GO / GO_LEFT
                    else:
                        velocity = base_velocity
                        control_state = signal_state

                else:
                    velocity = base_velocity
                    control_state = "CRUISE"

            # Console output
            # ------------------------------------------------
            print("\033[2J\033[H", end="")
            print("==============================================")
            print(" MORAI PURE PURSUIT + AUTO STOP LINE")
            print("==============================================")
            print(f"POSITION  : x={current_x:.3f}, y={current_y:.3f}")
            print(f"YAW       : {math.degrees(yaw):.2f} deg")
            print(f"NEAREST   : index={nearest_index}")
            print(f"PATH DIST : {nearest_distance:.3f} m")
            print()

            if result is None:
                print("LOOKAHEAD : NOT FOUND")
                print("STEERING  : 0.0000")
            else:
                print(f"LOOKAHEAD : index={result['target_index']}")
                print(f"STEERING  : {steering:.5f} rad ({math.degrees(steering):.2f} deg)")

            if args.traffic_light:
                signal_age = now - signal_last_update if signal_last_update else float("inf")
                line_age = now - stopline_last_update if stopline_last_update else float("inf")
                signal_age_text = "WAIT" if not math.isfinite(signal_age) else f"{signal_age:.2f}s"
                line_age_text = "WAIT" if not math.isfinite(line_age) else f"{line_age:.2f}s"
                line_text = "NONE" if not stopline_fresh or stopline_y is None else f"{stopline_y:.4f}"
                print()
                print(f"SIGNAL     : {signal_state} (age={signal_age_text})")
                print(f"STOPLINE   : y={line_text} conf={stopline_conf:.2f} age={line_age_text}")
                print(f"TARGET     : slow>={slow_trigger_ratio:.4f} stop>={stop_trigger_ratio:.4f}")
                print(f"ARMED/HOLD : {stopline_armed}/{intersection_stop_hold}")
                print(f"RED LATCH  : {red_latched} (GO confirm {go_release_count}/{GO_RELEASE_CONFIRM_FRAMES})")
                print(f"STATE      : {control_state}")

            print()
            if args.drive:
                print(f"[DRIVE] velocity={velocity:.1f} km/h")
            else:
                print(f"[DRY-RUN] velocity={velocity:.1f} km/h / CtrlCmd not sent")

            # ------------------------------------------------
            # Actual control
            # ------------------------------------------------
            if args.drive:
                cmd = EgoCtrlCmd()
                cmd.ctrl_mode = 2
                cmd.gear = 4
                cmd.cmd_type = 2
                cmd.velocity = velocity
                cmd.steer = max(-1.0, min(1.0, steering))
                ctrl_sender.send(cmd)

            elapsed = time.time() - loop_start
            if elapsed < period:
                time.sleep(period - elapsed)

    except KeyboardInterrupt:
        print("\n醫낅즺 以?..")
        if args.drive and ctrl_sender is not None:
            print("?뺤? 紐낅졊 ?≪떊")
            send_stop(ctrl_sender)
        if signal_receiver is not None:
            signal_receiver.close()
        print("?꾨줈洹몃옩 醫낅즺")


if __name__ == "__main__":
    main()



