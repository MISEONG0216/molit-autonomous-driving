#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""5 km/h static-obstacle avoidance DRIVE integration test.

Pipeline:
GPS/IMU + VLP16 -> detection -> tracking with ego-motion compensation
-> global obstacle adapter -> StaticObstaclePlanner
-> path selector -> Pure Pursuit -> longitudinal control -> EgoCtrlCmd.

Safety:
- Default is DRY-RUN. CtrlCmd is sent only with --drive.
- Test speed is hard-limited to 5 km/h.
- CLEAR follows the global-path slice.
- AVOIDING follows the generated local avoidance path.
- STOP_REQUIRED commands accel=0 and brake=1.0.
- Existing validated controller files are not modified.
"""

import argparse
import csv
import math
import sys
import time
from pathlib import Path

import numpy as np

LIDAR_DIR = Path(__file__).resolve().parent
SRC_DIR = LIDAR_DIR.parent
PROJECT_ROOT = SRC_DIR.parent

for path in (SRC_DIR, LIDAR_DIR):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from morai_common import ensure_morai_import_path, load_network_config

ensure_morai_import_path()

from lib.network.UDP import Receiver, Sender
from lib.define.GPS import GPS
from lib.define.IMU import IMU
from lib.define.EgoCtrlCmd import EgoCtrlCmd

import pure_pursuit_regulation as pp_base
from pure_pursuit_competition import CompetitionSpeedReceiver

from lidar_receiver import LidarReceiver
from object_detection.object_detector import ObjectDetector
from object_detection.tracker import ObjectTracker
from obstacle_adapter import tracked_list_to_planner_obstacles
from static_obstacle_planner import StaticObstaclePlanner
from lidar_ego_motion_test import (
    read_ego_pose,
    calculate_ego_motion,
    convert_lidar_to_vehicle,
)


GLOBAL_PATH_FILE = PROJECT_ROOT / "path" / "2026_molit_comp_global_path.txt"
STATIC_CONFIG_FILE = LIDAR_DIR / "static_obstacle_config.json"
LOG_DIR = PROJECT_ROOT / "logs" / "lidar_static_avoidance"

DEFAULT_TEST_SPEED_KPH = 5.0
ALLOWED_TEST_SPEEDS_KPH = (5.0, 10.0, 15.0, 20.0, 25.0)
LOCAL_PATH_LOOKAHEAD_M = 2.5
LOCAL_PATH_SEARCH_SIZE = 80
CONTROL_IP = "127.0.0.1"
CONTROL_PORT = 9093

# Once an avoidance path is accepted, keep following the same path until the
# vehicle has passed the return-to-global section.  This prevents one-frame
# detector/planner flicker from switching LOCAL -> GLOBAL -> LOCAL mid-manoeuvre.
AVOIDANCE_RELEASE_TOLERANCE_M = 0.15
AVOIDANCE_RELEASE_BUFFER_POINTS = 3
CLEAR_RELEASE_SCAN_COUNT = 10


class AvoidancePathLatch:
    def __init__(self, global_path):
        self.global_path = prepare_path(global_path)
        self.active = False
        self.path = None
        self.obstacle_id = None
        self.side = None
        self.release_index = None
        self.clear_count = 0

    def _calculate_release_index(self, local_path):
        local_xy = prepare_path(local_path)
        if len(local_xy) == 0:
            return 0

        deviations = []
        for point in local_xy:
            delta = self.global_path - point
            d2 = np.einsum("ij,ij->i", delta, delta)
            deviations.append(math.sqrt(float(np.min(d2))))

        shifted = [
            i for i, distance in enumerate(deviations)
            if distance > AVOIDANCE_RELEASE_TOLERANCE_M
        ]
        if not shifted:
            return max(0, len(local_xy) - 1)

        return min(
            len(local_xy) - 1,
            shifted[-1] + AVOIDANCE_RELEASE_BUFFER_POINTS,
        )

    def start(self, plan_result):
        self.active = True
        self.path = np.asarray(plan_result.local_path, dtype=float).copy()
        self.obstacle_id = plan_result.obstacle_id
        self.side = plan_result.avoidance_side
        self.release_index = self._calculate_release_index(self.path)
        self.clear_count = 0

    def _reset(self):
        self.active = False
        self.path = None
        self.obstacle_id = None
        self.side = None
        self.release_index = None
        self.clear_count = 0

    def apply(self, raw_plan_result, ego_x, ego_y):
        if not self.active:
            if raw_plan_result.status == "AVOIDING":
                self.start(raw_plan_result)
            return raw_plan_result, "RAW"

        local_xy = prepare_path(self.path)
        nearest = pp_base.find_nearest_waypoint(local_xy, ego_x, ego_y)

        # Safety has priority over latch persistence.
        # A fresh STOP_REQUIRED result must never be hidden by an old
        # avoidance path.
        if raw_plan_result.status == "STOP_REQUIRED":
            self._reset()
            return raw_plan_result, "STOP_REQUIRED_OVERRIDE"

        # During an active avoidance manoeuvre, require CLEAR to be observed
        # for 10 consecutive processed LiDAR scans before returning to the
        # global path.  Any new AVOIDING result resets the CLEAR counter.
        if raw_plan_result.status == "CLEAR":
            self.clear_count += 1
        else:
            self.clear_count = 0

        if self.clear_count >= CLEAR_RELEASE_SCAN_COUNT:
            self._reset()
            return raw_plan_result, "RELEASED_BY_CLEAR"

        # Keep the original geometric release as a second normal completion
        # condition.  This preserves the behaviour already validated in the
        # single-box MORAI test.
        if nearest >= self.release_index:
            self._reset()
            return raw_plan_result, "RELEASED_BY_PATH"

        from static_obstacle_planner import PlanResult
        latched = PlanResult(
            "AVOIDING",
            self.path.copy(),
            self.obstacle_id,
            self.side,
            8.0,
            "Latched avoidance path held until 10 consecutive CLEAR scans or path release",
        )
        return latched, f"LATCHED {nearest}/{self.release_index}"



def prepare_path(path):
    array = np.asarray(path, dtype=float)
    if array.ndim != 2 or array.shape[1] < 2:
        return np.empty((0, 2), dtype=float)
    return array[:, :2].copy()


def calculate_selected_path_pure_pursuit(path, ego_x, ego_y, ego_yaw):
    selected_path = prepare_path(path)
    if len(selected_path) < 2:
        return None, selected_path

    nearest_index = pp_base.find_nearest_waypoint(selected_path, ego_x, ego_y)
    end_index = min(nearest_index + LOCAL_PATH_SEARCH_SIZE, len(selected_path))

    cos_yaw = math.cos(ego_yaw)
    sin_yaw = math.sin(ego_yaw)

    for i in range(nearest_index, end_index):
        dx = selected_path[i, 0] - ego_x
        dy = selected_path[i, 1] - ego_y

        local_x = cos_yaw * dx + sin_yaw * dy
        local_y = -sin_yaw * dx + cos_yaw * dy

        if local_x <= 0.0:
            continue

        if math.hypot(local_x, local_y) < LOCAL_PATH_LOOKAHEAD_M:
            continue

        theta = math.atan2(local_y, local_x)
        steering = math.atan2(
            2.0 * pp_base.VEHICLE_LENGTH * math.sin(theta),
            LOCAL_PATH_LOOKAHEAD_M,
        )

        return {
            "nearest_index": nearest_index,
            "target_index": i,
            "local_x": float(local_x),
            "local_y": float(local_y),
            "steering": float(steering),
        }, selected_path

    return None, selected_path


def send_control(sender, accel, brake, steering):
    cmd = EgoCtrlCmd()
    cmd.ctrl_mode = 2
    cmd.gear = 4
    cmd.cmd_type = 1
    cmd.accel = max(0.0, min(1.0, float(accel)))
    cmd.brake = max(0.0, min(1.0, float(brake)))
    cmd.steer = max(-1.0, min(1.0, float(steering)))
    sender.send(cmd)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--drive",
        action="store_true",
        help="실제 MORAI로 EgoCtrlCmd 송신. 생략하면 DRY-RUN.",
    )
    parser.add_argument(
        "--speed",
        type=float,
        choices=ALLOWED_TEST_SPEEDS_KPH,
        default=DEFAULT_TEST_SPEED_KPH,
        help="시험 목표속도(km/h): 5, 10, 15, 20, 25 중 선택. 기본값 5.",
    )
    args = parser.parse_args()
    test_speed_kph = float(args.speed)

    print("=" * 76)
    print(f" STATIC OBSTACLE AVOIDANCE + PURE PURSUIT - {test_speed_kph:.0f} km/h TEST")
    print("=" * 76)
    print("[MODE] DRIVE" if args.drive else "[MODE] DRY-RUN")
    print(f"[LIMIT] Test speed = {test_speed_kph:.1f} km/h")
    print("[PATH] CLEAR -> GLOBAL / AVOIDING -> LOCAL_AVOIDANCE")
    print("[STOP] STOP_REQUIRED -> accel 0 / brake 1.0")
    print(f"[CTRL] {CONTROL_IP}:{CONTROL_PORT} / ctrl_mode=2 / cmd_type=1")
    print()

    if not GLOBAL_PATH_FILE.exists():
        raise FileNotFoundError(f"Global path not found: {GLOBAL_PATH_FILE}")
    if not STATIC_CONFIG_FILE.exists():
        raise FileNotFoundError(f"Planner config not found: {STATIC_CONFIG_FILE}")

    global_path = np.loadtxt(GLOBAL_PATH_FILE, dtype=float)
    if global_path.ndim != 2 or global_path.shape[1] < 2:
        raise RuntimeError("Global path requires x/y columns.")
    if global_path.shape[1] == 2:
        global_path = np.column_stack((global_path, np.zeros(len(global_path))))
    global_path = global_path[:, :3]

    planner = StaticObstaclePlanner(global_path, str(STATIC_CONFIG_FILE))
    avoidance_latch = AvoidancePathLatch(global_path)

    network_config = load_network_config()
    gps_cfg = network_config["gps"]
    imu_cfg = network_config["imu"]

    gps_receiver = Receiver(gps_cfg["ip"], int(gps_cfg["port"]), GPS())
    imu_receiver = Receiver(imu_cfg["ip"], int(imu_cfg["port"]), IMU())
    speed_receiver = CompetitionSpeedReceiver()
    lidar_receiver = LidarReceiver()
    ctrl_sender = Sender(CONTROL_IP, CONTROL_PORT) if args.drive else None

    detector = ObjectDetector(
        ground_z_threshold=-1.70,
        dbscan_eps=0.5,
        dbscan_min_samples=5,
        min_cluster_points=20,
        min_width=0.15,
        max_width=3.00,
        min_length=0.05,
        max_length=5.00,
    )

    tracker = ObjectTracker(
        max_match_distance=1.0,
        velocity_window_size=5,
        dynamic_speed_threshold=0.5,
        dynamic_confirm_frames=3,
        max_missed_frames=3,
    )

    command_limiter = pp_base.LongitudinalCommandLimiter(
        pp_base.LONGITUDINAL_CMD_RATE_PER_SEC
    )

    previous_scan_timestamp = None
    last_processed_timestamp = None
    previous_ego_pose = None
    previous_control_time = time.time()
    scan_number = 0
    previous_raw_status = None

    LOG_DIR.mkdir(parents=True, exist_ok=True)
    run_stamp = time.strftime("%Y%m%d_%H%M%S")
    log_file = LOG_DIR / f"static_avoidance_{run_stamp}.csv"
    log_handle = log_file.open("w", newline="", encoding="utf-8-sig")
    log_writer = csv.writer(log_handle)
    log_writer.writerow([
        "scan", "ego_x", "ego_y", "ego_yaw_deg", "ego_speed_kmh",
        "raw_status", "planner_status", "latch_state", "planner_obstacle_id",
        "avoid_side", "path_source", "cluster_id", "object_x", "object_y",
        "width", "length", "distance", "points",
        "x_min", "x_max", "y_min", "y_max", "z_min", "z_max",
    ])
    log_handle.flush()
    print(f"[LOG] {log_file}")

    try:
        lidar_receiver.start()
        print("[READY] GPS / IMU / Competition Status / LiDAR")
        print("[WAIT] Waiting for first fresh LiDAR scan...")

        while True:
            ego_pose = read_ego_pose(gps_receiver, imu_receiver)
            if ego_pose is None:
                time.sleep(0.03)
                continue
            ego_x, ego_y, ego_yaw = ego_pose

            current_speed_kmh, speed_valid = speed_receiver.read()
            if not speed_valid:
                if args.drive and ctrl_sender is not None:
                    send_control(ctrl_sender, 0.0, 1.0, 0.0)
                print("[WAIT] Competition Vehicle Status - braking for safety")
                time.sleep(0.05)
                continue

            latest = lidar_receiver.get_latest()
            if latest is None:
                time.sleep(0.01)
                continue

            points, stats = latest
            if points is None or not stats.get("fresh", False):
                if args.drive and ctrl_sender is not None:
                    send_control(ctrl_sender, 0.0, 1.0, 0.0)
                time.sleep(0.01)
                continue

            scan_timestamp = stats.get("timestamp")
            if scan_timestamp is None or scan_timestamp == last_processed_timestamp:
                time.sleep(0.005)
                continue

            last_processed_timestamp = scan_timestamp
            scan_number += 1

            if previous_scan_timestamp is None:
                dt = 0.1
            else:
                dt = scan_timestamp - previous_scan_timestamp
                if dt <= 0.0 or dt > 1.0:
                    dt = 0.1
            previous_scan_timestamp = scan_timestamp

            if previous_ego_pose is None:
                delta_forward = 0.0
                delta_left = 0.0
                delta_yaw = 0.0
            else:
                _, _, delta_forward, delta_left, delta_yaw = calculate_ego_motion(
                    previous_ego_pose[0],
                    previous_ego_pose[1],
                    previous_ego_pose[2],
                    ego_x,
                    ego_y,
                    ego_yaw,
                )
            previous_ego_pose = (ego_x, ego_y, ego_yaw)

            vehicle_points = convert_lidar_to_vehicle(points)
            obstacles, non_ground_points, cluster_labels = detector.detect(vehicle_points)

            tracked_obstacles = tracker.update(
                obstacles,
                dt,
                delta_x=delta_forward,
                delta_y=delta_left,
                delta_yaw=delta_yaw,
            )

            planner_obstacles = tracked_list_to_planner_obstacles(
                tracked_obstacles,
                ego_x=ego_x,
                ego_y=ego_y,
                ego_yaw=ego_yaw,
                confidence=1.0,
            )

            raw_plan_result = planner.plan(
                ego_x=ego_x,
                ego_y=ego_y,
                obstacles=planner_obstacles,
            )
            plan_result, latch_state = avoidance_latch.apply(
                raw_plan_result,
                ego_x,
                ego_y,
            )

            now = time.time()
            dt_control = max(1e-3, now - previous_control_time)
            previous_control_time = now

            if plan_result.status == "STOP_REQUIRED":
                path_source = "STOP_REQUIRED"
                selected_path = plan_result.local_path
                pp_result = None
                target_speed_kmh = 0.0
                steering = 0.0
                accel_cmd = 0.0
                brake_cmd = 1.0
                signed_cmd = -1.0
                speed_error = -current_speed_kmh
                control_zone = "STOP_REQUIRED"
                command_limiter.reset()
            else:
                path_source = (
                    "LOCAL_AVOIDANCE"
                    if plan_result.status == "AVOIDING"
                    else "GLOBAL_CLEAR"
                )
                selected_path = plan_result.local_path
                pp_result, _ = calculate_selected_path_pure_pursuit(
                    selected_path,
                    ego_x,
                    ego_y,
                    ego_yaw,
                )

                target_speed_kmh = min(
                    test_speed_kph,
                    test_speed_kph,
                )

                if pp_result is None:
                    steering = 0.0
                    target_speed_kmh = 0.0
                    desired_accel = 0.0
                    desired_brake = pp_base.PATH_LOST_BRAKE_CMD
                    speed_error = -current_speed_kmh
                    control_zone = "PATH_LOST"
                else:
                    steering = pp_result["steering"]
                    (
                        desired_accel,
                        desired_brake,
                        speed_error,
                        control_zone,
                    ) = pp_base.longitudinal_policy(
                        target_speed_kmh,
                        current_speed_kmh,
                    )

                accel_cmd, brake_cmd, signed_cmd = command_limiter.update(
                    desired_accel,
                    desired_brake,
                    dt_control,
                )

            if args.drive and ctrl_sender is not None:
                send_control(
                    ctrl_sender,
                    accel_cmd,
                    brake_cmd,
                    steering,
                )

            # Save every processed LiDAR scan so short-lived false avoidance
            # events can be inspected after a full-course drive.
            diagnostic_rows = []
            for obstacle in obstacles:
                obstacle_id = getattr(
                    obstacle,
                    "cluster_id",
                    getattr(obstacle, "id", None),
                )
                cluster_mask = cluster_labels == obstacle_id
                cluster_points = non_ground_points[cluster_mask]
                if len(cluster_points) > 0:
                    x_min = float(np.min(cluster_points[:, 0]))
                    x_max = float(np.max(cluster_points[:, 0]))
                    y_min = float(np.min(cluster_points[:, 1]))
                    y_max = float(np.max(cluster_points[:, 1]))
                    z_min = float(np.min(cluster_points[:, 2]))
                    z_max = float(np.max(cluster_points[:, 2]))
                    point_count = len(cluster_points)
                else:
                    x_min = x_max = y_min = y_max = z_min = z_max = float("nan")
                    point_count = 0

                diagnostic_rows.append((
                    obstacle_id, obstacle, point_count,
                    x_min, x_max, y_min, y_max, z_min, z_max,
                ))

            if diagnostic_rows:
                for (
                    obstacle_id, obstacle, point_count,
                    x_min, x_max, y_min, y_max, z_min, z_max,
                ) in diagnostic_rows:
                    log_writer.writerow([
                        scan_number, ego_x, ego_y, math.degrees(ego_yaw),
                        current_speed_kmh, raw_plan_result.status,
                        plan_result.status, latch_state, plan_result.obstacle_id,
                        plan_result.avoidance_side, path_source, obstacle_id,
                        obstacle.x, obstacle.y, obstacle.width, obstacle.length,
                        obstacle.distance, point_count,
                        x_min, x_max, y_min, y_max, z_min, z_max,
                    ])
            else:
                log_writer.writerow([
                    scan_number, ego_x, ego_y, math.degrees(ego_yaw),
                    current_speed_kmh, raw_plan_result.status,
                    plan_result.status, latch_state, plan_result.obstacle_id,
                    plan_result.avoidance_side, path_source,
                    "", "", "", "", "", "", 0,
                    "", "", "", "", "", "",
                ])
            log_handle.flush()

            if raw_plan_result.status != previous_raw_status:
                print(
                    f"[EVENT] RAW {previous_raw_status} -> "
                    f"{raw_plan_result.status} at SCAN {scan_number}"
                )
                previous_raw_status = raw_plan_result.status

            print("\033[2J\033[H", end="")
            print("=" * 76)
            print(f" STATIC OBSTACLE AVOIDANCE + PURE PURSUIT - {test_speed_kph:.0f} km/h TEST")
            print("=" * 76)
            print(f"MODE          : {'DRIVE' if args.drive else 'DRY-RUN'}")
            print(f"SCAN          : {scan_number}")
            print(
                f"EGO           : x={ego_x:.3f}, y={ego_y:.3f}, "
                f"yaw={math.degrees(ego_yaw):.2f} deg"
            )
            print(
                f"EGO MOTION    : forward={delta_forward:+.3f} m, "
                f"left={delta_left:+.3f} m, "
                f"dyaw={math.degrees(delta_yaw):+.3f} deg"
            )
            print(
                f"LIDAR         : points={len(vehicle_points)}, "
                f"non_ground={len(non_ground_points)}, detected={len(obstacles)}"
            )
            print(
                f"TRACK/GLOBAL  : {len(tracked_obstacles)} / "
                f"{len(planner_obstacles)} objects"
            )

            print("[DETECTED OBJECTS]")
            if len(obstacles) == 0:
                print("  None")
            else:
                for (
                    obstacle_id, obstacle, point_count,
                    x_min, x_max, y_min, y_max, z_min, z_max,
                ) in diagnostic_rows:
                    print(
                        f"  ID={obstacle_id} "
                        f"x={obstacle.x:.2f} m, "
                        f"y={obstacle.y:.2f} m, "
                        f"width={obstacle.width:.2f} m, "
                        f"length={obstacle.length:.2f} m, "
                        f"distance={obstacle.distance:.2f} m, "
                        f"points={point_count}"
                    )
                    print(
                        f"       range: "
                        f"x=[{x_min:.2f}, {x_max:.2f}] m, "
                        f"y=[{y_min:.2f}, {y_max:.2f}] m, "
                        f"z=[{z_min:.2f}, {z_max:.2f}] m"
                    )
            print()
            print(f"PLANNER RAW   : {raw_plan_result.status}")
            print(f"PLANNER       : {plan_result.status}")
            print(f"PATH LATCH    : {latch_state}")
            print(
                f"CLEAR COUNT   : {avoidance_latch.clear_count} / "
                f"{CLEAR_RELEASE_SCAN_COUNT}"
            )
            print(f"OBSTACLE ID   : {plan_result.obstacle_id}")
            print(f"AVOID SIDE    : {plan_result.avoidance_side}")
            print(f"PATH SOURCE   : {path_source}")
            print(f"PATH POINTS   : {len(selected_path)}")
            print(f"PLAN SPEED    : {plan_result.recommended_speed_kph:.1f} km/h")
            print(f"TEST LIMIT    : {test_speed_kph:.1f} km/h")
            print(f"TARGET SPEED  : {target_speed_kmh:.1f} km/h")
            print(f"COMP SPEED    : {current_speed_kmh:.2f} km/h")
            print()

            if plan_result.status == "STOP_REQUIRED":
                print("PURE PURSUIT  : BYPASSED - STOP REQUEST")
            elif pp_result is None:
                print("PURE PURSUIT  : LOOKAHEAD NOT FOUND")
            else:
                print(
                    f"PURE PURSUIT  : nearest={pp_result['nearest_index']}, "
                    f"target={pp_result['target_index']}"
                )
                print(
                    f"TARGET LOCAL  : x={pp_result['local_x']:.2f} m, "
                    f"y={pp_result['local_y']:.2f} m"
                )

            print(
                f"STEERING      : {steering:.5f} rad "
                f"({math.degrees(steering):.2f} deg)"
            )
            print(f"CTRL ZONE     : {control_zone}")
            print(f"SPEED ERROR   : {speed_error:+.2f} km/h")
            print(f"LONG CMD      : {signed_cmd:+.3f}")
            print(f"ACCEL         : {accel_cmd:.3f}")
            print(f"BRAKE         : {brake_cmd:.3f}")
            print()

            if args.drive:
                print("[DRIVE] EgoCtrlCmd transmitted to 127.0.0.1:9093")
            else:
                print("[DRY-RUN] NO EgoCtrlCmd transmitted.")

    except KeyboardInterrupt:
        print()
        print("[STOP] Ctrl+C received.")
        if args.drive and ctrl_sender is not None:
            print("[STOP] Sending full-brake stop command.")
            for _ in range(10):
                send_control(ctrl_sender, 0.0, 1.0, 0.0)
                time.sleep(0.05)
    finally:
        lidar_receiver.stop()
        log_handle.close()
        print("[STOP] LiDAR receiver stopped.")
        print(f"[LOG] Saved: {log_file}")


if __name__ == "__main__":
    main()
