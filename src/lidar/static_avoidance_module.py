#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""10/06 static avoidance pipeline without steering/throttle/brake/UDP control.

Usage from existing driving process:
    module = StaticAvoidanceModule()
    module.start()
    try:
        result = module.update()  # call repeatedly; returns None until fresh scan
        # result.status: CLEAR / AVOIDING / STOP_REQUIRED / SENSOR_UNAVAILABLE
        # result.local_path: numpy array of x,y,z (or None if unavailable)
        # Driving controller must decide safe stop, path tracking and actuator commands.
    finally:
        module.stop()

Do not run alongside another process binding the same GPS/IMU/LiDAR UDP ports.
"""
from dataclasses import dataclass
from pathlib import Path
import sys
import time
import numpy as np

LIDAR_DIR = Path(__file__).resolve().parent
SRC_DIR = LIDAR_DIR.parent
ROOT = SRC_DIR.parent
for p in (SRC_DIR, LIDAR_DIR):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

from morai_common import ensure_morai_import_path, load_network_config
ensure_morai_import_path()
from safe_udp_receiver import Receiver
from lib.define.GPS import GPS
from lib.define.IMU import IMU
from lidar_receiver import LidarReceiver
from object_detection.object_detector import ObjectDetector
from object_detection.tracker import ObjectTracker
from obstacle_adapter import tracked_list_to_planner_obstacles
from static_obstacle_planner import StaticObstaclePlanner, PlanResult
from lidar_ego_motion_test import read_ego_pose, calculate_ego_motion, convert_lidar_to_vehicle

GLOBAL_PATH = ROOT / 'path' / '2026_molit_comp_global_path.txt'
CONFIG = LIDAR_DIR / 'static_obstacle_config.json'

@dataclass
class AvoidanceOutput:
    status: str
    local_path: object
    obstacle_id: object = None
    avoidance_side: object = None
    recommended_speed_kph: float = 0.0
    reason: str = ''
    scan_timestamp: object = None
    detected_count: int = 0
    tracked_count: int = 0
    latch_state: str = ''


def _xy(path):
    arr = np.asarray(path, dtype=float)
    if arr.ndim != 2 or arr.shape[1] < 2 or len(arr) == 0:
        return np.empty((0, 2))
    return arr[:, :2]

class AvoidancePathLatch:
    """Retain accepted avoidance path, matching the 10/06 original behavior."""
    def __init__(self, global_path):
        self.global_xy = _xy(global_path)
        self.reset()

    def reset(self):
        self.active = False
        self.path = None
        self.obstacle_id = None
        self.side = None
        self.release_index = 0
        self.clear_count = 0

    def start(self, plan):
        self.path = np.asarray(plan.local_path, dtype=float).copy()
        local_xy = _xy(self.path)
        distances = np.linalg.norm(local_xy[:, None, :] - self.global_xy[None, :, :], axis=2).min(axis=1)
        shifted = np.flatnonzero(distances > 0.15)
        self.release_index = min(len(local_xy) - 1, int(shifted[-1]) + 3) if len(shifted) else len(local_xy) - 1
        self.obstacle_id = plan.obstacle_id
        self.side = plan.avoidance_side
        self.clear_count = 0
        self.active = True

    def apply(self, raw, ego_x, ego_y):
        if not self.active:
            if raw.status == 'AVOIDING':
                self.start(raw)
            return raw, 'RAW'
        local_xy = _xy(self.path)
        nearest = int(np.argmin(np.sum((local_xy - [ego_x, ego_y]) ** 2, axis=1)))
        if raw.status == 'STOP_REQUIRED' and raw.obstacle_id != self.obstacle_id:
            self.reset()
            return raw, 'STOP_REQUIRED_NEW_OBSTACLE'
        self.clear_count = self.clear_count + 1 if raw.status == 'CLEAR' else 0
        if self.clear_count >= 10:
            self.reset()
            return raw, 'RELEASED_BY_CLEAR'
        if nearest >= self.release_index:
            self.reset()
            return raw, 'RELEASED_BY_PATH'
        return PlanResult('AVOIDING', self.path.copy(), self.obstacle_id, self.side, 8.0,
                          'Latched avoidance path'), f'LATCHED {nearest}/{self.release_index}'

class StaticAvoidanceModule:
    def __init__(self, global_path_file=GLOBAL_PATH, config_file=CONFIG, sensor_timeout_s=0.5):
        path = np.loadtxt(str(global_path_file), dtype=float)
        if path.ndim != 2 or path.shape[1] < 2:
            raise ValueError('Global path requires x/y columns')
        if path.shape[1] == 2:
            path = np.column_stack((path, np.zeros(len(path))))
        self.global_path = path[:, :3]
        self.planner = StaticObstaclePlanner(self.global_path, str(config_file))
        self.latch = AvoidancePathLatch(self.global_path)
        cfg = load_network_config()
        self.gps = Receiver(cfg['gps']['ip'], int(cfg['gps']['port']), GPS())
        self.imu = Receiver(cfg['imu']['ip'], int(cfg['imu']['port']), IMU())
        self.lidar = LidarReceiver()
        self.detector = ObjectDetector(ground_z_threshold=-1.70, dbscan_eps=0.5,
            dbscan_min_samples=5, min_cluster_points=20, min_width=0.15,
            max_width=3.0, min_length=0.05, max_length=5.0)
        self.tracker = ObjectTracker(max_match_distance=1.0, velocity_window_size=5,
            dynamic_speed_threshold=0.5, dynamic_confirm_frames=3, max_missed_frames=3)
        self.prev_scan = None
        self.prev_pose = None
        self.last_scan = None
        self.last_fresh_wall = time.monotonic()
        self.timeout = sensor_timeout_s
        self.started = False

    def start(self):
        if not self.started:
            self.lidar.start()
            self.started = True
            self.last_fresh_wall = time.monotonic()

    def stop(self):
        if self.started:
            self.lidar.stop()
            self.started = False

    def _unavailable(self, reason):
        self.latch.reset()
        self.prev_pose = None
        self.prev_scan = None
        return AvoidanceOutput('SENSOR_UNAVAILABLE', None, reason=reason)

    def update(self):
        """Non-blocking. None means no new frame; caller must enforce its own watchdog."""
        if not self.started:
            raise RuntimeError('Call start() first')
        pose = read_ego_pose(self.gps, self.imu)
        if pose is None:
            return self._unavailable('GPS/IMU unavailable')
        latest = self.lidar.get_latest()
        if latest is None:
            if time.monotonic() - self.last_fresh_wall > self.timeout:
                return self._unavailable('LiDAR timeout')
            return None
        points, stats = latest
        if points is None or not stats.get('fresh', False):
            if time.monotonic() - self.last_fresh_wall > self.timeout:
                return self._unavailable('LiDAR stale')
            return None
        stamp = stats.get('timestamp')
        if stamp is None or stamp == self.last_scan:
            if time.monotonic() - self.last_fresh_wall > self.timeout:
                return self._unavailable('LiDAR timestamp stalled')
            return None
        dt = 0.1 if self.prev_scan is None else float(stamp - self.prev_scan)
        if not 0 < dt <= 1.0:
            dt = 0.1
        self.prev_scan = self.last_scan = stamp
        self.last_fresh_wall = time.monotonic()
        x, y, yaw = pose
        if self.prev_pose is None:
            df = dl = dyaw = 0.0
        else:
            _, _, df, dl, dyaw = calculate_ego_motion(*self.prev_pose, x, y, yaw)
        self.prev_pose = (x, y, yaw)
        vehicle_points = convert_lidar_to_vehicle(points)
        obstacles, _, _ = self.detector.detect(vehicle_points)
        tracked = self.tracker.update(obstacles, dt, delta_x=df, delta_y=dl, delta_yaw=dyaw)
        global_obstacles = tracked_list_to_planner_obstacles(tracked, ego_x=x, ego_y=y,
                                                              ego_yaw=yaw, confidence=1.0)
        raw = self.planner.plan(ego_x=x, ego_y=y, obstacles=global_obstacles)
        result, latch_state = self.latch.apply(raw, x, y)
        return AvoidanceOutput(status=result.status, local_path=np.asarray(result.local_path).copy(),
            obstacle_id=result.obstacle_id, avoidance_side=result.avoidance_side,
            recommended_speed_kph=float(result.recommended_speed_kph), reason=result.reason,
            scan_timestamp=stamp, detected_count=len(obstacles), tracked_count=len(tracked),
            latch_state=latch_state)


def main():
    module = StaticAvoidanceModule()
    module.start()
    print('[MONITOR] Static avoidance only. NO EgoCtrlCmd sent.')
    try:
        while True:
            output = module.update()
            if output is not None:
                n = len(output.local_path) if output.local_path is not None else 0
                print(f'{output.status:<19} points={n:3d} side={output.avoidance_side} '
                      f'objects={output.detected_count} latch={output.latch_state}')
            time.sleep(0.01)
    except KeyboardInterrupt:
        pass
    finally:
        module.stop()

if __name__ == '__main__':
    main()
