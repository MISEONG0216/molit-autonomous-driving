#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Source-preserving avoidance path latch. No vehicle command sender.

Extracted from lidar_object_detection/src/lidar/lidar_static_avoidance_drive_5kph.py
so the perception worker can reuse the original accepted-path hold/release logic.
"""
import math
import numpy as np

from static_obstacle_planner import PlanResult

# Kept exactly equal to the validated 5-km/h source constants.
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
        nearest = int(np.argmin(np.sum((local_xy - np.array([ego_x, ego_y])) ** 2, axis=1)))

        # Safety still has priority for a different/new obstacle.
        # However, once an avoidance path has already been accepted and
        # latched, replanning the SAME tracked obstacle from a much closer
        # ego position can make every newly generated candidate look unsafe.
        # In that specific case, keep following the previously accepted path
        # instead of throwing it away and braking in front of the same box.
        if raw_plan_result.status == "STOP_REQUIRED":
            same_obstacle = (
                self.obstacle_id is not None
                and raw_plan_result.obstacle_id is not None
                and raw_plan_result.obstacle_id == self.obstacle_id
            )
            if not same_obstacle:
                self._reset()
                return raw_plan_result, "STOP_REQUIRED_NEW_OBSTACLE"

            self.clear_count = 0

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

        latched = PlanResult(
            "AVOIDING",
            self.path.copy(),
            self.obstacle_id,
            self.side,
            8.0,
            "Latched avoidance path held until 10 consecutive CLEAR scans or path release",
        )
        if raw_plan_result.status == "STOP_REQUIRED":
            return latched, (
                f"LATCHED_SAME_OBSTACLE_STOP {nearest}/{self.release_index}"
            )
        return latched, f"LATCHED {nearest}/{self.release_index}"



def prepare_path(path):
    array = np.asarray(path, dtype=float)
    if array.ndim != 2 or array.shape[1] < 2:
        return np.empty((0, 2), dtype=float)
    return array[:, :2].copy()



