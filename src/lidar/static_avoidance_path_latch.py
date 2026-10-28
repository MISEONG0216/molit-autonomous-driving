#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Lock the first accepted local avoidance path until geometric completion.

A tracking ID change is NOT a reason to abandon the accepted path.  On every
LiDAR scan, check all eligible tracked objects against the REMAINING path;
if an object overlaps that corridor, request a stop without discarding the
latched route.  This module never transmits vehicle controls.
"""
import math
import numpy as np

from static_obstacle_planner import PlanResult

AVOIDANCE_RELEASE_TOLERANCE_M = 0.15
AVOIDANCE_RELEASE_BUFFER_POINTS = 3


def prepare_path(path):
    array = np.asarray(path, dtype=float)
    if array.ndim != 2 or array.shape[1] < 2:
        return np.empty((0, 2), dtype=float)
    return array[:, :2].copy()


def _distance_to_polyline(point, polyline):
    """Minimum centre-to-segment distance, not just waypoint distance."""
    point = np.asarray(point, dtype=float)
    if len(polyline) == 0:
        return math.inf
    if len(polyline) == 1:
        return float(np.linalg.norm(point - polyline[0]))
    starts, ends = polyline[:-1], polyline[1:]
    vectors = ends - starts
    lens2 = np.einsum("ij,ij->i", vectors, vectors)
    lens2 = np.maximum(lens2, 1e-12)
    t = np.clip(np.einsum("ij,ij->i", point - starts, vectors) / lens2, 0.0, 1.0)
    foot = starts + t[:, None] * vectors
    return float(np.min(np.linalg.norm(foot - point, axis=1)))


class AvoidancePathLatch:
    def __init__(
        self,
        global_path,
        vehicle_width_m=1.8,
        safety_margin_m=0.4,
        min_obstacle_width_m=0.6,
        min_obstacle_height_m=0.2,
    ):
        self.global_path = prepare_path(global_path)
        self.vehicle_width_m = float(vehicle_width_m)
        self.safety_margin_m = float(safety_margin_m)
        self.min_obstacle_width_m = float(min_obstacle_width_m)
        self.min_obstacle_height_m = float(min_obstacle_height_m)
        self.active = False
        self.path = None
        self.obstacle_id = None
        self.side = None
        self.release_index = None

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
        local_path = np.asarray(plan_result.local_path, dtype=float)
        if local_path.ndim != 2 or len(local_path) < 2:
            raise ValueError("Cannot latch an invalid avoidance path")
        self.active = True
        self.path = local_path.copy()
        self.obstacle_id = plan_result.obstacle_id
        self.side = plan_result.avoidance_side
        self.release_index = self._calculate_release_index(self.path)

    def _reset(self):
        self.active = False
        self.path = None
        self.obstacle_id = None
        self.side = None
        self.release_index = None

    def _blocking_obstacle(self, ego_x, ego_y, nearest, obstacles):
        """Check tracked objects against the not-yet-traversed locked path.

        Uses the same width-based circular footprint as the source planner.
        Its geometric approximation is not proof that a path is road-legal.
        """
        xy = prepare_path(self.path)
        remaining = xy[nearest:self.release_index + 1]
        if len(remaining) == 0:
            return None
        ego = np.array([ego_x, ego_y], dtype=float)
        # Include travel from ego position to the next path sample.
        remaining = np.vstack((ego, remaining))
        for obs in obstacles:
            try:
                x, y = float(obs.x), float(obs.y)
                width = float(obs.width)
                height = float(getattr(obs, "height", 0.0))
                confidence = float(obs.confidence)
            except (AttributeError, ValueError, TypeError):
                continue
            if not all(math.isfinite(v) for v in (x, y, width, height, confidence)):
                continue
            if confidence < 0.5 or width < self.min_obstacle_width_m:
                continue
            if height < self.min_obstacle_height_m:
                continue
            # Do not exclude dynamic detections: an obstacle blocking this
            # path must cause a stop, even if its tracker label is unstable.
            safe_radius = (
                self.vehicle_width_m / 2.0
                + width / 2.0
                + self.safety_margin_m
            )
            if _distance_to_polyline((x, y), remaining) < safe_radius:
                return obs
        return None

    def apply(self, raw_plan_result, ego_x, ego_y, obstacles=None):
        if not self.active:
            if raw_plan_result.status == "AVOIDING":
                self.start(raw_plan_result)
                return raw_plan_result, "LOCKED_NEW_PATH"
            return raw_plan_result, "RAW"

        xy = prepare_path(self.path)
        nearest = int(np.argmin(np.sum(
            (xy - np.array([ego_x, ego_y])) ** 2, axis=1
        )))

        # Release ONLY after reaching the initial path's geometric end of
        # avoidance. A transient CLEAR (including ID churn) must not release it.
        if nearest >= self.release_index:
            self._reset()
            return raw_plan_result, "RELEASED_BY_PATH"

        if obstacles is None:
            # A caller without obstacle geometry cannot justify overriding
            # a newly reported STOP_REQUIRED. Production always passes it.
            if (raw_plan_result.status == "STOP_REQUIRED"
                    and raw_plan_result.obstacle_id != self.obstacle_id):
                return raw_plan_result, "STOP_REQUIRED_UNVERIFIED"
        else:
            blocking = self._blocking_obstacle(ego_x, ego_y, nearest, obstacles)
            if blocking is not None:
                # Keep latch active: if the blockage disappears we can resume
                # the original route without generating a different one.
                return PlanResult(
                    "STOP_REQUIRED", np.empty((0, 3), dtype=float),
                    getattr(blocking, "id", None), self.side, 0.0,
                    "Tracked object overlaps locked avoidance path",
                ), "STOP_REQUIRED_PATH_BLOCKED"

        latched = PlanResult(
            "AVOIDING", self.path.copy(), self.obstacle_id, self.side, 8.0,
            "Original avoidance path locked until geometric release",
        )
        # A fresh raw STOP from a different object is not itself a reason to
        # discard a proven-clear locked path. The geometry check above decides.
        if raw_plan_result.status == "STOP_REQUIRED":
            return latched, f"LOCKED_RAW_STOP_IGNORED {nearest}/{self.release_index}"
        return latched, f"LOCKED {nearest}/{self.release_index}"
