#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Dynamic-obstacle risk prediction.

Input:
    Tracked LiDAR obstacles from object_detection.tracker.ObjectTracker.

This module does NOT receive LiDAR UDP packets, cluster point clouds, or send
vehicle-control commands.  Detection/tracking stays in modules #1-#3.

Coordinate frame expected from #3:
    +X: vehicle forward
    +Y: vehicle left
    +Z: up

The first version intentionally preserves the teammate module's core idea:
- dynamic object already inside the ego-path corridor -> risk
- dynamic object outside the corridor but moving toward it -> predict entry time
- entry within a configured horizon -> risk

All tracked dynamic objects are evaluated, not only the nearest detection.
"""

from dataclasses import dataclass
from typing import Optional, Sequence
import math


@dataclass
class DynamicObstacleResult:
    status: str
    obstacle_id: Optional[int]
    distance_m: Optional[float]
    time_to_path_s: Optional[float]
    path_state: str
    prediction_state: str
    reason: str


class DynamicObstaclePlanner:
    def __init__(
        self,
        path_y_min=-1.5,
        path_y_max=1.5,
        prediction_time_s=5.0,
        min_forward_m=0.0,
        max_forward_m=20.0,
        min_lateral_speed_mps=0.01,
    ):
        self.path_y_min = float(path_y_min)
        self.path_y_max = float(path_y_max)
        self.prediction_time_s = float(prediction_time_s)
        self.min_forward_m = float(min_forward_m)
        self.max_forward_m = float(max_forward_m)
        self.min_lateral_speed_mps = float(min_lateral_speed_mps)

    def check_path(self, center_y):
        if self.path_y_min <= center_y <= self.path_y_max:
            return "IN_PATH"
        return "OUT_OF_PATH"

    def predict_path_entry(self, center_y, velocity_y):
        if self.path_y_min <= center_y <= self.path_y_max:
            return "IN_PATH", 0.0

        if abs(velocity_y) < self.min_lateral_speed_mps:
            return "NO_LATERAL_MOTION", None

        # Object is on the left (+Y). It must move toward -Y.
        if center_y > self.path_y_max:
            if velocity_y < 0.0:
                time_to_path = (self.path_y_max - center_y) / velocity_y
                if time_to_path >= 0.0:
                    return "APPROACHING_PATH", time_to_path
            return "MOVING_AWAY", None

        # Object is on the right (-Y). It must move toward +Y.
        if center_y < self.path_y_min:
            if velocity_y > 0.0:
                time_to_path = (self.path_y_min - center_y) / velocity_y
                if time_to_path >= 0.0:
                    return "APPROACHING_PATH", time_to_path
            return "MOVING_AWAY", None

        return "UNKNOWN", None

    def _object_id(self, obstacle):
        track_id = getattr(obstacle, "track_id", None)
        if track_id is not None:
            return track_id
        return getattr(obstacle, "cluster_id", getattr(obstacle, "id", None))

    def evaluate_obstacle(self, obstacle):
        obstacle_id = self._object_id(obstacle)
        x = float(getattr(obstacle, "x", 0.0))
        y = float(getattr(obstacle, "y", 0.0))
        velocity_y = float(getattr(obstacle, "velocity_y", 0.0))
        distance = float(getattr(obstacle, "distance", math.hypot(x, y)))

        if not bool(getattr(obstacle, "is_dynamic", False)):
            return DynamicObstacleResult(
                "CLEAR", obstacle_id, distance, None,
                self.check_path(y), "NOT_DYNAMIC",
                "Object is not classified as dynamic.",
            )

        if x < self.min_forward_m or x > self.max_forward_m:
            return DynamicObstacleResult(
                "CLEAR", obstacle_id, distance, None,
                self.check_path(y), "OUT_OF_FORWARD_ROI",
                "Dynamic object is outside the forward evaluation range.",
            )

        path_state = self.check_path(y)
        prediction_state, time_to_path = self.predict_path_entry(y, velocity_y)

        if path_state == "IN_PATH":
            return DynamicObstacleResult(
                "RISK", obstacle_id, distance, 0.0,
                path_state, prediction_state,
                "Dynamic object is inside the ego-path corridor.",
            )

        if (
            prediction_state == "APPROACHING_PATH"
            and time_to_path is not None
            and time_to_path <= self.prediction_time_s
        ):
            return DynamicObstacleResult(
                "RISK", obstacle_id, distance, time_to_path,
                path_state, prediction_state,
                "Dynamic object is predicted to enter the ego-path corridor.",
            )

        return DynamicObstacleResult(
            "CLEAR", obstacle_id, distance, time_to_path,
            path_state, prediction_state,
            "No dynamic path-entry risk detected.",
        )

    def plan(self, tracked_obstacles: Sequence):
        results = [
            self.evaluate_obstacle(obstacle)
            for obstacle in tracked_obstacles
            if bool(getattr(obstacle, "is_dynamic", False))
        ]

        if not results:
            return DynamicObstacleResult(
                "CLEAR", None, None, None,
                "NO_DYNAMIC_OBJECT", "NO_DYNAMIC_OBJECT",
                "No tracked dynamic obstacle.",
            )

        risky = [result for result in results if result.status == "RISK"]
        if not risky:
            nearest = min(
                results,
                key=lambda result: (
                    float("inf") if result.distance_m is None else result.distance_m
                ),
            )
            return DynamicObstacleResult(
                "CLEAR",
                nearest.obstacle_id,
                nearest.distance_m,
                nearest.time_to_path_s,
                nearest.path_state,
                nearest.prediction_state,
                "Dynamic objects exist, but none currently meet the risk condition.",
            )

        # Prefer the earliest predicted path entry. Objects already in-path have
        # time_to_path_s=0, so they naturally receive the highest priority.
        return min(
            risky,
            key=lambda result: (
                float("inf")
                if result.time_to_path_s is None
                else result.time_to_path_s,
                float("inf")
                if result.distance_m is None
                else result.distance_m,
            ),
        )
