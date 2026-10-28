#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Adapter between #3 ObjectTracker and #4 StaticObstaclePlanner.

#3 tracker coordinate system
----------------------------
x = Forward
y = Left

#4 planner coordinate system
----------------------------
x = MORAI global/local-map X
y = MORAI global/local-map Y

This adapter:
1. Converts vehicle-relative obstacle coordinates to global coordinates.
2. Converts track_id to the planner's id field.
3. Produces the obstacle data format expected by #4.

This file does NOT send vehicle control commands.
"""

import math
from dataclasses import dataclass


@dataclass
class PlannerObstacle:
    """
    Obstacle format expected by #4 StaticObstaclePlanner.

    This intentionally matches the fields currently defined
    in static_obstacle_planner.py.
    """

    id: int
    x: float
    y: float
    width: float
    length: float
    distance: float
    velocity_x: float
    velocity_y: float
    is_dynamic: bool
    confidence: float
    height: float = 0.0


def vehicle_to_global(
    local_x,
    local_y,
    ego_x,
    ego_y,
    ego_yaw,
):
    """
    Convert one point from vehicle coordinates to global coordinates.

    Vehicle coordinates:
        +X = Forward
        +Y = Left

    ego_yaw:
        vehicle heading in radians in the same global coordinate
        convention used by the existing Pure Pursuit controller.

    Rotation + translation:

        global_x =
            ego_x
            + cos(yaw) * local_x
            - sin(yaw) * local_y

        global_y =
            ego_y
            + sin(yaw) * local_x
            + cos(yaw) * local_y
    """

    cos_yaw = math.cos(ego_yaw)
    sin_yaw = math.sin(ego_yaw)

    global_x = (
        ego_x
        + cos_yaw * local_x
        - sin_yaw * local_y
    )

    global_y = (
        ego_y
        + sin_yaw * local_x
        + cos_yaw * local_y
    )

    return global_x, global_y


def tracked_to_planner_obstacle(
    tracked_obstacle,
    ego_x,
    ego_y,
    ego_yaw,
    confidence=1.0,
):
    """
    Convert one #3 tracked obstacle into the format required by #4.

    IMPORTANT:
    The current #2 detector initializes confidence to 0.0.
    #4 ignores confidence values below 0.5.

    During integration testing, confidence defaults to 1.0 so that
    a successfully detected/tracked object can reach the planner.

    This is an integration placeholder, NOT a real probabilistic
    detection-confidence estimator.
    """

    global_x, global_y = vehicle_to_global(
        tracked_obstacle.x,
        tracked_obstacle.y,
        ego_x,
        ego_y,
        ego_yaw,
    )

    return PlannerObstacle(
        id=int(tracked_obstacle.track_id),
        x=float(global_x),
        y=float(global_y),
        width=float(tracked_obstacle.width),
        length=float(tracked_obstacle.length),
        distance=float(tracked_obstacle.distance),
        velocity_x=float(tracked_obstacle.velocity_x),
        velocity_y=float(tracked_obstacle.velocity_y),
        is_dynamic=bool(tracked_obstacle.is_dynamic),
        confidence=float(confidence),
        height=float(getattr(tracked_obstacle, "height", 0.0)),
    )


def tracked_list_to_planner_obstacles(
    tracked_obstacles,
    ego_x,
    ego_y,
    ego_yaw,
    confidence=1.0,
):
    """
    Convert a list of #3 tracked obstacles for #4.
    """

    planner_obstacles = []

    for tracked_obstacle in tracked_obstacles:
        planner_obstacle = tracked_to_planner_obstacle(
            tracked_obstacle,
            ego_x,
            ego_y,
            ego_yaw,
            confidence,
        )

        planner_obstacles.append(planner_obstacle)

    return planner_obstacles


def run_coordinate_test():
    """
    Simple coordinate-conversion self-test.

    These tests do not require MORAI, GPS, IMU, or LiDAR.
    """

    print("============================================================")
    print(" Obstacle Adapter Coordinate Test")
    print("============================================================")
    print()

    # ---------------------------------------------------------
    # TEST 1
    #
    # Ego:
    #   global position = (100, 200)
    #   yaw = 0 deg
    #
    # Obstacle:
    #   10 m forward
    #   0 m left
    #
    # Expected:
    #   global = (110, 200)
    # ---------------------------------------------------------

    x1, y1 = vehicle_to_global(
        local_x=10.0,
        local_y=0.0,
        ego_x=100.0,
        ego_y=200.0,
        ego_yaw=math.radians(0.0),
    )

    print("[TEST 1]")
    print("Ego       : (100.0, 200.0), yaw=0 deg")
    print("Obstacle  : forward=10.0, left=0.0")
    print(f"Result    : ({x1:.3f}, {y1:.3f})")
    print("Expected  : (110.000, 200.000)")
    print()

    # ---------------------------------------------------------
    # TEST 2
    #
    # Ego yaw = 90 deg.
    # Vehicle forward direction therefore points along global +Y.
    #
    # 10 m forward should become:
    #   (100, 210)
    # ---------------------------------------------------------

    x2, y2 = vehicle_to_global(
        local_x=10.0,
        local_y=0.0,
        ego_x=100.0,
        ego_y=200.0,
        ego_yaw=math.radians(90.0),
    )

    print("[TEST 2]")
    print("Ego       : (100.0, 200.0), yaw=90 deg")
    print("Obstacle  : forward=10.0, left=0.0")
    print(f"Result    : ({x2:.3f}, {y2:.3f})")
    print("Expected  : (100.000, 210.000)")
    print()

    # ---------------------------------------------------------
    # TEST 3
    #
    # yaw = 0 deg
    # 10 m forward + 3 m left
    #
    # Expected:
    #   (110, 203)
    # ---------------------------------------------------------

    x3, y3 = vehicle_to_global(
        local_x=10.0,
        local_y=3.0,
        ego_x=100.0,
        ego_y=200.0,
        ego_yaw=math.radians(0.0),
    )

    print("[TEST 3]")
    print("Ego       : (100.0, 200.0), yaw=0 deg")
    print("Obstacle  : forward=10.0, left=3.0")
    print(f"Result    : ({x3:.3f}, {y3:.3f})")
    print("Expected  : (110.000, 203.000)")
    print()

    tolerance = 1e-6

    test1_ok = (
        abs(x1 - 110.0) < tolerance
        and abs(y1 - 200.0) < tolerance
    )

    test2_ok = (
        abs(x2 - 100.0) < tolerance
        and abs(y2 - 210.0) < tolerance
    )

    test3_ok = (
        abs(x3 - 110.0) < tolerance
        and abs(y3 - 203.0) < tolerance
    )

    print("============================================================")

    if test1_ok and test2_ok and test3_ok:
        print("[PASS] All coordinate conversion tests passed.")
    else:
        print("[FAIL] Coordinate conversion test failed.")

    print("============================================================")


if __name__ == "__main__":
    run_coordinate_test()