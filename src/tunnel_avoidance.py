#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import numpy as np


# ============================================================
# Tunnel section
# ============================================================

TUNNEL_START_INDEX = 3768
TUNNEL_END_INDEX = 4045

TUNNEL_START = (
    43.761510782991536,
    -476.41094469698146,
)

TUNNEL_END = (
    -74.86074459552765,
    -544.5780326710083,
)


# ============================================================
# Vehicle / safety
# ============================================================

VEHICLE_HALF_WIDTH_M = 0.90
SAFETY_MARGIN_M = 0.35

COLLISION_RADIUS_M = (
    VEHICLE_HALF_WIDTH_M
    + SAFETY_MARGIN_M
)

PREFERRED_CLEARANCE_M = 1.50


# ============================================================
# Avoidance settings
# ============================================================

CANDIDATE_OFFSETS_M = (
    2.0,
    -2.0,
    3.0,
    -3.0,
)

DETECT_FORWARD_M = 22.0

CENTER_CORRIDOR_HALF_WIDTH_M = 1.60

MIN_OBSTACLE_POINTS = 3

DEFAULT_AVOID_HORIZON_M = 35.0


# ============================================================
# Speed limits
# ============================================================

TUNNEL_SPEED_CAP_KMH = 25.0

AVOID_SPEED_CAP_KMH = 3.0

TIGHT_AVOID_SPEED_CAP_KMH = 2.0

FALLBACK_CRAWL_KMH = 1.5


# ============================================================
# Ground estimation
# ============================================================

GROUND_FORWARD_MIN_M = 2.0
GROUND_FORWARD_MAX_M = 15.0

GROUND_SIDE_HALF_WIDTH_M = 3.0

GROUND_PERCENTILE = 10.0

GROUND_FALLBACK_Z_M = -1.86

MIN_OBSTACLE_HEIGHT_M = 0.15
MAX_OBSTACLE_HEIGHT_M = 2.50


# ============================================================
# Utility
# ============================================================

def smoothstep(t):

    t = max(
        0.0,
        min(
            1.0,
            float(t),
        ),
    )

    return (
        10.0 * t**3
        - 15.0 * t**4
        + 6.0 * t**5
    )


def global_to_vehicle(
    path,
    vehicle_x,
    vehicle_y,
    yaw,
):

    path = np.asarray(
        path,
        dtype=float,
    )

    dx = (
        path[:, 0]
        - vehicle_x
    )

    dy = (
        path[:, 1]
        - vehicle_y
    )

    c = math.cos(yaw)
    s = math.sin(yaw)

    forward = (
        c * dx
        + s * dy
    )

    left = (
        -s * dx
        + c * dy
    )

    return np.column_stack(
        (
            forward,
            left,
        )
    )


# ============================================================
# Ground estimation
# ============================================================

def estimate_ground_z(
    lidar_points,
):

    if (
        lidar_points is None
        or len(lidar_points) == 0
    ):

        return (
            GROUND_FALLBACK_Z_M
        )

    p = np.asarray(
        lidar_points,
        dtype=float,
    )

    forward = p[:, 0]
    left = p[:, 1]
    sensor_z = p[:, 2]

    mask = (
        (forward > GROUND_FORWARD_MIN_M)
        & (forward < GROUND_FORWARD_MAX_M)
        & (
            np.abs(left)
            < GROUND_SIDE_HALF_WIDTH_M
        )
    )

    samples = (
        sensor_z[
            mask
        ]
    )

    if len(samples) < 20:

        return (
            GROUND_FALLBACK_Z_M
        )

    return float(
        np.percentile(
            samples,
            GROUND_PERCENTILE,
        )
    )


# ============================================================
# LiDAR obstacle filtering
# ============================================================

def filter_obstacle_points(
    lidar_points,
):

    if lidar_points is None:

        return np.empty(
            (0, 2),
            dtype=float,
        )

    if len(lidar_points) == 0:

        return np.empty(
            (0, 2),
            dtype=float,
        )

    p = np.asarray(
        lidar_points,
        dtype=float,
    )

    forward = p[:, 0]
    left = p[:, 1]
    sensor_z = p[:, 2]

    ground_z = (
        estimate_ground_z(
            p
        )
    )

    height = (
        sensor_z
        - ground_z
    )

    mask = (
        (forward > 1.0)
        & (forward < 40.0)
        & (np.abs(left) < 7.0)
        & (
            height
            > MIN_OBSTACLE_HEIGHT_M
        )
        & (
            height
            < MAX_OBSTACLE_HEIGHT_M
        )
    )

    return np.column_stack(
        (
            forward[mask],
            left[mask],
        )
    )


# ============================================================
# Front obstacle detector
# ============================================================

def front_obstacle_distance(
    obstacle_points,
):

    if (
        obstacle_points is None
        or len(obstacle_points) == 0
    ):

        return None

    p = np.asarray(
        obstacle_points,
        dtype=float,
    )

    forward = p[:, 0]
    left = p[:, 1]

    mask = (
        (forward > 2.0)
        & (
            forward
            < DETECT_FORWARD_M
        )
        & (
            np.abs(left)
            < CENTER_CORRIDOR_HALF_WIDTH_M
        )
    )

    candidate = (
        forward[
            mask
        ]
    )

    if (
        len(candidate)
        < MIN_OBSTACLE_POINTS
    ):

        return None

    return float(
        np.percentile(
            candidate,
            10,
        )
    )


# ============================================================
# Temporary avoidance path
# ============================================================

def build_avoidance_path(
    global_path,
    start_index,
    peak_offset_m,
    obstacle_distance_m,
):

    global_path = np.asarray(
        global_path,
        dtype=float,
    )

    points = []

    total_distance = 0.0

    previous = (
        global_path[
            start_index,
            :2
        ]
    )

    end_index = start_index

    horizon = max(
        DEFAULT_AVOID_HORIZON_M,
        obstacle_distance_m + 18.0,
    )

    for i in range(
        start_index,
        len(global_path),
    ):

        p = (
            global_path[
                i,
                :2
            ]
        )

        if i > start_index:

            total_distance += (
                math.hypot(
                    p[0] - previous[0],
                    p[1] - previous[1],
                )
            )

        points.append(
            (
                p[0],
                p[1],
                i,
                total_distance,
            )
        )

        previous = p

        end_index = i

        if (
            total_distance
            >= horizon
        ):
            break

    if len(points) < 3:

        return (
            None,
            start_index,
        )

    temp = np.zeros(
        (
            len(points),
            2,
        ),
        dtype=float,
    )

    # 최대 lateral offset에 장애물보다 먼저 도달
    peak_distance = max(
        3.0,
        obstacle_distance_m - 1.5,
    )

    # 장애물을 지난 뒤 부드럽게 원래 global path 복귀
    end_distance = max(
        peak_distance + 14.0,
        total_distance,
    )

    for j, (
        x,
        y,
        global_idx,
        distance_m,
    ) in enumerate(points):

        if (
            distance_m
            <= peak_distance
        ):

            t = (
                distance_m
                / peak_distance
            )

            lateral_offset = (
                peak_offset_m
                * smoothstep(t)
            )

        else:

            recover_length = max(
                1.0,
                end_distance
                - peak_distance,
            )

            t = (
                distance_m
                - peak_distance
            ) / recover_length

            lateral_offset = (
                peak_offset_m
                * (
                    1.0
                    - smoothstep(t)
                )
            )

        if global_idx <= 0:

            p0 = (
                global_path[
                    global_idx,
                    :2
                ]
            )

            p1 = (
                global_path[
                    global_idx + 1,
                    :2
                ]
            )

        elif (
            global_idx
            >= len(global_path) - 1
        ):

            p0 = (
                global_path[
                    global_idx - 1,
                    :2
                ]
            )

            p1 = (
                global_path[
                    global_idx,
                    :2
                ]
            )

        else:

            p0 = (
                global_path[
                    global_idx - 1,
                    :2
                ]
            )

            p1 = (
                global_path[
                    global_idx + 1,
                    :2
                ]
            )

        tx = (
            p1[0]
            - p0[0]
        )

        ty = (
            p1[1]
            - p0[1]
        )

        norm = (
            math.hypot(
                tx,
                ty,
            )
        )

        if norm < 1e-6:

            nx = 0.0
            ny = 0.0

        else:

            tx /= norm
            ty /= norm

            nx = -ty
            ny = tx

        temp[j, 0] = (
            x
            + lateral_offset
            * nx
        )

        temp[j, 1] = (
            y
            + lateral_offset
            * ny
        )

    # 마지막 점은 반드시 원래 경로
    temp[-1, 0] = (
        points[-1][0]
    )

    temp[-1, 1] = (
        points[-1][1]
    )

    return (
        temp,
        end_index,
    )


# ============================================================
# Candidate clearance
# ============================================================

def path_clearance(
    candidate_path,
    obstacle_points,
    vehicle_x,
    vehicle_y,
    yaw,
):

    if (
        candidate_path is None
        or len(candidate_path) == 0
    ):

        return 0.0

    if (
        obstacle_points is None
        or len(obstacle_points) == 0
    ):

        return 999.0

    local_path = (
        global_to_vehicle(
            candidate_path,
            vehicle_x,
            vehicle_y,
            yaw,
        )
    )

    valid_path = (
        (local_path[:, 0] > 1.0)
        & (local_path[:, 0] < 40.0)
    )

    local_path = (
        local_path[
            valid_path
        ]
    )

    if len(local_path) == 0:

        return 0.0

    points = np.asarray(
        obstacle_points,
        dtype=float,
    )

    if len(points) > 3000:

        step = max(
            1,
            len(points) // 3000,
        )

        points = (
            points[
                ::step
            ]
        )

    diff = (
        points[:, None, :]
        - local_path[None, :, :]
    )

    dist_sq = (
        np.sum(
            diff * diff,
            axis=2,
        )
    )

    return math.sqrt(
        float(
            np.min(
                dist_sq
            )
        )
    )


# ============================================================
# Tunnel avoidance planner V5
# ============================================================

class TunnelAvoidancePlanner:

    def __init__(self):

        self.active_path = None

        self.active_offset = 0.0

        self.end_global_index = None

        self.state = "NORMAL"

        self.last_clearance = 999.0

        # V5:
        # 최초 회피 방향을 저장한다.
        #
        # +1 = global path 기준 + offset
        # -1 = global path 기준 - offset
        self.locked_sign = None


    def in_tunnel(
        self,
        nearest_index,
    ):

        return (
            TUNNEL_START_INDEX
            <= nearest_index
            <= TUNNEL_END_INDEX
        )


    def clear_plan(
        self,
        reset_side=True,
    ):

        self.active_path = None

        self.active_offset = 0.0

        self.end_global_index = None

        self.last_clearance = 999.0

        if reset_side:

            self.locked_sign = None


    def _candidate_offsets(
        self,
        same_side_only=False,
    ):

        if (
            not same_side_only
            or self.locked_sign is None
        ):

            return list(
                CANDIDATE_OFFSETS_M
            )

        sign = (
            1.0
            if self.locked_sign > 0
            else -1.0
        )

        # V5 핵심:
        # 한 번 선택한 방향의 반대편 후보는 아예 생성하지 않는다.
        return [
            2.0 * sign,
            3.0 * sign,
        ]


    def _choose_path(
        self,
        global_path,
        nearest_index,
        vehicle_x,
        vehicle_y,
        yaw,
        obstacle_points,
        obstacle_distance,
        candidate_offsets,
    ):

        candidates = []

        for offset in candidate_offsets:

            path, end_idx = (
                build_avoidance_path(
                    global_path,
                    nearest_index,
                    offset,
                    obstacle_distance,
                )
            )

            if path is None:

                continue

            clearance = (
                path_clearance(
                    path,
                    obstacle_points,
                    vehicle_x,
                    vehicle_y,
                    yaw,
                )
            )

            candidates.append(
                (
                    float(offset),
                    path,
                    int(end_idx),
                    float(clearance),
                )
            )

        if not candidates:

            return None

        preferred = [
            c
            for c in candidates
            if (
                c[3]
                >= PREFERRED_CLEARANCE_M
            )
        ]

        if preferred:

            # 가장 작은 회피량을 우선.
            # 같은 회피량이면 clearance 큰 쪽.
            preferred.sort(
                key=lambda c: (
                    abs(c[0]),
                    -c[3],
                )
            )

            return (
                preferred[0],
                "AVOID",
            )

        minimum_safe = [
            c
            for c in candidates
            if (
                c[3]
                >= COLLISION_RADIUS_M
            )
        ]

        if minimum_safe:

            chosen = max(
                minimum_safe,
                key=lambda c: (
                    c[3],
                    -abs(c[0]),
                ),
            )

            return (
                chosen,
                "AVOID_TIGHT",
            )

        chosen = max(
            candidates,
            key=lambda c: (
                c[3],
                -abs(c[0]),
            ),
        )

        return (
            chosen,
            "AVOID_FALLBACK",
        )


    def update(
        self,
        global_path,
        nearest_index,
        vehicle_x,
        vehicle_y,
        yaw,
        lidar_points,
    ):

        # ----------------------------------------------------
        # Tunnel outside
        # ----------------------------------------------------

        if not self.in_tunnel(
            nearest_index
        ):

            self.state = "NORMAL"

            self.clear_plan(
                reset_side=True
            )

            return {
                "state": self.state,
                "path": None,
                "speed_cap": None,
                "offset": 0.0,
                "obstacle_distance": None,
                "clearance": None,
                "side_locked": False,
            }

        obstacle_points = (
            filter_obstacle_points(
                lidar_points
            )
        )

        obstacle_distance = (
            front_obstacle_distance(
                obstacle_points
            )
        )

        # ----------------------------------------------------
        # Existing avoidance path
        # ----------------------------------------------------

        if self.active_path is not None:

            if (
                self.end_global_index
                is not None
                and nearest_index
                < self.end_global_index - 2
            ):

                clearance = (
                    path_clearance(
                        self.active_path,
                        obstacle_points,
                        vehicle_x,
                        vehicle_y,
                        yaw,
                    )
                )

                self.last_clearance = (
                    clearance
                )

                self.state = "AVOID"

                return {
                    "state": self.state,
                    "path": self.active_path,
                    "speed_cap": AVOID_SPEED_CAP_KMH,
                    "offset": self.active_offset,
                    "obstacle_distance": obstacle_distance,
                    "clearance": clearance,
                    "side_locked": (
                        self.locked_sign
                        is not None
                    ),
                }

            # 회피 경로 끝 도달
            self.clear_plan(
                reset_side=True
            )

            self.state = "RECOVER"

            return {
                "state": self.state,
                "path": None,
                "speed_cap": TUNNEL_SPEED_CAP_KMH,
                "offset": 0.0,
                "obstacle_distance": obstacle_distance,
                "clearance": None,
                "side_locked": False,
            }

        # ----------------------------------------------------
        # No obstacle
        # ----------------------------------------------------

        if obstacle_distance is None:

            self.state = "TUNNEL_CLEAR"

            return {
                "state": self.state,
                "path": None,
                "speed_cap": TUNNEL_SPEED_CAP_KMH,
                "offset": 0.0,
                "obstacle_distance": None,
                "clearance": None,
                "side_locked": False,
            }

        # ----------------------------------------------------
        # First obstacle detection
        # ----------------------------------------------------

        selection = (
            self._choose_path(
                global_path,
                nearest_index,
                vehicle_x,
                vehicle_y,
                yaw,
                obstacle_points,
                obstacle_distance,
                self._candidate_offsets(
                    same_side_only=False
                ),
            )
        )

        if selection is None:

            self.state = (
                "AVOID_FALLBACK"
            )

            return {
                "state": self.state,
                "path": None,
                "speed_cap": FALLBACK_CRAWL_KMH,
                "offset": 0.0,
                "obstacle_distance": obstacle_distance,
                "clearance": 0.0,
                "side_locked": False,
            }

        chosen, state = selection

        (
            offset,
            path,
            end_idx,
            clearance,
        ) = chosen

        self.active_path = path

        self.active_offset = offset

        self.end_global_index = (
            end_idx
        )

        self.last_clearance = (
            clearance
        )

        # V5 핵심:
        # 최초 회피 방향 고정
        self.locked_sign = (
            1
            if offset > 0.0
            else -1
        )

        self.state = state

        if (
            state
            == "AVOID_FALLBACK"
        ):

            speed_cap = (
                FALLBACK_CRAWL_KMH
            )

        elif (
            state
            == "AVOID_TIGHT"
        ):

            speed_cap = (
                TIGHT_AVOID_SPEED_CAP_KMH
            )

        else:

            speed_cap = (
                AVOID_SPEED_CAP_KMH
            )

        return {
            "state": self.state,
            "path": self.active_path,
            "speed_cap": speed_cap,
            "offset": self.active_offset,
            "obstacle_distance": obstacle_distance,
            "clearance": clearance,
            "side_locked": True,
        }


    # ========================================================
    # V5 controlled replan
    # ========================================================

    def replan_same_side(
        self,
        global_path,
        nearest_index,
        vehicle_x,
        vehicle_y,
        yaw,
        lidar_points,
    ):

        if (
            self.active_path is None
            or self.locked_sign is None
        ):

            return False

        obstacle_points = (
            filter_obstacle_points(
                lidar_points
            )
        )

        obstacle_distance = (
            front_obstacle_distance(
                obstacle_points
            )
        )

        if obstacle_distance is None:

            return False

        current_abs_offset = (
            abs(
                self.active_offset
            )
        )

        sign = (
            1.0
            if self.locked_sign > 0
            else -1.0
        )

        # 이미 3m 회피 중이면 더 이상
        # 매 프레임 재생성하지 않는다.
        if current_abs_offset >= 2.9:

            return False

        # V5:
        # +2 -> +3
        # -2 -> -3
        # 반대 방향 후보 없음
        candidate_offsets = [
            3.0 * sign
        ]

        selection = (
            self._choose_path(
                global_path,
                nearest_index,
                vehicle_x,
                vehicle_y,
                yaw,
                obstacle_points,
                obstacle_distance,
                candidate_offsets,
            )
        )

        if selection is None:

            return False

        chosen, state = selection

        (
            offset,
            path,
            end_idx,
            clearance,
        ) = chosen

        self.active_path = path

        self.active_offset = offset

        self.end_global_index = (
            end_idx
        )

        self.last_clearance = (
            clearance
        )

        self.state = state

        # locked_sign은 절대 변경하지 않는다.

        return True