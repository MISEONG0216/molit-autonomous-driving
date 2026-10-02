#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import numpy as np


# ============================================================
# Competition tunnel section
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
# LiDAR / vehicle configuration
# ============================================================

LIDAR_HEIGHT_M = 1.30

# 실제 차량 폭 확인 후 조정 가능
VEHICLE_HALF_WIDTH_M = 0.90

SAFETY_MARGIN_M = 0.35

COLLISION_RADIUS_M = (
    VEHICLE_HALF_WIDTH_M
    + SAFETY_MARGIN_M
)


# ============================================================
# Tunnel avoidance parameters
# ============================================================

# 차선 침범 허용.
# 터널 벽은 LiDAR collision 검사로 제한한다.
CANDIDATE_OFFSETS_M = (
    0.0,
    1.0,
    -1.0,
    2.0,
    -2.0,
    3.0,
    -3.0,
)

DETECT_FORWARD_M = 22.0

# 원래 경로 바로 앞에 장애물이 있는지 판단
CENTER_CORRIDOR_HALF_WIDTH_M = 1.15

MIN_OBSTACLE_POINTS = 5

# 회피 시작 후 다시 원래 경로로 돌아올 거리
DEFAULT_AVOID_HORIZON_M = 35.0

# 터널 주행 속도 제한
TUNNEL_SPEED_CAP_KMH = 25.0

# 장애물 회피 중
AVOID_SPEED_CAP_KMH = 15.0

# 모든 후보가 좋지 않아도 정지하지 않고 계속 움직인다.
FALLBACK_CRAWL_KMH = 6.0


# ============================================================
# Utility
# ============================================================

def smoothstep(t):

    t = max(
        0.0,
        min(1.0, float(t))
    )

    return (
        10.0 * t**3
        - 15.0 * t**4
        + 6.0 * t**5
    )


def path_distance(path):

    if len(path) <= 1:
        return np.zeros(len(path))

    diff = np.diff(
        path[:, :2],
        axis=0
    )

    ds = np.hypot(
        diff[:, 0],
        diff[:, 1]
    )

    return np.concatenate(
        (
            [0.0],
            np.cumsum(ds),
        )
    )


def global_to_vehicle(
    path,
    vehicle_x,
    vehicle_y,
    yaw
):

    dx = path[:, 0] - vehicle_x
    dy = path[:, 1] - vehicle_y

    c = math.cos(yaw)
    s = math.sin(yaw)

    forward = c * dx + s * dy
    left = -s * dx + c * dy

    return np.column_stack(
        (
            forward,
            left,
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

    """
    global_path의 일부를 복사한 후,
    좌/우 lateral offset을 주어 S자 회피경로 생성.

    시작 offset = 0
    장애물 부근 = peak_offset
    마지막 offset = 0

    따라서 최종적으로 원래 global_path로 자동 복귀.
    """

    points = []

    total_distance = 0.0

    previous = global_path[
        start_index,
        :2
    ]

    end_index = start_index

    horizon = max(
        DEFAULT_AVOID_HORIZON_M,
        obstacle_distance_m + 18.0,
    )

    for i in range(
        start_index,
        len(global_path)
    ):

        p = global_path[i, :2]

        if i > start_index:

            total_distance += math.hypot(
                p[0] - previous[0],
                p[1] - previous[1],
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

        if total_distance >= horizon:
            break

    if len(points) < 3:

        return (
            None,
            start_index
        )

    temp = np.zeros(
        (len(points), 2),
        dtype=float
    )

    # 최대 회피 위치를 장애물보다 약간 뒤로
    peak_distance = max(
        7.0,
        obstacle_distance_m + 2.0
    )

    # 충분히 회피 후 복귀하도록 함
    end_distance = max(
        peak_distance + 12.0,
        total_distance
    )

    for j, (
        x,
        y,
        global_idx,
        distance_m
    ) in enumerate(points):

        # --------------------------------
        # lateral offset profile
        # --------------------------------

        if distance_m <= peak_distance:

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
                - peak_distance
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

        # --------------------------------
        # global path tangent
        # --------------------------------

        if global_idx <= 0:

            p0 = global_path[
                global_idx,
                :2
            ]

            p1 = global_path[
                global_idx + 1,
                :2
            ]

        elif global_idx >= len(global_path) - 1:

            p0 = global_path[
                global_idx - 1,
                :2
            ]

            p1 = global_path[
                global_idx,
                :2
            ]

        else:

            p0 = global_path[
                global_idx - 1,
                :2
            ]

            p1 = global_path[
                global_idx + 1,
                :2
            ]

        tx = p1[0] - p0[0]
        ty = p1[1] - p0[1]

        norm = math.hypot(
            tx,
            ty
        )

        if norm < 1e-6:

            nx = 0.0
            ny = 0.0

        else:

            tx /= norm
            ty /= norm

            # 좌측 normal
            nx = -ty
            ny = tx

        temp[j, 0] = (
            x
            + lateral_offset * nx
        )

        temp[j, 1] = (
            y
            + lateral_offset * ny
        )

    # 마지막 점은 반드시 global_path에 붙임
    temp[-1, 0] = points[-1][0]
    temp[-1, 1] = points[-1][1]

    return (
        temp,
        end_index
    )


# ============================================================
# LiDAR point filtering
# ============================================================

def filter_obstacle_points(
    lidar_points
):

    """
    input:
        Nx3
        [forward, left, sensor_z]

    output:
        실제 회피 판단용 Nx2
        [forward, left]
    """

    if lidar_points is None:

        return np.empty(
            (0, 2),
            dtype=float
        )

    if len(lidar_points) == 0:

        return np.empty(
            (0, 2),
            dtype=float
        )

    p = np.asarray(
        lidar_points,
        dtype=float
    )

    forward = p[:, 0]
    left = p[:, 1]

    sensor_z = p[:, 2]

    # LiDAR 장착높이 1.30m 반영
    ground_height = (
        sensor_z
        + LIDAR_HEIGHT_M
    )

    mask = (
        (forward > 1.0)
        & (forward < 40.0)
        & (np.abs(left) < 7.0)
        & (ground_height > 0.15)
        & (ground_height < 2.50)
    )

    return np.column_stack(
        (
            forward[mask],
            left[mask],
        )
    )


# ============================================================
# Obstacle detection
# ============================================================

def front_obstacle_distance(
    obstacle_points
):

    if len(obstacle_points) == 0:
        return None

    forward = obstacle_points[:, 0]
    left = obstacle_points[:, 1]

    mask = (
        (forward > 2.0)
        & (forward < DETECT_FORWARD_M)
        & (
            np.abs(left)
            < CENTER_CORRIDOR_HALF_WIDTH_M
        )
    )

    candidate = forward[mask]

    if (
        len(candidate)
        < MIN_OBSTACLE_POINTS
    ):
        return None

    # 한 개의 튀는 점보다 안정적인 값 사용
    return float(
        np.percentile(
            candidate,
            10
        )
    )


# ============================================================
# Candidate path collision score
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

    if len(obstacle_points) == 0:

        return 999.0

    local_path = global_to_vehicle(
        candidate_path,
        vehicle_x,
        vehicle_y,
        yaw,
    )

    # 뒤쪽 path 제거
    valid_path = (
        (local_path[:, 0] > 1.0)
        & (local_path[:, 0] < 40.0)
    )

    local_path = local_path[
        valid_path
    ]

    if len(local_path) == 0:
        return 0.0

    points = obstacle_points

    # 계산량 감소
    if len(points) > 3000:

        step = max(
            1,
            len(points) // 3000
        )

        points = points[::step]

    # obstacle x path 거리 계산
    diff = (
        points[:, None, :]
        - local_path[None, :, :]
    )

    dist_sq = np.sum(
        diff * diff,
        axis=2
    )

    min_dist = math.sqrt(
        float(
            np.min(
                dist_sq
            )
        )
    )

    return min_dist


# ============================================================
# Planner
# ============================================================

class TunnelAvoidancePlanner:

    def __init__(self):

        self.active_path = None

        self.active_offset = 0.0

        self.end_global_index = None

        self.state = "NORMAL"

        self.last_clearance = 999.0

    def in_tunnel(
        self,
        nearest_index
    ):

        return (
            TUNNEL_START_INDEX
            <= nearest_index
            <= TUNNEL_END_INDEX
        )

    def clear_plan(self):

        self.active_path = None
        self.active_offset = 0.0
        self.end_global_index = None

    def update(
        self,
        global_path,
        nearest_index,
        vehicle_x,
        vehicle_y,
        yaw,
        lidar_points,
    ):

        # ---------------------------------------
        # Tunnel 밖
        # ---------------------------------------

        if not self.in_tunnel(
            nearest_index
        ):

            self.state = "NORMAL"

            self.clear_plan()

            return {
                "state": self.state,
                "path": None,
                "speed_cap": None,
                "offset": 0.0,
                "obstacle_distance": None,
                "clearance": None,
            }

        obstacle_points = (
            filter_obstacle_points(
                lidar_points
            )
        )

        # ---------------------------------------
        # 이미 회피경로 진행 중
        # ---------------------------------------

        if self.active_path is not None:

            if (
                self.end_global_index
                is not None
                and nearest_index
                < self.end_global_index - 2
            ):

                self.state = "AVOID"

                clearance = path_clearance(
                    self.active_path,
                    obstacle_points,
                    vehicle_x,
                    vehicle_y,
                    yaw,
                )

                self.last_clearance = clearance

                return {
                    "state": self.state,
                    "path": self.active_path,
                    "speed_cap": AVOID_SPEED_CAP_KMH,
                    "offset": self.active_offset,
                    "obstacle_distance": None,
                    "clearance": clearance,
                }

            # 회피경로 끝 -> global path 복귀
            self.clear_plan()

            self.state = "RECOVER"

            return {
                "state": self.state,
                "path": None,
                "speed_cap": TUNNEL_SPEED_CAP_KMH,
                "offset": 0.0,
                "obstacle_distance": None,
                "clearance": None,
            }

        # ---------------------------------------
        # 새로운 장애물 탐색
        # ---------------------------------------

        obstacle_distance = (
            front_obstacle_distance(
                obstacle_points
            )
        )

        if obstacle_distance is None:

            self.state = "TUNNEL_CLEAR"

            return {
                "state": self.state,
                "path": None,
                "speed_cap": TUNNEL_SPEED_CAP_KMH,
                "offset": 0.0,
                "obstacle_distance": None,
                "clearance": None,
            }

        # ---------------------------------------
        # 후보 회피경로 생성
        # ---------------------------------------

        candidates = []

        for offset in (
            CANDIDATE_OFFSETS_M
        ):

            path, end_idx = (
                build_avoidance_path(
                    global_path,
                    nearest_index,
                    offset,
                    obstacle_distance,
                )
            )

            clearance = path_clearance(
                path,
                obstacle_points,
                vehicle_x,
                vehicle_y,
                yaw,
            )

            candidates.append(
                (
                    offset,
                    path,
                    end_idx,
                    clearance,
                )
            )

        # ---------------------------------------
        # 충돌 없는 후보
        # ---------------------------------------

        safe = [
            c
            for c in candidates
            if c[3]
            >= COLLISION_RADIUS_M
        ]

        if safe:

            # 같은 정도의 lateral 이동이면
            # clearance가 큰 쪽 선택.
            # 필요 이상으로 반대차선까지 가지 않도록
            # 작은 offset 우선.
            safe.sort(
                key=lambda c: (
                    abs(c[0]),
                    -c[3],
                )
            )

            chosen = safe[0]

            self.state = "AVOID"

        else:

            # -----------------------------------
            # 정지하지 않는 fallback
            # -----------------------------------
            #
            # 모든 후보가 막힌 것으로 보일 경우
            # 가장 clearance가 큰 경로를 선택하고
            # 매우 느리게 계속 전진.
            #
            # 실제 대회에서는 회피 가능한 공간이
            # 존재한다는 전제.
            #

            chosen = max(
                candidates,
                key=lambda c: c[3]
            )

            self.state = (
                "AVOID_FALLBACK"
            )

        (
            offset,
            path,
            end_idx,
            clearance
        ) = chosen

        self.active_path = path
        self.active_offset = offset
        self.end_global_index = end_idx
        self.last_clearance = clearance

        speed_cap = (
            FALLBACK_CRAWL_KMH
            if self.state
            == "AVOID_FALLBACK"
            else AVOID_SPEED_CAP_KMH
        )

        return {
            "state": self.state,
            "path": self.active_path,
            "speed_cap": speed_cap,
            "offset": offset,
            "obstacle_distance": obstacle_distance,
            "clearance": clearance,
        }
