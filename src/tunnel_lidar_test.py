import math
from pathlib import Path

import numpy as np

from lidar_receiver import MoraiLidarReceiver

from tunnel_avoidance import (
    TunnelAvoidancePlanner,
    TUNNEL_START_INDEX,
    TUNNEL_END_INDEX,
    filter_obstacle_points,
    front_obstacle_distance,
)


# ============================================================
# Independent tunnel test
#
# IMPORTANT:
#
# main.py is NOT used.
# Vehicle control is NOT sent.
#
# This script only tests:
#
# LiDAR
#   ->
# obstacle filtering
#   ->
# tunnel avoidance planner
#
# ============================================================


SCAN_DURATION_S = 0.10


# 실제 차량을 터널까지 옮기지 않고도
# planner 연결 자체를 확인하기 위한 테스트 index.
TEST_INDEX = TUNNEL_START_INDEX


# ============================================================
# Project root
# ============================================================

THIS_FILE = Path(
    __file__
).resolve()

SRC_DIR = (
    THIS_FILE.parent
)

PROJECT_ROOT = (
    SRC_DIR.parent
)


# ============================================================
# Global path search
# ============================================================

def load_global_path():

    # 먼저 흔한 경로 확인
    preferred = [
        PROJECT_ROOT
        / "2026_molit_comp_global_path.txt",

        PROJECT_ROOT
        / "global_path.txt",

        PROJECT_ROOT
        / "path"
        / "2026_molit_comp_global_path.txt",

        PROJECT_ROOT
        / "data"
        / "2026_molit_comp_global_path.txt",
    ]


    candidates = []


    for path in preferred:

        if path.exists():

            candidates.append(
                path
            )


    # 위에서 못 찾았으면
    # 프로젝트 안에서 global/path 이름의 txt 탐색
    if not candidates:

        for path in (
            PROJECT_ROOT.rglob(
                "*.txt"
            )
        ):

            name = (
                path.name.lower()
            )


            if (
                "global" in name
                and "path" in name
            ):

                candidates.append(
                    path
                )


    for path in candidates:

        try:

            data = np.loadtxt(
                path,
                dtype=float
            )


        except Exception:

            continue


        if data.ndim != 2:

            continue


        if data.shape[1] < 2:

            continue


        if (
            len(data)
            <= TUNNEL_END_INDEX
        ):

            continue


        print(
            f"[PATH] Loaded: "
            f"{path}"
        )

        print(
            f"[PATH] Points: "
            f"{len(data)}"
        )


        return data


    raise FileNotFoundError(
        "\n"
        "Global path txt 파일을 찾지 못했습니다.\n"
        "\n"
        "프로젝트 폴더 안에\n"
        "2026_molit_comp_global_path.txt\n"
        "또는 global_path가 포함된 txt 파일이 있는지 확인하세요."
    )


# ============================================================
# Pose from global path
# ============================================================

def pose_from_path(
    global_path,
    index,
):

    index = int(
        np.clip(
            index,
            1,
            len(global_path) - 2
        )
    )


    vehicle_x = float(
        global_path[
            index,
            0
        ]
    )


    vehicle_y = float(
        global_path[
            index,
            1
        ]
    )


    previous = global_path[
        index - 1,
        :2
    ]


    next_point = global_path[
        index + 1,
        :2
    ]


    dx = (
        next_point[0]
        - previous[0]
    )


    dy = (
        next_point[1]
        - previous[1]
    )


    yaw = math.atan2(
        dy,
        dx
    )


    return (
        vehicle_x,
        vehicle_y,
        yaw,
    )


# ============================================================
# Helper
# ============================================================

def fmt(
    value,
    digits=2
):

    if value is None:

        return "NONE"


    if isinstance(
        value,
        (float, np.floating)
    ):

        return (
            f"{value:.{digits}f}"
        )


    return str(
        value
    )


# ============================================================
# Main
# ============================================================

def main():

    print(
        "=" * 76
    )

    print(
        " MORAI LiDAR -> Tunnel Avoidance Independent Test"
    )

    print(
        "=" * 76
    )

    print()

    print(
        "main.py       : NOT USED"
    )

    print(
        "vehicle drive : DISABLED"
    )

    print(
        "purpose       : LiDAR + tunnel planner test"
    )

    print()


    # ========================================================
    # Global path
    # ========================================================

    global_path = (
        load_global_path()
    )


    nearest_index = (
        TEST_INDEX
    )


    (
        vehicle_x,
        vehicle_y,
        yaw,
    ) = pose_from_path(
        global_path,
        nearest_index,
    )


    print()

    print(
        f"[TEST INDEX] "
        f"{nearest_index}"
    )


    print(
        f"[TUNNEL] "
        f"{TUNNEL_START_INDEX}"
        f" ~ "
        f"{TUNNEL_END_INDEX}"
    )


    print(
        f"[TEST POSE] "
        f"x="
        f"{vehicle_x:.3f}"
        f" | "
        f"y="
        f"{vehicle_y:.3f}"
        f" | "
        f"yaw="
        f"{math.degrees(yaw):.2f} deg"
    )


    print()

    print(
        "NPC는 Ego 정면 중앙 "
        "3~5m 정도에 두세요."
    )

    print()

    print(
        "Press Ctrl+C to stop."
    )

    print(
        "=" * 76
    )

    print()


    # ========================================================
    # Modules
    # ========================================================

    lidar = (
        MoraiLidarReceiver()
    )


    planner = (
        TunnelAvoidancePlanner()
    )


    try:

        while True:

            # =================================================
            # 1. LiDAR scan
            # =================================================

            scan = (
                lidar.receive_scan(
                    duration_s=SCAN_DURATION_S
                )
            )


            if len(scan) == 0:

                print(
                    "[WAIT] "
                    "No LiDAR scan"
                )

                continue


            # =================================================
            # 2. Same filter used by tunnel_avoidance.py
            # =================================================

            obstacle_points = (
                filter_obstacle_points(
                    scan
                )
            )


            # =================================================
            # 3. Check obstacle directly ahead
            # =================================================

            obstacle_distance = (
                front_obstacle_distance(
                    obstacle_points
                )
            )


            print(
                f"[LIDAR] "
                f"raw="
                f"{len(scan):5d}"
                f" | "
                f"filtered="
                f"{len(obstacle_points):4d}"
                f" | "
                f"front_obstacle="
                f"{fmt(obstacle_distance)} m"
            )


            # =================================================
            # 4. Tunnel planner
            # =================================================

            result = (
                planner.update(
                    global_path=global_path,
                    nearest_index=nearest_index,
                    vehicle_x=vehicle_x,
                    vehicle_y=vehicle_y,
                    yaw=yaw,
                    lidar_points=scan,
                )
            )


            state = result.get(
                "state"
            )


            speed_cap = result.get(
                "speed_cap"
            )


            offset = result.get(
                "offset"
            )


            clearance = result.get(
                "clearance"
            )


            planner_obstacle_distance = (
                result.get(
                    "obstacle_distance"
                )
            )


            path = result.get(
                "path"
            )


            print(
                f"[PLANNER] "
                f"state="
                f"{state}"
                f" | "
                f"offset="
                f"{fmt(offset)} m"
                f" | "
                f"speed_cap="
                f"{fmt(speed_cap)} km/h"
            )


            print(
                f"[PLAN INFO] "
                f"obstacle_distance="
                f"{fmt(planner_obstacle_distance)} m"
                f" | "
                f"clearance="
                f"{fmt(clearance)} m"
            )


            # =================================================
            # 5. Avoidance path info
            # =================================================

            if (
                path is not None
                and len(path) > 0
            ):

                print(
                    f"[AVOID PATH] "
                    f"points="
                    f"{len(path)}"
                    f" | "
                    f"start=("
                    f"{path[0, 0]:.2f}, "
                    f"{path[0, 1]:.2f}"
                    f")"
                    f" | "
                    f"end=("
                    f"{path[-1, 0]:.2f}, "
                    f"{path[-1, 1]:.2f}"
                    f")"
                )

            else:

                print(
                    "[AVOID PATH] NONE"
                )


            print(
                "-" * 76
            )


    except KeyboardInterrupt:

        print()

        print(
            "[STOP] "
            "Tunnel LiDAR test stopped."
        )


    finally:

        lidar.close()


if __name__ == "__main__":

    main()