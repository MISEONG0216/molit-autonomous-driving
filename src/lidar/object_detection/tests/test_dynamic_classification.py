import numpy as np

from obstacle import Obstacle
from tracker import ObjectTracker


DT = 0.1


tracker = ObjectTracker(
    max_match_distance=1.0,
    velocity_window_size=3,
    dynamic_speed_threshold=0.5,
    dynamic_confirm_frames=3
)


print()
print("==============================================")
print("DYNAMIC CLASSIFICATION TEST")
print("==============================================")
print()


# ============================================================
# Frame별 위치
#
# Object A:
#   정지 물체
#   LiDAR Noise만 약간 존재
#
# Object B:
#   +1.0 m/s로 전진하는 동적 물체
# ============================================================

static_positions = [
    8.00,
    8.02,
    7.99,
    8.01,
    8.00,
    8.02,
    8.00
]


moving_positions = [
    12.00,
    12.10,
    12.20,
    12.30,
    12.40,
    12.50,
    12.60
]


for frame_index in range(
    len(static_positions)
):

    # --------------------------------------------------------
    # 정지 장애물
    # --------------------------------------------------------

    static_obstacle = Obstacle(
        cluster_id=0,
        x=static_positions[frame_index],
        y=1.0,
        width=1.0,
        length=1.0,
        distance=static_positions[frame_index]
    )


    # --------------------------------------------------------
    # 이동 장애물
    # --------------------------------------------------------

    moving_obstacle = Obstacle(
        cluster_id=1,
        x=moving_positions[frame_index],
        y=-1.0,
        width=1.0,
        length=1.0,
        distance=moving_positions[frame_index]
    )


    result = tracker.update(
        [
            static_obstacle,
            moving_obstacle
        ],
        dt=DT,
        delta_x=0.0,
        delta_y=0.0,
        delta_yaw=0.0
    )


    print(
        f"FRAME {frame_index + 1}"
    )


    for obstacle in result:

        speed = np.sqrt(
            obstacle.velocity_x ** 2
            +
            obstacle.velocity_y ** 2
        )


        count = tracker.dynamic_count[
            obstacle.track_id
        ]


        print(
            f"  Track {obstacle.track_id} | "
            f"x={obstacle.x:5.2f} | "
            f"speed={speed:4.2f} m/s | "
            f"count={count} | "
            f"is_dynamic={obstacle.is_dynamic}"
        )


    print()
