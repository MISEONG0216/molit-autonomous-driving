import numpy as np

from obstacle import Obstacle
from tracker import ObjectTracker


DT = 0.1


tracker = ObjectTracker(
    max_match_distance=1.0,
    velocity_window_size=5
)


# ============================================================
# 정지 장애물인데 측정 중심점이 조금씩 흔들리는 상황
# ============================================================

positions = [
    10.00,
    10.03,
    9.98,
    10.02,
    9.99,
    10.01,
    10.00
]


print()
print("========================================")
print("VELOCITY FILTER TEST")
print("========================================")
print()


previous_x = None


for frame_number, x in enumerate(
    positions
):

    obstacle = Obstacle(
        cluster_id=0,
        x=x,
        y=0.0,
        width=1.0,
        length=1.0,
        distance=x
    )


    result = tracker.update(
        [obstacle],
        dt=DT,
        delta_x=0.0,
        delta_y=0.0,
        delta_yaw=0.0
    )


    tracked = result[0]


    # 첫 Frame은 Raw velocity 계산 불가능
    if previous_x is None:

        raw_velocity = 0.0

    else:

        raw_velocity = (
            x - previous_x
        ) / DT


    filtered_speed = np.sqrt(
        tracked.velocity_x ** 2
        +
        tracked.velocity_y ** 2
    )


    print(
        f"Frame {frame_number + 1}: "
        f"x={x:.2f} m, "
        f"raw_vx={raw_velocity:+.2f} m/s, "
        f"filtered_vx={tracked.velocity_x:+.2f} m/s, "
        f"filtered_speed={filtered_speed:.2f} m/s"
    )


    previous_x = x

