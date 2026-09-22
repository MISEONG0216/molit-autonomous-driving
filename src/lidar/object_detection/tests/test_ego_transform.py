import numpy as np

from obstacle import Obstacle
from tracker import ObjectTracker


# ============================================================
# 공통 설정
# ============================================================

DT = 0.1


# ============================================================
# 결과 출력 함수
# ============================================================

def print_result(title, obstacle):

    speed = np.sqrt(
        obstacle.velocity_x ** 2
        +
        obstacle.velocity_y ** 2
    )

    print()
    print("========================================")
    print(title)
    print("========================================")

    print(
        f"Track ID : {obstacle.track_id}"
    )

    print(
        f"vx       : {obstacle.velocity_x:.2f} m/s"
    )

    print(
        f"vy       : {obstacle.velocity_y:.2f} m/s"
    )

    print(
        f"speed    : {speed:.2f} m/s"
    )


# ============================================================
# TEST 1
#
# 자차 직진 + 정지 장애물
# ============================================================

tracker1 = ObjectTracker(
    max_match_distance=1.0
)


# Frame 1
frame1 = [

    Obstacle(
        cluster_id=0,
        x=10.0,
        y=0.0,
        width=1.0,
        length=1.0,
        distance=10.0
    )
]


tracker1.update(
    frame1,
    dt=DT
)


# ------------------------------------------------------------
# 0.1초 동안 자차가 0.2m 전진
#
# 장애물은 실제로 정지
#
# LiDAR에서는
#
# 10.0m -> 9.8m
#
# 로 보인다.
# ------------------------------------------------------------

frame2 = [

    Obstacle(
        cluster_id=0,
        x=9.8,
        y=0.0,
        width=1.0,
        length=1.0,
        distance=9.8
    )
]


result1 = tracker1.update(
    frame2,
    dt=DT,
    delta_x=0.2,
    delta_y=0.0,
    delta_yaw=0.0
)


print_result(
    "TEST 1 : STRAIGHT + STATIC OBJECT",
    result1[0]
)


# ============================================================
# TEST 2
#
# 자차 직진 + 움직이는 장애물
# ============================================================

tracker2 = ObjectTracker(
    max_match_distance=1.0
)


# Frame 1
frame1 = [

    Obstacle(
        cluster_id=0,
        x=10.0,
        y=0.0,
        width=1.0,
        length=1.0,
        distance=10.0
    )
]


tracker2.update(
    frame1,
    dt=DT
)


# ------------------------------------------------------------
# 자차 속도:
# 2 m/s
#
# 장애물 실제 속도:
# 1 m/s
#
# dt = 0.1초
#
# 자차 이동량:
# 0.2m
#
# 장애물 이동량:
# 0.1m
#
# 따라서 LiDAR 상대 위치:
#
# 10.0m -> 9.9m
# ------------------------------------------------------------

frame2 = [

    Obstacle(
        cluster_id=0,
        x=9.9,
        y=0.0,
        width=1.0,
        length=1.0,
        distance=9.9
    )
]


result2 = tracker2.update(
    frame2,
    dt=DT,
    delta_x=0.2,
    delta_y=0.0,
    delta_yaw=0.0
)


print_result(
    "TEST 2 : STRAIGHT + MOVING OBJECT",
    result2[0]
)


# ============================================================
# TEST 3
#
# 자차 Yaw 회전 + 정지 장애물
# ============================================================

tracker3 = ObjectTracker(
    max_match_distance=1.0
)


# Frame 1
frame1 = [

    Obstacle(
        cluster_id=0,
        x=10.0,
        y=0.0,
        width=1.0,
        length=1.0,
        distance=10.0
    )
]


tracker3.update(
    frame1,
    dt=DT
)


# ------------------------------------------------------------
# 자차가 왼쪽으로 10도 회전
# ------------------------------------------------------------

delta_yaw = np.deg2rad(
    10.0
)


# ------------------------------------------------------------
# 정지 장애물이 현재 차량 좌표계에서
# 보이는 위치 계산
#
# 약:
#
# x =  9.85
# y = -1.74
# ------------------------------------------------------------

observed_x = (
    np.cos(delta_yaw) * 10.0
)

observed_y = (
    -np.sin(delta_yaw) * 10.0
)


frame2 = [

    Obstacle(
        cluster_id=0,
        x=observed_x,
        y=observed_y,
        width=1.0,
        length=1.0,
        distance=10.0
    )
]


result3 = tracker3.update(
    frame2,
    dt=DT,
    delta_x=0.0,
    delta_y=0.0,
    delta_yaw=delta_yaw
)


print_result(
    "TEST 3 : YAW + STATIC OBJECT",
    result3[0]
)
