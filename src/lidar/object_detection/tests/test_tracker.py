from obstacle import Obstacle
from tracker import ObjectTracker


# ============================================================
# 1. 설정
# ============================================================

# 가상 LiDAR Frame 간격
DT = 0.1


# ============================================================
# 2. Tracker 생성
# ============================================================

tracker = ObjectTracker(
    max_match_distance=1.0
)


# ============================================================
# 3. Frame 1
# ============================================================

frame1 = [

    # 물체 A
    Obstacle(
        cluster_id=0,
        x=8.0,
        y=1.0,
        width=1.0,
        length=1.0,
        distance=8.06
    ),

    # 물체 B
    Obstacle(
        cluster_id=1,
        x=15.0,
        y=-2.0,
        width=1.0,
        length=1.0,
        distance=15.13
    )
]


print()
print("========== FRAME 1 ==========")


tracked_frame1 = tracker.update(
    frame1,
    dt=DT
)


for obstacle in tracked_frame1:

    print(
        f"Cluster ID={obstacle.cluster_id}, "
        f"Track ID={obstacle.track_id}, "
        f"Position=({obstacle.x:.2f}, {obstacle.y:.2f}), "
        f"Velocity=({obstacle.velocity_x:.2f}, "
        f"{obstacle.velocity_y:.2f}) m/s"
    )


# ============================================================
# 4. Frame 2
# ============================================================

# 0.1초 후라고 가정
#
# A:
# (8.0, 1.0)
#      ↓
# (8.2, 1.0)
#
# 따라서
# vx = (8.2 - 8.0) / 0.1
#    = +2.0 m/s
#
#
# B:
# (15.0, -2.0)
#      ↓
# (14.8, -2.0)
#
# 따라서
# vx = (14.8 - 15.0) / 0.1
#    = -2.0 m/s


# DBSCAN Cluster ID는 일부러 뒤집는다.

frame2 = [

    # 물체 B
    Obstacle(
        cluster_id=0,
        x=14.8,
        y=-2.0,
        width=1.0,
        length=1.0,
        distance=14.93
    ),

    # 물체 A
    Obstacle(
        cluster_id=1,
        x=8.2,
        y=1.0,
        width=1.0,
        length=1.0,
        distance=8.26
    )
]


print()
print("========== FRAME 2 ==========")


tracked_frame2 = tracker.update(
    frame2,
    dt=DT
)


for obstacle in tracked_frame2:

    print(
        f"Cluster ID={obstacle.cluster_id}, "
        f"Track ID={obstacle.track_id}, "
        f"Position=({obstacle.x:.2f}, {obstacle.y:.2f}), "
        f"Velocity=({obstacle.velocity_x:.2f}, "
        f"{obstacle.velocity_y:.2f}) m/s"
    )


# ============================================================
# 5. 결과
# ============================================================

print()
print("========== RESULT ==========")


for obstacle in tracked_frame2:

    speed = (
        obstacle.velocity_x ** 2
        +
        obstacle.velocity_y ** 2
    ) ** 0.5


    print(
        f"Track {obstacle.track_id}: "
        f"vx={obstacle.velocity_x:.2f} m/s, "
        f"vy={obstacle.velocity_y:.2f} m/s, "
        f"speed={speed:.2f} m/s"
    )
