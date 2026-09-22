import numpy as np

from object_detector import ObjectDetector
from tracker import ObjectTracker


# ============================================================
# 테스트 설정
# ============================================================

DT = 0.1

np.random.seed(42)


# ============================================================
# Object Detector
# ============================================================

detector = ObjectDetector(
    ground_z_threshold=0.08,

    # 우리가 지금까지 사용한 DBSCAN 설정
    dbscan_eps=0.5,
    dbscan_min_samples=5,

    min_cluster_points=20,

    min_width=0.20,
    max_width=3.00,

    min_length=0.20,
    max_length=5.00
)


# ============================================================
# Object Tracker
# ============================================================

tracker = ObjectTracker(
    max_match_distance=1.0,

    velocity_window_size=3,

    dynamic_speed_threshold=0.5,
    dynamic_confirm_frames=3,

    max_missed_frames=3
)


# ============================================================
# Ground Point 생성
# ============================================================

def create_ground_points(
    point_count=1000
):

    x = np.random.uniform(
        0.0,
        20.0,
        point_count
    )

    y = np.random.uniform(
        -5.0,
        5.0,
        point_count
    )

    # Ground Removal 기준 0.08m보다 아래
    z = np.random.uniform(
        0.0,
        0.05,
        point_count
    )

    return np.column_stack(
        (
            x,
            y,
            z
        )
    )


# ============================================================
# 장애물 Point Cloud 생성
# ============================================================

def create_obstacle_points(
    center_x,
    center_y,
    length=1.0,
    width=1.0,
    height=1.0,
    point_count=200
):

    x = np.random.uniform(
        center_x - length / 2.0,
        center_x + length / 2.0,
        point_count
    )

    y = np.random.uniform(
        center_y - width / 2.0,
        center_y + width / 2.0,
        point_count
    )

    # Ground Removal을 통과하도록
    # 최소 높이를 0.20m로 설정
    z = np.random.uniform(
        0.20,
        height,
        point_count
    )

    return np.column_stack(
        (
            x,
            y,
            z
        )
    )


# ============================================================
# Frame 하나 생성
# ============================================================

def create_frame(
    static_x,
    moving_x
):

    # Ground
    ground = create_ground_points()


    # --------------------------------------------------------
    # 정지 장애물
    #
    # 위치:
    #   약 (8, +1)
    # --------------------------------------------------------

    static_obstacle = create_obstacle_points(
        center_x=static_x,
        center_y=1.0
    )


    # --------------------------------------------------------
    # 이동 장애물
    #
    # 위치:
    #   약 (12~, -1)
    # --------------------------------------------------------

    moving_obstacle = create_obstacle_points(
        center_x=moving_x,
        center_y=-1.0
    )


    # 전체 LiDAR Point Cloud
    points = np.vstack(
        (
            ground,
            static_obstacle,
            moving_obstacle
        )
    )


    return points


# ============================================================
# 테스트 위치
# ============================================================

# 정지 장애물
static_positions = [
    8.00,
    8.00,
    8.00,
    8.00,
    8.00,
    8.00,
    8.00
]


# 약 1 m/s 이동
#
# DT = 0.1초
# Frame당 0.1m 이동
#
moving_positions = [
    12.00,
    12.10,
    12.20,
    12.30,
    12.40,
    12.50,
    12.60
]


# ============================================================
# Full Pipeline 실행
# ============================================================

print()
print("==================================================")
print("LiDAR FULL PIPELINE TEST")
print("==================================================")
print()


for frame_index in range(
    len(static_positions)
):

    # --------------------------------------------------------
    # 1. 가상 LiDAR 생성
    # --------------------------------------------------------

    points = create_frame(
        static_x=static_positions[frame_index],
        moving_x=moving_positions[frame_index]
    )


    # --------------------------------------------------------
    # 2. Object Detection
    #
    # Point Cloud
    #     ↓
    # Ground Removal
    #     ↓
    # DBSCAN
    #     ↓
    # Obstacle[]
    # --------------------------------------------------------

    (
        detected_obstacles,
        non_ground_points,
        labels
    ) = detector.detect(
        points
    )


    # --------------------------------------------------------
    # 3. Tracking
    #
    # 이번 테스트에서는 자차가 정지해 있으므로
    # Ego Motion = 0
    # --------------------------------------------------------

    tracked_obstacles = tracker.update(
        detected_obstacles,

        dt=DT,

        delta_x=0.0,
        delta_y=0.0,
        delta_yaw=0.0
    )


    # --------------------------------------------------------
    # 결과 출력
    # --------------------------------------------------------

    print(
        f"FRAME {frame_index + 1}"
    )

    print(
        f"  Input points      : {len(points)}"
    )

    print(
        f"  Non-ground points : {len(non_ground_points)}"
    )

    print(
        f"  Detected objects  : {len(detected_obstacles)}"
    )


    for obstacle in tracked_obstacles:

        speed = np.sqrt(
            obstacle.velocity_x ** 2
            +
            obstacle.velocity_y ** 2
        )


        print(
            f"  Track {obstacle.track_id} | "
            f"x={obstacle.x:5.2f} | "
            f"y={obstacle.y:+5.2f} | "
            f"size=({obstacle.length:.2f}, "
            f"{obstacle.width:.2f}) | "
            f"speed={speed:.2f} m/s | "
            f"dynamic={obstacle.is_dynamic}"
        )


    print()
