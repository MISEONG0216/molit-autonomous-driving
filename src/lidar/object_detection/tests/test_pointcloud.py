import numpy as np
import matplotlib.pyplot as plt

from object_detector import ObjectDetector


# ============================================================
# 1. 가상 Point Cloud 설정
# ============================================================

np.random.seed(42)


# ============================================================
# 2. Ground 생성
# ============================================================

ground_count = 2000

ground_x = np.random.uniform(
    0,
    30,
    ground_count
)

ground_y = np.random.uniform(
    -5,
    5,
    ground_count
)

ground_z = np.random.normal(
    0,
    0.02,
    ground_count
)

ground_points = np.column_stack(
    (
        ground_x,
        ground_y,
        ground_z
    )
)


# ============================================================
# 3. 가상 박스 생성 함수
# ============================================================

def create_box(
    center_x,
    center_y,
    width,
    length,
    height,
    num_points
):

    x = np.random.uniform(
        center_x - length / 2,
        center_x + length / 2,
        num_points
    )

    y = np.random.uniform(
        center_y - width / 2,
        center_y + width / 2,
        num_points
    )

    z = np.random.uniform(
        0.1,
        height,
        num_points
    )

    return np.column_stack(
        (
            x,
            y,
            z
        )
    )


# ============================================================
# 4. 테스트 장애물
# ============================================================

obstacle_1 = create_box(
    center_x=8.0,
    center_y=1.0,
    width=1.0,
    length=1.0,
    height=1.0,
    num_points=200
)

obstacle_2 = create_box(
    center_x=9.8,
    center_y=1.0,
    width=1.0,
    length=1.0,
    height=1.0,
    num_points=200
)


# Filtering 테스트용 작은 Cluster
fake_cluster = create_box(
    center_x=13.0,
    center_y=-2.0,
    width=0.10,
    length=0.10,
    height=0.5,
    num_points=50
)


# ============================================================
# 5. Point Cloud 결합
# ============================================================

points = np.vstack(
    (
        ground_points,
        obstacle_1,
        obstacle_2,
        fake_cluster
    )
)


print(
    "Input points:",
    len(points)
)


# ============================================================
# 6. ObjectDetector 생성
# ============================================================

detector = ObjectDetector(
    ground_z_threshold=0.08,

    dbscan_eps=0.5,
    dbscan_min_samples=5,

    min_cluster_points=20,

    min_width=0.20,
    max_width=3.00,

    min_length=0.20,
    max_length=5.00
)


# ============================================================
# 7. 객체 검출
# ============================================================

obstacles, non_ground_points, labels = detector.detect(
    points
)


# ============================================================
# 8. 결과 출력
# ============================================================

print()

print(
    "Non-ground points:",
    len(non_ground_points)
)

print(
    "Detected obstacles:",
    len(obstacles)
)


print()
print("===== Obstacle[] =====")


for obstacle in obstacles:

    print(
        obstacle
    )


# ============================================================
# 9. 결과 시각화
# ============================================================

plt.figure()


# Ground 제거 후 모든 Point
plt.scatter(
    non_ground_points[:, 0],
    non_ground_points[:, 1],
    c="lightgray",
    s=10,
    label="Non-ground"
)


# 최종 Obstacle만 표시
for obstacle in obstacles:

    cluster_points = non_ground_points[
        labels == obstacle.cluster_id
    ]


    plt.scatter(
        cluster_points[:, 0],
        cluster_points[:, 1],
        s=15,
        label=f"Obstacle {obstacle.cluster_id}"
    )


    # 중심 위치
    plt.scatter(
        obstacle.x,
        obstacle.y,
        marker="x",
        s=100
    )


    plt.text(
        obstacle.x,
        obstacle.y + 0.1,
        f"Cluster ID {obstacle.cluster_id}"
    )


plt.xlabel(
    "X - Forward (m)"
)

plt.ylabel(
    "Y - Left/Right (m)"
)

plt.title(
    "ObjectDetector Module Test"
)

plt.grid()

plt.axis(
    "equal"
)

plt.legend()

plt.show()
