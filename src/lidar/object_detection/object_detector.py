import numpy as np
from sklearn.cluster import DBSCAN

from obstacle import Obstacle


class ObjectDetector:
    """
    전처리된 LiDAR Point Cloud를 입력받아
    장애물 목록 Obstacle[]을 생성한다.
    """

    def __init__(
        self,
        ground_z_threshold=0.08,
        dbscan_eps=0.5,
        dbscan_min_samples=5,
        min_cluster_points=20,
        min_width=0.20,
        max_width=3.00,
        min_length=0.20,
        max_length=5.00
    ):
        # Ground Removal
        self.ground_z_threshold = ground_z_threshold

        # DBSCAN
        self.dbscan_eps = dbscan_eps
        self.dbscan_min_samples = dbscan_min_samples

        # Cluster Filtering
        self.min_cluster_points = min_cluster_points

        self.min_width = min_width
        self.max_width = max_width

        self.min_length = min_length
        self.max_length = max_length


    # ========================================================
    # Ground Removal
    # ========================================================

    def remove_ground(self, points):

        if len(points) == 0:
            return points

        non_ground_points = points[
            points[:, 2] > self.ground_z_threshold
        ]

        return non_ground_points


    # ========================================================
    # DBSCAN
    # ========================================================

    def cluster_points(self, points):

        if len(points) == 0:
            return np.array([], dtype=int)

        # 현재는 X, Y 평면에서 Clustering
        xy_points = points[:, :2]

        dbscan = DBSCAN(
            eps=self.dbscan_eps,
            min_samples=self.dbscan_min_samples
        )

        labels = dbscan.fit_predict(
            xy_points
        )

        return labels


    # ========================================================
    # Cluster 하나를 Obstacle로 변환
    # ========================================================

    def create_obstacle(
        self,
        cluster_id,
        cluster_points
    ):

        point_count = len(cluster_points)

        # ----------------------------------------------------
        # Point 개수 Filtering
        # ----------------------------------------------------

        if point_count < self.min_cluster_points:

            return None


        # ----------------------------------------------------
        # Bounding Box
        # ----------------------------------------------------

        x_min = np.min(cluster_points[:, 0])
        x_max = np.max(cluster_points[:, 0])

        y_min = np.min(cluster_points[:, 1])
        y_max = np.max(cluster_points[:, 1])


        length = x_max - x_min
        width = y_max - y_min


        # ----------------------------------------------------
        # Width Filtering
        # ----------------------------------------------------

        if (
            width < self.min_width
            or
            width > self.max_width
        ):
            return None


        # ----------------------------------------------------
        # Length Filtering
        # ----------------------------------------------------

        if (
            length < self.min_length
            or
            length > self.max_length
        ):
            return None


        # ----------------------------------------------------
        # 객체 중심
        # ----------------------------------------------------

        center_x = (x_min + x_max) / 2.0
        center_y = (y_min + y_max) / 2.0


        # ----------------------------------------------------
        # 차량 원점으로부터 거리
        # ----------------------------------------------------

        distance = np.sqrt(
            center_x ** 2
            +
            center_y ** 2
        )


        # ----------------------------------------------------
        # Obstacle 생성
        # ----------------------------------------------------

        obstacle = Obstacle(
            cluster_id=cluster_id,

            x=center_x,
            y=center_y,

            width=width,
            length=length,

            distance=distance,

            # Tracking에서 이후 갱신
            velocity_x=0.0,
            velocity_y=0.0,
            is_dynamic=False,

            # 아직 confidence 정의 안 함
            confidence=0.0
        )

        return obstacle


    # ========================================================
    # 전체 객체 검출
    # ========================================================

    def detect(self, points):

        # 1. Ground 제거
        non_ground_points = self.remove_ground(
            points
        )

        # Point가 하나도 없으면 빈 결과 반환
        if len(non_ground_points) == 0:

            return [], non_ground_points, np.array([], dtype=int)


        # 2. DBSCAN
        labels = self.cluster_points(
            non_ground_points
        )


        # 3. Cluster ID 추출
        cluster_ids = set(labels)

        # -1은 DBSCAN Noise
        cluster_ids.discard(-1)


        # 4. 최종 Obstacle[]
        obstacles = []


        for cluster_id in sorted(cluster_ids):

            cluster_points = non_ground_points[
                labels == cluster_id
            ]

            obstacle = self.create_obstacle(
                cluster_id,
                cluster_points
            )


            # Filtering을 통과한 경우에만 추가
            if obstacle is not None:

                obstacles.append(
                    obstacle
                )


        return (
            obstacles,
            non_ground_points,
            labels
        )
