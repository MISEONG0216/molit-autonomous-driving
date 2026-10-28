class Obstacle:
    """
    LiDAR에서 검출된 하나의 객체.

    Object Detection:
        cluster_id
        x, y
        width, length
        distance
        confidence

    Tracking:
        track_id
        velocity_x
        velocity_y
        is_dynamic
    """

    def __init__(
        self,
        cluster_id,
        x,
        y,
        width,
        length,
        distance,
        track_id=None,
        velocity_x=0.0,
        velocity_y=0.0,
        is_dynamic=False,
        confidence=0.0,
        height=0.0
    ):
        # Object Detection에서 생성
        self.cluster_id = cluster_id

        # Tracking에서 생성
        self.track_id = track_id

        self.x = x
        self.y = y

        self.width = width
        self.length = length

        self.distance = distance

        self.velocity_x = velocity_x
        self.velocity_y = velocity_y

        self.is_dynamic = is_dynamic

        self.confidence = confidence
        self.height = height


    def __repr__(self):
        return (
            f"Obstacle("
            f"cluster_id={self.cluster_id}, "
            f"track_id={self.track_id}, "
            f"x={self.x:.2f}, "
            f"y={self.y:.2f}, "
            f"width={self.width:.2f}, "
            f"length={self.length:.2f}, "
            f"distance={self.distance:.2f}, "
            f"velocity_x={self.velocity_x:.2f}, "
            f"velocity_y={self.velocity_y:.2f}, "
            f"is_dynamic={self.is_dynamic}, "
            f"confidence={self.confidence:.2f}, "
            f"height={self.height:.2f}"
            f")"
        )
