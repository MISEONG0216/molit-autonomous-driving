import numpy as np
from collections import deque


class Track:
    """
    하나의 추적 중인 객체 상태.

    obstacle:
        마지막으로 검출된 Obstacle

    missed_frames:
        연속으로 검출되지 않은 Frame 수
    """

    def __init__(self, track_id, obstacle):

        self.track_id = track_id
        self.obstacle = obstacle

        self.missed_frames = 0


class ObjectTracker:

    def __init__(
        self,
        max_match_distance=1.0,
        velocity_window_size=5,
        dynamic_speed_threshold=0.5,
        dynamic_confirm_frames=3,
        max_missed_frames=3
    ):

        # 현재 살아 있는 Track
        self.tracks = {}

        self.next_track_id = 0

        self.max_match_distance = (
            max_match_distance
        )

        self.velocity_window_size = (
            velocity_window_size
        )

        self.dynamic_speed_threshold = (
            dynamic_speed_threshold
        )

        self.dynamic_confirm_frames = (
            dynamic_confirm_frames
        )

        self.max_missed_frames = (
            max_missed_frames
        )


        # Track별 속도 기록
        self.velocity_history = {}

        # Track별 연속 동적 판정 횟수
        self.dynamic_count = {}


    # ========================================================
    # 거리 계산
    # ========================================================

    def calculate_distance(
        self,
        x1,
        y1,
        x2,
        y2
    ):

        return np.sqrt(
            (x1 - x2) ** 2
            +
            (y1 - y2) ** 2
        )


    # ========================================================
    # Ego Motion Compensation
    # ========================================================

    def transform_previous_position(
        self,
        x,
        y,
        delta_x,
        delta_y,
        delta_yaw
    ):

        translated_x = (
            x - delta_x
        )

        translated_y = (
            y - delta_y
        )


        cos_yaw = np.cos(
            delta_yaw
        )

        sin_yaw = np.sin(
            delta_yaw
        )


        predicted_x = (
            cos_yaw * translated_x
            +
            sin_yaw * translated_y
        )

        predicted_y = (
            -sin_yaw * translated_x
            +
            cos_yaw * translated_y
        )


        return (
            predicted_x,
            predicted_y
        )


    # ========================================================
    # Track 상태 초기화
    # ========================================================

    def initialize_track_state(
        self,
        track_id
    ):

        if track_id not in self.velocity_history:

            self.velocity_history[
                track_id
            ] = deque(
                maxlen=self.velocity_window_size
            )


        if track_id not in self.dynamic_count:

            self.dynamic_count[
                track_id
            ] = 0


    # ========================================================
    # Moving Average
    # ========================================================

    def filter_velocity(
        self,
        track_id,
        raw_velocity_x,
        raw_velocity_y
    ):

        self.initialize_track_state(
            track_id
        )


        history = self.velocity_history[
            track_id
        ]


        history.append(
            (
                raw_velocity_x,
                raw_velocity_y
            )
        )


        average_velocity_x = np.mean(
            [
                velocity[0]
                for velocity in history
            ]
        )

        average_velocity_y = np.mean(
            [
                velocity[1]
                for velocity in history
            ]
        )


        return (
            average_velocity_x,
            average_velocity_y
        )


    # ========================================================
    # Dynamic 판단
    # ========================================================

    def classify_dynamic(
        self,
        track_id,
        velocity_x,
        velocity_y
    ):

        self.initialize_track_state(
            track_id
        )


        speed = np.sqrt(
            velocity_x ** 2
            +
            velocity_y ** 2
        )


        if speed >= self.dynamic_speed_threshold:

            self.dynamic_count[
                track_id
            ] += 1

        else:

            self.dynamic_count[
                track_id
            ] = 0


        return (
            self.dynamic_count[track_id]
            >=
            self.dynamic_confirm_frames
        )


    # ========================================================
    # 새로운 Track 생성
    # ========================================================

    def create_track(
        self,
        obstacle
    ):

        track_id = self.next_track_id

        self.next_track_id += 1


        obstacle.track_id = track_id

        obstacle.velocity_x = 0.0
        obstacle.velocity_y = 0.0

        obstacle.is_dynamic = False


        self.initialize_track_state(
            track_id
        )


        self.tracks[
            track_id
        ] = Track(
            track_id,
            obstacle
        )


        return track_id


    # ========================================================
    # Track 삭제
    # ========================================================

    def delete_track(
        self,
        track_id
    ):

        if track_id in self.tracks:
            del self.tracks[track_id]


        if track_id in self.velocity_history:
            del self.velocity_history[track_id]


        if track_id in self.dynamic_count:
            del self.dynamic_count[track_id]


    # ========================================================
    # Tracking Update
    # ========================================================

    def update(
        self,
        current_obstacles,
        dt,
        delta_x=0.0,
        delta_y=0.0,
        delta_yaw=0.0
    ):

        if dt <= 0:

            raise ValueError(
                "dt must be greater than 0"
            )


        # ====================================================
        # Track이 하나도 없는 경우
        # ====================================================

        if len(self.tracks) == 0:

            for obstacle in current_obstacles:

                self.create_track(
                    obstacle
                )


            return current_obstacles


        # ====================================================
        # 이번 Frame에서 이미 사용된 Track
        # ====================================================

        matched_track_ids = set()


        # ====================================================
        # 현재 검출 객체 Matching
        # ====================================================

        for current in current_obstacles:

            best_track_id = None
            best_distance = float("inf")

            best_predicted_x = None
            best_predicted_y = None


            for track_id, track in self.tracks.items():

                if track_id in matched_track_ids:
                    continue


                previous = track.obstacle


                predicted_x, predicted_y = (
                    self.transform_previous_position(
                        previous.x,
                        previous.y,
                        delta_x,
                        delta_y,
                        delta_yaw
                    )
                )


                distance = self.calculate_distance(
                    predicted_x,
                    predicted_y,
                    current.x,
                    current.y
                )


                if distance < best_distance:

                    best_distance = distance

                    best_track_id = track_id

                    best_predicted_x = predicted_x
                    best_predicted_y = predicted_y


            # =================================================
            # 기존 Track과 Matching 성공
            # =================================================

            if (
                best_track_id is not None
                and
                best_distance
                <=
                self.max_match_distance
            ):

                track = self.tracks[
                    best_track_id
                ]


                current.track_id = (
                    best_track_id
                )


                residual_dx = (
                    current.x
                    -
                    best_predicted_x
                )

                residual_dy = (
                    current.y
                    -
                    best_predicted_y
                )


                raw_velocity_x = (
                    residual_dx / dt
                )

                raw_velocity_y = (
                    residual_dy / dt
                )


                filtered_vx, filtered_vy = (
                    self.filter_velocity(
                        best_track_id,
                        raw_velocity_x,
                        raw_velocity_y
                    )
                )


                current.velocity_x = (
                    filtered_vx
                )

                current.velocity_y = (
                    filtered_vy
                )


                current.is_dynamic = (
                    self.classify_dynamic(
                        best_track_id,
                        filtered_vx,
                        filtered_vy
                    )
                )


                # 다시 검출됐으므로 Lost count 초기화
                track.missed_frames = 0

                # Track의 최신 객체 갱신
                track.obstacle = current


                matched_track_ids.add(
                    best_track_id
                )


            # =================================================
            # Matching 실패 → 새로운 Track
            # =================================================

            else:

                new_track_id = (
                    self.create_track(
                        current
                    )
                )


                matched_track_ids.add(
                    new_track_id
                )


        # ====================================================
        # 이번 Frame에서 검출되지 않은 Track 처리
        # ====================================================

        tracks_to_delete = []


        for track_id, track in self.tracks.items():

            if track_id in matched_track_ids:
                continue


            track.missed_frames += 1


            if (
                track.missed_frames
                >
                self.max_missed_frames
            ):

                tracks_to_delete.append(
                    track_id
                )


        # ====================================================
        # 오래 Lost된 Track 삭제
        # ====================================================

        for track_id in tracks_to_delete:

            self.delete_track(
                track_id
            )


        return current_obstacles
