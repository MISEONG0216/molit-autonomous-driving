from obstacle import Obstacle
from tracker import ObjectTracker


DT = 0.1


tracker = ObjectTracker(
    max_match_distance=1.0,
    velocity_window_size=3,
    dynamic_speed_threshold=0.5,
    dynamic_confirm_frames=3,
    max_missed_frames=3
)


def make_obstacle(x):

    return Obstacle(
        cluster_id=0,
        x=x,
        y=1.0,
        width=1.0,
        length=1.0,
        distance=x
    )


# ============================================================
# 테스트 Frame
#
# None = 해당 Frame에서 LiDAR 검출 실패
# ============================================================

frames = [

    10.00,     # Frame 1
    10.02,     # Frame 2

    None,      # Frame 3 - 검출 실패

    10.04,     # Frame 4 - 다시 검출

    None,      # Frame 5
    None,      # Frame 6

    10.06      # Frame 7 - 다시 검출
]


print()
print("========================================")
print("TRACK LOST TEST")
print("========================================")
print()


for frame_number, position in enumerate(
    frames,
    start=1
):

    if position is None:

        obstacles = []

    else:

        obstacles = [
            make_obstacle(
                position
            )
        ]


    result = tracker.update(
        obstacles,
        dt=DT,
        delta_x=0.0,
        delta_y=0.0,
        delta_yaw=0.0
    )


    print(
        f"FRAME {frame_number}"
    )


    if len(result) == 0:

        print(
            "  LiDAR detection : NONE"
        )

    else:

        for obstacle in result:

            print(
                f"  Detection       : "
                f"x={obstacle.x:.2f}"
            )

            print(
                f"  Track ID        : "
                f"{obstacle.track_id}"
            )


    print(
        "  Active tracks:"
    )


    if len(tracker.tracks) == 0:

        print(
            "    NONE"
        )


    for track_id, track in tracker.tracks.items():

        print(
            f"    Track {track_id} | "
            f"missed_frames="
            f"{track.missed_frames}"
        )


    print()
