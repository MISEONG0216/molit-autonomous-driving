import socket
import struct
import time

import numpy as np


# ============================================================
# MORAI VLP16 UDP
# ============================================================

LIDAR_IP = "127.0.0.1"
LIDAR_PORT = 1116

BUFFER_SIZE = 65535
SOCKET_TIMEOUT = 1.0


# ============================================================
# VLP16 packet configuration
# ============================================================

PACKET_SIZE = 1206

BLOCK_COUNT = 12
BLOCK_SIZE = 100

POINTS_PER_BLOCK = 32
LASERS_PER_FIRING = 16

DISTANCE_RESOLUTION = 0.002


VERTICAL_ANGLES_DEG = np.array(
    [
        -15.0,
          1.0,
        -13.0,
          3.0,
        -11.0,
          5.0,
         -9.0,
          7.0,
         -7.0,
          9.0,
         -5.0,
         11.0,
         -3.0,
         13.0,
         -1.0,
         15.0,
    ],
    dtype=np.float64
)

VERTICAL_ANGLES_RAD = np.deg2rad(
    VERTICAL_ANGLES_DEG
)


# ============================================================
# LiDAR Receiver
# ============================================================

class MoraiLidarReceiver:

    def __init__(
        self,
        ip=LIDAR_IP,
        port=LIDAR_PORT,
    ):

        self.ip = ip
        self.port = port

        self.sock = socket.socket(
            socket.AF_INET,
            socket.SOCK_DGRAM
        )

        self.sock.setsockopt(
            socket.SOL_SOCKET,
            socket.SO_REUSEADDR,
            1
        )

        self.sock.bind(
            (self.ip, self.port)
        )

        self.sock.settimeout(
            SOCKET_TIMEOUT
        )

        self.packet_count = 0
        self.total_point_count = 0

        self.start_time = time.monotonic()


    # ========================================================
    # UDP packet
    # ========================================================

    def receive_packet(self):

        try:

            data, addr = self.sock.recvfrom(
                BUFFER_SIZE
            )

        except socket.timeout:

            return None, None

        self.packet_count += 1

        return data, addr


    # ========================================================
    # 1206-byte VLP16 packet -> Nx3
    #
    # output:
    #
    # [
    #   forward,
    #   left,
    #   sensor_z
    # ]
    #
    # +forward : Ego front
    # +left    : Ego left
    # -left    : Ego right
    # ========================================================

    def parse_packet(
        self,
        data
    ):

        if len(data) != PACKET_SIZE:

            return np.empty(
                (0, 3),
                dtype=np.float32
            )


        block_azimuths = []


        # ----------------------------------------------------
        # Read azimuth of all 12 blocks
        # ----------------------------------------------------

        for block_idx in range(
            BLOCK_COUNT
        ):

            block_offset = (
                block_idx
                * BLOCK_SIZE
            )


            flag = data[
                block_offset:
                block_offset + 2
            ]


            if flag != b"\xff\xee":

                return np.empty(
                    (0, 3),
                    dtype=np.float32
                )


            azimuth_raw = struct.unpack_from(
                "<H",
                data,
                block_offset + 2
            )[0]


            azimuth_deg = (
                azimuth_raw
                / 100.0
            )


            block_azimuths.append(
                azimuth_deg
            )


        points = []


        # ----------------------------------------------------
        # Convert returns into XYZ
        # ----------------------------------------------------

        for block_idx in range(
            BLOCK_COUNT
        ):

            block_offset = (
                block_idx
                * BLOCK_SIZE
            )


            current_azimuth = (
                block_azimuths[
                    block_idx
                ]
            )


            if block_idx < BLOCK_COUNT - 1:

                next_azimuth = (
                    block_azimuths[
                        block_idx + 1
                    ]
                )


                azimuth_diff = (
                    next_azimuth
                    - current_azimuth
                ) % 360.0

            else:

                prev_azimuth = (
                    block_azimuths[
                        block_idx - 1
                    ]
                )


                azimuth_diff = (
                    current_azimuth
                    - prev_azimuth
                ) % 360.0


            for point_idx in range(
                POINTS_PER_BLOCK
            ):

                point_offset = (
                    block_offset
                    + 4
                    + point_idx * 3
                )


                distance_raw = struct.unpack_from(
                    "<H",
                    data,
                    point_offset
                )[0]


                if distance_raw == 0:

                    continue


                distance = (
                    distance_raw
                    * DISTANCE_RESOLUTION
                )


                laser_id = (
                    point_idx
                    % LASERS_PER_FIRING
                )


                firing_id = (
                    point_idx
                    // LASERS_PER_FIRING
                )


                if firing_id == 0:

                    azimuth_deg = (
                        current_azimuth
                    )

                else:

                    azimuth_deg = (
                        current_azimuth
                        + azimuth_diff * 0.5
                    ) % 360.0


                azimuth_rad = np.deg2rad(
                    azimuth_deg
                )


                vertical_rad = (
                    VERTICAL_ANGLES_RAD[
                        laser_id
                    ]
                )


                cos_vertical = np.cos(
                    vertical_rad
                )


                # ---------------------------------------------
                # Velodyne sensor coordinate
                # ---------------------------------------------

                sensor_x = (
                    distance
                    * cos_vertical
                    * np.sin(
                        azimuth_rad
                    )
                )


                sensor_y = (
                    distance
                    * cos_vertical
                    * np.cos(
                        azimuth_rad
                    )
                )


                sensor_z = (
                    distance
                    * np.sin(
                        vertical_rad
                    )
                )


                # ---------------------------------------------
                # Final coordinate convention
                #
                # Verified in MORAI:
                #
                # sensor_y = forward
                # sensor_x = right
                #
                # Therefore:
                #
                # forward = sensor_y
                # left    = -sensor_x
                # ---------------------------------------------

                forward = sensor_y

                left = -sensor_x


                points.append(
                    [
                        forward,
                        left,
                        sensor_z,
                    ]
                )


        if not points:

            return np.empty(
                (0, 3),
                dtype=np.float32
            )


        points = np.asarray(
            points,
            dtype=np.float32
        )


        self.total_point_count += (
            len(points)
        )


        return points


    # ========================================================
    # Receive one UDP packet and return its points
    # ========================================================

    def receive_points(self):

        data, _ = (
            self.receive_packet()
        )


        if data is None:

            return None


        return self.parse_packet(
            data
        )


    # ========================================================
    # Accumulate several UDP packets into one short scan
    #
    # default:
    # approximately 0.10 sec
    # ========================================================

    def receive_scan(
        self,
        duration_s=0.10,
    ):

        scan_parts = []


        start = time.monotonic()


        while (
            time.monotonic()
            - start
            < duration_s
        ):

            points = (
                self.receive_points()
            )


            if points is None:

                continue


            if len(points) == 0:

                continue


            scan_parts.append(
                points
            )


        if not scan_parts:

            return np.empty(
                (0, 3),
                dtype=np.float32
            )


        return np.vstack(
            scan_parts
        )


    # ========================================================

    def close(self):

        try:

            self.sock.close()

        except OSError:

            pass


# ============================================================
# Standalone test
# ============================================================

def main():

    receiver = (
        MoraiLidarReceiver()
    )


    print("=" * 72)
    print(" MORAI VLP16 LiDAR Receiver")
    print("=" * 72)

    print(
        f"Listening : "
        f"{LIDAR_IP}:{LIDAR_PORT}"
    )

    print(
        "Output    : "
        "Nx3 [forward, left, sensor_z]"
    )

    print()

    print(
        "+forward = Ego front"
    )

    print(
        "+left    = Ego left"
    )

    print(
        "-left    = Ego right"
    )

    print()

    print(
        "Press Ctrl+C to stop."
    )

    print()


    try:

        while True:

            scan = (
                receiver.receive_scan(
                    duration_s=0.10
                )
            )


            if len(scan) == 0:

                print(
                    "[WAIT] "
                    "No LiDAR points"
                )

                continue


            # ---------------------------------------------
            # Simple front-object diagnostic only
            # ---------------------------------------------

            mask = (
                (scan[:, 0] >= 1.0)
                &
                (scan[:, 0] <= 8.0)
                &
                (
                    np.abs(
                        scan[:, 1]
                    )
                    <= 3.0
                )
                &
                (scan[:, 2] >= -1.60)
                &
                (scan[:, 2] <= 2.00)
            )


            obstacle = scan[
                mask
            ]


            print(
                f"[SCAN] "
                f"points="
                f"{len(scan):5d}"
                f" | "
                f"obstacle_points="
                f"{len(obstacle):4d}"
            )


            if len(obstacle) > 0:

                center = np.mean(
                    obstacle,
                    axis=0
                )


                print(
                    f"[CENTER] "
                    f"forward="
                    f"{center[0]:.2f} m"
                    f" | "
                    f"left="
                    f"{center[1]:.2f} m"
                    f" | "
                    f"sensor_z="
                    f"{center[2]:.2f} m"
                )


            print(
                "-" * 72
            )


    except KeyboardInterrupt:

        print()

        print(
            "[STOP] "
            "LiDAR receiver stopped."
        )


    finally:

        receiver.close()


if __name__ == "__main__":

    main()