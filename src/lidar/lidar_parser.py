import numpy as np


# ============================================================
# MORAI VLP16 LiDAR Parser
#
# Module #1 - Packet Parsing
#
# Packet structure / distance / intensity / azimuth:
#   MORAI SensorExample LIDARprocess.py 기준
#
# VLP16 vertical laser angles:
#   VLP-16 User Manual Table 9-1 기준
#
# Output:
#   np.ndarray, shape = (N, 4)
#
#   columns:
#       [x, y, z, intensity]
#
# Coordinate convention:
#   +X = Right
#   -X = Left
#   +Y = Forward
#   -Y = Backward
#   +Z = Up
#   -Z = Down
# ============================================================


CHANNEL = 16

PACKET_SIZE = 1206
PAYLOAD_SIZE = 1200
BLOCK_SIZE = 100

MEASUREMENTS_PER_BLOCK = 32
FIRINGS_PER_BLOCK = 2


# ============================================================
# VLP-16 vertical angles
#
# VLP-16 User Manual Table 9-1
# Laser firing order
# ============================================================

VERTICAL_ANGLE_DEG = np.array(
    [[
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
    ]],
    dtype=np.float32
)


VERTICAL_ANGLE_RAD = np.deg2rad(
    VERTICAL_ANGLE_DEG
)


# ============================================================
# Coordinate conversion
# ============================================================

def sph2cart(distance, azimuth):
    """
    Convert MORAI VLP16 spherical measurements to Cartesian.

    Parameters
    ----------
    distance : ndarray, shape (N, 16)
        Distance in meters.

    azimuth : ndarray, shape (N, 1)
        Horizontal azimuth in degrees.

    Returns
    -------
    x, y, z : ndarray
        Flattened Cartesian coordinates.

    Coordinate convention
    ---------------------
    +X = Right
    +Y = Forward
    +Z = Up
    """

    distance = np.asarray(
        distance,
        dtype=np.float32
    )

    azimuth = np.asarray(
        azimuth,
        dtype=np.float32
    )


    if (
        distance.ndim != 2
        or
        distance.shape[1] != CHANNEL
    ):
        raise ValueError(
            "distance must have shape (N, 16)"
        )


    if (
        azimuth.ndim != 2
        or
        azimuth.shape[1] != 1
        or
        azimuth.shape[0] != distance.shape[0]
    ):
        raise ValueError(
            "azimuth must have shape (N, 1) "
            "and match distance rows"
        )


    azimuth_rad = np.deg2rad(
        azimuth
    )


    cos_vertical = np.cos(
        VERTICAL_ANGLE_RAD
    )


    x = (
        distance
        * cos_vertical
        * np.sin(azimuth_rad)
    )


    y = (
        distance
        * cos_vertical
        * np.cos(azimuth_rad)
    )


    z = (
        distance
        * np.sin(VERTICAL_ANGLE_RAD)
    )


    return (
        x.reshape(-1),
        y.reshape(-1),
        z.reshape(-1)
    )


# ============================================================
# MORAI VLP16 payload parser
# ============================================================

def parse_buffer(buffer):
    """
    Parse concatenated MORAI VLP16 UDP payloads.

    Parameters
    ----------
    buffer : bytes-like
        Concatenated 1200-byte LiDAR payloads.

        The final 6 bytes of each 1206-byte UDP packet must
        already have been removed by the receiver.

    Returns
    -------
    points : ndarray, shape (N, 4)
        [x, y, z, intensity]
    """

    if not isinstance(
        buffer,
        (bytes, bytearray, memoryview)
    ):
        raise TypeError(
            "buffer must be bytes-like"
        )


    buffer_length = len(buffer)


    if buffer_length == 0:

        return np.empty(
            (0, 4),
            dtype=np.float32
        )


    if buffer_length % BLOCK_SIZE != 0:

        raise ValueError(
            "LiDAR buffer size must be "
            "a multiple of 100 bytes"
        )


    # ========================================================
    # 100-byte measurement blocks
    # ========================================================

    buffer_np = np.frombuffer(
        buffer,
        dtype=np.uint8
    ).reshape(
        -1,
        BLOCK_SIZE
    )


    block_count = (
        buffer_np.shape[0]
    )


    # ========================================================
    # Azimuth
    #
    # MORAI SensorExample VLP16 handling:
    #
    # firing #1:
    #     base azimuth
    #
    # firing #2:
    #     base azimuth + 20 raw units
    #
    # 1 raw unit = 0.01 degree
    # therefore +20 = +0.20 degree
    # ========================================================

    base_azimuth_raw = (
        buffer_np[:, 2].astype(
            np.float32
        )
        +
        256.0
        * buffer_np[:, 3].astype(
            np.float32
        )
    )


    azimuth = np.empty(
        block_count * FIRINGS_PER_BLOCK,
        dtype=np.float32
    )


    azimuth[0::2] = (
        base_azimuth_raw
    )


    azimuth[1::2] = (
        base_azimuth_raw
        + 20.0
    )


    azimuth = (
        azimuth.reshape(-1, 1)
        / 100.0
    )


    azimuth = np.mod(
        azimuth,
        360.0
    )


    # ========================================================
    # Distance
    #
    # raw =
    #     low byte + 256 * high byte
    #
    # MORAI SensorExample conversion:
    #
    #     raw * 2 / 1000
    #
    # result:
    #     meters
    # ========================================================

    distance_raw = (
        buffer_np[:, 4::3].astype(
            np.float32
        )
        +
        256.0
        * buffer_np[:, 5::3].astype(
            np.float32
        )
    )


    if (
        distance_raw.shape[1]
        != MEASUREMENTS_PER_BLOCK
    ):
        raise ValueError(
            "Unexpected LiDAR measurement "
            "count in 100-byte block"
        )


    distance = (
        distance_raw
        * 2.0
        / 1000.0
    )


    # 32 measurements per block:
    #
    #   16 channels - firing #1
    #   16 channels - firing #2

    distance = distance.reshape(
        -1,
        CHANNEL
    )


    # ========================================================
    # Intensity
    # ========================================================

    intensity = (
        buffer_np[:, 6::3]
        .astype(np.float32)
        .reshape(-1)
    )


    # ========================================================
    # XYZ
    # ========================================================

    x, y, z = sph2cart(
        distance,
        azimuth
    )


    expected_points = (
        block_count
        * MEASUREMENTS_PER_BLOCK
    )


    if not (
        len(x)
        == len(y)
        == len(z)
        == len(intensity)
        == expected_points
    ):
        raise ValueError(
            "Parsed LiDAR array lengths "
            "are inconsistent"
        )


    # ========================================================
    # Final Point Cloud
    # ========================================================

    points = np.column_stack(
        (
            x,
            y,
            z,
            intensity
        )
    ).astype(
        np.float32,
        copy=False
    )


    return points
