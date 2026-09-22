import numpy as np


# ============================================================
# LiDAR Point Cloud Basic Preprocessing
#
# Module #1 - Basic Point Cloud Filtering
#
# Input / Output:
#   np.ndarray, shape = (N, 4)
#   [x, y, z, intensity]
#
# Coordinate convention:
#   +X = Right
#   -X = Left
#   +Y = Forward
#   -Y = Backward
#   +Z = Up
#   -Z = Down
#
# Responsibilities:
#   1. Invalid point removal
#   2. ROI filtering
#
# NOT performed here:
#   - Ground removal
#   - Clustering
#   - Object detection
#   - Tracking
#   - Vehicle control
# ============================================================


POINT_COLUMNS = 4

MIN_VALID_DISTANCE_M = 0.01


# ============================================================
# Input validation
# ============================================================

def validate_points(points):
    """
    Validate Point Cloud array.

    Parameters
    ----------
    points : ndarray
        Expected shape = (N, 4)

    Raises
    ------
    TypeError
        If points is not a numpy ndarray.

    ValueError
        If shape is not (N, 4).
    """

    if not isinstance(points, np.ndarray):
        raise TypeError(
            "points must be a numpy ndarray"
        )

    if (
        points.ndim != 2
        or
        points.shape[1] != POINT_COLUMNS
    ):
        raise ValueError(
            "points must have shape (N, 4)"
        )


# ============================================================
# Invalid point removal
# ============================================================

def remove_invalid_points(points):
    """
    Remove invalid LiDAR points.

    Removes:
        - NaN
        - Inf
        - points extremely close to sensor origin

    Parameters
    ----------
    points : ndarray, shape (N, 4)
        [x, y, z, intensity]

    Returns
    -------
    ndarray, shape (M, 4)
    """

    validate_points(points)


    if len(points) == 0:
        return points


    # --------------------------------------------------------
    # NaN / Inf
    # --------------------------------------------------------

    finite_mask = np.isfinite(
        points
    ).all(
        axis=1
    )


    filtered = points[
        finite_mask
    ]


    if len(filtered) == 0:
        return filtered


    # --------------------------------------------------------
    # Sensor-origin invalid points
    #
    # XYZ only. Intensity does not participate in distance.
    # --------------------------------------------------------

    distance = np.linalg.norm(
        filtered[:, :3],
        axis=1
    )


    valid_mask = (
        distance
        > MIN_VALID_DISTANCE_M
    )


    return filtered[
        valid_mask
    ]


# ============================================================
# ROI filtering
# ============================================================

def roi_filter(
    points,
    x_min=None,
    x_max=None,
    y_min=None,
    y_max=None,
    z_min=None,
    z_max=None
):
    """
    Apply rectangular 3D ROI filtering.

    Coordinate convention
    ---------------------
    X:
        negative = Left
        positive = Right

    Y:
        negative = Backward
        positive = Forward

    Z:
        negative = Down
        positive = Up

    A limit set to None is not applied.
    """

    validate_points(points)


    # --------------------------------------------------------
    # Validate ROI bounds
    # --------------------------------------------------------

    bounds = (
        ("x", x_min, x_max),
        ("y", y_min, y_max),
        ("z", z_min, z_max),
    )


    for axis, minimum, maximum in bounds:

        if (
            minimum is not None
            and
            maximum is not None
            and
            minimum > maximum
        ):

            raise ValueError(
                f"{axis}_min must be <= "
                f"{axis}_max"
            )


    if len(points) == 0:
        return points


    mask = np.ones(
        len(points),
        dtype=bool
    )


    # X : Left / Right

    if x_min is not None:
        mask &= (
            points[:, 0] >= x_min
        )

    if x_max is not None:
        mask &= (
            points[:, 0] <= x_max
        )


    # Y : Backward / Forward

    if y_min is not None:
        mask &= (
            points[:, 1] >= y_min
        )

    if y_max is not None:
        mask &= (
            points[:, 1] <= y_max
        )


    # Z : Down / Up

    if z_min is not None:
        mask &= (
            points[:, 2] >= z_min
        )

    if z_max is not None:
        mask &= (
            points[:, 2] <= z_max
        )


    return points[
        mask
    ]


# ============================================================
# Preprocessing pipeline
# ============================================================

def preprocess_points(
    points,
    roi=None
):
    """
    Basic Point Cloud preprocessing pipeline.

    Pipeline
    --------
    Raw Point Cloud
        ↓
    Invalid Point Removal
        ↓
    ROI Filtering
        ↓
    Filtered Point Cloud

    Parameters
    ----------
    points : ndarray, shape (N, 4)

    roi : dict or None
        Optional ROI limits.

    Returns
    -------
    filtered_points : ndarray, shape (M, 4)

    stats : dict
        raw   : input point count
        valid : point count after invalid removal
        roi   : final point count
    """

    validate_points(points)


    if (
        roi is not None
        and
        not isinstance(roi, dict)
    ):
        raise TypeError(
            "roi must be a dict or None"
        )


    raw_count = len(points)


    # --------------------------------------------------------
    # Invalid removal
    # --------------------------------------------------------

    filtered = remove_invalid_points(
        points
    )


    valid_count = len(
        filtered
    )


    # --------------------------------------------------------
    # ROI
    # --------------------------------------------------------

    if roi is not None:

        filtered = roi_filter(
            filtered,
            **roi
        )


    roi_count = len(
        filtered
    )


    # --------------------------------------------------------
    # Statistics
    # --------------------------------------------------------

    stats = {
        "raw": raw_count,
        "valid": valid_count,
        "roi": roi_count,
    }


    return (
        filtered,
        stats
    )
