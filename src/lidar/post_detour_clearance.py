"""GPS-shadow post-detour gate: use current lane steering ONLY as a
conservative predicted swept-corridor check when far-centre LiDAR hits exceed
the old count threshold. An uncalibrated/unavailable lane steer NEVER clears
a failure. New route approval and emergency checks are separate.
"""
import math
import numpy as np


def _trajectory_clearance(roi, steer, cfg, half):
    """Nearest distance from all 4-12m returns to a bicycle-model arc.

    Same vehicle forward-left coordinates as LiDAR. The returned margin is
    point-to-centreline distance minus body half-width plus original margin.
    It is an estimate, NOT a guaranteed camera lane localisation.
    """
    wheelbase = float(cfg.get("post_detour_path_wheelbase_m", 2.7))
    max_abs = float(cfg.get("post_detour_max_lane_steer_rad", 0.22))
    if (not math.isfinite(wheelbase) or wheelbase <= 1.5 or
            not math.isfinite(steer) or abs(steer) > max_abs):
        return None
    k = math.tan(steer) / wheelbase
    x = np.linspace(0., 12., 49)
    if abs(k) * 12. >= 0.7:  # curved path outside reliable short-arc range
        return None
    if abs(k) < 1e-7:
        y = np.zeros_like(x)
    else:
        y = (1. - np.sqrt(1. - (k*x)**2)) / k
    path = np.column_stack((x, y))
    relevant = roi[roi[:, 0] >= float(cfg.get("post_detour_near_limit_m", 4.))]
    if len(relevant) == 0:
        return float("inf"), float(y[-1])
    best = np.full(len(relevant), np.inf)
    for a, b in zip(path[:-1], path[1:]):
        v = b-a
        t = np.clip(np.sum((relevant[:, :2]-a)*v, axis=1) /
                    max(np.dot(v, v), 1e-8), 0., 1.)
        d = np.linalg.norm(relevant[:, :2]-(a+t[:, None]*v), axis=1)
        best = np.minimum(best, d)
    return float(np.min(best)-half), float(y[-1])


def evaluate_handoff_clearance(vehicle_points, cfg, lane_steering=None):
    points = np.asarray(vehicle_points, dtype=float)
    report = {
        "verified": False, "reason": "INVALID_CLOUD", "roi_points": 0,
        "near_hits": 0, "far_hits": 0, "left_support": 0,
        "right_support": 0, "trajectory_margin_m": None,
        "trajectory_end_lateral_m": None,
    }
    if points.ndim != 2 or points.shape[1] < 3:
        return report
    cloud = points[np.isfinite(points[:, :3]).all(axis=1), :3]
    roi = cloud[
        (cloud[:, 0] >= 1.0) & (cloud[:, 0] <= 12.0)
        & (cloud[:, 2] >= float(cfg.get("z_min", -1.45)))
        & (cloud[:, 2] <= float(cfg.get("z_max", 1.25)))
    ]
    half = (float(cfg.get("vehicle_half_width_m", 0.9))
            + float(cfg.get("margin_m", 0.45)))
    near_end = float(cfg.get("post_detour_near_limit_m", 4.0))
    max_far = int(cfg.get("post_detour_far_allowed_points", 2))
    support_min = int(cfg.get("post_detour_min_side_points", 8))
    central = np.abs(roi[:, 1]) <= half
    near_hits = int(np.count_nonzero(
        central & (roi[:, 0] < near_end)))
    far_hits = int(np.count_nonzero(
        central & (roi[:, 0] >= near_end)))
    left = int(np.count_nonzero(
        (roi[:, 0] >= near_end) & (roi[:, 1] >= half)
        & (roi[:, 1] <= 7.0)))
    right = int(np.count_nonzero(
        (roi[:, 0] >= near_end) & (roi[:, 1] <= -half)
        & (roi[:, 1] >= -7.0)))
    report.update(
        roi_points=len(roi), near_hits=near_hits, far_hits=far_hits,
        left_support=left, right_support=right,
    )
    if not 3.0 <= near_end <= 6.0 or not 0 <= max_far <= 3 or support_min < 1:
        report["reason"] = "INVALID_CONFIG"
    elif len(roi) < int(cfg.get("post_detour_min_roi_points", 80)):
        report["reason"] = "SPARSE_FORWARD_SCAN"
    elif near_hits > 0:
        report["reason"] = "NEAR_CENTER_RETURN"
    elif max(left, right) < support_min:
        report["reason"] = "BOTH_SIDES_UNOBSERVED"
    elif far_hits > max_far:
        if lane_steering is None:
            report["reason"] = "FAR_HITS_NO_LANE_TARGET"
            return report
        clearance = _trajectory_clearance(roi, float(lane_steering), cfg, half)
        if clearance is None:
            report["reason"] = "INVALID_LANE_TRAJECTORY"
            return report
        margin, offset = clearance
        report["trajectory_margin_m"] = round(margin, 3)
        report["trajectory_end_lateral_m"] = round(offset, 3)
        # Require a genuine lateral trajectory; never declare a straight
        # obstacle-free path based on an insignificant steering correction.
        if abs(offset) < float(cfg.get("post_detour_min_curve_offset_m", 0.6)):
            report["reason"] = "FAR_HITS_STRAIGHT_PATH"
        elif margin < float(cfg.get("post_detour_path_extra_margin_m", 0.15)):
            report["reason"] = "PREDICTED_PATH_OBSTRUCTED"
        else:
            report["verified"] = True
            report["reason"] = "FAR_HITS_OUTSIDE_PREDICTED_PATH"
    else:
        report["verified"] = True
        if min(left, right) < support_min:
            report["reason"] = "ONE_SIDE_OBSERVED_FORWARD_CLEAR"
        else:
            report["reason"] = (
                "FAR_SPARSE_RETURNS_TOLERATED" if far_hits else "CLEAR"
            )
    return report
