"""LiDAR evidence gate for GPS-shadow post-detour camera lane handoff.

This is NOT an obstacle detector. It rejects near-center returns, tolerates
at most a few isolated FAR-center returns (typical sparse wall/noise), and
requires at least one observed side sector during post-detour handoff.
Missing the opposite wall does not itself prove an obstacle; the independent
LiDAR emergency and dynamic
object checks in the worker are always evaluated separately.
"""
import numpy as np


def evaluate_handoff_clearance(vehicle_points, cfg):
    points = np.asarray(vehicle_points, dtype=float)
    report = {
        "verified": False, "reason": "INVALID_CLOUD", "roi_points": 0,
        "near_hits": 0, "far_hits": 0, "left_support": 0,
        "right_support": 0,
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
    elif far_hits > max_far:
        report["reason"] = "FAR_CENTER_RETURNS"
    elif max(left, right) < support_min:
        # No reliable side reference on EITHER side: keep fail-stop.
        report["reason"] = "BOTH_SIDES_UNOBSERVED"
    else:
        report["verified"] = True
        if min(left, right) < support_min:
            # In a tunnel one wall may leave the LiDAR ROI while the
            # centre corridor stays clear. Advisory, not an obstruction.
            report["reason"] = "ONE_SIDE_OBSERVED_FORWARD_CLEAR"
        else:
            report["reason"] = (
                "FAR_SPARSE_RETURNS_TOLERATED" if far_hits else "CLEAR"
            )
    return report
