"""One-shot, LiDAR-verified RIGHT lane change after first shadow LEFT detour.

Vehicle frame: forward +x, left +y. No control commands are sent here.
This intentionally does not claim painted lane recognition or road-rule compliance.
"""
import math
import numpy as np

from shadow_wall_guard import check_wall_clearance, close_forward_collision


def _smoothstep(t):
    t = np.clip(t, 0.0, 1.0)
    return t * t * (3.0 - 2.0 * t)


def right_shift_local_path(offset, cfg):
    """Smooth right shift, then HOLD the offset (no immediate return)."""
    shift_start = float(cfg.get("shift_start_m", 1.5))
    shift_end = float(cfg.get("shift_end_m", 10.0))
    horizon = float(cfg.get("horizon_m", 18.0))
    if (not math.isfinite(offset) or offset <= 0 or
            not (0 <= shift_start < shift_end < horizon <= 18.0)):
        raise ValueError("invalid right-shift shape")
    x = np.arange(0.0, horizon + 0.001, 0.5)
    y = -offset * _smoothstep((x - shift_start) /
                               max(0.1, shift_end - shift_start))
    return np.column_stack((x, y))


def plan_verified_right_shift(vehicle_points, ego_pose, wall_cfg, cfg):
    """Return dict with SAFE path or STOP (never bypasses wall-sweep checks).

    This function must be called by the worker ONLY after the existing
    obstacle planner reported CLEAR and the requested wait has elapsed.
    """
    result = {"status": "STOP_REQUIRED", "reason": "RIGHT_SHIFT_UNVERIFIED",
              "path": [], "offset_m": None, "diagnostics": []}
    pts = np.asarray(vehicle_points, dtype=float)
    if pts.ndim != 2 or pts.shape[1] < 3 or not np.isfinite(pts).all():
        result["reason"] = "RIGHT_SHIFT_BAD_SCAN"
        return result
    if close_forward_collision(pts, wall_cfg):
        result["reason"] = "RIGHT_SHIFT_FORWARD_BLOCKED"
        return result

    x0, y0, yaw = (float(v) for v in ego_pose)
    if not all(math.isfinite(v) for v in (x0, y0, yaw)):
        result["reason"] = "RIGHT_SHIFT_INVALID_POSE"
        return result
    cy, sy = math.cos(yaw), math.sin(yaw)
    offsets = cfg.get("offsets_m", [2.4, 2.8])
    for value in offsets:
        offset = float(value)
        path = right_shift_local_path(offset, cfg)
        xy_global = np.column_stack((
            x0 + cy * path[:, 0] - sy * path[:, 1],
            y0 + sy * path[:, 0] + cy * path[:, 1],
            np.zeros(len(path)),
        ))
        alignment = {}
        clear, reason = check_wall_clearance(
            xy_global, ego_pose, pts, wall_cfg,
            alignment_diagnostics=alignment,
        )
        result["diagnostics"].append({
            "offset_m": offset,
            "reason": reason,
            "verified": bool(clear),
            "alignment_error_m": alignment.get("segment_error_m"),
        })
        if clear:
            return {
                "status": "AVOIDING",
                "reason": "RIGHT_SHIFT_VERIFIED",
                "path": xy_global,
                "offset_m": offset,
                "diagnostics": result["diagnostics"],
            }
    result["reason"] = "RIGHT_SHIFT_NO_VERIFIED_CORRIDOR"
    return result
