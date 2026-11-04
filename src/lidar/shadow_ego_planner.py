"""GPS-shadow LiDAR detour planner (vehicle coordinates, no GPS waypoints).

+x is forward, +y is left and +z is up. Evaluate both sides with variable
lateral offsets and obstacle-dependent return distances. Every candidate must
clear all observed wall/obstacle returns along its swept footprint. Unobserved
space and emergency-close objects are NOT assumed safe. No vehicle controls.
"""
import numpy as np


def _distance_points_to_polyline(points, path):
    distances = np.full(len(points), np.inf, dtype=float)
    for a, b in zip(path[:-1], path[1:]):
        v = b - a
        t = np.clip(np.sum((points - a) * v, axis=1) /
                    max(float(np.dot(v, v)), 1e-9), 0.0, 1.0)
        distances = np.minimum(
            distances, np.linalg.norm(points - (a + t[:, None] * v), axis=1)
        )
    return distances


def _smoothstep(t):
    t = np.clip(t, 0.0, 1.0)
    return t * t * (3.0 - 2.0 * t)


def _path(side, cfg, offset=None, shift_end=None, return_start=None,
          return_end=None):
    """Polyline in forward-left metres, 0.5m sampling for legacy controller."""
    if offset is None:
        offset = float(cfg.get("offset_m", 1.6))
    if shift_end is None:
        shift_end = float(cfg.get("shift_end_m", 7.0))
    if return_start is None:
        return_start = float(cfg.get("return_start_m", 10.0))
    if return_end is None:
        return_end = float(cfg.get("horizon_m", 15.0))
    x = np.arange(0.0, return_end + 0.001, 0.5)
    enter_t = (x - 1.5) / max(0.1, shift_end - 1.5)
    exit_t = (x - return_start) / max(0.1, return_end - return_start)
    y = float(side) * float(offset) * _smoothstep(enter_t) * (
        1.0 - _smoothstep(exit_t)
    )
    return np.column_stack((x, y))


def _candidate_geometry(obstacle, cfg, clearance):
    """Longer/wider objects get a later rejoin; don't rejoin inside object."""
    first_x = float(np.min(obstacle[:, 0]))
    last_x = float(np.max(obstacle[:, 0]))
    # This is NOT a promise of free space. It is only a shape proposal.
    shift_end = min(float(cfg.get("shift_end_m", 7.0)),
                    first_x - float(cfg.get("pre_obstacle_buffer_m", 1.5)))
    if shift_end < float(cfg.get("min_shift_end_m", 4.0)):
        return None, "TOO_CLOSE_TO_SHIFT"
    return_start = max(float(cfg.get("return_start_m", 10.0)),
                       last_x + float(cfg.get("post_obstacle_hold_m", 2.5)))
    return_end = max(float(cfg.get("horizon_m", 15.0)),
                     return_start + float(cfg.get("return_length_m", 5.0)))
    if return_end > float(cfg.get("max_horizon_m", 24.0)):
        return None, "OBSTACLE_EXTENDS_BEYOND_HORIZON"
    return (shift_end, return_start, return_end), None


def choose_local_detour(points, cfg, active_side=None):
    """Return status/side/path/reason/diagnostics.

    This function is called only for NEW paths. The caller retains its
    single-path latch and rechecks its remaining path on future scans.
    """
    fail = {"status": "STOP_REQUIRED", "side": None, "path": [],
            "reason": "UNVERIFIED", "diagnostics": []}
    pts = np.asarray(points, dtype=float)
    if pts.ndim != 2 or pts.shape[1] < 3 or len(pts) < int(cfg.get("min_points", 80)):
        return dict(fail, reason="SPARSE_SCAN")
    pts = pts[np.isfinite(pts).all(axis=1)]
    pts = pts[(pts[:, 2] > float(cfg.get("z_min", -1.45))) &
              (pts[:, 2] < float(cfg.get("z_max", 1.25)))]
    max_horizon = float(cfg.get("max_horizon_m", 24.0))
    pts = pts[(pts[:, 0] >= 1.0) & (pts[:, 0] <= max_horizon + 1.0)]
    if len(pts) < int(cfg.get("min_points", 80)):
        return dict(fail, reason="SPARSE_FORWARD_SCAN")

    clearance = (float(cfg.get("vehicle_half_width_m", 0.9)) +
                 float(cfg.get("margin_m", 0.45)))
    front_close = pts[
        (pts[:, 0] <= float(cfg.get("emergency_forward_m", 2.5))) &
        (np.abs(pts[:, 1]) <= clearance)
    ]
    if len(front_close) >= int(cfg.get("obstacle_point_threshold", 5)):
        return dict(fail, reason="IMMEDIATE_OBSTACLE")

    obstacle = pts[
        (pts[:, 0] >= float(cfg.get("obstacle_min_forward_m", 3.0))) &
        (pts[:, 0] <= float(cfg.get("obstacle_max_forward_m", 12.0))) &
        (np.abs(pts[:, 1]) <= clearance)
    ]
    if len(obstacle) < int(cfg.get("obstacle_point_threshold", 5)) and active_side not in (1, -1):
        return {"status": "CLEAR", "side": None, "path": [],
                "reason": "EGO_LANE_CLEAR", "diagnostics": []}
    if len(obstacle) == 0:
        return dict(fail, reason="OBSTACLE_GEOMETRY_UNKNOWN")

    # A missing wall/side view is not proof of drivable space. Do not try
    # larger offsets when side visibility is insufficient.
    n = int(cfg.get("min_side_points", 8))
    side_counts = {}
    for sign, name in ((1, "LEFT"), (-1, "RIGHT")):
        side_counts[name] = int(np.count_nonzero(
            (pts[:, 1] * sign >= clearance) &
            (pts[:, 1] * sign <= float(cfg.get("side_observation_max_m", 7.0))) &
            (pts[:, 0] >= 2.0)
        ))
    if min(side_counts.values()) < n:
        return dict(fail, reason="SIDE_WALL_NOT_OBSERVED")

    geom, error = _candidate_geometry(obstacle, cfg, clearance)
    if error:
        return dict(fail, reason=error)
    shift_end, return_start, return_end = geom

    # A late return is accepted only when wall sectors are actually
    # observed near the proposed endpoint, not just near the ego vehicle.
    endpoint_window = float(cfg.get("end_visibility_window_m", 3.0))
    endpoint_min = int(cfg.get("end_visibility_min_points", 3))
    far_observed = {}
    for sign, name in ((1, "LEFT"), (-1, "RIGHT")):
        far_observed[name] = int(np.count_nonzero(
            (pts[:, 0] >= return_end - endpoint_window) &
            (pts[:, 0] <= return_end + 1.0) &
            (pts[:, 1] * sign >= clearance) &
            (pts[:, 1] * sign <= float(cfg.get("side_observation_max_m", 7.0)))
        ))
    if min(far_observed.values()) < endpoint_min:
        return dict(fail, reason="RETURN_CORRIDOR_UNOBSERVED",
                    diagnostics=[{"return_end_m": round(return_end, 2),
                                  "far_side_points": far_observed}])

    offsets = [float(x) for x in cfg.get("candidate_offsets_m", [1.6, 2.0, 2.4])]
    if not offsets or any(not np.isfinite(v) or v <= 0.0 for v in offsets):
        return dict(fail, reason="BAD_CANDIDATE_OFFSETS")
    candidates = []
    diagnostics = []
    side_order = (active_side,) if active_side in (1, -1) else (1, -1)
    for side in side_order:
        for offset in sorted(set(offsets)):
            path = _path(side, cfg, offset, shift_end, return_start, return_end)
            distances = _distance_points_to_polyline(pts[:, :2], path)
            hits = int(np.count_nonzero(distances < clearance))
            min_distance = float(np.min(distances))
            direction = "LEFT" if side == 1 else "RIGHT"
            diagnostics.append({
                "side": direction, "offset_m": offset,
                "shift_end_m": round(shift_end, 2),
                "return_start_m": round(return_start, 2),
                "return_end_m": round(return_end, 2),
                "hit_points": hits, "min_distance_m": round(min_distance, 3),
                "reason": "SAFE" if hits <= int(cfg.get("allowed_points", 0))
                          else "SWEEP_COLLISION",
            })
            if hits <= int(cfg.get("allowed_points", 0)):
                candidates.append((offset, 0 if side == 1 else 1, side, path))
    if not candidates:
        return dict(fail, reason="NO_FREE_SIDE", diagnostics=diagnostics)
    offset, _, side, path = min(candidates, key=lambda item: item[:2])
    return {"status": "AVOIDING", "side": side, "path": path.tolist(),
            "reason": "EGO_LIDAR_ADAPTIVE_LEFT" if side == 1
                      else "EGO_LIDAR_ADAPTIVE_RIGHT",
            "diagnostics": diagnostics}
