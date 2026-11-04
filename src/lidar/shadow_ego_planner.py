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


def _sweep_metrics(points, path, clearance, shift_end, return_start):
    """Per-return closest segment, stage and clearance (diagnostics only).

    Assign each LiDAR point to the path segment nearest to that point.
    Stage follows that closest segment's forward position, not the point's
    world X; all three hit counts therefore sum to the original total.
    """
    points = np.asarray(points, dtype=float)
    min_dist = np.full(len(points), np.inf, dtype=float)
    closest_stage = np.zeros(len(points), dtype=np.int8)
    for a, b in zip(path[:-1], path[1:]):
        v = b - a
        t = np.clip(np.sum((points - a) * v, axis=1) /
                    max(float(np.dot(v, v)), 1e-9), 0.0, 1.0)
        dist = np.linalg.norm(points - (a + t[:, None] * v), axis=1)
        better = dist < min_dist
        min_dist[better] = dist[better]
        x_at_segment = 0.5 * (a[0] + b[0])
        stage = (0 if x_at_segment < shift_end else
                 1 if x_at_segment <= return_start else 2)
        closest_stage[better] = stage
    hits = min_dist < clearance
    stages = ("ENTER", "PASS", "RETURN")
    counts = {name: int(np.count_nonzero(hits & (closest_stage == i)))
              for i, name in enumerate(stages)}
    minima = {
        name: (round(float(np.min(min_dist[closest_stage == i])), 3)
               if np.any(closest_stage == i) else None)
        for i, name in enumerate(stages)
    }
    return min_dist, counts, minima


def _smoothstep(t):
    t = np.clip(t, 0.0, 1.0)
    return t * t * (3.0 - 2.0 * t)


def _max_discrete_curvature(path):
    """Estimate maximum turn curvature (1/metre) on a sampled XY polyline.

    Geometric penalty only; not a vehicle steering feasibility guarantee.
    """
    xy = np.asarray(path, dtype=float)[:, :2]
    if len(xy) < 3:
        return float("inf")
    a = xy[1:-1] - xy[:-2]
    b = xy[2:] - xy[1:-1]
    la = np.linalg.norm(a, axis=1)
    lb = np.linalg.norm(b, axis=1)
    cross = np.abs(a[:, 0]*b[:, 1] - a[:, 1]*b[:, 0])
    chord = np.linalg.norm(xy[2:] - xy[:-2], axis=1)
    denominator = np.maximum(la * lb * chord, 1e-9)
    return float(np.max(2.0 * cross / denominator))


def score_verified_candidate(min_distance, clearance, max_curvature,
                             offset, delay, cfg):
    """Rank ONLY candidates already verified clear and observable.

    Reward excess clearance, penalize sharp curvature/lateral deviation and
    unnecessary return delay. Cap clearance reward to avoid favouring overly
    wide routes merely because an obstacle is far away.
    """
    if not all(np.isfinite(v) for v in (
            min_distance, clearance, max_curvature, offset, delay)):
        return float("-inf")
    excess = max(0.0, min(min_distance - clearance,
                          float(cfg.get("score_clearance_cap_m", 1.0))))
    return (float(cfg.get("score_clearance_weight", 2.0)) * excess
            - float(cfg.get("score_curvature_weight", 0.8)) * max_curvature
            - float(cfg.get("score_offset_weight", 0.2)) * offset
            - float(cfg.get("score_delay_weight", 0.03)) * delay)


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


def choose_local_detour(points, cfg, active_side=None, visibility_history=None):
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

    # Historic returns are aligned into current ego frame by the worker.
    # They supplement visibility and ADD collision evidence; only the
    # latest scan chooses the obstacle geometry / initiates the maneuver.
    observed = pts
    history_count = 0
    if visibility_history is not None:
        past = np.asarray(visibility_history, dtype=float)
        if past.ndim == 2 and past.shape[1] >= 3:
            past = past[np.isfinite(past[:, :3]).all(axis=1), :3]
            past = past[
                (past[:, 2] > float(cfg.get("z_min", -1.45))) &
                (past[:, 2] < float(cfg.get("z_max", 1.25))) &
                (past[:, 0] >= 1.) & (past[:, 0] <= max_horizon + 1.)
            ]
            if len(past):
                observed = np.vstack((pts[:, :3], past))
                history_count = len(past)

    clearance = (float(cfg.get("vehicle_half_width_m", 0.9)) +
                 float(cfg.get("margin_m", 0.45)))
    front_close = pts[
        (pts[:, 0] <= float(cfg.get("emergency_forward_m", 2.5))) &
        (np.abs(pts[:, 1]) <= clearance)
    ]
    if len(front_close) >= int(cfg.get("obstacle_point_threshold", 5)):
        return dict(fail, reason="IMMEDIATE_OBSTACLE")

    # Detect the object within the near window, then extend its observed
    # forward extent beyond that window. Otherwise a 10-17m long diagonal
    # vehicle can appear truncated at 12m and the path rejoins too early.
    central = pts[
        (pts[:, 0] >= float(cfg.get("obstacle_min_forward_m", 3.0))) &
        (pts[:, 0] <= max_horizon) &
        (np.abs(pts[:, 1]) <= clearance)
    ]
    front_seed = central[
        central[:, 0] <= float(cfg.get("obstacle_max_forward_m", 12.0))
    ]
    threshold = int(cfg.get("obstacle_point_threshold", 5))
    if len(front_seed) < threshold and active_side not in (1, -1):
        return {"status": "CLEAR", "side": None, "path": [],
                "reason": "EGO_LANE_CLEAR", "diagnostics": []}
    if len(front_seed) == 0:
        return dict(fail, reason="OBSTACLE_GEOMETRY_UNKNOWN")
    # Group returns by contiguous forward distance. Far separate obstacles
    # must not silently be merged with the nearest one; they are still
    # checked by the swept-corridor collision test for every candidate.
    central = central[np.argsort(central[:, 0])]
    gap = float(cfg.get("obstacle_extent_gap_m", 0.8))
    split = np.flatnonzero(np.diff(central[:, 0]) > gap) + 1
    groups = np.split(central, split)
    matching = [group for group in groups
                if len(group) >= threshold and
                np.min(group[:, 0]) <= float(cfg.get(
                    "obstacle_max_forward_m", 12.0))]
    if not matching:
        return dict(fail, reason="OBSTACLE_GEOMETRY_UNKNOWN")
    obstacle = min(matching, key=lambda group: float(np.min(group[:, 0])))

    # A missing wall/side view is not proof of drivable space. Do not try
    # larger offsets when side visibility is insufficient.
    n = int(cfg.get("min_side_points", 8))
    side_counts = {}
    for sign, name in ((1, "LEFT"), (-1, "RIGHT")):
        side_counts[name] = int(np.count_nonzero(
            (observed[:, 1] * sign >= clearance) &
            (observed[:, 1] * sign <= float(cfg.get("side_observation_max_m", 7.0))) &
            (observed[:, 0] >= 2.0)
        ))
    if min(side_counts.values()) < n:
        return dict(fail, reason="SIDE_WALL_NOT_OBSERVED")

    geom, error = _candidate_geometry(obstacle, cfg, clearance)
    if error:
        return dict(fail, reason=error)
    shift_end, return_start, return_end = geom

    # Return timing is a candidate dimension, not a single fixed value.
    # Never create an unobserved return: each candidate must independently
    # meet BOTH far-side observation and full swept-footprint constraints.
    endpoint_window = float(cfg.get("end_visibility_window_m", 3.0))
    endpoint_min = int(cfg.get("end_visibility_min_points", 3))
    offsets = [float(v) for v in cfg.get(
        "candidate_offsets_m", [1.6, 1.8, 2.0, 2.2, 2.4, 2.6, 2.8])]
    diagnostic_offsets = [float(v) for v in cfg.get(
        "diagnostic_offsets_m", [2.9, 3.0, 3.1, 3.2])]
    delay_options = [float(v) for v in cfg.get(
        "return_delay_options_m", [0.0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0])]
    if (not offsets or not delay_options or
            any(not np.isfinite(v) or v <= 0.0
                for v in offsets + diagnostic_offsets) or
            any(not np.isfinite(v) or v < 0.0 for v in delay_options)):
        return dict(fail, reason="BAD_CANDIDATE_CONFIG")
    active_offsets = set(offsets)
    all_offsets = sorted(active_offsets | set(diagnostic_offsets))
    side_order = (active_side,) if active_side in (1, -1) else (1, -1)
    candidates = []
    diagnostics = []
    visibility_passed = False
    max_path = float(cfg.get("max_horizon_m", 24.0))
    min_return_length = float(cfg.get("return_length_m", 5.0))
    for delay in sorted(set(delay_options)):
        candidate_return_start = return_start + delay
        candidate_return_end = max(
            float(cfg.get("horizon_m", 15.0)),
            candidate_return_start + min_return_length,
        )
        within_horizon = candidate_return_end <= max_path + 1e-6
        far_observed = {}
        for sign, name in ((1, "LEFT"), (-1, "RIGHT")):
            far_observed[name] = int(np.count_nonzero(
                (observed[:, 0] >= candidate_return_end - endpoint_window) &
                (observed[:, 0] <= candidate_return_end + 1.0) &
                (observed[:, 1] * sign >= clearance) &
                (observed[:, 1] * sign <= float(
                    cfg.get("side_observation_max_m", 7.0)))
            ))
        return_visible = (within_horizon and
                          min(far_observed.values()) >= endpoint_min)
        visibility_passed = visibility_passed or return_visible
        diagnostics.append({
            "return_start_m": round(candidate_return_start, 2),
            "return_end_m": round(candidate_return_end, 2),
            "return_delay_m": round(delay, 2),
            "far_side_points": far_observed,
            "history_obstacle_points": history_count,
            "required_points_per_side": endpoint_min,
            "visibility_ok": return_visible,
            "reason": ("OBSERVED" if return_visible else
                       "RETURN_HORIZON_EXCEEDED" if not within_horizon else
                       "RETURN_CORRIDOR_UNOBSERVED"),
        })
        # Endpoints beyond maximum LiDAR planning horizon are not sampled
        # or approved. All other candidates get diagnostics, even if their
        # endpoint coverage fails, to reveal PASS vs RETURN collisions.
        if not within_horizon:
            continue
        for side in side_order:
            for offset in all_offsets:
                path = _path(
                    side, cfg, offset, shift_end,
                    candidate_return_start, candidate_return_end
                )
                distances, stage_hits, stage_minima = _sweep_metrics(
                    observed[:, :2], path, clearance, shift_end,
                    candidate_return_start
                )
                hits = int(sum(stage_hits.values()))
                diagnostic_only = offset not in active_offsets
                collision_free = hits <= int(cfg.get("allowed_points", 0))
                reason = ("SWEEP_COLLISION" if not collision_free else
                          "RETURN_CORRIDOR_UNOBSERVED" if not return_visible else
                          "DIAGNOSTIC_ONLY" if diagnostic_only else "SAFE")
                min_distance = float(np.min(distances))
                max_curvature = _max_discrete_curvature(path)
                candidate_score = (score_verified_candidate(
                    min_distance, clearance, max_curvature, offset,
                    delay, cfg) if (collision_free and return_visible
                                    and not diagnostic_only) else None)
                diagnostics.append({
                    "side": "LEFT" if side == 1 else "RIGHT",
                    "offset_m": offset,
                    "score": (round(candidate_score, 5)
                              if candidate_score is not None else None),
                    "max_curvature_1pm": round(max_curvature, 5),
                    "diagnostic_only": diagnostic_only,
                    "return_delay_m": round(delay, 2),
                    "shift_end_m": round(shift_end, 2),
                    "return_start_m": round(candidate_return_start, 2),
                    "return_end_m": round(candidate_return_end, 2),
                    "far_side_points": far_observed,
                    "visibility_ok": return_visible,
                    "hit_points": hits,
                    "min_distance_m": round(min_distance, 3),
                    "stage_hits": stage_hits,
                    "stage_min_distance_m": stage_minima,
                    "reason": reason,
                })
                if candidate_score is not None:
                    # Evaluate score ONLY after original sweep/visibility gates.
                    # Never let diagnostic-only offsets authorize steering.
                    candidates.append((
                        -candidate_score, offset, delay,
                        0 if side == 1 else 1, side, path,
                        min_distance, max_curvature
                    ))
    if not candidates:
        return dict(
            fail,
            reason=("NO_FREE_SIDE" if visibility_passed
                    else "RETURN_CORRIDOR_UNOBSERVED"),
            diagnostics=diagnostics,
        )
    neg_score, offset, delay, _, side, path, min_distance, max_curvature = min(
        candidates, key=lambda item: item[:4])
    return {"status": "AVOIDING", "side": side, "path": path.tolist(),
            "selected_offset_m": offset, "selected_return_delay_m": delay,
            "selected_score": round(-neg_score, 5),
            "selected_min_distance_m": round(min_distance, 3),
            "selected_max_curvature_1pm": round(max_curvature, 5),
            "reason": "EGO_LIDAR_ADAPTIVE_LEFT" if side == 1
                      else "EGO_LIDAR_ADAPTIVE_RIGHT",
            "diagnostics": diagnostics}
