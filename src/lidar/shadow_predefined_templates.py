"""Predefined left-then-right GPS-shadow LiDAR detours (Python 3.8).

A fixed manoeuvre SHAPE is read from configuration, anchored to a newly
observed static obstacle in the CURRENT vehicle frame. No GPS waypoints.
This module does not send vehicle controls or modify camera lane following.
"""
import numpy as np
from shadow_ego_planner import _path, _sweep_metrics


def choose_template_detour(points, config, completed_count=0):
    empty = dict(status="STOP_REQUIRED", side=None, path=[],
                 reason="NO_VERIFIED_TEMPLATE", diagnostics=[],
                 template_id=None)
    if not config.get("enabled", False):
        return dict(empty, reason="TEMPLATES_DISABLED")
    templates = config["templates"]
    if completed_count >= len(templates):
        return dict(status="CLEAR", side=None, path=[],
                    reason="ALL_TEMPLATES_COMPLETED", diagnostics=[],
                    template_id=None)
    template = templates[completed_count]
    name = template["id"]
    side = 1 if template["side"] == "LEFT" else -1
    pts = np.asarray(points, dtype=float)
    if pts.ndim != 2 or pts.shape[1] < 3:
        return dict(empty, reason="BAD_SCAN", template_id=name)
    pts = pts[np.isfinite(pts).all(axis=1)]
    zmin, zmax = config["lidar_z_range_m"]
    pts = pts[(pts[:, 2] >= zmin) & (pts[:, 2] <= zmax) &
              (pts[:, 0] >= 1.0) &
              (pts[:, 0] <= config["max_path_horizon_m"] + 1.0)]
    if len(pts) < int(config["min_total_points"]):
        return dict(empty, reason="SPARSE_SCAN", template_id=name)

    clearance = float(config["vehicle_half_width_m"] +
                      config["safety_margin_m"])
    fmin, fmax = (float(x) for x in config["detection_forward_m"])
    center_limit = float(config["detection_lateral_abs_max_m"])
    center = pts[(np.abs(pts[:, 1]) <= center_limit) &
                 (pts[:, 0] >= 2.0)]
    too_close = center[center[:, 0] < fmin]
    near_window = center[(center[:, 0] >= fmin) &
                         (center[:, 0] <= fmax)]
    threshold = int(config["min_obstacle_points"])
    if len(too_close) >= threshold:
        return dict(empty, reason="OBSTACLE_TOO_CLOSE_TO_TEMPLATE",
                    template_id=name)
    if len(near_window) < threshold:
        return dict(status="CLEAR", side=None, path=[],
                    reason="WAITING_FOR_TEMPLATE_OBSTACLE",
                    diagnostics=[], template_id=name)

    # Determine nearest *connected longitudinal group*, not all obstacles
    # across the tunnel. Everything else remains in the sweep test.
    groups = np.split(
        center[np.argsort(center[:, 0])],
        np.flatnonzero(np.diff(np.sort(center[:, 0])) > .8) + 1,
    )
    valid = [group for group in groups
             if len(group) >= threshold and
             float(np.min(group[:, 0])) <= fmax]
    if not valid:
        return dict(empty, reason="OBSTACLE_GROUP_UNKNOWN", template_id=name)
    obstacle = min(valid, key=lambda group: float(np.min(group[:, 0])))
    first_x = float(np.min(obstacle[:, 0]))
    last_x = float(np.max(obstacle[:, 0]))
    base_shift_end = first_x - float(
        config["shift_clearance_before_obstacle_m"])
    if base_shift_end < float(config["min_shift_distance_m"]):
        return dict(empty, reason="NOT_ENOUGH_SHIFT_DISTANCE",
                    template_id=name)

    min_side = int(config["min_side_points"])
    side_counts = {}
    for direction, label in ((1, "LEFT"), (-1, "RIGHT")):
        side_counts[label] = int(np.count_nonzero(
            (pts[:, 0] >= 2.0) &
            (pts[:, 1] * direction >= clearance) &
            (pts[:, 1] * direction <= 8.0)
        ))
    if min(side_counts.values()) < min_side:
        return dict(empty, reason="SIDE_WALL_NOT_OBSERVED",
                    diagnostics=[{"side_counts": side_counts}],
                    template_id=name)
    base_return_start = last_x + float(config["hold_after_obstacle_m"])
    diagnostics = []
    approved = []
    for offset in (float(v) for v in template["offset_options_m"]):
        for delay in (float(v) for v in
                      config["return_delay_options_m"]):
            return_start = base_return_start + delay
            return_end = return_start + float(config["return_length_m"])
            if return_end > float(config["max_path_horizon_m"]):
                diagnostics.append(dict(side=template["side"],
                    offset_m=offset, return_delay_m=delay,
                    reason="BEYOND_HORIZON", hit_points=-1,
                    min_distance_m=float("inf"), stage_hits={},
                    stage_min_distance_m={}, visible=False,
                    shift_end_m=base_shift_end,
                    return_start_m=return_start, return_end_m=return_end))
                continue
            window = float(config["return_observation_window_m"])
            observed = {}
            for direction, label in ((1, "LEFT"), (-1, "RIGHT")):
                observed[label] = int(np.count_nonzero(
                    (pts[:, 0] >= return_end - window) &
                    (pts[:, 0] <= return_end + 1.0) &
                    (pts[:, 1] * direction >= clearance) &
                    (pts[:, 1] * direction <= 8.0)
                ))
            visible = min(observed.values()) >= int(
                config["min_return_observation_points_per_side"])
            shape = _path(
                side, config, offset=offset,
                shift_end=base_shift_end,
                return_start=return_start,
                return_end=return_end,
            )
            distances, phase_hits, minima = _sweep_metrics(
                pts[:, :2], shape, clearance, base_shift_end, return_start)
            count = int(sum(phase_hits.values()))
            safe = count == 0 and visible
            reason = ("SWEEP_COLLISION" if count > 0 else
                      "RETURN_CORRIDOR_UNOBSERVED" if not visible else
                      "SAFE")
            diagnostics.append(dict(
                side=template["side"], offset_m=offset,
                return_delay_m=delay, shift_end_m=round(base_shift_end, 2),
                return_start_m=round(return_start, 2),
                return_end_m=round(return_end, 2),
                hit_points=count,
                min_distance_m=round(float(np.min(distances)), 3),
                stage_hits=phase_hits, stage_min_distance_m=minima,
                far_side_points=observed, visible=visible,
                reason=reason))
            if safe:
                approved.append((offset, delay, shape))
    if not approved:
        return dict(empty, reason="NO_VERIFIED_TEMPLATE_PATH",
                    template_id=name, diagnostics=diagnostics)
    offset, delay, shape = min(approved, key=lambda item: item[:2])
    return dict(
        status="AVOIDING", side=side, path=shape.tolist(),
        reason="TEMPLATE_VERIFIED_" + name,
        template_id=name, selected_offset_m=offset,
        selected_return_delay_m=delay, diagnostics=diagnostics,
    )
