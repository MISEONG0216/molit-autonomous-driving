"""Predefined left-then-right GPS-shadow LiDAR detours (Python 3.8).

A fixed manoeuvre SHAPE is read from configuration, anchored to a newly
observed static obstacle in the CURRENT vehicle frame. No GPS waypoints.
This module does not send vehicle controls or modify camera lane following.
"""
import numpy as np
from shadow_ego_planner import _path, _sweep_metrics, _smoothstep


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
    # FIRST_LEFT remains the exact V48 planner. V49 only replaces
    # SECOND_RIGHT; its anchor is an observed 2-D obstruction signature
    # from the separate stationary survey, not any GPS waypoint.
    if completed_count == 1 and config.get("second_survey_match", {}).get(
            "enabled", False):
        return choose_second_survey_hold(pts, config, template, clearance)

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


def _two_side_observed(pts, clearance, minimum, start_x, end_x):
    """Side-sector evidence. Unobserved space is never declared free."""
    result = {}
    for sign, label in ((1, "LEFT"), (-1, "RIGHT")):
        result[label] = int(np.count_nonzero(
            (pts[:, 0] >= start_x) & (pts[:, 0] <= end_x) &
            (pts[:, 1] * sign >= clearance) &
            (pts[:, 1] * sign <= 8.0)
        ))
    return result, min(result.values()) >= minimum


def choose_second_survey_hold(points, config, template, clearance):
    """Return fixed right-shift-and-HOLD path, without blind return.

    Survey was measured from a different stationary pose, so we require
    the observed forward/lateral pattern to resemble that scan rather
    than treating survey coordinates as world/GPS positions.
    """
    cfg = config["second_survey_match"]
    failure = dict(status="STOP_REQUIRED", side=None, path=[],
                   reason="SECOND_SURVEY_UNVERIFIED", diagnostics=[],
                   template_id=template["id"])
    pts = np.asarray(points, dtype=float)
    threshold = int(cfg["min_points"])
    close = pts[(pts[:, 0] >= 1.0) &
                (pts[:, 0] < float(cfg["near_unmatched_stop_m"])) &
                (np.abs(pts[:, 1]) <= clearance)]
    if len(close) >= int(config["min_obstacle_points"]):
        return dict(failure, reason="SECOND_APPROACH_TOO_CLOSE")

    xmin, xmax = (float(v) for v in cfg["forward_window_m"])
    ymin, ymax = (float(v) for v in cfg["lateral_window_m"])
    candidate = pts[(pts[:, 0] >= xmin) & (pts[:, 0] <= xmax) &
                    (pts[:, 1] >= ymin) & (pts[:, 1] <= ymax)]
    if len(candidate) < threshold:
        # If the expected object arrives TOO CLOSE without being matched,
        # stop instead of allowing the camera to blindly approach it.
        late = pts[(pts[:, 0] >= float(cfg["near_unmatched_stop_m"])) &
                   (pts[:, 0] < xmin) &
                   (pts[:, 1] >= ymin) & (pts[:, 1] <= ymax)]
        if len(late) >= threshold:
            return dict(failure, reason="SECOND_MATCH_WINDOW_MISSED")
        return dict(status="CLEAR", side=None, path=[],
                    reason="WAITING_SECOND_SURVEY_MATCH", diagnostics=[],
                    template_id=template["id"], match=False)
    # Never merge point returns from multiple distant obstacles into a
    # bogus long rectangle. Divide at 0.7m longitudinal discontinuities.
    ordered = candidate[np.argsort(candidate[:, 0])]
    gaps = np.where(np.diff(ordered[:, 0]) > 0.7)[0] + 1
    groups = [g for g in np.split(ordered, gaps) if len(g) >= threshold]
    matched = []
    for group in groups:
        xlo, xhi = float(np.min(group[:, 0])), float(np.max(group[:, 0]))
        center_forward = (xlo + xhi) * 0.5
        lateral_mid = float((np.min(group[:, 1]) +
                             np.max(group[:, 1])) / 2.0)
        lateral_span = float(np.ptp(group[:, 1]))
        if (xhi - xlo <= float(cfg["max_group_forward_span_m"]) and
                abs(center_forward - float(cfg["target_forward_m"])) <=
                float(cfg["forward_tolerance_m"]) and
                abs(lateral_mid - float(cfg["target_left_center_m"])) <=
                float(cfg["left_tolerance_m"]) and
                lateral_span >= float(cfg["min_lateral_span_m"])):
            matched.append((abs(center_forward -
                              float(cfg["target_forward_m"])), group))
    if not matched:
        # A nearby unmatched object could be a wall or a differently placed
        # barrier; stop if it is within the critical 15m approach zone.
        late = pts[(pts[:, 0] >= float(cfg["near_unmatched_stop_m"])) &
                   (pts[:, 0] < xmin) &
                   (pts[:, 1] >= ymin) & (pts[:, 1] <= ymax)]
        if len(late) >= threshold:
            return dict(failure, reason="SECOND_SURVEY_UNMATCHED_CLOSE")
        return dict(status="CLEAR", side=None, path=[],
                    reason="SECOND_SURVEY_PATTERN_MISMATCH",
                    diagnostics=[{"candidate_points":len(candidate)}],
                    template_id=template["id"], match=False)
    obstacle = min(matched, key=lambda item: item[0])[1]
    side_counts, observed = _two_side_observed(
        pts, clearance, int(config["min_side_points"]), 2.0, 15.0)
    if not observed:
        return dict(failure, reason="SIDE_WALL_NOT_OBSERVED",
                    diagnostics=[{"side_counts":side_counts}], match=True)

    endpoint = float(cfg["hold_endpoint_m"])
    if endpoint > float(config["max_path_horizon_m"]):
        return dict(failure, reason="HOLD_BEYOND_HORIZON", match=True)
    far_counts, far_visible = _two_side_observed(
        pts, clearance, int(cfg["require_end_observations_per_side"]),
        endpoint - 3.0, endpoint + 1.0)
    diagnostics, approved = [], []
    for offset in (float(v) for v in template["offset_options_m"]):
        x = np.arange(0.0, endpoint + 0.001, float(
            config.get("sample_step_m", .5)))
        shift_start = float(config.get("start_shift_m", 1.5))
        shift_end = float(cfg["shift_end_m"])
        y = -offset * _smoothstep(
            (x-shift_start) / (shift_end-shift_start))
        shape = np.column_stack((x, y))
        # RETURN phase is intentionally absent. Every segment through the
        # HOLD endpoint must be measured, including tunnel wall points.
        distances, stages, minima = _sweep_metrics(
            pts[:, :2], shape, clearance, shift_end, endpoint)
        hits = int(sum(stages.values()))
        safe = hits == 0 and far_visible
        reason = ("SWEEP_COLLISION" if hits else
                  "HOLD_ENDPOINT_UNOBSERVED" if not far_visible else "SAFE")
        diagnostics.append(dict(
            side="RIGHT", offset_m=offset, return_delay_m=0.0,
            shift_end_m=shift_end, return_start_m=endpoint,
            return_end_m=endpoint, visible=far_visible,
            far_side_points=far_counts, hit_points=hits,
            min_distance_m=round(float(np.min(distances)), 3),
            stage_hits=stages, stage_min_distance_m=minima,
            reason=reason,
        ))
        if safe:
            approved.append((offset, shape))
    if not approved:
        return dict(failure, reason="NO_VERIFIED_SECOND_HOLD",
                    diagnostics=diagnostics, match=True)
    offset, shape = min(approved, key=lambda item: item[0])
    return dict(status="AVOIDING", side=-1, path=shape.tolist(),
                reason="SECOND_SURVEY_RIGHT_HOLD_VERIFIED",
                template_id=template["id"],
                selected_offset_m=offset,
                selected_return_delay_m=0.0,
                diagnostics=diagnostics, match=True)


def plan_second_verified_return(points, config, ego_pose, wall_cfg, offset):
    """From right-hand hold, propose an entirely separate leftward return.

    The existing main controller will follow it ONLY if current LiDAR
    confirms the entire path via the original wall-guard. Failure means STOP.
    """
    from shadow_wall_guard import check_wall_clearance
    cfg = config["second_survey_match"]
    result = dict(status="STOP_REQUIRED", path=[],
                  reason="RETURN_NOT_VERIFIED", diagnostics=[])
    pts = np.asarray(points, dtype=float)
    pose = np.asarray(ego_pose, dtype=float)
    if (pts.ndim != 2 or pts.shape[1] < 3 or
            not np.isfinite(pts).all() or not np.isfinite(pose).all()):
        return dict(result, reason="BAD_RETURN_SCAN_OR_POSE")
    clearance = float(config["vehicle_half_width_m"] +
                      config["safety_margin_m"])
    side_counts, side_ok = _two_side_observed(
        pts, clearance, int(config["min_side_points"]), 2.0, 12.0)
    if not side_ok:
        return dict(result, reason="RETURN_SIDE_UNOBSERVED",
                    diagnostics=[{"side_counts":side_counts}])
    endpoint = float(cfg["return_endpoint_m"])
    far_counts, far_ok = _two_side_observed(
        pts, clearance, int(cfg["require_end_observations_per_side"]),
        endpoint-3.0, endpoint+1.0)
    if not far_ok:
        return dict(result, reason="RETURN_END_UNOBSERVED",
                    diagnostics=[{"far_side_points":far_counts}])
    offset = float(offset)
    if (not np.isfinite(offset) or offset not in [float(v) for v in
            config["templates"][1]["offset_options_m"]]):
        return dict(result, reason="RETURN_OFFSET_INVALID")
    x = np.arange(0.0, endpoint + .001, float(
        config.get("sample_step_m", .5)))
    y = offset * _smoothstep(
        (x-float(cfg["return_shift_start_m"])) /
        (float(cfg["return_shift_end_m"]) -
         float(cfg["return_shift_start_m"])))
    shape = np.column_stack((x, y))
    px, py, yaw = pose
    c, s = np.cos(yaw), np.sin(yaw)
    global_path = np.column_stack((
        px+c*x-s*y, py+s*x+c*y, np.zeros(len(x))))
    metrics = {}
    wall_ok, why = check_wall_clearance(
        global_path, ego_pose, pts, wall_cfg,
        alignment_diagnostics=metrics)
    result["diagnostics"] = [dict(
        clearance_ok=wall_ok, reason=why,
        offset_m=offset, alignment=metrics)]
    if not wall_ok:
        return dict(result, reason="RETURN_" + why)
    return dict(status="AVOIDING", path=global_path,
                reason="SECOND_VERIFIED_RETURN", diagnostics=result["diagnostics"])
