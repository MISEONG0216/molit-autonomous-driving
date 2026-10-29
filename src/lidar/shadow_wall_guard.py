"""Conservative LiDAR wall-clearance gate for GPS-shadow static detours.

The original planner generates candidate global-XY paths. This module evaluates
those candidates against the *current vehicle-frame LiDAR scan* before a path
may be latched. Unknown scan coverage is NOT proof that a path is safe.

Coordinate system of points: forward (+X), left (+Y), up (+Z).
This module never sends MORAI controls.
"""
import math
import numpy as np

from static_obstacle_planner import PlanResult


def _vehicle_xy(global_xy, ego_x, ego_y, yaw):
    xy = np.asarray(global_xy, dtype=float)[:, :2]
    delta = xy - [ego_x, ego_y]
    c, s = math.cos(yaw), math.sin(yaw)
    return np.column_stack((c * delta[:, 0] + s * delta[:, 1],
                            -s * delta[:, 0] + c * delta[:, 1]))


def _closest_distance_to_polyline(points, path):
    """Vectorized point-to-segment distances, in manageable memory chunks."""
    starts, ends = path[:-1], path[1:]
    v = ends - starts
    v2 = np.maximum(np.sum(v * v, axis=1), 1e-9)
    minimum = np.full(len(points), np.inf)
    for a, d, d2 in zip(starts, v, v2):
        t = np.clip(np.sum((points - a) * d, axis=1) / d2, 0, 1)
        foot = a + t[:, None] * d
        minimum = np.minimum(minimum, np.linalg.norm(points-foot, axis=1))
    return minimum


def check_wall_clearance(local_path_global, ego_pose, vehicle_points, cfg):
    """Return (verified, reason). All seen obstacles/walls must clear sweep.

    Points must be fresh from current scan; worker ensures freshness. Vehicle
    dimensions include conservative body half-width and added clearance.
    Reject unobserved lateral sectors and paths whose ego-frame coordinates
    are inconsistent with a nearby original global waypoint.
    """
    try:
        x, y, yaw = (float(v) for v in ego_pose)
        pts = np.asarray(vehicle_points, dtype=float)
        path = _vehicle_xy(local_path_global, x, y, yaw)
        if (pts.ndim != 2 or pts.shape[1] < 3 or path.shape[0] < 2
                or not np.isfinite(path).all() or not np.isfinite(pts).all()):
            return False, "BAD_LIDAR_OR_PATH"
        if float(np.min(np.linalg.norm(path - [0, 0], axis=1))) > float(
                cfg.get("max_path_pose_error_m", 1.8)):
            return False, "PATH_NOT_ALIGNED"
        scan = pts[(pts[:, 2] > float(cfg.get("z_min_m", -1.45)))
                   & (pts[:, 2] < float(cfg.get("z_max_m", 1.25)))
                   & (pts[:, 0] >= 1.0)
                   & (pts[:, 0] <= float(cfg.get("check_ahead_m", 18.0)) + 2.0)]
        if len(scan) < int(cfg.get("minimum_obstacle_points", 40)):
            return False, "SCAN_SPARSE"
        # Require observed lateral sectors ON BOTH sides of the ego vehicle.
        # The lack of wall evidence is uncertainty, NOT free space.
        lateral_min = float(cfg.get("side_observation_min_m", 1.3))
        lateral_max = float(cfg.get("side_observation_max_m", 8.0))
        min_side = int(cfg.get("min_side_observation_points", 8))
        for sign, label in ((1, "LEFT"), (-1, "RIGHT")):
            side = scan[(scan[:, 0] >= 2.0) & (scan[:, 0] <= 15.0)
                        & (scan[:, 1]*sign >= lateral_min)
                        & (scan[:, 1]*sign <= lateral_max)]
            if len(side) < min_side:
                return False, label + "_BOUNDARY_UNOBSERVED"

        # Sample only the forward horizon; ignore rearward pieces of a long
        # latched plan. Include current vehicle position as sweep start.
        ahead = float(cfg.get("check_ahead_m", 18.0))
        path = path[(path[:, 0] >= -0.5) & (path[:, 0] <= ahead)]
        if len(path) < 2 or path[-1, 0] < float(cfg.get("minimum_path_ahead_m", 6.0)):
            return False, "PATH_TOO_SHORT"
        if np.linalg.norm(path[0]) > 2.0:
            path = np.vstack((np.array([[0.0, 0.0]]), path))
        clearance = float(cfg.get("vehicle_half_width_m", .9)) + float(
            cfg.get("safety_margin_m", .45))
        # Evaluate all non-ground 3D returns, including long tunnel-wall
        # clusters that the object detector's max-width filtering discards.
        distance = _closest_distance_to_polyline(scan[:, :2], path)
        near = np.count_nonzero(distance < clearance)
        if near > int(cfg.get("allow_sweep_hit_points", 0)):
            return False, "WALL_OR_OBSTACLE_IN_SWEPT_PATH"
        return True, "LIDAR_SWEEP_CLEAR"
    except (ValueError, TypeError, IndexError, OverflowError):
        return False, "INVALID_GEOMETRY"


def pick_shadow_detour(planner, ego_pose, obstacles, vehicle_points, cfg):
    """Try BOTH original planner offsets; choose first geometrically clear.

    Reuses existing static obstacle selection/path generation and size filters.
    No side is trusted purely because the original global-path width is 7m.
    """
    x, y, yaw = ego_pose
    ego_index = planner_nearest(planner.path, x, y)
    selected = planner._select_obstacle(ego_index, obstacles)
    if selected is None:
        return planner.plan(x, y, obstacles), False, "NO_SELECTED_STATIC"
    obs, obs_index, obs_d = selected
    candidates = []
    decisions = []
    for offset in planner.cfg["candidate_offsets_m"]:
        candidate, shifted = planner._candidate_path(ego_index, obs_index, float(offset))
        original_check = planner._candidate_safety_diagnostic(candidate, shifted, obs)
        if not original_check["safe"]:
            decisions.append((offset, "ORIGINAL_"+original_check["reason"]))
            continue
        clear, reason = check_wall_clearance(candidate, ego_pose, vehicle_points, cfg)
        decisions.append((offset, reason))
        if clear:
            # Prefer moving opposite the obstruction and smaller offset,
            # never prefer an unsafe side simply because it is shorter.
            score = (0 if offset*obs_d <= 0 else 10) + abs(offset)
            candidates.append((score, offset, candidate))
    if not candidates:
        return PlanResult("STOP_REQUIRED", np.empty((0,3)), obs.id, None, 0.,
                          "NO_LIDAR_VERIFIED_DETOUR "+repr(decisions)), False, "NO_SAFE_SIDE"
    _, offset, path = min(candidates, key=lambda c:c[0])
    return PlanResult("AVOIDING", path, obs.id,
                      "LEFT" if offset > 0 else "RIGHT",
                      min(planner.avoidance_speed_kph, float(cfg.get("avoidance_speed_kmh",6.0))),
                      "LiDAR sweep checked shadow detour"), True, "SAFE_"+("LEFT" if offset>0 else "RIGHT")


def close_forward_collision(vehicle_points, cfg):
    """Independent emergency STOP for objects or wall encroaching ego path.

    Used in shadow mode even when original object clustering returns CLEAR.
    Vehicle-frame LiDAR cannot prove all free space; this is an extra brake
    layer, not permission to move without lane perception.
    """
    try:
        pts = np.asarray(vehicle_points, dtype=float)
        if pts.ndim != 2 or pts.shape[1] < 3 or not np.isfinite(pts).all():
            return True
        half = float(cfg.get("vehicle_half_width_m", .9)) + float(
            cfg.get("safety_margin_m", .45))
        danger = (pts[:, 0] >= float(cfg.get("emergency_min_forward_m", 1.5))) & (
            pts[:, 0] <= float(cfg.get("emergency_stop_ahead_m", 6.0))) & (
            np.abs(pts[:, 1]) <= half) & (
            pts[:, 2] >= float(cfg.get("z_min_m", -1.45))) & (
            pts[:, 2] <= float(cfg.get("z_max_m", 1.25)))
        return int(np.count_nonzero(danger)) >= int(
            cfg.get("emergency_min_points", 5))
    except (ValueError, TypeError, IndexError):
        return True


def planner_nearest(path, x, y):
    d = path[:, :2] - [x, y]
    return int(np.argmin(np.sum(d*d, axis=1)))
