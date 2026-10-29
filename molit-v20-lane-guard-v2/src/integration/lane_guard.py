"""Optional, waypoint-scoped Pure Pursuit reference-path offset for MORAI.

Lane boundaries are NOT perceived here. Never enable a zone before manually
checking its source path, road markings and logs in MORAI. All dimensions m.
"""
import math
import numpy as np


def read_guard_config(cfg, path_length):
    enabled = bool(cfg.get("enabled", False))
    zones = []
    if enabled:
        for zone in cfg.get("zones", []):
            a = int(zone["start_wp"])
            b = int(zone["end_wp"])
            offset = float(zone["right_offset_m"])
            if not (0 <= a < b < path_length):
                raise ValueError(f"Invalid lane-guard waypoint range {a}..{b}")
            if not (0.0 < offset <= 1.0):
                raise ValueError("right_offset_m must be >0 and <=1.0 m")
            zones.append((a, b, offset))
        if not zones:
            raise ValueError("lane_guard enabled but no zones specified")
        ordered = sorted(zones)
        if any(ordered[i][0] <= ordered[i-1][1] for i in range(1, len(ordered))):
            raise ValueError("lane-guard zones may not overlap")
    # Optional additional shift in a *verified* tight right-hand bend.
    # Ramps use physical path distance rather than WP count to prevent jumps.
    bends = []
    if enabled:
        for sec in cfg.get("curve_sections", []):
            a = int(sec["start_wp"])
            p = int(sec["full_start_wp"])
            q = int(sec["full_end_wp"])
            b = int(sec["end_wp"])
            extra = float(sec.get("extra_right_m", 0.0))
            bend_speed = float(sec.get("max_target_speed_kmh", 8.0))
            if not (0 <= a < p <= q < b < path_length):
                raise ValueError("curve section indices must be ordered start<full_start<=full_end<end")
            if not any(za <= a and b <= zb for za, zb, _ in zones):
                raise ValueError("curve section must be inside a lane-guard zone")
            if not (0 <= extra <= 0.30):
                raise ValueError("extra_right_m must be 0..0.30m")
            if not (5 <= bend_speed <= 30):
                raise ValueError("curve max_target_speed_kmh must be 5..30")
            bends.append((a, p, q, b, extra, bend_speed))
    taper = float(cfg.get("taper_m", 15.0))
    approach = float(cfg.get("approach_m", 65.0))
    speed = float(cfg.get("max_target_speed_kmh", 20.0))
    warning = float(cfg.get("cross_track_warn_m", 0.70))
    stop = float(cfg.get("cross_track_stop_m", 1.50))
    if not (3 <= taper <= 100 and 5 <= approach <= 200):
        raise ValueError("Guard taper/approach distances outside supported range")
    if not (5 <= speed <= 40):
        raise ValueError("Guard speed cap must be between 5 and 40 km/h")
    if not (0 < warning < stop <= 3):
        raise ValueError("Guard warning/stop thresholds inconsistent")
    return dict(enabled=enabled, zones=zones, bends=bends, taper_m=taper,
                approach_m=approach, speed=speed, warn=warning, stop=stop)


def build_guarded_path(path, guard):
    """Make a copy; global waypoint indices remain intact for other missions."""
    result = np.array(path, dtype=float, copy=True)
    cumulative = np.r_[0., np.cumsum(np.hypot(*np.diff(result[:, :2], axis=0).T))]
    if not guard["enabled"]:
        return result, np.zeros(len(result), dtype=float), cumulative
    tangents = np.empty((len(result), 2), dtype=float)
    tangents[1:-1] = result[2:, :2] - result[:-2, :2]
    tangents[0] = result[1, :2] - result[0, :2]
    tangents[-1] = result[-1, :2] - result[-2, :2]
    norms = np.linalg.norm(tangents, axis=1)
    if np.any(norms < 1e-9):
        raise ValueError("Path contains zero-length local tangents")
    normal_right = np.column_stack((tangents[:, 1], -tangents[:, 0])) / norms[:, None]
    offsets = np.zeros(len(result), dtype=float)
    taper = guard["taper_m"]
    for a, b, value in guard["zones"]:
        dist_from_start = (cumulative[a:b+1] - cumulative[a]) / taper
        dist_to_end = (cumulative[b] - cumulative[a:b+1]) / taper
        u = np.clip(np.minimum(dist_from_start, dist_to_end), 0., 1.)
        smooth = u*u*(3. - 2.*u)
        offsets[a:b+1] = value*smooth
    for a, p, q, b, extra, _ in guard.get("bends", []):
        # Smoothstep entering and leaving the turn; no discontinuous steering target.
        section = np.arange(a, b + 1)
        turn_offset = np.full(len(section), extra)
        enter = section < p
        u = np.clip((cumulative[section[enter]] - cumulative[a]) /
                    max(cumulative[p] - cumulative[a], 1e-9), 0., 1.)
        turn_offset[enter] *= u*u*(3. - 2.*u)
        leave = section > q
        u = np.clip((cumulative[b] - cumulative[section[leave]]) /
                    max(cumulative[b] - cumulative[q], 1e-9), 0., 1.)
        turn_offset[leave] *= u*u*(3. - 2.*u)
        offsets[section] += turn_offset
    if np.any(offsets > 1.0 + 1e-9):
        raise ValueError("Combined rightward offset exceeds 1.0m")
    result[:, :2] += normal_right * offsets[:, None]
    return result, offsets, cumulative


def zone_state(index, guard, distances):
    """Return (within_zone, approaching_zone)."""
    if not guard["enabled"]:
        return False, False
    in_zone = any(a <= index <= b for a, b, _ in guard["zones"])
    approaching = any(index < a and 0. <= distances[a] - distances[index] <= guard["approach_m"]
                      for a, b, _ in guard["zones"])
    return in_zone, approaching


def guard_policy(path_error_m, in_zone, approaching, guard):
    """Return (speed_cap or None, full_stop, warning).

    A path-error stop is only useful for a checked reference path; it cannot
    identify paint, lane widths or oncoming traffic.
    """
    if not guard["enabled"] or not (in_zone or approaching):
        return None, False, False
    if not math.isfinite(path_error_m):
        return 0.0, True, True
    warning = in_zone and path_error_m > guard["warn"]
    stop = in_zone and path_error_m >= guard["stop"]
    speed = guard["speed"]
    if warning:
        speed = min(speed, 10.)
    return speed, stop, warning


def bend_speed_cap(index, guard):
    """Optional additional speed cap only inside manually checked bend intervals."""
    if not guard.get("enabled", False):
        return None
    values = [speed for a, _p, _q, b, _extra, speed in guard.get("bends", [])
              if a <= index <= b]
    return min(values) if values else None


def signed_lateral_error(path, index, x, y):
    """Signed offset from *guarded GPS reference*, +right, -left of travel.

    This is NOT the distance to a painted road line.
    """
    n = len(path)
    if index < 0 or index >= n or n < 2:
        raise ValueError("invalid path index")
    prev = max(0, index - 2)
    next_ = min(n - 1, index + 2)
    dx, dy = (path[next_, :2] - path[prev, :2])
    norm = math.hypot(dx, dy)
    if norm < 1e-9:
        raise ValueError("zero local tangent")
    right_x, right_y = dy / norm, -dx / norm
    return float((x - path[index, 0])*right_x + (y - path[index, 1])*right_y)
