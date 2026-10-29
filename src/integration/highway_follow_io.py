"""Highway in-lane forward vehicle risk -> *speed cap*, never direct control.

The score is the supplied YOLO/MiDaS image-space proxy, not metric TTC,
headway or collision prediction. The only EgoCtrlCmd publisher is main.
"""
import json
import math


# Full-frame trapezoid: narrow at the horizon, wider near the ego bumper.
# Still only an image-space lane approximation, NOT calibrated lane geometry.
LANE_ROI_RATIO = ((0.44, 0.38), (0.56, 0.38), (0.88, 0.97), (0.12, 0.97))


def vehicle_overlaps_lane(x1, y1, x2, y2, width, height,
                          min_overlap=0.30):
    """Accept a close/partially clipped vehicle overlapping the ego-lane ROI.

    Use bottom-of-box horizontal overlap, not only the bottom-center point.
    Return False for boxes entirely in an adjacent lane. Geometry must be
    validated against the actual camera extrinsics and road curvature.
    """
    if width <= 0 or height <= 0 or x2 <= x1 or y2 <= y1:
        return False
    # Do not include sky objects; clamp y to the near-vehicle trapezoid.
    bottom = min(0.97, max(0.38, y2 / float(height)))
    t = (bottom - 0.38) / (0.97 - 0.38)
    left = (0.44 + (0.12 - 0.44) * t) * width
    right = (0.56 + (0.88 - 0.56) * t) * width
    overlap = max(0.0, min(x2, right) - max(x1, left))
    ratio = overlap / max(x2 - x1, 1e-6)
    center = 0.5 * (x1 + x2)
    return ratio >= min_overlap and (left <= center <= right or ratio >= 0.50)


def retained_follow_risk(raw_risk, last_risk, seconds_since_seen,
                         hold_sec=0.8):
    """Temporarily retain last risk when a nearby car is briefly lost.

    Bounded retention; cannot substitute for range sensing or tracking.
    """
    if not math.isfinite(seconds_since_seen) or seconds_since_seen < 0:
        return max(0.0, min(1.0, raw_risk))
    if seconds_since_seen <= hold_sec:
        return max(raw_risk, last_risk)
    if seconds_since_seen <= 2 * hold_sec:
        return max(raw_risk, last_risk * 0.60)
    return raw_risk
RISK_LEVELS = (
    (0.80, 4, 20.0),
    (0.60, 3, 40.0),
    (0.40, 2, 65.0),
    (0.20, 1, 90.0),
    (0.00, 0, 120.0),
)


def in_follow_zone(wp, cfg, path_length):
    """Never cross the loop boundary implicitly for the unspecified toll end."""
    if type(wp) is not int or not 0 <= wp < path_length:
        return False
    for start, end in cfg.get("zones_wp", []):
        if type(start) is not int or start < 0:
            continue
        if end is None:
            end = path_length - 1
        if type(end) is int and start <= wp <= min(end, path_length - 1):
            return True
    return False


def highway_speed_zones(cfg, path_length):
    """No unverified fast-speed segment past the unspecified toll-gate end."""
    zones = [tuple(z) for z in cfg.get("verified_highway_speed_zones_wp", [])]
    second_end = cfg.get("tollgate_exit_wp")
    if type(second_end) is int and 3200 <= second_end < path_length:
        zones.append((3200, second_end))
    return zones


def grade_speed_cap(risk):
    """Monotonic conservative speed caps from the supplied risk thresholds."""
    try:
        r = float(risk)
    except (ValueError, TypeError):
        return 4, 20.0
    if not math.isfinite(r) or not 0.0 <= r <= 1.0:
        return 4, 20.0
    for threshold, grade, speed in RISK_LEVELS:
        if r >= threshold:
            return grade, speed
    return 4, 20.0


def parse_follow_message(raw):
    result = {"valid": False, "risk": 1.0, "grade": 4,
              "waypoint_index": None, "frame": None, "vehicles": 0}
    if not raw:
        return result
    try:
        obj = json.loads(raw.decode("utf-8"))
        wp = obj.get("waypoint_index")
        frame = obj.get("frame")
        risk = obj.get("risk")
        count = obj.get("vehicles")
        if obj.get("kind") != "HIGHWAY_FOLLOW" or type(wp) is not int:
            return result
        if type(frame) is not int or frame < 1:
            return result
        if type(count) is not int or count < 0:
            return result
        if type(risk) not in (int, float) or not math.isfinite(risk):
            return result
        if not 0 <= risk <= 1:
            return result
        grade, _ = grade_speed_cap(risk)
        result.update(valid=True, risk=float(risk), grade=grade,
                      waypoint_index=wp, frame=frame, vehicles=count)
    except (ValueError, TypeError, AttributeError, UnicodeError):
        pass
    return result
