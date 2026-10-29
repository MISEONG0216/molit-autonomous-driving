"""Highway in-lane forward vehicle risk -> *speed cap*, never direct control.

The score is the supplied YOLO/MiDaS image-space proxy, not metric TTC,
headway or collision prediction. The only EgoCtrlCmd publisher is main.
"""
import json
import math


LANE_ROI_RATIO = ((0.48, 0.48), (0.52, 0.48), (0.66, 0.78), (0.34, 0.78))
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
