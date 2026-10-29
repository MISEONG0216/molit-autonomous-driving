"""Helpers for MORAI left-side and left-rear merging observations."""

import math


class JpegAssembler:
    """Extract the most recent complete JPEG from fragmented UDP camera data."""
    def __init__(self, limit=4 * 1024 * 1024):
        self.data = bytearray()
        self.limit = limit

    def feed(self, packet):
        self.data.extend(packet)
        if len(self.data) > self.limit:
            start = self.data.rfind(bytes((255, 216)))
            self.data = self.data[start:] if start >= 0 else bytearray()
        end = self.data.rfind(bytes((255, 217)))
        if end < 0:
            return None
        start = self.data.rfind(bytes((255, 216)), 0, end)
        jpeg = bytes(self.data[start:end + 2]) if 0 <= start < end else None
        del self.data[:end + 2]
        return jpeg


def in_merge_zone(wp, cfg, all_zones=False):
    if type(wp) is not int or wp < 0:
        return False
    if all_zones:
        return True
    ranges = list(cfg.get("zones", ())) + list(cfg.get("roundabout_zones", ()))
    return any(
        isinstance(pair, (list, tuple)) and len(pair) == 2
        and all(type(x) is int for x in pair)
        and pair[0] <= wp <= pair[1]
        for pair in ranges
    )


def clamp(value):
    return min(1.0, max(0.0, float(value)))


def vehicle_risk(x1, y1, x2, y2, width, height, previous_area_ratio=0.0):
    """Conservative *image-space proxy*, not calibrated distance or TTC."""
    if width <= 0 or height <= 0:
        return 0.0
    box_w = max(0.0, x2 - x1)
    box_h = max(0.0, y2 - y1)
    area = clamp(box_w * box_h / float(width * height))
    width_ratio = clamp(box_w / float(width))
    growth = 0.0
    if previous_area_ratio > 0.001:
        growth = max(0.0, (area - previous_area_ratio) / previous_area_ratio)
    return max(
        0.80 * clamp(area / 0.045),
        0.80 * clamp(width_ratio / 0.30),
        0.85 * clamp(growth / 0.10),
    )


def format_result(side, rear, decision):
    if decision not in ("MERGE_REJECT", "MERGE_ACCEPT"):
        raise ValueError("Invalid decision")
    if not all(math.isfinite(float(x)) for x in (side, rear)):
        raise ValueError("Nonfinite risk")
    side, rear = clamp(side), clamp(rear)
    return ("MERGE|%.4f|%.4f|%.4f|%s" % (
        side, rear, max(side, rear), decision
    )).encode("ascii")


def merge_camera_mode(wp, cfg, all_zones=False):
    """Return enabled camera mode. Roundabout uses LEFT only.

    Outside known zones --merge-all is an explicit debugging override,
    never a reason to activate rear-left in the configured roundabout.
    """
    if type(wp) is not int or wp < 0:
        return "off"
    for pair in cfg.get("roundabout_zones", ()):
        if len(pair) == 2 and pair[0] <= wp <= pair[1]:
            return "roundabout"
    # Latest MORAI capture: first merge completed by WP2538.
    # After this point, rear camera no longer vetoes continued forward motion.
    # Second merge's completion WP is unknown and stays protected.
    completed_at = cfg.get("rear_camera_release_wp_by_zone", {})
    for pair in cfg.get("zones", ()):
        if len(pair) == 2 and pair[0] <= wp <= pair[1]:
            release_wp = completed_at.get(str(pair[0]))
            if type(release_wp) is int and pair[0] < release_wp <= pair[1] and wp >= release_wp:
                return "post_merge"
            return "highway_merge"
    return "highway_merge" if all_zones else "off"


def active_merge_cameras(mode):
    if mode in ("roundabout", "post_merge"):
        return ("left",)
    if mode == "highway_merge":
        return ("left", "left_rear")
    return ()


def attenuate_distant_risk(raw_risk, area_ratio, cfg):
    """Reduce false STOP from very small boxes without lowering near risk.

    Image size is only a distance *proxy*. This is not a lane classifier,
    and a distant fast-approaching vehicle may still be missed.
    """
    if not bool(cfg.get("distance_sensitivity_enabled", True)):
        return clamp(raw_risk)
    near = float(cfg.get("near_full_risk_area_ratio", 0.030))
    far = float(cfg.get("far_area_ratio", 0.006))
    far_weight = float(cfg.get("far_risk_weight", 0.45))
    if not (0.0 <= far < near <= 1.0 and 0.0 < far_weight <= 1.0):
        raise ValueError("Invalid merge distance-sensitivity configuration")
    area = clamp(area_ratio)
    fraction = clamp((area - far) / (near - far))
    weight = far_weight + (1.0 - far_weight) * fraction
    return clamp(raw_risk) * weight
