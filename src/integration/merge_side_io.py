"""Helpers for conservative left/right MORAI camera merge observations."""

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
