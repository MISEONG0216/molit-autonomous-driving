"""Bounded recovery gate for repeated GPS-shadow planner observation failures.

This module never sends vehicle controls. A timeout ALONE is never authority
to move: verified fresh LiDAR straight-corridor evidence and camera lane data
are mandatory. Collision, immediate-obstacle, dynamic and stale-data stops
remain non-recoverable.
"""
import math
import time


RECOVERABLE_REASONS = frozenset({
    "RETURN_CORRIDOR_UNOBSERVED",
    "SIDE_WALL_NOT_OBSERVED",
})


class ShadowRecoveryGate:
    def __init__(self, delay_sec=8.0, confirmations=3):
        if not 1 <= float(delay_sec) <= 60:
            raise ValueError("delay_sec must be 1..60")
        if not 1 <= int(confirmations) <= 20:
            raise ValueError("confirmations must be 1..20")
        self.delay_sec = float(delay_sec)
        self.confirmations = int(confirmations)
        self.reset()

    def reset(self):
        self.started_at = None
        self.clear_scans = 0
        self.last_scan = None

    def update(self, shadow_active, static_fresh, static_plan,
               lane_fresh, now=None):
        if now is None:
            now = time.monotonic()
        reason = str(static_plan.get("reason", ""))
        status = static_plan.get("status")
        if not (shadow_active and static_fresh and status == "STOP_REQUIRED"
                and reason in RECOVERABLE_REASONS):
            self.reset()
            return "INACTIVE", False, 0.0

        if self.started_at is None:
            self.started_at = now
        stopped_sec = max(0.0, now - self.started_at)

        scan = static_plan.get("scan")
        if scan != self.last_scan:
            self.last_scan = scan
            if bool(static_plan.get("forward_clear_verified", False)):
                self.clear_scans += 1
            else:
                self.clear_scans = 0

        if stopped_sec < self.delay_sec:
            return "COUNTDOWN", False, stopped_sec
        if not lane_fresh:
            return "LANE_UNAVAILABLE", False, stopped_sec
        if self.clear_scans < self.confirmations:
            return "FORWARD_UNVERIFIED", False, stopped_sec
        return "RECOVERY_ELIGIBLE", True, stopped_sec
