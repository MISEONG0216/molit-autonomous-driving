"""GPS-shadow entry LiDAR observation gate (Python 3.8)."""
import math
import numpy as np

class EntryObservationGate:
    def __init__(self, config):
        self.config = config
        self.started = None
        self.stable_since = None
        self.last_pose = None
        self.good = 0
        self.done = False

    def reset(self):
        self.started = None
        self.stable_since = None
        self.last_pose = None
        self.good = 0
        self.done = False

    def check(self, now, pose, points):
        if self.done:
            return True, "READY", {}
        if self.started is None:
            self.started = now
        cfg = self.config
        x, y, yaw = pose
        pts = np.asarray(points)
        pts = pts[np.isfinite(pts).all(axis=1)]
        pts = pts[(pts[:, 2] > -1.45) & (pts[:, 2] < 1.25)
                  & (pts[:, 0] >= 2.0) & (pts[:, 0] <= 16.0)]
        sides = [
            int(np.count_nonzero((pts[:, 1] * sign >= 1.35) &
                                 (pts[:, 1] * sign <= 8.0)))
            for sign in (1, -1)
        ]
        close = int(np.count_nonzero((pts[:, 0] <= 8) &
                                     (np.abs(pts[:, 1]) <= 1.35)))
        moving = True
        if self.last_pose is not None:
            px, py, pyaw = self.last_pose
            moving = (math.hypot(x-px, y-py) > 0.08 or
                      abs(math.atan2(math.sin(yaw-pyaw),
                                     math.cos(yaw-pyaw))) > 0.052)
        self.last_pose = pose
        if moving:
            self.stable_since = None
            self.good = 0
        elif self.stable_since is None:
            self.stable_since = now
        sufficient = len(pts) >= 40 and min(sides) >= 8 and close == 0
        self.good = self.good + 1 if not moving and sufficient else 0
        stable_time = 0 if self.stable_since is None else now-self.stable_since
        diagnostic = dict(sides=sides, points=len(pts), close=close,
                          good=self.good, stable_sec=round(stable_time, 2),
                          elapsed_sec=round(now-self.started, 2))
        if close:
            return False, "CLOSE_OBSTACLE", diagnostic
        if moving:
            return False, "WAIT_STOP", diagnostic
        if now-self.started < cfg.get("minimum_hold_sec", 2.0):
            return False, "WAIT_TIMER", diagnostic
        if stable_time < cfg.get("minimum_stable_sec", 0.8):
            return False, "WAIT_STABLE", diagnostic
        if not sufficient:
            return False, "INSUFFICIENT_LIDAR", diagnostic
        if self.good < cfg.get("minimum_good_scans", 5):
            return False, "COLLECTING_SCANS", diagnostic
        self.done = True
        return True, "READY", diagnostic
