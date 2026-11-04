"""Short rolling LiDAR history aligned by existing dead-reckoned ego pose.

Only supplements endpoint observability. It never creates clearance by itself;
all observed historical points are checked against each candidate's swept body.
No extra MORAI data or networks.
"""
from collections import deque
import math
import numpy as np


class TemporalShadowObservations:
    def __init__(self, max_frames=5, max_age_sec=1.8, voxel_m=0.15):
        self.frames = deque(maxlen=max(1, int(max_frames)))
        self.max_age_sec = float(max_age_sec)
        self.voxel_m = float(voxel_m)

    def clear(self):
        self.frames.clear()

    def update(self, points, pose, stamp):
        if pose is None or not all(map(math.isfinite, pose)):
            self.clear()
            return
        cloud = np.asarray(points, dtype=float)
        if cloud.ndim != 2 or cloud.shape[1] < 3:
            return
        cloud = cloud[np.isfinite(cloud[:, :3]).all(axis=1), :3]
        # Downsample retained evidence for CPU/memory control.
        cloud = cloud[(cloud[:, 0] >= 1.) & (cloud[:, 0] <= 26.) &
                      (np.abs(cloud[:, 1]) <= 7.) &
                      (cloud[:, 2] >= -1.45) & (cloud[:, 2] <= 1.25)]
        if len(cloud) == 0:
            return
        cells = np.floor(cloud / max(0.05, self.voxel_m)).astype(np.int32)
        _, unique = np.unique(cells, axis=0, return_index=True)
        cloud = cloud[unique]
        x, y, yaw = pose
        c, s = math.cos(yaw), math.sin(yaw)
        xy = np.empty_like(cloud)
        xy[:, 0] = x + c * cloud[:, 0] - s * cloud[:, 1]
        xy[:, 1] = y + s * cloud[:, 0] + c * cloud[:, 1]
        xy[:, 2] = cloud[:, 2]
        # Reject clock regression / out-of-order scans rather than mixing poses.
        if self.frames and stamp <= self.frames[-1][0]:
            self.clear()
        self.frames.append((float(stamp), xy))

    def get_prior_in_vehicle(self, pose, now):
        if not self.frames or pose is None:
            return np.empty((0, 3), dtype=np.float32)
        cutoff = float(now) - self.max_age_sec
        while self.frames and self.frames[0][0] < cutoff:
            self.frames.popleft()
        # Last frame is the CURRENT scan, already present in the planner.
        previous = [world for stamp, world in list(self.frames)[:-1]
                    if stamp >= cutoff]
        if not previous:
            return np.empty((0, 3), dtype=np.float32)
        world = np.vstack(previous)
        x, y, yaw = pose
        c, s = math.cos(yaw), math.sin(yaw)
        dx = world[:, 0] - x
        dy = world[:, 1] - y
        transformed = np.column_stack((c*dx+s*dy, -s*dx+c*dy, world[:, 2]))
        transformed = transformed[
            (transformed[:, 0] >= 1.) & (transformed[:, 0] <= 25.) &
            (np.abs(transformed[:, 1]) <= 7.)]
        if len(transformed) == 0:
            return np.empty((0, 3), dtype=np.float32)
        cells = np.floor(transformed / max(0.05, self.voxel_m)).astype(np.int32)
        _, unique = np.unique(cells, axis=0, return_index=True)
        return transformed[unique].astype(np.float32)
