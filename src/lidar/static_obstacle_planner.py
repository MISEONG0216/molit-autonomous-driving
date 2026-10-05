from dataclasses import dataclass
from typing import List, Optional, Tuple
import json
import math

import numpy as np


@dataclass
class Obstacle:
    id: int
    x: float
    y: float
    width: float
    length: float
    distance: float
    velocity_x: float
    velocity_y: float
    is_dynamic: bool
    confidence: float


@dataclass
class PlanResult:
    status: str
    local_path: np.ndarray
    obstacle_id: Optional[int]
    avoidance_side: Optional[str]
    recommended_speed_kph: float
    reason: str


def load_path(path_file: str) -> np.ndarray:
    path = np.loadtxt(path_file, dtype=float)
    if path.ndim != 2 or path.shape[1] < 2:
        raise ValueError("Global path must contain x y [z] columns")
    if path.shape[1] == 2:
        path = np.column_stack((path, np.zeros(len(path))))
    return path[:, :3]


def cumulative_s(path: np.ndarray) -> np.ndarray:
    ds = np.linalg.norm(np.diff(path[:, :2], axis=0), axis=1)
    return np.concatenate(([0.0], np.cumsum(ds)))


def nearest_index(path: np.ndarray, x: float, y: float) -> int:
    delta = path[:, :2] - np.array([x, y])
    return int(np.argmin(np.einsum("ij,ij->i", delta, delta)))


def signed_lateral(path: np.ndarray, index: int, x: float, y: float) -> float:
    prev_i = max(0, index - 1)
    next_i = min(len(path) - 1, index + 1)
    tangent = path[next_i, :2] - path[prev_i, :2]
    norm = float(np.linalg.norm(tangent))
    if norm < 1e-9:
        return 0.0
    tangent /= norm
    normal = np.array([-tangent[1], tangent[0]])
    return float(np.dot(np.array([x, y]) - path[index, :2], normal))


def smoothstep5(t: np.ndarray) -> np.ndarray:
    t = np.clip(t, 0.0, 1.0)
    return 10.0 * t**3 - 15.0 * t**4 + 6.0 * t**5


class StaticObstaclePlanner:
    """Produces a local path only. It never sends Accel/Brake/Steer."""

    def __init__(self, global_path: np.ndarray, config_file: str):
        self.path = global_path
        self.s = cumulative_s(global_path)
        with open(config_file, "r", encoding="utf-8") as f:
            self.cfg = json.load(f)

    def _forward_indices(self, ego_index: int, distance_m: float) -> np.ndarray:
        # The supplied competition path is a closed loop. Walk forward with wraparound.
        result = [ego_index]
        travelled = 0.0
        i = ego_index
        while travelled < distance_m and len(result) <= len(self.path):
            j = (i + 1) % len(self.path)
            travelled += float(np.linalg.norm(self.path[j, :2] - self.path[i, :2]))
            result.append(j)
            i = j
        return np.asarray(result, dtype=int)

    def _select_obstacle(
        self, ego_index: int, obstacles: List[Obstacle]
    ) -> Optional[Tuple[Obstacle, int, float]]:
        horizon_idx = self._forward_indices(ego_index, self.cfg["detect_ahead_m"])
        order = {int(idx): n for n, idx in enumerate(horizon_idx)}
        best = None
        best_order = math.inf
        half_width = self.cfg["drivable_width_m"] / 2.0

        for obs in obstacles:
            if obs.is_dynamic or obs.confidence < 0.5:
                continue

            # Static-mission-only size filter.
            # Keep small/thin detections available to ObjectDetector/ObjectTracker
            # (and later dynamic-obstacle logic), but do not use them as #4
            # static avoidance targets. The threshold is intentionally configured
            # here rather than in the shared detector.
            min_static_width = float(self.cfg.get("min_static_obstacle_width_m", 0.0))
            if obs.width < min_static_width:
                continue

            idx = nearest_index(self.path, obs.x, obs.y)
            if idx not in order:
                continue
            d = signed_lateral(self.path, idx, obs.x, obs.y)
            if abs(d) > half_width + obs.width / 2.0:
                continue
            if order[idx] < best_order:
                best = (obs, idx, d)
                best_order = order[idx]
        return best

    def _candidate_path(
        self, ego_index: int, obstacle_index: int, target_d: float
    ) -> Tuple[np.ndarray, np.ndarray]:
        total = (
            self.cfg["detect_ahead_m"]
            + self.cfg["hold_after_m"]
            + self.cfg["return_m"]
        )
        indices = self._forward_indices(ego_index, total)
        base = self.path[indices].copy()

        # Distance along this local slice, avoiding global-path wraparound issues.
        local_s = cumulative_s(base)
        obs_pos = int(np.where(indices == obstacle_index)[0][0])
        obs_s = local_s[obs_pos]
        enter_start = max(0.0, obs_s - self.cfg["approach_m"])
        enter_end = max(enter_start + 0.1, obs_s - self.cfg["hold_before_m"])
        exit_start = obs_s + self.cfg["hold_after_m"]
        exit_end = exit_start + self.cfg["return_m"]

        offsets = np.zeros(len(base))
        entering = (local_s >= enter_start) & (local_s < enter_end)
        holding = (local_s >= enter_end) & (local_s <= exit_start)
        exiting = (local_s > exit_start) & (local_s < exit_end)
        offsets[entering] = target_d * smoothstep5(
            (local_s[entering] - enter_start) / (enter_end - enter_start)
        )
        offsets[holding] = target_d
        offsets[exiting] = target_d * (
            1.0
            - smoothstep5((local_s[exiting] - exit_start) / (exit_end - exit_start))
        )

        xy = base[:, :2]
        tangent = np.gradient(xy, axis=0)
        norms = np.linalg.norm(tangent, axis=1)
        norms[norms < 1e-9] = 1.0
        tangent /= norms[:, None]
        normals = np.column_stack((-tangent[:, 1], tangent[:, 0]))
        base[:, :2] += normals * offsets[:, None]
        return base, offsets

    def _is_safe(self, path: np.ndarray, offsets: np.ndarray, obs: Obstacle) -> bool:
        boundary_limit = (
            self.cfg["drivable_width_m"] / 2.0
            - self.cfg["vehicle_width_m"] / 2.0
            - self.cfg["safety_margin_m"]
        )
        if np.max(np.abs(offsets)) > boundary_limit:
            return False

        # Conservative circular footprint for the first implementation.
        safe_radius = (
            self.cfg["vehicle_width_m"] / 2.0
            + obs.width / 2.0
            + self.cfg["safety_margin_m"]
        )
        distances = np.linalg.norm(path[:, :2] - np.array([obs.x, obs.y]), axis=1)
        return bool(np.min(distances) >= safe_radius)

    def plan(
        self, ego_x: float, ego_y: float, obstacles: List[Obstacle]
    ) -> PlanResult:
        ego_index = nearest_index(self.path, ego_x, ego_y)
        selected = self._select_obstacle(ego_index, obstacles)
        default_indices = self._forward_indices(ego_index, self.cfg["detect_ahead_m"])
        if selected is None:
            return PlanResult(
                "CLEAR", self.path[default_indices].copy(), None, None, 20.0,
                "No static obstacle in the planning corridor",
            )

        obs, obs_idx, obs_d = selected
        candidates = []
        for offset in self.cfg["candidate_offsets_m"]:
            candidate, offsets = self._candidate_path(ego_index, obs_idx, float(offset))
            if self._is_safe(candidate, offsets, obs):
                # Prefer the opposite side of the obstacle, then smaller displacement.
                opposite_penalty = 0.0 if offset * obs_d <= 0.0 else 10.0
                candidates.append((opposite_penalty + abs(offset), offset, candidate))

        if not candidates:
            return PlanResult(
                "STOP_REQUIRED", self.path[default_indices].copy(), obs.id, None, 0.0,
                "No collision-free candidate inside the configured drivable boundary",
            )

        _, offset, local_path = min(candidates, key=lambda item: item[0])
        return PlanResult(
            "AVOIDING",
            local_path,
            obs.id,
            "LEFT" if offset > 0.0 else "RIGHT",
            8.0,
            "Collision-free mock avoidance path generated",
        )

