from dataclasses import dataclass
from typing import List, Optional, Tuple
import math

import numpy as np


@dataclass
class AvoidanceObstacle:
    """Obstacle information required only by the path generator.

    x, y are global-map coordinates in metres.
    width is used for the conservative collision check.
    id is carried through only for diagnostics.
    """
    id: int
    x: float
    y: float
    width: float


@dataclass
class AvoidancePathResult:
    status: str
    local_path: np.ndarray
    obstacle_id: Optional[int]
    avoidance_side: Optional[str]
    reason: str
    candidate_diagnostics: Optional[List[dict]] = None


@dataclass
class AvoidancePathConfig:
    """Geometry values copied from the validated #4 avoidance-path logic."""

    drivable_width_m: float = 7.0
    vehicle_width_m: float = 1.8
    safety_margin_m: float = 0.4
    detect_ahead_m: float = 35.0
    approach_m: float = 12.0
    hold_before_m: float = 3.0
    hold_after_m: float = 5.0
    return_m: float = 12.0
    candidate_offsets_m: Tuple[float, ...] = (-2.2, -1.8, 1.8, 2.2)


def cumulative_s(path: np.ndarray) -> np.ndarray:
    ds = np.linalg.norm(np.diff(path[:, :2], axis=0), axis=1)
    return np.concatenate(([0.0], np.cumsum(ds)))


def nearest_index(path: np.ndarray, x: float, y: float) -> int:
    delta = path[:, :2] - np.array([x, y], dtype=float)
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
    """Fifth-order smoothstep used by #4 for smooth lane departure/return."""
    t = np.clip(t, 0.0, 1.0)
    return 10.0 * t**3 - 15.0 * t**4 + 6.0 * t**5


class AvoidancePathGenerator:
    """Independent local avoidance-path generator.

    This module deliberately does NOT:
    - receive LiDAR/camera/GPS/IMU,
    - decide static vs dynamic,
    - decide whether an obstacle is a mission target,
    - send Accel/Brake/Steer,
    - run Pure Pursuit.

    Input:
        global path + ego global position + selected obstacle global position/width

    Output:
        CLEAR / AVOIDING / STOP_REQUIRED and a local path.

    The candidate-path geometry and safety test are extracted from the validated
    static-obstacle (#4) planner so the same generator can later be shared by
    static and dynamic mission logic.
    """

    def __init__(
        self,
        global_path: np.ndarray,
        config: Optional[AvoidancePathConfig] = None,
    ):
        path = np.asarray(global_path, dtype=float)
        if path.ndim != 2 or path.shape[1] < 2:
            raise ValueError("global_path must contain x y [z] columns")
        if path.shape[1] == 2:
            path = np.column_stack((path, np.zeros(len(path))))
        self.path = path[:, :3].copy()
        self.cfg = config or AvoidancePathConfig()

    def _forward_indices(self, ego_index: int, distance_m: float) -> np.ndarray:
        # Competition path is treated as a closed loop, matching #4.
        result = [ego_index]
        travelled = 0.0
        i = ego_index
        while travelled < distance_m and len(result) <= len(self.path):
            j = (i + 1) % len(self.path)
            travelled += float(
                np.linalg.norm(self.path[j, :2] - self.path[i, :2])
            )
            result.append(j)
            i = j
        return np.asarray(result, dtype=int)

    def default_path(self, ego_x: float, ego_y: float) -> np.ndarray:
        """Return the normal forward global-path slice."""
        ego_index = nearest_index(self.path, ego_x, ego_y)
        indices = self._forward_indices(ego_index, self.cfg.detect_ahead_m)
        return self.path[indices].copy()

    def _candidate_path(
        self,
        ego_index: int,
        obstacle_index: int,
        target_offset_m: float,
    ) -> Tuple[np.ndarray, np.ndarray]:
        total = (
            self.cfg.detect_ahead_m
            + self.cfg.hold_after_m
            + self.cfg.return_m
        )
        indices = self._forward_indices(ego_index, total)
        base = self.path[indices].copy()

        matches = np.where(indices == obstacle_index)[0]
        if len(matches) == 0:
            raise ValueError("Obstacle is not inside the forward path horizon")

        local_s = cumulative_s(base)
        obs_pos = int(matches[0])
        obs_s = local_s[obs_pos]

        enter_start = max(0.0, obs_s - self.cfg.approach_m)
        enter_end = max(
            enter_start + 0.1,
            obs_s - self.cfg.hold_before_m,
        )
        exit_start = obs_s + self.cfg.hold_after_m
        exit_end = exit_start + self.cfg.return_m

        offsets = np.zeros(len(base), dtype=float)

        entering = (local_s >= enter_start) & (local_s < enter_end)
        holding = (local_s >= enter_end) & (local_s <= exit_start)
        exiting = (local_s > exit_start) & (local_s < exit_end)

        offsets[entering] = target_offset_m * smoothstep5(
            (local_s[entering] - enter_start) / (enter_end - enter_start)
        )
        offsets[holding] = target_offset_m
        offsets[exiting] = target_offset_m * (
            1.0
            - smoothstep5(
                (local_s[exiting] - exit_start) / (exit_end - exit_start)
            )
        )

        xy = base[:, :2]
        tangent = np.gradient(xy, axis=0)
        norms = np.linalg.norm(tangent, axis=1)
        norms[norms < 1e-9] = 1.0
        tangent /= norms[:, None]

        normals = np.column_stack((-tangent[:, 1], tangent[:, 0]))
        base[:, :2] += normals * offsets[:, None]

        return base, offsets

    def _candidate_safety_diagnostic(
        self,
        path: np.ndarray,
        offsets: np.ndarray,
        obstacle: AvoidanceObstacle,
    ) -> dict:
        boundary_limit = (
            self.cfg.drivable_width_m / 2.0
            - self.cfg.vehicle_width_m / 2.0
            - self.cfg.safety_margin_m
        )
        max_abs_offset = float(np.max(np.abs(offsets)))

        # Same conservative circular-footprint check used by #4.
        safe_radius = (
            self.cfg.vehicle_width_m / 2.0
            + obstacle.width / 2.0
            + self.cfg.safety_margin_m
        )

        distances = np.linalg.norm(
            path[:, :2] - np.array([obstacle.x, obstacle.y]),
            axis=1,
        )
        min_clearance = float(np.min(distances))

        boundary_ok = max_abs_offset <= boundary_limit
        collision_ok = min_clearance >= safe_radius

        if not boundary_ok:
            reason = "BOUNDARY"
        elif not collision_ok:
            reason = "COLLISION"
        else:
            reason = "SAFE"

        return {
            "safe": bool(boundary_ok and collision_ok),
            "reason": reason,
            "boundary_limit_m": float(boundary_limit),
            "max_abs_offset_m": max_abs_offset,
            "safe_radius_m": float(safe_radius),
            "min_clearance_m": min_clearance,
        }

    def generate(
        self,
        ego_x: float,
        ego_y: float,
        obstacle: Optional[AvoidanceObstacle],
    ) -> AvoidancePathResult:
        """Generate an avoidance path for one already-selected obstacle.

        Important:
        The caller is responsible for deciding *whether* this obstacle should be
        avoided. For a future dynamic mission, prediction/risk logic should make
        that decision before calling this method.
        """
        ego_index = nearest_index(self.path, ego_x, ego_y)
        default_indices = self._forward_indices(
            ego_index,
            self.cfg.detect_ahead_m,
        )
        default_path = self.path[default_indices].copy()

        if obstacle is None:
            return AvoidancePathResult(
                status="CLEAR",
                local_path=default_path,
                obstacle_id=None,
                avoidance_side=None,
                reason="No avoidance target supplied",
            )

        obstacle_index = nearest_index(
            self.path,
            obstacle.x,
            obstacle.y,
        )

        horizon_indices = self._forward_indices(
            ego_index,
            self.cfg.detect_ahead_m,
        )
        if obstacle_index not in set(int(i) for i in horizon_indices):
            return AvoidancePathResult(
                status="CLEAR",
                local_path=default_path,
                obstacle_id=obstacle.id,
                avoidance_side=None,
                reason="Avoidance target is outside the forward planning horizon",
            )

        obstacle_lateral = signed_lateral(
            self.path,
            obstacle_index,
            obstacle.x,
            obstacle.y,
        )

        candidates = []
        diagnostics = []

        for target_offset in self.cfg.candidate_offsets_m:
            candidate, offsets = self._candidate_path(
                ego_index,
                obstacle_index,
                float(target_offset),
            )

            diagnostic = self._candidate_safety_diagnostic(
                candidate,
                offsets,
                obstacle,
            )
            diagnostic["offset_m"] = float(target_offset)
            diagnostics.append(diagnostic)

            if diagnostic["safe"]:
                # Same #4 preference:
                # first prefer the side opposite the obstacle, then the smaller
                # lateral displacement.
                opposite_penalty = (
                    0.0 if target_offset * obstacle_lateral <= 0.0 else 10.0
                )
                score = opposite_penalty + abs(target_offset)
                candidates.append((score, target_offset, candidate))

        if not candidates:
            return AvoidancePathResult(
                status="STOP_REQUIRED",
                local_path=default_path,
                obstacle_id=obstacle.id,
                avoidance_side=None,
                reason=(
                    "No collision-free candidate inside the configured "
                    "drivable boundary"
                ),
                candidate_diagnostics=diagnostics,
            )

        _, selected_offset, local_path = min(
            candidates,
            key=lambda item: item[0],
        )

        return AvoidancePathResult(
            status="AVOIDING",
            local_path=local_path,
            obstacle_id=obstacle.id,
            avoidance_side="LEFT" if selected_offset > 0.0 else "RIGHT",
            reason="Collision-free avoidance path generated",
            candidate_diagnostics=diagnostics,
        )
