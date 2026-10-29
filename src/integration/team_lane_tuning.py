#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Opt-in V38 tracking-path refinement assembled from team V20/V22 branches.

Only a separate Pure Pursuit reference is moved. Official global waypoints,
nearest-WP lookup, traffic/mission indices, LiDAR planner, V7 speed profile,
and GPS-shadow camera steering remain unchanged. This is NOT a lane-boundary
detector and cannot guarantee tire-to-line or tunnel-wall clearance.

Provenance: V22 corner offsets (WP125..215, WP231..300, apex WP248..292),
V20-Beomsu yellow-line candidates (WP1480..1715, WP1859..1902,
WP3640..3820, WP4050..4300) and roundabout slowdown WP1770..1902.
WP195 exit-right and WP144 steering-pulse experiments were alternatives,
not additive to the later WP231/right and WP150/left tracking-path tuning.
"""
import math
import numpy as np

from integration.beomsu_lane_guard import make_zones, make_tracking_path
from integration.first_corner_left_offset import make_first_corner_left_path
from integration.second_corner_right_offset import make_second_corner_right_path
from integration.second_corner_apex_extra_right import make_apex_extra_right_path
from integration.roundabout_exit_offset import make_roundabout_exit_path
from integration.roundabout_speed_cap import build_roundabout_speed_caps

EXPECTED_WAYPOINTS = 4430
MAX_TEST_SPEED_KMH = 120.0
OFFSET_ZONE_SPEED_CAP_KMH = 20.0

# Fixed candidates from the team's experimental branches (not proven tire clearance).
GUARDRAIL_RIGHT_M = 0.25
FIRST_CORNER_LEFT_M = 0.30
SECOND_CORNER_RIGHT_M = 0.40
SECOND_CORNER_APEX_EXTRA_RIGHT_M = 0.10
ROUNDABOUT_EXIT_RIGHT_M = 0.25
ROUNDABOUT_HOLD_SPEED_KMH = 8.0
EXTRA_ZONES = ("3640:3820:0.12", "4050:4300:0.25")


def build_team_lane_plan(official_path, requested_max_kmh):
    """Return (tracking_path_copy, per-WP speed_caps, shift_magnitude_by_wp).

    Requires the exact 4430-point competition-path length. No source mutation,
    no sockets, and no direct vehicle control. Speed caps only reduce targets.
    """
    official = np.asarray(official_path, dtype=float)
    if official.ndim != 2 or official.shape[1] < 2:
        raise ValueError("Official global path must be Nx2 or Nx3")
    if len(official) != EXPECTED_WAYPOINTS:
        raise ValueError("Team WP tuning requires the known 4430-point global path")
    if not np.all(np.isfinite(official[:, :2])):
        raise ValueError("Official global path has non-finite XY coordinates")

    maximum = float(requested_max_kmh)
    if not math.isfinite(maximum) or not 0.0 < maximum <= MAX_TEST_SPEED_KMH:
        raise ValueError("Team lane tuning requires 0 < --max-speed <= 120 km/h")

    zones = make_zones(len(official), GUARDRAIL_RIGHT_M, EXTRA_ZONES)
    tracking = make_tracking_path(official, zones)
    tracking = make_first_corner_left_path(tracking, FIRST_CORNER_LEFT_M)
    # Never stack the older WP195 exit-right option with the newer WP231 turn.
    tracking = make_second_corner_right_path(tracking, SECOND_CORNER_RIGHT_M)
    tracking = make_apex_extra_right_path(tracking, SECOND_CORNER_APEX_EXTRA_RIGHT_M)
    tracking = make_roundabout_exit_path(tracking, ROUNDABOUT_EXIT_RIGHT_M)

    # Signed lateral normal orientation is already enforced by source modules.
    # This magnitude is for post-drive verification, NOT physical paint clearance.
    shifts = np.linalg.norm(tracking[:, :2] - official[:, :2], axis=1)
    if not np.all(np.isfinite(shifts)) or float(np.max(shifts)) > 0.501:
        raise ValueError("Unsafe/unexpected tracking path offset (>0.50 m)")

    # The V20 roundabout cap has long braking/release ramps. At a 120 km/h
    # request those ramps would otherwise throttle the V38 high-speed zone
    # WP2570-3040, even where the team's path is completely unchanged.
    # Bound the roundabout ramp to the existing V38 normal-road 57 km/h
    # reference. Outside that ramp, leave the ORIGINAL V38 speed profile
    # (including the verified highway fast zone) free to set the target.
    normal_road_reference = min(maximum, 57.0)
    roundabout_caps, _zones = build_roundabout_speed_caps(
        official, normal_road_reference, ROUNDABOUT_HOLD_SPEED_KMH,
    )
    caps = np.asarray(roundabout_caps, dtype=float)
    if maximum > normal_road_reference:
        caps[caps >= normal_road_reference - 1e-8] = maximum
    caps[shifts > 1e-8] = np.minimum(
        caps[shifts > 1e-8], OFFSET_ZONE_SPEED_CAP_KMH,
    )
    if not np.all(np.isfinite(caps)) or np.any(caps <= 0.0):
        raise ValueError("Invalid refined speed cap")
    if np.any(caps > maximum + 1e-8):
        raise ValueError("Refined cap may never increase the requested speed")
    if np.shares_memory(tracking, official):
        raise ValueError("Tracking path unexpectedly aliases official path")
    return tracking, caps, shifts
