#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Longitudinal speed controller for final sensor integration.

Preserves the validated 57 km/h staged speed-profile behavior and the final
0.10 brake-command ceiling. This module calculates targets/commands only; it
does not receive UDP data or send EgoCtrlCmd.
"""
import numpy as np
import path_speed_profile as profile_builder

MAX_SPEED_KMH = 57.0
MIN_PROFILE_SPEED_KMH = 5.0

ACCEL_KP = 0.10
MAX_ACCEL_CMD = 0.30
ACCEL_CUTOFF_BELOW_TARGET_KMH = 0.20
BRAKE_TRIGGER_ABOVE_TARGET_KMH = 2.00
BRAKE_KP = 0.04
MAX_BRAKE_CMD = 0.10
PATH_LOST_BRAKE_CMD = 0.12
LONGITUDINAL_CMD_RATE_PER_SEC = 0.80

def build_speed_profile(path, max_speed_kmh=MAX_SPEED_KMH):
    """Reproduce the staged profile used by the validated 57 km/h run."""
    _, ds = profile_builder.cumulative_distance(path)
    curvature = profile_builder.compute_curvature(path)
    radius = profile_builder.curvature_to_radius(curvature)
    seed = profile_builder.build_seed_profile(radius)
    full_profile = profile_builder.apply_braking_preview(seed, ds)

    full_min = float(np.min(full_profile))
    full_max = float(np.max(full_profile))
    if full_max - full_min < 1e-6:
        target = np.full_like(full_profile, max_speed_kmh)
    else:
        normalized = (full_profile - full_min) / (full_max - full_min)
        target = MIN_PROFILE_SPEED_KMH + normalized * (
            max_speed_kmh - MIN_PROFILE_SPEED_KMH
        )
    return target

class SpeedController:
    def __init__(self, path, max_speed_kmh=MAX_SPEED_KMH):
        if max_speed_kmh < MIN_PROFILE_SPEED_KMH or max_speed_kmh > MAX_SPEED_KMH:
            raise ValueError("max_speed_kmh는 5~57 km/h 범위여야 합니다.")
        self.target_profile = build_speed_profile(path, max_speed_kmh)
        self.signed_command = 0.0

    def target_speed(self, waypoint_index):
        return float(self.target_profile[waypoint_index])

    def reset(self):
        self.signed_command = 0.0

    def calculate(self, target_kmh, current_kmh, dt):
        """Return accel/brake using the validated accel/coast/brake policy."""
        error = target_kmh - current_kmh
        accel_cutoff = max(0.0, target_kmh - ACCEL_CUTOFF_BELOW_TARGET_KMH)
        brake_trigger = target_kmh + BRAKE_TRIGGER_ABOVE_TARGET_KMH

        if current_kmh < accel_cutoff:
            desired = min(MAX_ACCEL_CMD, max(0.0, ACCEL_KP * error))
            zone = "ACCEL"
        elif current_kmh <= brake_trigger:
            desired = 0.0
            zone = "COAST"
        else:
            overspeed = current_kmh - brake_trigger
            desired = -min(MAX_BRAKE_CMD, max(0.0, BRAKE_KP * overspeed))
            zone = "BRAKE"

        max_delta = LONGITUDINAL_CMD_RATE_PER_SEC * max(float(dt), 1e-3)
        delta = max(-max_delta, min(max_delta, desired - self.signed_command))
        self.signed_command += delta
        self.signed_command = max(
            -PATH_LOST_BRAKE_CMD,
            min(MAX_ACCEL_CMD, self.signed_command),
        )

        accel = max(0.0, self.signed_command)
        brake = max(0.0, -self.signed_command)
        return {
            "accel": accel,
            "brake": brake,
            "target_speed": float(target_kmh),
            "speed_error": error,
            "zone": zone,
        }

    def path_lost(self, dt):
        """Generate the existing conservative path-lost braking command."""
        max_delta = LONGITUDINAL_CMD_RATE_PER_SEC * max(float(dt), 1e-3)
        desired = -PATH_LOST_BRAKE_CMD
        delta = max(-max_delta, min(max_delta, desired - self.signed_command))
        self.signed_command += delta
        self.signed_command = max(-PATH_LOST_BRAKE_CMD, self.signed_command)
        return {"accel": 0.0, "brake": max(0.0, -self.signed_command),
                "target_speed": 0.0, "speed_error": 0.0, "zone": "PATH_LOST"}
