#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""50 km/h staged profile test with negative-speed clamp + dynamic LFD.

This test changes ONE lateral variable relative to the validated 50 km/h clamp run:
lookahead distance becomes speed dependent.

Reference structure from MORAI-ADModule Pure Pursuit:
    lfd = lfd_gain * vehicle_speed
    lfd = clip(lfd, min_lfd, max_lfd)

For this competition test we preserve the empirically proven 2.5 m low-speed
lookahead, use MORAI's example gain 0.65 with speed converted from km/h to m/s,
and cap the first high-speed validation at 9.0 m.

    dynamic_lfd = clip(0.65 * speed_mps, 2.5, 9.0)

Approximate values:
    5 km/h  -> 2.5 m (minimum)
    20 km/h -> 3.6 m
    30 km/h -> 5.4 m
    40 km/h -> 7.2 m
    50 km/h -> 9.0 m

Longitudinal control, waypoint speed profile, negative-speed clamp, steering
formula, and competition-valid cmd_type=1 / ctrl_mode=2 remain unchanged.
"""

import csv
import math
from datetime import datetime

import pure_pursuit_profile_45_clamp_test as clamp_test


profile_test = clamp_test.profile_test
base = profile_test.base
profile_test.ALLOWED_TEST_MAX_KMH = 50.0

LFD_GAIN = 0.65
MIN_LFD_M = 2.5
MAX_LFD_M = 9.0

_STATE = {
    "control_speed_kmh": 0.0,
    "dynamic_lfd_m": MIN_LFD_M,
}


class DynamicLfdSpeedReceiver(clamp_test.ClampedCompetitionSpeedReceiver):
    """Reuse validated negative-speed clamp and expose control speed to LFD logic."""

    def read(self):
        speed_kmh, valid = super().read()
        if valid:
            _STATE["control_speed_kmh"] = speed_kmh
        return speed_kmh, valid


def dynamic_lfd_m(speed_kmh):
    speed_mps = max(0.0, speed_kmh) / 3.6
    lfd = LFD_GAIN * speed_mps
    return max(MIN_LFD_M, min(MAX_LFD_M, lfd))


def calculate_pure_pursuit_dynamic(path, current_index, x, y, yaw):
    """Same proven Pure Pursuit geometry, with only LFD made speed-dependent."""
    lfd = dynamic_lfd_m(_STATE["control_speed_kmh"])
    _STATE["dynamic_lfd_m"] = lfd

    end_index = min(current_index + base.LOCAL_PATH_SIZE, len(path))
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)

    target = None
    target_local_x = None
    target_local_y = None

    for i in range(current_index, end_index):
        dx = path[i, 0] - x
        dy = path[i, 1] - y

        local_x = cos_yaw * dx + sin_yaw * dy
        local_y = -sin_yaw * dx + cos_yaw * dy

        if local_x <= 0.0:
            continue

        if math.hypot(local_x, local_y) >= lfd:
            target = i
            target_local_x = local_x
            target_local_y = local_y
            break

    if target is None:
        return None

    theta = math.atan2(target_local_y, target_local_x)
    steering = math.atan2(
        2.0 * base.VEHICLE_LENGTH * math.sin(theta),
        lfd,
    )

    return {
        "target_index": target,
        "target_x": path[target, 0],
        "target_y": path[target, 1],
        "local_x": target_local_x,
        "local_y": target_local_y,
        "theta": theta,
        "steering": steering,
    }


class DynamicDiagnosticWriter:
    """Append raw speed diagnostics, clamp state, and active LFD."""

    def __init__(self, writer):
        self.writer = writer

    def writerow(self, row):
        self.writer.writerow(
            list(row)
            + [
                f"{clamp_test._DIAG['raw_signed_vel_kmh']:.3f}",
                f"{clamp_test._DIAG['vel_x_kmh']:.3f}",
                clamp_test._DIAG["negative_clamp_active"],
                f"{_STATE['dynamic_lfd_m']:.3f}",
            ]
        )


def open_run_log_dynamic(max_speed_kmh, drive_mode):
    log_dir = base.PROJECT_ROOT / "logs"
    log_dir.mkdir(parents=True, exist_ok=True)

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    mode = "drive" if drive_mode else "dryrun"
    speed_tag = f"{max_speed_kmh:g}kph_dynamic_lfd"
    log_path = log_dir / f"run_{timestamp}_{speed_tag}_{mode}.csv"

    log_file = open(log_path, "w", newline="", buffering=1)
    raw_writer = csv.writer(log_file)
    raw_writer.writerow([
        "elapsed_sec",
        "waypoint_index",
        "x_m",
        "y_m",
        "yaw_deg",
        "path_dist_m",
        "curve_radius_m",
        "full_profile_kmh",
        "target_speed_kmh",
        "comp_speed_kmh",
        "speed_error_kmh",
        "ctrl_zone",
        "steering_rad",
        "steering_deg",
        "long_cmd",
        "accel_cmd",
        "brake_cmd",
        "raw_signed_vel_kmh",
        "vel_x_kmh",
        "negative_clamp_active",
        "dynamic_lfd_m",
    ])
    log_file.flush()
    return log_path, log_file, DynamicDiagnosticWriter(raw_writer)


# Test-only injections. Validated shared files remain unchanged.
profile_test.CompetitionSpeedReceiver = DynamicLfdSpeedReceiver
profile_test.open_run_log = open_run_log_dynamic
base.calculate_pure_pursuit = calculate_pure_pursuit_dynamic


if __name__ == "__main__":
    print(
        f"[DYNAMIC LFD] gain={LFD_GAIN:.2f}, "
        f"min={MIN_LFD_M:.1f} m, max={MAX_LFD_M:.1f} m"
    )
    profile_test.main()
