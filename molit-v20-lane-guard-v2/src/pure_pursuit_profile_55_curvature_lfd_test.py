#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""55 km/h staged test: speed-based dynamic LFD + curvature cap.

Purpose
-------
The 55 km/h speed-only dynamic-LFD run reduced straight-line hunting, but
corner tracking error increased. This test keeps the validated speed-based LFD
on straights and caps LFD only where the global-path curvature becomes tighter.

Base speed LFD (unchanged):
    speed_lfd = clip(0.65 * speed_mps, 2.5, 9.0)

Curvature-dependent maximum LFD used only as an upper bound:
    radius < 20 m       -> cap 3.0 m
    20 <= radius < 40   -> cap 4.0 m
    40 <= radius < 80   -> cap 5.0 m
    80 <= radius < 150  -> cap 6.5 m
    radius >= 150 m     -> cap 9.0 m

Effective LFD:
    lfd = max(2.5, min(speed_lfd, curvature_cap))

Only lateral lookahead selection is changed. Competition speed source,
negative-speed clamp, speed profile, braking preview, longitudinal control,
Pure Pursuit steering formula, ctrl_mode=2 and cmd_type=1 remain unchanged.
"""

import csv
import math
from datetime import datetime

import pure_pursuit_profile_50_dynamic_lfd_test as dynamic_test


profile_test = dynamic_test.profile_test
base = dynamic_test.base
profile_test.ALLOWED_TEST_MAX_KMH = 55.0

# Keep the validated speed-based LFD parameters.
LFD_GAIN = dynamic_test.LFD_GAIN
MIN_LFD_M = dynamic_test.MIN_LFD_M
MAX_LFD_M = dynamic_test.MAX_LFD_M

# Radius profile is captured from the same profile builder used by the test.
_RADIUS_PROFILE = None

_STATE = {
    "speed_lfd_m": MIN_LFD_M,
    "curve_lfd_cap_m": MAX_LFD_M,
    "effective_lfd_m": MIN_LFD_M,
}


def curvature_lfd_cap(radius_m):
    """Return a conservative LFD upper bound for the current path radius."""
    if not math.isfinite(radius_m):
        return MAX_LFD_M
    if radius_m < 20.0:
        return 3.0
    if radius_m < 40.0:
        return 4.0
    if radius_m < 80.0:
        return 5.0
    if radius_m < 150.0:
        return 6.5
    return MAX_LFD_M


_original_build_test_profile = profile_test.build_test_profile


def build_test_profile_capture_radius(global_path, max_speed_kmh):
    """Reuse the validated profile builder and retain its radius array."""
    global _RADIUS_PROFILE
    test_profile, radius_profile, full_profile = _original_build_test_profile(
        global_path, max_speed_kmh
    )
    _RADIUS_PROFILE = radius_profile
    return test_profile, radius_profile, full_profile


def calculate_pure_pursuit_curvature_lfd(path, current_index, x, y, yaw):
    """Same Pure Pursuit geometry with speed LFD capped by local curvature."""
    speed_kmh = dynamic_test._STATE["control_speed_kmh"]
    speed_lfd = dynamic_test.dynamic_lfd_m(speed_kmh)

    if _RADIUS_PROFILE is None:
        radius_m = math.inf
    else:
        radius_m = float(_RADIUS_PROFILE[current_index])

    curve_cap = curvature_lfd_cap(radius_m)
    lfd = max(MIN_LFD_M, min(speed_lfd, curve_cap))

    _STATE["speed_lfd_m"] = speed_lfd
    _STATE["curve_lfd_cap_m"] = curve_cap
    _STATE["effective_lfd_m"] = lfd
    # Preserve the existing dynamic-LFD diagnostic field semantics as the
    # actually active LFD for easy comparison with prior logs.
    dynamic_test._STATE["dynamic_lfd_m"] = lfd

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


class CurvatureDiagnosticWriter:
    """Append raw speed diagnostics plus speed/corner/effective LFD values."""

    def __init__(self, writer):
        self.writer = writer

    def writerow(self, row):
        self.writer.writerow(
            list(row)
            + [
                f"{dynamic_test.clamp_test._DIAG['raw_signed_vel_kmh']:.3f}",
                f"{dynamic_test.clamp_test._DIAG['vel_x_kmh']:.3f}",
                dynamic_test.clamp_test._DIAG["negative_clamp_active"],
                f"{_STATE['effective_lfd_m']:.3f}",
                f"{_STATE['speed_lfd_m']:.3f}",
                f"{_STATE['curve_lfd_cap_m']:.3f}",
            ]
        )


def open_run_log_curvature(max_speed_kmh, drive_mode):
    log_dir = base.PROJECT_ROOT / "logs"
    log_dir.mkdir(parents=True, exist_ok=True)

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    mode = "drive" if drive_mode else "dryrun"
    speed_tag = f"{max_speed_kmh:g}kph_curvature_lfd"
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
        "speed_lfd_m",
        "curve_lfd_cap_m",
    ])
    log_file.flush()
    return log_path, log_file, CurvatureDiagnosticWriter(raw_writer)


# Test-only injections. Existing validated files remain unchanged.
profile_test.build_test_profile = build_test_profile_capture_radius
profile_test.open_run_log = open_run_log_curvature
base.calculate_pure_pursuit = calculate_pure_pursuit_curvature_lfd


if __name__ == "__main__":
    print("[STAGE] 55 km/h curvature-capped dynamic-LFD validation")
    print(
        f"[LFD] speed gain={LFD_GAIN:.2f}, min={MIN_LFD_M:.1f} m, "
        f"straight max={MAX_LFD_M:.1f} m"
    )
    print("[LFD CAPS] R<20:3.0 / <40:4.0 / <80:5.0 / <150:6.5 / else:9.0 m")
    profile_test.main()
