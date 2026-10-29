#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""57 km/h corner V3: preview-curvature LFD test.

Experiment design
-----------------
Keep the V2 speed profile unchanged. Change only lateral lookahead selection:
instead of using only the radius at the current nearest waypoint, inspect a
short distance ahead and use the tightest upcoming radius for the LFD cap.

Diagnostics added:
- current curve radius
- preview minimum radius
- preview endpoint waypoint
- speed-based LFD
- curvature cap
- effective LFD
- Pure Pursuit target waypoint

Existing validated baseline/V2 files are not modified.
"""

import csv
import math
from datetime import datetime

import pure_pursuit_profile_57_corner_v2 as v2
import pure_pursuit_profile_55_curvature_lfd_test as curvature_base

profile_test = v2.profile_test
base = v2.base

# Keep V2 longitudinal/profile behavior exactly.
profile_test.build_test_profile = v2.build_corner_v2_profile
profile_test.DEFAULT_TEST_MAX_KMH = 57.0
profile_test.ALLOWED_TEST_MAX_KMH = 57.0

# Reuse the validated dynamic-LFD state/speed source.
dynamic_test = curvature_base.dynamic_test
MIN_LFD_M = curvature_base.MIN_LFD_M
MAX_LFD_M = curvature_base.MAX_LFD_M

# V3 test variable: preview distance for upcoming curvature.
PREVIEW_DISTANCE_M = 15.0

_RADIUS_PROFILE = None
_PATH_DS = None

_DIAG = {
    "current_radius_m": math.inf,
    "preview_radius_m": math.inf,
    "preview_end_index": -1,
    "speed_lfd_m": MIN_LFD_M,
    "curve_lfd_cap_m": MAX_LFD_M,
    "effective_lfd_m": MIN_LFD_M,
    "pp_target_index": -1,
}


def build_v3_profile_capture(global_path, max_speed_kmh):
    global _RADIUS_PROFILE, _PATH_DS
    target, radius, full_profile = v2.build_corner_v2_profile(
        global_path, max_speed_kmh
    )
    _, ds = profile_test.profile_builder.cumulative_distance(global_path)
    _RADIUS_PROFILE = radius
    _PATH_DS = ds
    return target, radius, full_profile


def curvature_lfd_cap(radius_m):
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


def preview_min_radius(current_index):
    """Tightest radius within PREVIEW_DISTANCE_M ahead on the closed path."""
    if _RADIUS_PROFILE is None or _PATH_DS is None:
        return math.inf, current_index

    n = len(_RADIUS_PROFILE)
    idx = current_index
    travelled = 0.0
    min_radius = float(_RADIUS_PROFILE[idx])
    end_idx = idx

    # Follow waypoint order and wrap at END because the official path is closed.
    while travelled < PREVIEW_DISTANCE_M:
        next_idx = (idx + 1) % n
        ds_idx = idx if idx < len(_PATH_DS) else 0
        if idx == n - 1:
            # Approximate the closing segment using path spacing near END.
            step = float(_PATH_DS[-1])
        else:
            step = float(_PATH_DS[ds_idx])
        travelled += max(step, 1e-6)
        idx = next_idx
        end_idx = idx
        r = float(_RADIUS_PROFILE[idx])
        if r < min_radius:
            min_radius = r

        # Defensive bound.
        if end_idx == current_index:
            break

    return min_radius, end_idx


def calculate_pure_pursuit_preview_lfd(path, current_index, x, y, yaw):
    speed_kmh = dynamic_test._STATE["control_speed_kmh"]
    speed_lfd = dynamic_test.dynamic_lfd_m(speed_kmh)

    current_radius = (
        float(_RADIUS_PROFILE[current_index])
        if _RADIUS_PROFILE is not None
        else math.inf
    )
    preview_radius, preview_end = preview_min_radius(current_index)

    curve_cap = curvature_lfd_cap(preview_radius)
    lfd = max(MIN_LFD_M, min(speed_lfd, curve_cap))

    _DIAG["current_radius_m"] = current_radius
    _DIAG["preview_radius_m"] = preview_radius
    _DIAG["preview_end_index"] = preview_end
    _DIAG["speed_lfd_m"] = speed_lfd
    _DIAG["curve_lfd_cap_m"] = curve_cap
    _DIAG["effective_lfd_m"] = lfd
    _DIAG["pp_target_index"] = -1

    # Preserve the inherited diagnostic state where other wrappers expect it.
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

    _DIAG["pp_target_index"] = target

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


class V3DiagnosticWriter:
    def __init__(self, writer):
        self.writer = writer

    @staticmethod
    def fmt_radius(value):
        return "inf" if not math.isfinite(value) else f"{value:.3f}"

    def writerow(self, row):
        self.writer.writerow(
            list(row)
            + [
                self.fmt_radius(_DIAG["current_radius_m"]),
                self.fmt_radius(_DIAG["preview_radius_m"]),
                _DIAG["preview_end_index"],
                f"{_DIAG['speed_lfd_m']:.3f}",
                f"{_DIAG['curve_lfd_cap_m']:.3f}",
                f"{_DIAG['effective_lfd_m']:.3f}",
                _DIAG["pp_target_index"],
                f"{PREVIEW_DISTANCE_M:.1f}",
            ]
        )


def open_run_log_v3(max_speed_kmh, drive_mode):
    log_dir = base.PROJECT_ROOT / "logs"
    log_dir.mkdir(parents=True, exist_ok=True)

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    mode = "drive" if drive_mode else "dryrun"
    speed_tag = f"{max_speed_kmh:g}kph_corner_v3_preview_lfd"
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
        "current_radius_m",
        "preview_min_radius_m",
        "preview_end_index",
        "speed_lfd_m",
        "curve_lfd_cap_m",
        "effective_lfd_m",
        "pp_target_index",
        "preview_distance_m",
    ])
    log_file.flush()
    return log_path, log_file, V3DiagnosticWriter(raw_writer)


# V3-only injections.
profile_test.build_test_profile = build_v3_profile_capture
profile_test.open_run_log = open_run_log_v3
base.calculate_pure_pursuit = calculate_pure_pursuit_preview_lfd


if __name__ == "__main__":
    print("[STAGE] 57 km/h CORNER V3 - PREVIEW CURVATURE LFD")
    print("[SPEED PROFILE] V2 unchanged")
    print(f"[LFD PREVIEW] {PREVIEW_DISTANCE_M:.1f} m ahead")
    print("[LFD CAPS] R<20:3.0 / <40:4.0 / <80:5.0 / <150:6.5 / else:9.0 m")
    print("[LOG] current/preview radius + LFD + PP target diagnostics enabled")
    profile_test.main()
