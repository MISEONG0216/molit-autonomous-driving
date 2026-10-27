#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""57 km/h corner V4: preview-curvature LFD + preview-radius steering gain.

Goal
----
Keep V3 speed profile and 15 m curvature preview unchanged.
Add a small, bounded steering gain only when a tight curve is already visible
inside the preview window. This targets the remaining large corner tracking
errors without changing longitudinal control or the validated LFD logic.

Diagnostics added for root-cause analysis:
- PP target waypoint / waypoint gap
- target local x/y
- theta
- raw Pure Pursuit steering
- steering gain
- final steering

Existing baseline, V2 and V3 files are not modified.
"""

import csv
import math
from datetime import datetime

import pure_pursuit_profile_57_corner_v3_preview_lfd as v3

profile_test = v3.profile_test
base = v3.base
dynamic_test = v3.dynamic_test

# Keep V2/V3 speed profile exactly.
profile_test.build_test_profile = v3.build_v3_profile_capture
profile_test.DEFAULT_TEST_MAX_KMH = 57.0
profile_test.ALLOWED_TEST_MAX_KMH = 57.0

# Keep V3 preview distance exactly.
PREVIEW_DISTANCE_M = v3.PREVIEW_DISTANCE_M

# V4 test variable: modest steering amplification only for upcoming tight curves.
# Team tuning values, not competition-rule values.
STEER_GAIN_R20 = 1.12
STEER_GAIN_R40 = 1.08
STEER_GAIN_R80 = 1.04
STEER_GAIN_MAX = 1.12

_DIAG = {
    "pp_target_index": -1,
    "pp_target_gap": -1,
    "target_local_x_m": 0.0,
    "target_local_y_m": 0.0,
    "theta_deg": 0.0,
    "raw_steering_rad": 0.0,
    "steering_gain": 1.0,
    "final_steering_rad": 0.0,
}


def steering_gain_from_preview_radius(radius_m):
    if not math.isfinite(radius_m):
        return 1.0
    if radius_m < 20.0:
        return STEER_GAIN_R20
    if radius_m < 40.0:
        return STEER_GAIN_R40
    if radius_m < 80.0:
        return STEER_GAIN_R80
    return 1.0


def calculate_pure_pursuit_v4(path, current_index, x, y, yaw):
    # Same V3 LFD calculation.
    speed_kmh = dynamic_test._STATE["control_speed_kmh"]
    speed_lfd = dynamic_test.dynamic_lfd_m(speed_kmh)

    current_radius = (
        float(v3._RADIUS_PROFILE[current_index])
        if v3._RADIUS_PROFILE is not None
        else math.inf
    )
    preview_radius, preview_end = v3.preview_min_radius(current_index)

    curve_cap = v3.curvature_lfd_cap(preview_radius)
    lfd = max(v3.MIN_LFD_M, min(speed_lfd, curve_cap))

    v3._DIAG["current_radius_m"] = current_radius
    v3._DIAG["preview_radius_m"] = preview_radius
    v3._DIAG["preview_end_index"] = preview_end
    v3._DIAG["speed_lfd_m"] = speed_lfd
    v3._DIAG["curve_lfd_cap_m"] = curve_cap
    v3._DIAG["effective_lfd_m"] = lfd
    v3._DIAG["pp_target_index"] = -1
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
    raw_steering = math.atan2(
        2.0 * base.VEHICLE_LENGTH * math.sin(theta),
        lfd,
    )

    gain = min(
        STEER_GAIN_MAX,
        steering_gain_from_preview_radius(preview_radius),
    )
    final_steering = raw_steering * gain

    # Keep the command inside the same downstream normalized steering envelope.
    final_steering = max(-1.0, min(1.0, final_steering))

    v3._DIAG["pp_target_index"] = target
    _DIAG["pp_target_index"] = target
    _DIAG["pp_target_gap"] = target - current_index
    _DIAG["target_local_x_m"] = target_local_x
    _DIAG["target_local_y_m"] = target_local_y
    _DIAG["theta_deg"] = math.degrees(theta)
    _DIAG["raw_steering_rad"] = raw_steering
    _DIAG["steering_gain"] = gain
    _DIAG["final_steering_rad"] = final_steering

    return {
        "target_index": target,
        "target_x": path[target, 0],
        "target_y": path[target, 1],
        "local_x": target_local_x,
        "local_y": target_local_y,
        "theta": theta,
        "steering": final_steering,
    }


class V4DiagnosticWriter:
    def __init__(self, writer):
        self.writer = writer

    @staticmethod
    def fmt_radius(value):
        return "inf" if not math.isfinite(value) else f"{value:.3f}"

    def writerow(self, row):
        self.writer.writerow(
            list(row)
            + [
                self.fmt_radius(v3._DIAG["current_radius_m"]),
                self.fmt_radius(v3._DIAG["preview_radius_m"]),
                v3._DIAG["preview_end_index"],
                f"{v3._DIAG['speed_lfd_m']:.3f}",
                f"{v3._DIAG['curve_lfd_cap_m']:.3f}",
                f"{v3._DIAG['effective_lfd_m']:.3f}",
                _DIAG["pp_target_index"],
                _DIAG["pp_target_gap"],
                f"{_DIAG['target_local_x_m']:.3f}",
                f"{_DIAG['target_local_y_m']:.3f}",
                f"{_DIAG['theta_deg']:.3f}",
                f"{_DIAG['raw_steering_rad']:.6f}",
                f"{_DIAG['steering_gain']:.3f}",
                f"{_DIAG['final_steering_rad']:.6f}",
                f"{PREVIEW_DISTANCE_M:.1f}",
            ]
        )


def open_run_log_v4(max_speed_kmh, drive_mode):
    log_dir = base.PROJECT_ROOT / "logs"
    log_dir.mkdir(parents=True, exist_ok=True)

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    mode = "drive" if drive_mode else "dryrun"
    speed_tag = f"{max_speed_kmh:g}kph_corner_v4_steer_gain"
    log_path = log_dir / f"run_{timestamp}_{speed_tag}_{mode}.csv"

    log_file = open(log_path, "w", newline="", buffering=1)
    raw_writer = csv.writer(log_file)
    raw_writer.writerow([
        "elapsed_sec", "waypoint_index", "x_m", "y_m", "yaw_deg",
        "path_dist_m", "curve_radius_m", "full_profile_kmh",
        "target_speed_kmh", "comp_speed_kmh", "speed_error_kmh",
        "ctrl_zone", "steering_rad", "steering_deg", "long_cmd",
        "accel_cmd", "brake_cmd",
        "current_radius_m", "preview_min_radius_m", "preview_end_index",
        "speed_lfd_m", "curve_lfd_cap_m", "effective_lfd_m",
        "pp_target_index", "pp_target_gap",
        "target_local_x_m", "target_local_y_m", "theta_deg",
        "raw_steering_rad", "steering_gain", "final_steering_rad",
        "preview_distance_m",
    ])
    log_file.flush()
    return log_path, log_file, V4DiagnosticWriter(raw_writer)


# V4-only injections.
profile_test.build_test_profile = v3.build_v3_profile_capture
profile_test.open_run_log = open_run_log_v4
base.calculate_pure_pursuit = calculate_pure_pursuit_v4


if __name__ == "__main__":
    print("[STAGE] 57 km/h CORNER V4 - PREVIEW LFD + STEERING GAIN")
    print("[SPEED PROFILE] V2/V3 unchanged")
    print(f"[LFD PREVIEW] {PREVIEW_DISTANCE_M:.1f} m unchanged")
    print(
        "[STEER GAIN] "
        f"R<20:{STEER_GAIN_R20:.2f} / "
        f"<40:{STEER_GAIN_R40:.2f} / "
        f"<80:{STEER_GAIN_R80:.2f} / else:1.00"
    )
    print("[LOG] PP target/local geometry/raw+final steering diagnostics enabled")
    profile_test.main()
