#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""57 km/h corner V7: V5 + earlier braking preview for tight curves.

Only new test variable
----------------------
Detect an upcoming left/right curvature-sign change inside the same 15 m
preview window and, near that transition, bias the Pure Pursuit target toward
the post-transition path. V4 speed profile, LFD logic and steering gains stay
unchanged.

This is aimed at S-bend transitions where V4 showed large heading error after
recovering from the previous bend.
"""

import csv
import math
from datetime import datetime
import numpy as np

import pure_pursuit_profile_57_corner_v4_steer_gain as v4

profile_test = v4.profile_test
base = v4.base
v3 = v4.v3
dynamic_test = v4.dynamic_test

PREVIEW_DISTANCE_M = v4.PREVIEW_DISTANCE_M

# V5 lateral tuning unchanged. Values are team test parameters, not regulation values.
SIGN_CURV_WINDOW = 20
SIGN_CURV_EPS = 0.002       # 1/m: ignore nearly-straight numerical sign noise
SWITCH_ADVANCE_M = 3.0      # when switch is close, allow target past transition

_SIGNED_CURVATURE = None
_PATH = None

_DIAG = {
    "current_sign": 0,
    "next_sign": 0,
    "sign_switch_found": 0,
    "sign_switch_index": -1,
    "sign_switch_distance_m": -1.0,
    "target_bias_active": 0,
    "original_target_index": -1,
    "selected_target_index": -1,
    "selected_target_gap": -1,
    "target_local_x_m": 0.0,
    "target_local_y_m": 0.0,
    "theta_deg": 0.0,
    "raw_steering_rad": 0.0,
    "steering_gain": 1.0,
    "final_steering_rad": 0.0,
}


def compute_signed_curvature(path, window=SIGN_CURV_WINDOW):
    """Signed 3-point curvature using the same ~10 m window as base profile."""
    n = len(path)
    out = np.zeros(n, dtype=float)
    for i in range(window, n - window):
        p1 = path[i - window, :2]
        p2 = path[i, :2]
        p3 = path[i + window, :2]

        a = np.linalg.norm(p2 - p1)
        b = np.linalg.norm(p3 - p2)
        c = np.linalg.norm(p3 - p1)
        denom = a * b * c
        if denom < 1e-9:
            continue

        v1 = p2 - p1
        v2 = p3 - p1
        cross = v1[0] * v2[1] - v1[1] * v2[0]
        out[i] = 2.0 * cross / denom

    # Closed-path boundary: copy nearby stable values rather than inventing
    # wrap geometry; competition run begins away from this boundary issue.
    out[:window] = out[window]
    out[-window:] = out[-window - 1]
    return out


TIGHT_CURVE_PREVIEW_DECEL_MPS2 = 1.5


def build_v7_profile_capture(global_path, max_speed_kmh):
    global _SIGNED_CURVATURE, _PATH
    # Build the same curvature/radius seed used by V5, but use a lower
    # backward-pass deceleration assumption. In v^2 = v_next^2 + 2*a*ds,
    # lowering 'a' makes the target speed fall earlier before a slow corner.
    _, ds = profile_test.profile_builder.cumulative_distance(global_path)
    curvature = profile_test.profile_builder.compute_curvature(global_path)
    radius = profile_test.profile_builder.curvature_to_radius(curvature)
    seed = profile_test.profile_builder.build_seed_profile(radius)
    braking_profile = profile_test.profile_builder.apply_braking_preview(
        seed, ds, decel_mps2=TIGHT_CURVE_PREVIEW_DECEL_MPS2
    )
    capped_profile = np.minimum(braking_profile, float(max_speed_kmh))
    target = v3.v2.apply_forward_accel_limit(capped_profile, ds)

    # Preserve V3/V5 diagnostic state.
    v3._RADIUS_PROFILE = radius
    v3._PATH_DS = ds
    _PATH = global_path
    _SIGNED_CURVATURE = compute_signed_curvature(global_path)
    return target, radius, braking_profile


def curvature_sign(k):
    if abs(k) < SIGN_CURV_EPS:
        return 0
    return 1 if k > 0.0 else -1


def find_upcoming_sign_switch(current_index):
    """Return first meaningful opposite curvature sign within 15 m."""
    if _SIGNED_CURVATURE is None or v3._PATH_DS is None:
        return 0, 0, -1, -1.0

    n = len(_SIGNED_CURVATURE)
    current_sign = curvature_sign(float(_SIGNED_CURVATURE[current_index]))

    # If currently near straight, find the first meaningful bend as reference.
    ref_sign = current_sign
    idx = current_index
    travelled = 0.0
    while travelled < PREVIEW_DISTANCE_M and ref_sign == 0:
        step = float(v3._PATH_DS[idx if idx < n - 1 else -1])
        travelled += max(step, 1e-6)
        idx = (idx + 1) % n
        ref_sign = curvature_sign(float(_SIGNED_CURVATURE[idx]))

    if ref_sign == 0:
        return current_sign, 0, -1, -1.0

    idx = current_index
    travelled = 0.0
    while travelled < PREVIEW_DISTANCE_M:
        step = float(v3._PATH_DS[idx if idx < n - 1 else -1])
        travelled += max(step, 1e-6)
        idx = (idx + 1) % n
        s = curvature_sign(float(_SIGNED_CURVATURE[idx]))
        if s != 0 and s != ref_sign:
            return current_sign, s, idx, travelled

    return current_sign, ref_sign, -1, -1.0


def local_point(path, i, x, y, cos_yaw, sin_yaw):
    dx = path[i, 0] - x
    dy = path[i, 1] - y
    return (
        cos_yaw * dx + sin_yaw * dy,
        -sin_yaw * dx + cos_yaw * dy,
    )


def calculate_pure_pursuit_v5(path, current_index, x, y, yaw):
    # V4/V3 LFD logic unchanged.
    speed_kmh = dynamic_test._STATE["control_speed_kmh"]
    speed_lfd = dynamic_test.dynamic_lfd_m(speed_kmh)
    current_radius = (
        float(v3._RADIUS_PROFILE[current_index])
        if v3._RADIUS_PROFILE is not None else math.inf
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
    dynamic_test._STATE["dynamic_lfd_m"] = lfd

    current_sign, next_sign, switch_idx, switch_dist = (
        find_upcoming_sign_switch(current_index)
    )
    switch_found = switch_idx >= 0

    end_index = min(current_index + base.LOCAL_PATH_SIZE, len(path))
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)

    original_target = None
    original_local = None
    for i in range(current_index, end_index):
        lx, ly = local_point(path, i, x, y, cos_yaw, sin_yaw)
        if lx <= 0.0:
            continue
        if math.hypot(lx, ly) >= lfd:
            original_target = i
            original_local = (lx, ly)
            break

    if original_target is None:
        return None

    target = original_target
    target_local_x, target_local_y = original_local
    bias_active = 0

    # Single V5 behavior change:
    # when the curvature direction switch is closer than 3 m, do not keep a
    # target on the old bend; choose the first valid lookahead point at/after
    # the transition. This advances steering-sign reversal without changing LFD.
    if switch_found and switch_dist <= SWITCH_ADVANCE_M:
        search_start = max(original_target, switch_idx)
        for i in range(search_start, end_index):
            lx, ly = local_point(path, i, x, y, cos_yaw, sin_yaw)
            if lx <= 0.0:
                continue
            if math.hypot(lx, ly) >= lfd:
                target = i
                target_local_x, target_local_y = lx, ly
                bias_active = 1
                break

    theta = math.atan2(target_local_y, target_local_x)
    raw_steering = math.atan2(
        2.0 * base.VEHICLE_LENGTH * math.sin(theta), lfd
    )

    # V4 steering gain unchanged.
    gain = min(
        v4.STEER_GAIN_MAX,
        v4.steering_gain_from_preview_radius(preview_radius),
    )
    final_steering = max(-1.0, min(1.0, raw_steering * gain))

    v3._DIAG["pp_target_index"] = target
    _DIAG.update({
        "current_sign": current_sign,
        "next_sign": next_sign,
        "sign_switch_found": int(switch_found),
        "sign_switch_index": switch_idx,
        "sign_switch_distance_m": switch_dist,
        "target_bias_active": bias_active,
        "original_target_index": original_target,
        "selected_target_index": target,
        "selected_target_gap": target - current_index,
        "target_local_x_m": target_local_x,
        "target_local_y_m": target_local_y,
        "theta_deg": math.degrees(theta),
        "raw_steering_rad": raw_steering,
        "steering_gain": gain,
        "final_steering_rad": final_steering,
    })

    return {
        "target_index": target,
        "target_x": path[target, 0],
        "target_y": path[target, 1],
        "local_x": target_local_x,
        "local_y": target_local_y,
        "theta": theta,
        "steering": final_steering,
    }


class V7DiagnosticWriter:
    def __init__(self, writer):
        self.writer = writer

    @staticmethod
    def fmt_radius(value):
        return "inf" if not math.isfinite(value) else f"{value:.3f}"

    def writerow(self, row):
        self.writer.writerow(list(row) + [
            self.fmt_radius(v3._DIAG["current_radius_m"]),
            self.fmt_radius(v3._DIAG["preview_radius_m"]),
            v3._DIAG["preview_end_index"],
            f"{v3._DIAG['speed_lfd_m']:.3f}",
            f"{v3._DIAG['curve_lfd_cap_m']:.3f}",
            f"{v3._DIAG['effective_lfd_m']:.3f}",
            _DIAG["current_sign"],
            _DIAG["next_sign"],
            _DIAG["sign_switch_found"],
            _DIAG["sign_switch_index"],
            f"{_DIAG['sign_switch_distance_m']:.3f}",
            _DIAG["target_bias_active"],
            _DIAG["original_target_index"],
            _DIAG["selected_target_index"],
            _DIAG["selected_target_gap"],
            f"{_DIAG['target_local_x_m']:.3f}",
            f"{_DIAG['target_local_y_m']:.3f}",
            f"{_DIAG['theta_deg']:.3f}",
            f"{_DIAG['raw_steering_rad']:.6f}",
            f"{_DIAG['steering_gain']:.3f}",
            f"{_DIAG['final_steering_rad']:.6f}",
            f"{PREVIEW_DISTANCE_M:.1f}",
            f"{SWITCH_ADVANCE_M:.1f}",
        ])


def open_run_log_v7(max_speed_kmh, drive_mode):
    log_dir = base.PROJECT_ROOT / "logs"
    log_dir.mkdir(parents=True, exist_ok=True)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    mode = "drive" if drive_mode else "dryrun"
    log_path = log_dir / (
        f"run_{timestamp}_{max_speed_kmh:g}kph_corner_v7_early_brake_{mode}.csv"
    )
    log_file = open(log_path, "w", newline="", buffering=1)
    raw = csv.writer(log_file)
    raw.writerow([
        "elapsed_sec", "waypoint_index", "x_m", "y_m", "yaw_deg",
        "path_dist_m", "curve_radius_m", "full_profile_kmh",
        "target_speed_kmh", "comp_speed_kmh", "speed_error_kmh",
        "ctrl_zone", "steering_rad", "steering_deg", "long_cmd",
        "accel_cmd", "brake_cmd",
        "current_radius_m", "preview_min_radius_m", "preview_end_index",
        "speed_lfd_m", "curve_lfd_cap_m", "effective_lfd_m",
        "current_curve_sign", "next_curve_sign", "sign_switch_found",
        "sign_switch_index", "sign_switch_distance_m", "target_bias_active",
        "original_target_index", "selected_target_index", "selected_target_gap",
        "target_local_x_m", "target_local_y_m", "theta_deg",
        "raw_steering_rad", "steering_gain", "final_steering_rad",
        "preview_distance_m", "switch_advance_m",
    ])
    log_file.flush()
    return log_path, log_file, V7DiagnosticWriter(raw)


profile_test.build_test_profile = build_v7_profile_capture
profile_test.open_run_log = open_run_log_v7
base.calculate_pure_pursuit = calculate_pure_pursuit_v5


if __name__ == "__main__":
    print("[STAGE] 57 km/h CORNER V7 - EARLY BRAKING PREVIEW")
    print("[BASE] V5 lateral control unchanged")
    print(f"[BRAKING] preview decel assumption: 2.0 -> {TIGHT_CURVE_PREVIEW_DECEL_MPS2:.1f} m/s^2")
    print(f"[LATERAL] signed-curvature preview {PREVIEW_DISTANCE_M:.1f} m unchanged")
    print(f"[SWITCH] target transition advance within {SWITCH_ADVANCE_M:.1f} m")
    print("[LOG] curve sign/switch/original+selected PP target diagnostics enabled")
    profile_test.main()
