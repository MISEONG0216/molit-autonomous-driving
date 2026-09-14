#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""45 km/h negative-speed-clamp validation test.

Purpose:
- Keep the validated staged waypoint-profile controller unchanged.
- Keep signed_vel as the Competition Vehicle Status control-speed source.
- Clamp negative signed_vel to 0 km/h ONLY for longitudinal control.
- Preserve raw signed_vel and vel_x in the CSV for diagnosis.

This isolates the WP930 negative-speed anomaly without changing lateral control,
profile generation, braking preview, or the normal positive-speed behavior.
"""

import csv
from datetime import datetime

import pure_pursuit_profile_test as profile_test
from pure_pursuit_competition import (
    CompetitionSpeedReceiver as BaseCompetitionSpeedReceiver,
    COMP_STATUS_HEADER,
    COMP_STATUS_DATA_LENGTH,
)


profile_test.ALLOWED_TEST_MAX_KMH = 45.0

_DIAG = {
    "raw_signed_vel_kmh": 0.0,
    "vel_x_kmh": 0.0,
    "negative_clamp_active": 0,
}


class ClampedCompetitionSpeedReceiver(BaseCompetitionSpeedReceiver):
    """Clamp negative signed_vel only in the value returned to the controller."""

    def read(self):
        data = self.receiver.get_data()

        try:
            header = bytes(data.header).split(b"\x00", 1)[0]
            data_length = int(data.data_lenght)
        except Exception:
            return self.last_speed_kmh, False

        if header != COMP_STATUS_HEADER:
            return self.last_speed_kmh, False
        if data_length != COMP_STATUS_DATA_LENGTH:
            return self.last_speed_kmh, False

        raw_signed_vel_kmh = float(data.signed_vel)
        vel_x_kmh = float(data.vel_x)

        # Forward-driving safety guard for the repeated WP930 anomaly.
        # Raw values remain available in the CSV for post-run analysis.
        control_speed_kmh = max(0.0, raw_signed_vel_kmh)

        self.last_speed_kmh = control_speed_kmh
        self.valid = True

        _DIAG["raw_signed_vel_kmh"] = raw_signed_vel_kmh
        _DIAG["vel_x_kmh"] = vel_x_kmh
        _DIAG["negative_clamp_active"] = int(raw_signed_vel_kmh < 0.0)

        return control_speed_kmh, True


class DiagnosticWriter:
    """Append raw Competition speed fields and clamp state to telemetry rows."""

    def __init__(self, writer):
        self.writer = writer

    def writerow(self, row):
        self.writer.writerow(
            list(row)
            + [
                f"{_DIAG['raw_signed_vel_kmh']:.3f}",
                f"{_DIAG['vel_x_kmh']:.3f}",
                _DIAG["negative_clamp_active"],
            ]
        )


def open_run_log_45_clamp(max_speed_kmh, drive_mode):
    log_dir = profile_test.base.PROJECT_ROOT / "logs"
    log_dir.mkdir(parents=True, exist_ok=True)

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    mode = "drive" if drive_mode else "dryrun"
    speed_tag = f"{max_speed_kmh:g}kph_clamp"
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
    ])
    log_file.flush()
    return log_path, log_file, DiagnosticWriter(raw_writer)


# Test-only injection. Shared validated files remain unchanged.
profile_test.CompetitionSpeedReceiver = ClampedCompetitionSpeedReceiver
profile_test.open_run_log = open_run_log_45_clamp


if __name__ == "__main__":
    profile_test.main()
