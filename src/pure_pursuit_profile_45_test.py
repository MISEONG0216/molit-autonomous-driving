#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""45 km/h staged profile test.

Reuses the validated waypoint-profile controller unchanged for vehicle control,
while adding diagnostic logging of both Competition Vehicle Status speed fields:
- signed_vel: remains the CONTROL speed source for this test.
- vel_x: diagnostic only; it does not affect control.

This lets us inspect the repeated signed_vel anomaly around WP930 without
changing two variables at once.
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

# Shared diagnostic value used only by the CSV writer below.
_DIAG = {"vel_x_kmh": 0.0}


class ComparisonSpeedReceiver(BaseCompetitionSpeedReceiver):
    """Keep signed_vel for control and expose vel_x for diagnostic logging."""

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

        signed_vel_kmh = float(data.signed_vel)
        vel_x_kmh = float(data.vel_x)

        # Preserve the validated stopped-value handling.
        if -0.05 < signed_vel_kmh < 0.0:
            signed_vel_kmh = 0.0
        if -0.05 < vel_x_kmh < 0.0:
            vel_x_kmh = 0.0

        self.last_speed_kmh = signed_vel_kmh
        self.valid = True
        _DIAG["vel_x_kmh"] = vel_x_kmh
        return signed_vel_kmh, True


class DiagnosticWriter:
    """Append vel_x to each telemetry row produced by the validated controller."""

    def __init__(self, writer):
        self.writer = writer

    def writerow(self, row):
        self.writer.writerow(list(row) + [f"{_DIAG['vel_x_kmh']:.3f}"])


def open_run_log_45(max_speed_kmh, drive_mode):
    log_dir = profile_test.base.PROJECT_ROOT / "logs"
    log_dir.mkdir(parents=True, exist_ok=True)

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    mode = "drive" if drive_mode else "dryrun"
    speed_tag = f"{max_speed_kmh:g}kph"
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
        "vel_x_kmh",
    ])
    log_file.flush()
    return log_path, log_file, DiagnosticWriter(raw_writer)


# Test-only injection. The shared validated controller remains unchanged.
profile_test.CompetitionSpeedReceiver = ComparisonSpeedReceiver
profile_test.open_run_log = open_run_log_45


if __name__ == "__main__":
    profile_test.main()
