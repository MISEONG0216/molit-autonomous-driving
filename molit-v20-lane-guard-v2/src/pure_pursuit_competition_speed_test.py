#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Validation wrapper for the proven regulation-compliant Pure Pursuit controller.

It intentionally reuses src/pure_pursuit_regulation.py unchanged and replaces
only its temporary GPS-displacement speed estimator with the competition-
allowed Competition Vehicle Status signed_vel feedback.

MORAI Network Settings:
    Competition Vehicle Status (Sim -> User)
    Host Port        : 1908
    Destination Port : 1909

NOTE:
The reused baseline still prints labels such as "GPS SPEED" and "RAW GPS SPD".
In this validation wrapper, those displayed values are Competition Vehicle
Status signed_vel values, not GPS-derived speed.
"""

import sys
from pathlib import Path

SRC_DIR = Path(__file__).resolve().parent
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

import pure_pursuit_regulation as base

from lib.network.UDP import Receiver
from lib.define.EgoVehicleStatus import EgoVehicleStatus


COMP_STATUS_IP = "127.0.0.1"
COMP_STATUS_PORT = 1909
COMP_STATUS_DATA_LENGTH = 152
COMP_STATUS_HEADER = b"#MoraiInfo$"


class CompetitionSpeedEstimator:
    """Drop-in replacement for the baseline GPS speed estimator."""

    def __init__(self):
        self.receiver = Receiver(
            COMP_STATUS_IP,
            COMP_STATUS_PORT,
            EgoVehicleStatus(),
        )
        self.last_speed_kmh = 0.0

    def update(self, _x, _y, _now):
        data = self.receiver.get_data()

        try:
            header = bytes(data.header).split(b"\x00", 1)[0]
            data_length = int(data.data_lenght)
        except Exception:
            return (
                self.last_speed_kmh,
                self.last_speed_kmh,
                False,
                "WAIT_COMP",
            )

        if header != COMP_STATUS_HEADER or data_length != COMP_STATUS_DATA_LENGTH:
            return (
                self.last_speed_kmh,
                self.last_speed_kmh,
                False,
                "WAIT_COMP",
            )

        speed_kmh = float(data.signed_vel)

        # The competition vehicle is commanded in Drive (gear=4). Tiny negative
        # values can appear numerically while stationary, so clamp only those
        # near-zero reverse values to zero rather than taking abs().
        if -0.05 < speed_kmh < 0.0:
            speed_kmh = 0.0

        self.last_speed_kmh = speed_kmh

        return (
            speed_kmh,
            speed_kmh,
            True,
            "COMP_STATUS",
        )


if __name__ == "__main__":
    print("[SPEED SOURCE] Competition Vehicle Status signed_vel @ UDP 1909")
    print("[NOTE] Baseline screen labels 'GPS SPEED'/'RAW GPS SPD' now show Competition speed.")
    print()

    # Reuse the proven controller unchanged; replace only the speed source.
    base.GpsSpeedEstimator = CompetitionSpeedEstimator
    base.main()
