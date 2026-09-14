#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Competition Vehicle Status speed receiver test.

This reuses MORAI's official UDP Receiver and EgoVehicleStatus definition as
recommended by MORAI support. The simulator must be configured to publish
Competition Vehicle Status to 127.0.0.1:1909 (Host 1908 -> Destination 1909).

Only fields confirmed to be present/allowed for Competition Vehicle Status are
read here. Do not print the full EgoVehicleStatus object because the competition
packet omits trailing fields and the generic __repr__ can decode garbage bytes.
"""

import sys
import time
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
MORAI_MODULE = PROJECT_ROOT / "third_party" / "MORAI-NetworkModule"
sys.path.insert(0, str(MORAI_MODULE))

from lib.network.UDP import Receiver
from lib.define.EgoVehicleStatus import EgoVehicleStatus

IP = "127.0.0.1"
PORT = 1909


def main():
    receiver = Receiver(IP, PORT, EgoVehicleStatus())

    print("==============================================")
    print(" COMPETITION VEHICLE STATUS SPEED TEST")
    print("==============================================")
    print(f"[UDP] listening on {IP}:{PORT}")
    print("[SOURCE] Competition Vehicle Status")
    print("[FIELDS] signed_vel / vel_x only")
    print()

    try:
        while True:
            status = receiver.get_data()

            header = bytes(status.header).split(b"\x00", 1)[0]
            if header != b"#MoraiInfo$":
                print("[WAIT] Competition Vehicle Status 데이터 대기 중...")
                time.sleep(0.1)
                continue

            print(
                f"speed={float(status.signed_vel):7.3f} km/h | "
                f"vel_x={float(status.vel_x):7.3f} | "
                f"ctrl_mode={int(status.ctrl_mode)} | "
                f"gear={int(status.gear)} | "
                f"wheelbase={float(status.wheelbase):.3f} m"
            )
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\n종료")


if __name__ == "__main__":
    main()
