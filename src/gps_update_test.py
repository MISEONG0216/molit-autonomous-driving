#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""MORAI GPS update diagnostic.

Purpose:
- Verify that GPS latitude/longitude and competition local x/y keep updating
  while the ego vehicle moves.
- Detect a receiver that is alive but returning the same stale GPS sample.
- IMU yaw is printed only as a comparison signal.

This file sends NO vehicle-control command.
"""

import math
import sys
import time
from pathlib import Path

from pyproj import Proj

SRC_DIR = Path(__file__).resolve().parent
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from morai_common import ensure_morai_import_path, load_network_config
ensure_morai_import_path()

from lib.network.UDP import Receiver
from lib.define.GPS import GPS
from lib.define.IMU import IMU

PROJ_UTM = Proj(proj="utm", zone=52, ellps="WGS84", preserve_units=False)
EAST_OFFSET = 302595.0
NORTH_OFFSET = 4124145.0


def gps_to_local(lat, lon):
    if lat == 0.0 and lon == 0.0:
        return None
    utm_x, utm_y = PROJ_UTM(lon, lat)
    return utm_x - EAST_OFFSET, utm_y - NORTH_OFFSET


def quaternion_to_yaw(x, y, z, w):
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def read_gps(receiver):
    data = receiver.get_data()
    try:
        data.parsing()
        lat = float(data.gprmc.lat)
        lon = float(data.gprmc.lon)
    except Exception:
        return None

    local = gps_to_local(lat, lon)
    if local is None:
        return None
    return lat, lon, local[0], local[1]


def read_yaw(receiver):
    data = receiver.get_data()
    try:
        w = float(data.ori_w)
        x = float(data.ori_x)
        y = float(data.ori_y)
        z = float(data.ori_z)
    except Exception:
        return None

    if w == 0.0 and x == 0.0 and y == 0.0 and z == 0.0:
        return None
    return quaternion_to_yaw(x, y, z, w)


def main():
    cfg = load_network_config()
    gps_cfg = cfg["gps"]
    imu_cfg = cfg["imu"]

    print("=" * 78)
    print(" MORAI GPS UPDATE TEST")
    print("=" * 78)
    print(f"[GPS] {gps_cfg['ip']}:{gps_cfg['port']}")
    print(f"[IMU] {imu_cfg['ip']}:{imu_cfg['port']}")
    print("[SAFE] No EgoCtrlCmd is transmitted.")
    print("[TEST] Keep still for 3-5 s, then move the vehicle slowly.")
    print()

    gps_receiver = Receiver(gps_cfg["ip"], int(gps_cfg["port"]), GPS())
    imu_receiver = Receiver(imu_cfg["ip"], int(imu_cfg["port"]), IMU())

    previous_xy = None
    previous_change_time = time.monotonic()
    sample = 0

    try:
        while True:
            gps = read_gps(gps_receiver)
            yaw = read_yaw(imu_receiver)

            if gps is None:
                print("\r[WAIT] GPS data not ready...", end="", flush=True)
                time.sleep(0.1)
                continue

            sample += 1
            lat, lon, x, y = gps

            if previous_xy is None:
                dx = dy = step = 0.0
                changed = True
            else:
                dx = x - previous_xy[0]
                dy = y - previous_xy[1]
                step = math.hypot(dx, dy)
                changed = step > 0.0001

            if changed:
                previous_change_time = time.monotonic()

            stale_s = time.monotonic() - previous_change_time
            yaw_text = "N/A" if yaw is None else f"{math.degrees(yaw):.2f}"

            state = "UPDATING" if changed else f"SAME {stale_s:.1f}s"
            warning = "  <-- CHECK GPS" if stale_s >= 1.0 else ""

            print(
                f"[{sample:05d}] "
                f"lat={lat:.9f} lon={lon:.9f} | "
                f"x={x:.3f} y={y:.3f} | "
                f"dx={dx:+.4f} dy={dy:+.4f} step={step:.4f}m | "
                f"yaw={yaw_text}deg | GPS={state}{warning}"
            )

            previous_xy = (x, y)
            time.sleep(0.1)

    except KeyboardInterrupt:
        print()
        print("[STOP] GPS update test finished.")
        print("[SAFE] No vehicle-control command was transmitted.")


if __name__ == "__main__":
    main()
