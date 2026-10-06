#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""GPS update test using ThreadReceiver instead of MORAI multiprocessing Receiver."""

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

from lib.define.GPS import GPS
from morai_thread_receiver import ThreadReceiver

PROJ_UTM = Proj(proj="utm", zone=52, ellps="WGS84", preserve_units=False)
EAST_OFFSET = 302595.0
NORTH_OFFSET = 4124145.0


def gps_to_local(lat, lon):
    if lat == 0.0 and lon == 0.0:
        return None
    utm_x, utm_y = PROJ_UTM(lon, lat)
    return utm_x - EAST_OFFSET, utm_y - NORTH_OFFSET


def main():
    cfg = load_network_config()
    gps_cfg = cfg["gps"]

    print("=" * 78)
    print(" MORAI GPS THREAD RECEIVER TEST")
    print("=" * 78)
    print(f"[GPS] {gps_cfg['ip']}:{gps_cfg['port']}")
    print("[SAFE] Original UDP.py is NOT modified.")
    print("[SAFE] No IMU / LiDAR / EgoCtrlCmd is used.")
    print("[TEST] Keep still for 3-5 s, then move the vehicle slowly.")
    print()

    receiver = ThreadReceiver(gps_cfg["ip"], int(gps_cfg["port"]), GPS())

    previous_xy = None
    previous_packet_count = 0
    sample = 0

    try:
        while True:
            data = receiver.get_data()
            stats = receiver.get_stats()

            if data is None:
                print("\r[WAIT] GPS packet not ready...", end="", flush=True)
                time.sleep(0.1)
                continue

            try:
                # ThreadReceiver already calls parsing(); calling it again is
                # harmless for the current GPS class and keeps this test explicit.
                data.parsing()
                lat = float(data.gprmc.lat)
                lon = float(data.gprmc.lon)
            except Exception as exc:
                print(f"\n[PARSE ERROR] {exc!r}")
                time.sleep(0.1)
                continue

            local = gps_to_local(lat, lon)
            if local is None:
                time.sleep(0.1)
                continue

            x, y = local
            sample += 1

            if previous_xy is None:
                dx = dy = step = 0.0
            else:
                dx = x - previous_xy[0]
                dy = y - previous_xy[1]
                step = math.hypot(dx, dy)

            packet_delta = stats["packet_count"] - previous_packet_count

            print(
                f"[{sample:05d}] packets={stats['packet_count']:6d} "
                f"(+{packet_delta:2d}) | "
                f"lat={lat:.9f} lon={lon:.9f} | "
                f"x={x:.3f} y={y:.3f} | "
                f"dx={dx:+.4f} dy={dy:+.4f} step={step:.4f}m | "
                f"error={stats['last_error']}"
            )

            previous_xy = (x, y)
            previous_packet_count = stats["packet_count"]
            time.sleep(0.1)

    except KeyboardInterrupt:
        print()
        print("[STOP] ThreadReceiver GPS test finished.")
    finally:
        receiver.close()
        print("[SAFE] No vehicle-control command was transmitted.")


if __name__ == "__main__":
    main()
