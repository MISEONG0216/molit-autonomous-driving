#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import time
import msvcrt
from pathlib import Path

import numpy as np
from pyproj import Proj

PROJECT_ROOT = Path(__file__).resolve().parents[1]
MORAI_MODULE = PROJECT_ROOT / "third_party" / "MORAI-NetworkModule"
sys.path.insert(0, str(MORAI_MODULE))

from lib.network.UDP import Receiver
from lib.define.GPS import GPS

GPS_IP = "127.0.0.1"
GPS_PORT = 9201

PATH_FILE = PROJECT_ROOT / "path" / "2026_molit_comp_global_path.txt"
SAVE_FILE = PROJECT_ROOT / "path" / "traffic_stop_indices.txt"

PROJ_UTM = Proj(
    proj="utm",
    zone=52,
    ellps="WGS84",
    preserve_units=False
)

EAST_OFFSET = 302595.0
NORTH_OFFSET = 4124145.0


def load_path(path_file):
    points = []

    with open(path_file, "r") as f:
        for line in f:
            values = line.split()

            if len(values) >= 2:
                points.append(
                    (float(values[0]), float(values[1]))
                )

    return np.array(points, dtype=float)


def gps_to_local(lat, lon):
    if lat == 0.0 and lon == 0.0:
        return None

    utm_x, utm_y = PROJ_UTM(lon, lat)

    return (
        utm_x - EAST_OFFSET,
        utm_y - NORTH_OFFSET
    )


def find_nearest_waypoint(path, x, y):
    dx = path[:, 0] - x
    dy = path[:, 1] - y

    return int(np.argmin(dx * dx + dy * dy))


def save_indices(indices):
    with open(SAVE_FILE, "w", encoding="utf-8") as f:
        for idx in indices:
            f.write(f"{idx}\n")


def main():
    print("========================================")
    print(" MORAI STOP INDEX RECORDER")
    print("========================================")
    print("S = save current stop position")
    print("U = undo last saved position")
    print("Q = save and quit")
    print()

    global_path = load_path(PATH_FILE)

    gps_receiver = Receiver(
        GPS_IP,
        GPS_PORT,
        GPS()
    )

    stop_indices = []

    while True:
        gps_data = gps_receiver.get_data()
        gps_data.parsing()

        lat = gps_data.gprmc.lat
        lon = gps_data.gprmc.lon

        position = gps_to_local(lat, lon)

        if position is None:
            print("\rWaiting for GPS...", end="")
            time.sleep(0.1)
            continue

        x, y = position
        nearest_index = find_nearest_waypoint(
            global_path, x, y
        )

        print(
            f"\rCURRENT INDEX: {nearest_index:5d} | SAVED: {stop_indices}      ",
            end=""
        )

        if msvcrt.kbhit():
            key = msvcrt.getch().decode(
                errors="ignore"
            ).lower()

            if key == "s":
                if (
                    not stop_indices
                    or abs(nearest_index - stop_indices[-1]) > 2
                ):
                    stop_indices.append(nearest_index)
                    save_indices(stop_indices)

                    print(
                        f"\n[RECORDED] STOP #{len(stop_indices)} "
                        f"= index {nearest_index}"
                    )
                else:
                    print("\n[SKIP] Too close to previous index.")

            elif key == "u":
                if stop_indices:
                    removed = stop_indices.pop()
                    save_indices(stop_indices)
                    print(f"\n[UNDO] Removed index {removed}")

            elif key == "q":
                save_indices(stop_indices)

                print()
                print("[DONE]")
                print("STOP INDICES =", stop_indices)
                print("SAVED TO =", SAVE_FILE)
                break

        time.sleep(0.05)


if __name__ == "__main__":
    main()
