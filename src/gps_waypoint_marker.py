#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Manual MORAI waypoint survey tool (READ-ONLY).

Uses the same GPS UDP, UTM offsets, and global-path nearest-index routine
as main_integrated_v1.py. Does not subscribe to prohibited ego position,
and NEVER sends EgoCtrlCmd.

Drive the MORAI car manually; type a label + Enter to save a waypoint mark.
Examples: highway_entry, highway_merge, highway_exit, roundabout_entry,
roundabout_yield, roundabout_exit. Type 'q' + Enter to quit.

The global-path waypoint index is ZERO-BASED, matching NEAREST in V14.
"""
import argparse
import csv
import json
import math
import select
import sys
import time
from datetime import datetime
from pathlib import Path

# Same implementation/coordinate transforms as existing competition controller.
import pure_pursuit_regulation as base
from lib.network.UDP import Receiver
from lib.define.GPS import GPS

ROOT = Path(__file__).resolve().parents[1]
NETWORK = ROOT / "config" / "integration_network.json"


def load_config():
    with NETWORK.open(encoding="utf-8") as stream:
        return json.load(stream)


def user_command_available():
    """Unix terminal input; nonblocking until Enter is pressed."""
    ready, _, _ = select.select([sys.stdin], [], [], 0)
    return bool(ready)


def main():
    parser = argparse.ArgumentParser(
        description="Manually mark nearest global-path WP using GPS only."
    )
    parser.add_argument(
        "--interval", type=float, default=0.5,
        help="GPS display and trace interval in seconds (default 0.5)."
    )
    parser.add_argument(
        "--max-mark-distance", type=float, default=10.0,
        help="Warn if GPS is farther than this from the global path (m)."
    )
    args = parser.parse_args()
    if args.interval <= 0:
        parser.error("--interval must be positive")

    cfg = load_config()
    path = base.load_path(base.PATH_FILE)
    gps = Receiver(cfg["gps"]["bind_ip"], int(cfg["gps"]["port"]), GPS())

    outdir = ROOT / "logs"
    outdir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    marks_path = outdir / f"waypoint_marks_{stamp}.csv"
    trace_path = outdir / f"waypoint_trace_{stamp}.csv"
    columns = [
        "timestamp", "label", "waypoint_index",
        "gps_lat", "gps_lon", "x_m", "y_m",
        "path_x_m", "path_y_m", "path_distance_m",
    ]

    print("=" * 68)
    print("MORAI GPS -> global path WAYPOINT MARKER (manual driving)")
    print(f"GPS receive port : {cfg['gps']['port']}")
    print(f"Global path      : {base.PATH_FILE.name} / {len(path)} waypoints")
    print("Index convention : ZERO-BASED (same as main integrated NEAREST)")
    print("Vehicle control  : NONE - you control the car directly in MORAI")
    print("At an important location, type a label and press Enter:")
    print("  highway_entry / merge_point / highway_exit")
    print("  roundabout_entry / yield_line / roundabout_exit")
    print("  q + Enter = quit (or Ctrl+C)")
    print(f"Marks saved to   : {marks_path}")
    print(f"Trace saved to   : {trace_path}")
    print("=" * 68)

    latest = None
    trace_count = 0
    marks_count = 0
    try:
        with marks_path.open("w", newline="", encoding="utf-8") as marks_file, \
             trace_path.open("w", newline="", encoding="utf-8") as trace_file:
            mw = csv.DictWriter(marks_file, fieldnames=columns)
            tw = csv.DictWriter(trace_file, fieldnames=columns)
            mw.writeheader()
            tw.writeheader()

            while True:
                loop_start = time.monotonic()
                gps_data = gps.get_data()
                gps_data.parsing()
                lat = float(gps_data.gprmc.lat)
                lon = float(gps_data.gprmc.lon)
                pos = base.gps_to_local(lat, lon)

                if pos is None or not all(math.isfinite(x) for x in pos):
                    print("[WAIT GPS] GPS 0,0 or invalid. Check simulator GPS port.")
                    latest = None
                else:
                    x, y = pos
                    idx = base.find_nearest_waypoint(path, x, y)
                    px, py = float(path[idx, 0]), float(path[idx, 1])
                    dist = math.hypot(x - px, y - py)
                    latest = {
                        "timestamp": datetime.now().isoformat(timespec="milliseconds"),
                        "label": "",
                        "waypoint_index": idx,
                        "gps_lat": round(lat, 9),
                        "gps_lon": round(lon, 9),
                        "x_m": round(x, 3),
                        "y_m": round(y, 3),
                        "path_x_m": round(px, 3),
                        "path_y_m": round(py, 3),
                        "path_distance_m": round(dist, 3),
                    }
                    tw.writerow(latest)
                    trace_file.flush()
                    trace_count += 1
                    off_path = "  *** FAR FROM PATH ***" if dist > args.max_mark_distance else ""
                    print(
                        f"[WP {idx:4d}/{len(path)-1}] "
                        f"GPS {lat:.6f}, {lon:.6f} | "
                        f"XY ({x:.2f}, {y:.2f}) | "
                        f"nearest path ({px:.2f}, {py:.2f}) | "
                        f"error {dist:.2f}m{off_path}"
                    )

                # Accept marker labels at any time; no keyboard package needed.
                # Stdin is line-buffered: input is delivered after Enter.
                if user_command_available():
                    cmd = sys.stdin.readline().strip()
                    if cmd.lower() in ("q", "quit", "exit"):
                        break
                    if not cmd:
                        continue
                    if latest is None:
                        print("[SKIP] GPS not valid yet; marker not saved.")
                    else:
                        mark = dict(latest, label=cmd)
                        mw.writerow(mark)
                        marks_file.flush()
                        marks_count += 1
                        print(
                            f"*** SAVED [{cmd}]: WP {mark['waypoint_index']} "
                            f"(path error {mark['path_distance_m']}m) ***"
                        )

                elapsed = time.monotonic() - loop_start
                if elapsed < args.interval:
                    time.sleep(args.interval - elapsed)

    except KeyboardInterrupt:
        print("\n[EXIT] Ctrl+C")
    finally:
        print(f"[RESULT] {marks_count} marks in {marks_path}")
        print(f"[RESULT] {trace_count} GPS trace samples in {trace_path}")
        print("[SAFETY] No EgoCtrlCmd was sent.")


if __name__ == "__main__":
    main()
