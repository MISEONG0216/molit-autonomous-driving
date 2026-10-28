#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Make a time-aligned PNG chart from a lane-tuning CSV.

Examples:
  python scripts/plot_lane_csv.py --csv logs/integrated_v1_RUN.csv
  python scripts/plot_lane_csv.py --csv logs/integrated_v1_RUN.csv --start 42 --end 49
"""
import argparse
import csv
import math
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def value(row, key):
    try:
        return float(row.get(key, "nan"))
    except (TypeError, ValueError):
        return math.nan


def main():
    parser = argparse.ArgumentParser(description="Plot MORAI global path tracking diagnostics")
    parser.add_argument("--csv", required=True, type=Path)
    parser.add_argument("--start", type=float, default=None)
    parser.add_argument("--end", type=float, default=None)
    parser.add_argument("--output", type=Path, default=None)
    args = parser.parse_args()
    with args.csv.open("r", newline="", encoding="utf-8-sig") as stream:
        rows = list(csv.DictReader(stream))
    if not rows or "signed_cte_m" not in rows[0]:
        raise SystemExit("CSV missing lane diagnostics columns")

    filtered = []
    for row in rows:
        t = value(row, "elapsed_sec")
        if not math.isfinite(t):
            continue
        if args.start is not None and t < args.start:
            continue
        if args.end is not None and t > args.end:
            continue
        filtered.append(row)
    if not filtered:
        raise SystemExit("No samples in selected interval")

    times = [value(r, "elapsed_sec") for r in filtered]
    cte = [value(r, "signed_cte_m") for r in filtered]
    heading = [value(r, "heading_error_deg") for r in filtered]
    actual_speed = [value(r, "comp_speed_kmh") for r in filtered]
    target_speed = [value(r, "final_target_kmh") for r in filtered]
    steering = [value(r, "steering_rad") for r in filtered]
    gain = [value(r, "v7_steering_gain") for r in filtered]
    wp = [int(value(r, "waypoint_index")) for r in filtered]

    fig, axes = plt.subplots(4, 1, figsize=(14, 9), sharex=True)
    axes[0].plot(times, cte, label="signed CTE (+left / -right)")
    axes[0].axhline(0, linewidth=0.7)
    axes[0].set_ylabel("CTE (m)")
    axes[0].legend(loc="upper right")

    axes[1].plot(times, actual_speed, label="actual speed")
    axes[1].plot(times, target_speed, label="target speed", linestyle="--")
    axes[1].set_ylabel("Speed (km/h)")
    axes[1].legend(loc="upper right")

    axes[2].plot(times, steering, label="steering command (rad)")
    axes[2].plot(times, gain, label="V7 gain", linestyle=":")
    axes[2].set_ylabel("Steer / Gain")
    axes[2].legend(loc="upper right")

    axes[3].plot(times, heading, label="heading error")
    axes[3].set_ylabel("Heading (deg)")
    axes[3].set_xlabel("CSV elapsed_sec (seconds) / match VIDEO SYNC in recorded CMD")
    axes[3].legend(loc="upper right")

    # Sparse waypoint labels help map a timestamp to the original track.
    for fraction in (0.0, 0.25, 0.5, 0.75, 1.0):
        j = min(len(wp) - 1, round((len(wp) - 1) * fraction))
        axes[0].annotate("WP{}".format(wp[j]), (times[j], cte[j]),
                         xytext=(0, 9), textcoords="offset points", fontsize=8)

    for axis in axes:
        axis.grid(True, alpha=0.2)
    fig.suptitle("V20 lane review: global path deviation (NOT camera lane contact)")
    fig.tight_layout()
    output = args.output
    if output is None:
        suffix = "_lane_plot"
        if args.start is not None or args.end is not None:
            suffix += "_section"
        output = args.csv.with_name(args.csv.stem + suffix + ".png")
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(str(output), dpi=140)
    plt.close(fig)
    print("[PLOT SAVED]", output)


if __name__ == "__main__":
    main()
