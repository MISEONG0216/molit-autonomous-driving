#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Inspect V20 run CSV against video-selected times (no driving, no camera access).

Examples:
  python scripts/review_lane_csv.py --csv logs/integrated_v1_RUN.csv
  python scripts/review_lane_csv.py --csv logs/integrated_v1_RUN.csv --start 42 --end 46

The thresholds are REVIEW FILTERS for global path deviation, not lane boundaries.
"""
import argparse
import csv
import math
from pathlib import Path


def numeric(row, key):
    try:
        value = float(row.get(key, "nan"))
        return value if math.isfinite(value) else math.nan
    except (ValueError, TypeError):
        return math.nan


def read_log(csv_path):
    with open(csv_path, "r", newline="", encoding="utf-8-sig") as stream:
        reader = csv.DictReader(stream)
        required = {"elapsed_sec", "waypoint_index", "signed_cte_m",
                    "comp_speed_kmh", "heading_error_deg", "tracking_path_source"}
        missing = required.difference(reader.fieldnames or [])
        if missing:
            raise ValueError("CSV is not the lane-tuning format. Missing: " + ", ".join(sorted(missing)))
        rows = []
        for row in reader:
            elapsed = numeric(row, "elapsed_sec")
            cte = numeric(row, "signed_cte_m")
            if not math.isfinite(elapsed) or not math.isfinite(cte):
                continue
            row["_elapsed"] = elapsed
            row["_cte"] = cte
            rows.append(row)
    return sorted(rows, key=lambda row: row["_elapsed"])


def describe(rows):
    if not rows:
        return "No matching valid samples"
    worst = max(rows, key=lambda r: abs(r["_cte"]))
    speeds = [numeric(r, "comp_speed_kmh") for r in rows]
    speeds = [v for v in speeds if math.isfinite(v)]
    headings = [numeric(r, "heading_error_deg") for r in rows]
    headings = [v for v in headings if math.isfinite(v)]
    wp = [int(float(r["waypoint_index"])) for r in rows]
    modes = sorted(set(r["tracking_path_source"] for r in rows))
    return (
        f"t={rows[0]['_elapsed']:.2f}~{rows[-1]['_elapsed']:.2f}s "
        f"({rows[-1]['_elapsed'] - rows[0]['_elapsed']:.2f}s), "
        f"WP={min(wp)}~{max(wp)}, "
        f"peak_CTE={worst['_cte']:+.3f}m @WP{worst['waypoint_index']}, "
        f"max_speed={max(speeds) if speeds else math.nan:.1f}km/h, "
        f"max_abs_heading_err={max((abs(v) for v in headings), default=math.nan):.1f}deg, "
        f"mode={','.join(modes)}"
    )


def high_cte_runs(rows, threshold, max_gap):
    runs = []
    current = []
    for row in rows:
        flagged = abs(row["_cte"]) >= threshold
        continuous = current and (row["_elapsed"] - current[-1]["_elapsed"] <= max_gap)
        if not flagged or (current and not continuous):
            if current:
                runs.append(current)
                current = []
        if flagged:
            current.append(row)
    if current:
        runs.append(current)
    return runs


def main():
    parser = argparse.ArgumentParser(description="Review lane video times against CSV")
    parser.add_argument("--csv", required=True, type=Path)
    parser.add_argument("--start", type=float, help="Start ELAPSED seconds shown in video")
    parser.add_argument("--end", type=float, help="End ELAPSED seconds shown in video")
    parser.add_argument("--threshold", type=float, default=0.7,
                        help="CTE magnitude in metres for candidate review, NOT lane violation")
    parser.add_argument("--max-gap", type=float, default=0.35,
                        help="Maximum gap in seconds within a candidate segment")
    parser.add_argument("--top", type=int, default=12)
    args = parser.parse_args()
    if (args.start is None) != (args.end is None):
        parser.error("Use both --start and --end for a video interval")
    if args.start is not None and args.end < args.start:
        parser.error("--end must be >= --start")
    if args.threshold <= 0 or args.max_gap <= 0:
        parser.error("--threshold and --max-gap must be > 0")
    rows = read_log(args.csv)
    print(f"[CSV] {args.csv} / valid samples: {len(rows)}")
    print("[CAUTION] Global path deviation is NOT physical wheel-to-lane-line distance.")
    if args.start is not None:
        selected = [r for r in rows if args.start <= r["_elapsed"] <= args.end]
        print("[VIDEO INTERVAL] " + describe(selected))
        if selected:
            print("[FIRST] local time:", selected[0].get("wall_time_local", "unknown"))
            print("[LAST ] local time:", selected[-1].get("wall_time_local", "unknown"))
    else:
        runs = high_cte_runs(rows, args.threshold, args.max_gap)
        # Long and high-deviation intervals deserve early video review.
        runs.sort(key=lambda segment: (
            segment[-1]["_elapsed"] - segment[0]["_elapsed"],
            max(abs(r["_cte"]) for r in segment)), reverse=True)
        print(f"[REVIEW] |signed_CTE| >= {args.threshold:.2f}m (analysis filter only)")
        if not runs:
            print("No candidate intervals at this threshold.")
        for n, segment in enumerate(runs[:max(0, args.top)], 1):
            print(f"{n:02d}. {describe(segment)}")
    print("[NEXT] Identify lane-line contact in the video before changing steering/speed.")


if __name__ == "__main__":
    main()
