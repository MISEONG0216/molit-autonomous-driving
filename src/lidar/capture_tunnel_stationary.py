#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Collect LiDAR at a stationary MORAI vehicle; NEVER transmit driving commands.

Coordinates saved: +forward_m, +left_m, +up_m, intensity.
The values remain relative to a stationary LiDAR origin, NOT global/GPS XY.
"""
import argparse
import csv
import json
import sys
import time
from datetime import datetime
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from lidar_receiver import LidarReceiver


def vehicle_frame(raw):
    """MORAI receiver +X=right, +Y=forward -> +X=forward, +Y=left."""
    a = np.asarray(raw, dtype=float)
    return np.column_stack((a[:, 1], -a[:, 0], a[:, 2], a[:, 3]))


def summarize_cloud(points):
    """Observable returns only; count is NOT proof of empty or drivable space."""
    usable = points[(points[:, 2] >= -1.45) & (points[:, 2] <= 1.25) &
                    (points[:, 0] >= 1.0) & (points[:, 0] <= 24.0)]
    central = usable[np.abs(usable[:, 1]) <= 1.35]
    left = usable[(usable[:, 1] >= 1.35) & (usable[:, 1] <= 8.0)]
    right = usable[(usable[:, 1] <= -1.35) & (usable[:, 1] >= -8.0)]
    nearest = float(np.min(central[:, 0])) if len(central) else None
    return dict(usable_points=len(usable), center_points=len(central),
                left_points=len(left), right_points=len(right),
                nearest_center_forward_m=nearest)


def collect(ip, port, scans, timeout_sec, output_root):
    network = json.loads((ROOT / "config/integration_network.json").read_text(
        encoding="utf-8"))
    cfg = network["lidar"]
    ip = ip or cfg["bind_ip"]
    port = int(port or cfg["port"])
    out = output_root / ("survey_" + datetime.now().strftime("%Y%m%d_%H%M%S"))
    out.mkdir(parents=True, exist_ok=False)
    receiver = LidarReceiver(ip=ip, port=port)
    records = []
    seen = None
    print("[SURVEY] This tool only receives LiDAR; it does NOT stop the vehicle.")
    print("[SURVEY] Keep MORAI vehicle fully stationary. Close all other LiDAR workers.")
    print("[SURVEY] Collecting {} NEW valid scans, up to {:.1f} sec.".format(
        scans, timeout_sec))
    started = time.monotonic()
    receiver.start()
    try:
        while len(records) < scans and time.monotonic() - started < timeout_sec:
            cloud, stats = receiver.get_latest(copy=True)
            if cloud is None or not stats or not stats.get("fresh", False):
                time.sleep(0.05)
                continue
            frame_id = stats.get("scan")
            if frame_id is None or frame_id == seen:
                time.sleep(0.05)
                continue
            seen = frame_id
            if len(cloud) == 0:
                continue
            xyzr = vehicle_frame(cloud)
            if not np.isfinite(xyzr).all():
                continue
            info = summarize_cloud(xyzr)
            info.update(frame=int(frame_id), points=len(xyzr),
                        quality=str(stats.get("quality", "")),
                        timestamp_monotonic=float(stats["timestamp"]))
            records.append((info, xyzr))
            print("[SURVEY] scan={} points={} center={} left={} right={} near_center={}".format(
                frame_id, info["points"], info["center_points"],
                info["left_points"], info["right_points"],
                info["nearest_center_forward_m"]), flush=True)
    finally:
        receiver.stop()
    if not records:
        print("[SURVEY] No complete fresh scan. Check MORAI LiDAR and UDP 1116.")
        return 1
    with (out / "points_vehicle_frame.csv").open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(["scan", "forward_m", "left_m", "up_m", "intensity"])
        for info, cloud in records:
            for p in cloud:
                writer.writerow([info["frame"], *[round(float(v), 5) for v in p]])
    with (out / "scan_summary.csv").open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(records[0][0].keys()))
        writer.writeheader()
        for info, _ in records:
            writer.writerow(info)
    (out / "metadata.json").write_text(json.dumps({
        "coordinate_system": "vehicle origin, +X forward, +Y left, +Z up",
        "reference": "fixed stationary vehicle only; no GPS/world transform",
        "complete_scans": len(records),
        "requested_scans": scans,
        "lidar_bind_ip": ip, "lidar_port": port,
        "warning": "Point returns alone do not confirm drivable corridor. No ground segmentation or wall clustering. Do not blindly drive on this scan."
    }, ensure_ascii=False, indent=2), encoding="utf-8")
    # Optional visual only. Matplotlib is not required for CSV data capture.
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(11, 7))
        colors = plt.cm.viridis(np.linspace(0, 1, len(records)))
        for idx, (info, cloud) in enumerate(records):
            roi = cloud[(cloud[:, 0] >= 0) & (cloud[:, 0] <= 24) &
                        (cloud[:, 2] > -1.45) & (cloud[:, 2] < 1.25)]
            ax.scatter(roi[:, 1], roi[:, 0], s=1.3, alpha=.28,
                       color=colors[idx], label="scan " + str(info["frame"]))
        ax.scatter([0], [0], c="red", s=50, marker="^", label="ego")
        ax.set_xlabel("Left (+) / Right (-), m")
        ax.set_ylabel("Forward (+), m")
        ax.set_xlim(-10, 10)
        ax.set_ylim(0, 25)
        ax.set_aspect("equal", adjustable="box")
        ax.grid(alpha=.25)
        ax.legend(markerscale=4, fontsize=8, loc="upper right")
        fig.tight_layout()
        fig.savefig(str(out / "lidar_top_view.png"), dpi=180)
        plt.close(fig)
    except ImportError:
        print("[SURVEY] matplotlib not installed: CSV files saved; PNG skipped.")
    print("[SURVEY] Saved to: {}".format(out.resolve()))
    print("[SURVEY] Captured {} independent scans.".format(len(records)))
    return 0 if len(records) == scans else 2


def main():
    parser = argparse.ArgumentParser(description="Stationary MORAI LiDAR scan survey")
    parser.add_argument("--scans", type=int, default=5)
    parser.add_argument("--timeout-sec", type=float, default=40.0)
    parser.add_argument("--lidar-bind", default=None)
    parser.add_argument("--lidar-port", type=int, default=None)
    parser.add_argument("--output", default=str(ROOT / "logs" / "tunnel_surveys"))
    args = parser.parse_args()
    if not (1 <= args.scans <= 30 and 1.0 <= args.timeout_sec <= 300.0):
        parser.error("scans must be 1..30 and timeout 1..300 seconds")
    raise SystemExit(collect(
        args.lidar_bind, args.lidar_port, args.scans, args.timeout_sec,
        Path(args.output)))


if __name__ == "__main__":
    main()
