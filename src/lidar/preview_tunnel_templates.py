#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Offline LiDAR survey preview for V48 template (NO vehicle controls)."""
import argparse
import csv
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from shadow_predefined_templates import choose_template_detour


def load_first_scan(path):
    rows = []
    first_id = None
    with path.open("r", newline="", encoding="utf-8-sig") as handle:
        for row in csv.DictReader(handle):
            frame = int(row["scan"])
            if first_id is None:
                first_id = frame
            if frame != first_id:
                continue
            rows.append([float(row["forward_m"]), float(row["left_m"]),
                         float(row["up_m"])])
    if not rows:
        raise ValueError("No saved LiDAR points")
    return np.asarray(rows, dtype=float)


def main():
    parser = argparse.ArgumentParser(
        description="Dry-run-only first/second template on one stationary survey")
    parser.add_argument("--csv", required=True, type=Path)
    parser.add_argument("--stage", choices=("first", "second"), required=True)
    args = parser.parse_args()
    cfg = json.loads((ROOT / "config" /
        "tunnel_predefined_templates.json").read_text(encoding="utf-8"))
    points = load_first_scan(args.csv)
    completed = 0 if args.stage == "first" else 1
    result = choose_template_detour(points, cfg, completed)
    print("[TEMPLATE OFFLINE] stage={} points={} status={} reason={}".format(
        args.stage, len(points), result["status"], result["reason"]))
    if result["status"] == "AVOIDING":
        print("[TEMPLATE OFFLINE] side={} offset={}m return_delay={}m".format(
            "LEFT" if result["side"] == 1 else "RIGHT",
            result["selected_offset_m"],
            result["selected_return_delay_m"]))
    else:
        fmin = float(cfg["detection_forward_m"][0])
        fmax = float(cfg["detection_forward_m"][1])
        center = points[(np.abs(points[:, 1]) <=
            float(cfg["detection_lateral_abs_max_m"])) &
            (points[:, 0] >= 3.0)]
        nearest = float(np.min(center[:, 0])) if len(center) else None
        print("[TEMPLATE OFFLINE] central nearest={}m; trigger range={}..{}m".format(
            nearest, fmin, fmax))
    for d in result.get("diagnostics", []):
        if "offset_m" in d:
            print("  offset={}m delay={}m hits={} reason={} visible={}".format(
                d["offset_m"], d["return_delay_m"],
                d["hit_points"], d["reason"], d.get("visible")))
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        pts = points[(points[:, 2] >= -1.45) &
                     (points[:, 2] <= 1.25) &
                     (points[:, 0] >= 0) & (points[:, 0] <= 25)]
        fig, ax = plt.subplots(figsize=(9, 8))
        ax.scatter(pts[:, 1], pts[:, 0], s=1.0, alpha=.3,
                   label="Observed LiDAR points")
        if result["status"] == "AVOIDING":
            path = np.asarray(result["path"])
            ax.plot(path[:, 1], path[:, 0], linewidth=2.5,
                    label="Proposed verified template (offline only)")
        ax.scatter([0], [0], marker="^", s=70, label="Vehicle")
        ax.invert_xaxis()  # Positive left appears visually on LEFT.
        ax.set_aspect("equal", adjustable="box")
        ax.set(xlabel="Vehicle LEFT (+) / RIGHT (-), m",
               ylabel="Forward (+), m",
               xlim=(9, -9), ylim=(0, 25),
               title="OFFLINE PREVIEW: {} - {}".format(
                   args.stage, result["reason"]))
        ax.grid(alpha=.2)
        ax.legend()
        fig.tight_layout()
        output = args.csv.parent / ("template_preview_" + args.stage + ".png")
        fig.savefig(str(output), dpi=160)
        plt.close(fig)
        print("[TEMPLATE OFFLINE] Preview saved:", output)
    except ImportError:
        print("[TEMPLATE OFFLINE] matplotlib unavailable, text result only")


if __name__ == "__main__":
    main()
