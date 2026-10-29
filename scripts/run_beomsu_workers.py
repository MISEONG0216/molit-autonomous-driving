#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Windows CMD-friendly worker supervisor for V39+Beomsu.

Run exactly one instance of each group in separate CMD windows:
    .venv\Scripts\python.exe scripts\run_beomsu_workers.py vision
    .venv\Scripts\python.exe scripts\run_beomsu_workers.py safety

This process NEVER sends EgoCtrlCmd. Only the separate integrated main does.
Child stdout/stderr go to individually named logs under project/logs.
Press Ctrl+C in this CMD to terminate just its group.
"""
import argparse
import datetime
import json
import os
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
PYTHON = Path(sys.executable)
GROUPS = {
    "vision": (
        ("traffic", "src/traffic_light_detector.py", "--relay-pedestrian", "--relay-lane"),
        ("pedestrian", "src/dynamic_pedestrian_worker.py", "--follow"),
        ("lane", "src/gps_shadow_lane_worker.py"),
    ),
    "safety": (
        ("lidar", "src/lidar/static_avoidance_module.py"),
        ("merge", "src/merge_side_camera_worker.py"),
    ),
}


def preflight(group):
    cfg = json.loads(
        (ROOT / "config/integration_network.json").read_text(encoding="utf-8")
    )
    if cfg["control"]["target_ip"] != "127.0.0.1" or int(cfg["control"]["port"]) != 9093:
        raise SystemExit(
            "[ERROR] This test launcher expects MORAI on THIS PC "
            "and CtrlCmd target 127.0.0.1:9093. Check the actual simulator setup."
        )
    if group == "safety":
        missions = json.loads(
            (ROOT / "config/integration_missions.json").read_text(encoding="utf-8")
        )
        rear_port = missions["merge"].get("rear_left_camera_port")
        reserved = {9291, 9293, 9295, 9300, 9301, 9302, 9303, 9304,
                    9305, 9306, 9307, 9308, 9311, 9312}
        if type(rear_port) is not int or not 1 <= rear_port <= 65535 or rear_port in reserved:
            raise SystemExit(
                "[ERROR] LEFT_REAR camera UDP port is invalid. "
                "Check MORAI sensor settings; never use RIGHT camera UDP9295."
            )
        print("[MERGE] LEFT UDP9293 + LEFT_REAR UDP{} (must exist in MORAI)".format(rear_port),
              flush=True)
    for worker in GROUPS[group]:
        if not (ROOT / worker[1]).is_file():
            raise SystemExit("[ERROR] Missing worker script: " + worker[1])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("group", choices=tuple(GROUPS))
    args = parser.parse_args()
    os.chdir(str(ROOT))
    preflight(args.group)
    logdir = ROOT / "logs"
    logdir.mkdir(exist_ok=True)
    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    processes = []
    handles = []
    env = os.environ.copy()
    env["PYTHONUNBUFFERED"] = "1"
    # Full-run evidence, without changing any avoidance logic.
    if args.group == "safety":
        env["STATIC_CSV_LOG"] = "1"

    try:
        for spec in GROUPS[args.group]:
            name, script, *flags = spec
            logfile = logdir / ("beomsu_{}_{}_{}.log".format(args.group, name, stamp))
            handle = logfile.open("w", encoding="utf-8")
            handles.append(handle)
            cmd = [str(PYTHON), "-u", str(ROOT / script)] + flags
            # Start a child that shares this console's lifecycle, with output
            # redirected to disk to keep all three CMD windows readable.
            proc = subprocess.Popen(
                cmd, cwd=str(ROOT), stdout=handle,
                stderr=subprocess.STDOUT, env=env,
            )
            processes.append((name, proc, logfile))
            print("[START] {} pid={} -> {}".format(name, proc.pid, logfile), flush=True)
        print("[{}] Workers launched. No EgoCtrlCmd is sent here.".format(
            args.group.upper()), flush=True)
        print("[INFO] Wait until camera/GPS/LiDAR and worker logs show receiving data.",flush=True)
        print("[INFO] Check logs in: {}. Ctrl+C stops this worker group.".format(logdir),
              flush=True)

        last_report = 0.0
        while True:
            for name, proc, logfile in processes:
                if proc.poll() is not None:
                    raise RuntimeError(
                        "{} worker exited with code {}. Read {}".format(
                            name, proc.returncode, logfile
                        )
                    )
            if time.monotonic() - last_report > 12.0:
                print("[RUNNING] {} group: {}".format(args.group,
                    ", ".join("{}:{}".format(name,proc.pid)
                              for name,proc,_ in processes)),flush=True)
                last_report = time.monotonic()
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\n[STOP] Ctrl+C received. Ending {} worker group.".format(args.group),
              flush=True)
    except RuntimeError as exc:
        print("[ERROR] " + str(exc), file=sys.stderr, flush=True)
        raise SystemExit(1)
    finally:
        for _name, proc, _logfile in processes:
            if proc.poll() is None:
                proc.terminate()
        for _name, proc, _logfile in processes:
            try:
                proc.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait(timeout=3.0)
        for handle in handles:
            handle.close()
        print("[CLEANUP] Group terminated.", flush=True)


if __name__ == "__main__":
    main()
