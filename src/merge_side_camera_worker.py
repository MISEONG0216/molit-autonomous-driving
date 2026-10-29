#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""MORAI LEFT-side + rear-left-facing cameras -> merge-risk localhost:9301.

Only main_integrated_v1.py sends EgoCtrlCmd. Side-camera risk is an
uncalibrated image-space proxy, NOT metric range or time-to-collision.
"""
import argparse
import json
import select
import socket
import time
from pathlib import Path

import numpy as np
from integration.merge_side_io import (
    JpegAssembler, merge_camera_mode, active_merge_cameras,
    vehicle_risk, attenuate_distant_risk, format_result,
)

ROOT = Path(__file__).resolve().parents[1]
CARS = [2, 3, 5, 7]  # COCO: car, motorcycle, bus, truck


def take_latest(sock, assembler):
    latest = None
    for _ in range(192):
        try:
            packet, _ = sock.recvfrom(65535)
        except BlockingIOError:
            break
        jpeg = assembler.feed(packet)
        if jpeg:
            latest = jpeg
    return latest


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--show", action="store_true", help="show LEFT and LEFT-REAR cameras, never the right camera")
    p.add_argument("--imgsz", type=int, default=480)
    args = p.parse_args()

    import cv2
    import torch
    from ultralytics import YOLO

    with (ROOT / "config/integration_network.json").open(encoding="utf-8") as f:
        net = json.load(f)
    with (ROOT / "config/integration_missions.json").open(encoding="utf-8") as f:
        cfg = json.load(f)["merge"]
    if net["merge_gate"]["bind_ip"] != "127.0.0.1":
        raise RuntimeError("merge_gate must be localhost")
    rear_port = cfg.get("rear_left_camera_port")
    if type(rear_port) is not int or not 1 <= rear_port <= 65535:
        raise RuntimeError(
            "Rear-left-facing camera UDP port NOT configured. "
            "Set merge.rear_left_camera_port in config/integration_missions.json "
            "after verifying MORAI Sensor Settings. Right camera UDP9295 is forbidden."
        )
    if rear_port in (9291, 9293, 9295, 9300, 9301, 9302, 9303, 9304, 9305, 9306, 9307):
        raise RuntimeError(
            "Invalid rear-left-facing port: conflicts with front/left/right or local integration. "
            "Do NOT reuse right camera UDP9295."
        )
    device = "cuda" if torch.cuda.is_available() else "cpu"
    model = YOLO("yolov8n.pt")
    print(f"[MERGE CAM] device={device} LEFT UDP9293 / LEFT_REAR UDP{rear_port}; RIGHT DISABLED")
    print(f"[MERGE CAM] gate=127.0.0.1:{net['merge_gate']['port']}")
    print(f"[MERGE CAM] results=127.0.0.1:{net['merge_result']['port']}")
    print("[MERGE CAM] NO EgoCtrlCmd sender. Merge inference gated by main waypoint.")

    cameras = {}
    gate = None
    publisher = None
    try:
        for side, port in (("left", 9293), ("left_rear", rear_port)):
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 8 * 1024 * 1024)
            s.bind(("0.0.0.0", port))
            s.setblocking(False)
            cameras[side] = s

        gate = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        gate.bind(("127.0.0.1", int(net["merge_gate"]["port"])))
        gate.setblocking(False)
        publisher = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        target = ("127.0.0.1", int(net["merge_result"]["port"]))

        assemblers = {side: JpegAssembler() for side in cameras}
        frames = {}
        risk = {side: 1.0 for side in cameras}
        previous_area = {side: 0.0 for side in cameras}
        last_inferred = {side: 0.0 for side in cameras}
        latest_wp, gate_all, last_gate = None, False, 0.0
        roundabout_done = False
        previous_mode = 'off'
        last_state, last_log = None, 0.0

        if args.show:
            for side in cameras:
                name = "MORAI MERGE " + side.upper()
                cv2.namedWindow(name, cv2.WINDOW_NORMAL)
                cv2.resizeWindow(name, 640, 480)
        while True:
            while True:
                try:
                    raw, _ = gate.recvfrom(512)
                except BlockingIOError:
                    break
                try:
                    obj = json.loads(raw.decode("ascii"))
                    wp = obj.get("waypoint_index")
                    if type(wp) is int and 0 <= wp < 4430:
                        latest_wp = wp
                        gate_all = obj.get("merge_all") is True
                        roundabout_done = obj.get("roundabout_done") is True
                        last_gate = time.monotonic()
                except (ValueError, UnicodeError, TypeError, AttributeError):
                    pass

            gate_fresh = (latest_wp is not None
                          and time.monotonic() - last_gate <= 1.0)
            mode = (merge_camera_mode(latest_wp, cfg, gate_all, roundabout_done)
                    if gate_fresh else "off")
            active_sides = active_merge_cameras(mode)
            enabled = bool(active_sides)
            if mode != previous_mode:
                prearm_to_live = (
                    (previous_mode, mode) in (
                        ("roundabout_prearm", "roundabout"),
                        ("highway_prearm", "highway_merge"),
                    )
                )
                print(f"[MERGE ZONE] {mode.upper()} WP={latest_wp} "
                      f"active={active_sides} rear_left={'ON' if 'left_rear' in active_sides else 'OFF'}")
                # Never inherit previous-zone camera evidence or JPEG fragments.
                if not prearm_to_live:
                    previous_area = {side: 0.0 for side in cameras}
                    risk = {side: 1.0 for side in cameras}
                    last_inferred = {side: 0.0 for side in cameras}
                    for assembler in assemblers.values():
                        assembler.data.clear()
                if mode in ("roundabout", "roundabout_prearm", "post_merge", "roundabout_done"):
                    risk["left_rear"] = 0.0
                    frames.pop("left_rear", None)
                previous_mode = mode

            readable, _, _ = select.select(list(cameras.values()), [], [], 0.04)
            new_inference = False
            for side, cam in cameras.items():
                if cam not in readable:
                    continue
                if side not in active_sides:
                    # Disabled cameras are not decoded or inferred. Drain UDP
                    # to avoid stale frames when a later zone enables them.
                    for _ in range(192):
                        try:
                            cam.recvfrom(65535)
                        except BlockingIOError:
                            break
                    frames.pop(side, None)
                    continue
                jpeg = take_latest(cam, assemblers[side])
                if jpeg is None:
                    continue
                frame = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
                if frame is None or frame.size == 0:
                    continue
                frames[side] = frame
                if not enabled:
                    continue
                height, width = frame.shape[:2]
                # Only roundabout LEFT9293: infer on the RIGHT HALF of the
                # original image. Highway LEFT/LEFT_REAR remain full-frame.
                roundabout_roi = (side == "left" and mode in
                                  ("roundabout", "roundabout_prearm"))
                crop_x = int(width * float(cfg.get(
                    "roundabout_left_camera_min_x_ratio", 0.50
                ))) if roundabout_roi else 0
                inference_frame = frame[:, crop_x:] if roundabout_roi else frame
                if inference_frame.shape[1] < 16:
                    raise RuntimeError("Roundabout right-half ROI too narrow")
                result = model.predict(
                    inference_frame, imgsz=args.imgsz, classes=CARS,
                    conf=0.35, device=device, verbose=False,
                )[0]
                maximum = 0.0
                biggest_area = 0.0
                for box in result.boxes:
                    x1, y1, x2, y2 = map(float, box.xyxy[0].tolist())
                    # YOLO crop coordinates -> original-frame coordinates.
                    x1 += crop_x
                    x2 += crop_x
                    area = max(0.0, (x2-x1)*(y2-y1)/float(width*height))
                    current = vehicle_risk(
                        x1, y1, x2, y2, width, height,
                        previous_area_ratio=previous_area[side],
                    )
                    if mode in ("highway_merge", "highway_prearm", "post_merge"):
                        # Preserve original V23 risk near the vehicle, soften
                        # distant/small boxes that often belong to other lanes.
                        current = attenuate_distant_risk(current, area, cfg)
                    biggest_area = max(biggest_area, area)
                    maximum = max(maximum, current)
                    if args.show:
                        color = (0, 0, 255) if current >= cfg["risk_stop_threshold"] else (0, 220, 255)
                        cv2.rectangle(frame, (int(x1),int(y1)),(int(x2),int(y2)),color,2)
                        cv2.putText(frame, f"VEHICLE risk={current:.2f}",
                                    (int(x1), max(22,int(y1)-8)),
                                    cv2.FONT_HERSHEY_SIMPLEX,0.52,color,2)
                previous_area[side] = biggest_area
                risk[side] = maximum
                last_inferred[side] = time.monotonic()
                new_inference = True

            if args.show:
                for side in cameras:
                    frame = frames.get(side)
                    frame = frame.copy() if frame is not None else np.zeros((480,640,3),np.uint8)
                    if mode in ("roundabout", "roundabout_prearm") and side == "left":
                        cut_x = int(frame.shape[1] * float(cfg.get(
                            "roundabout_left_camera_min_x_ratio", 0.50)))
                        cv2.line(frame, (cut_x, 0), (cut_x, frame.shape[0]-1),
                                 (0, 0, 255), 2)
                        cv2.putText(frame, "LEFT HALF IGNORED",
                                    (8, 65), cv2.FONT_HERSHEY_SIMPLEX,
                                    0.58, (0, 0, 255), 2)
                    if mode in ("roundabout", "roundabout_prearm", "post_merge", "roundabout_done") and side == "left_rear":
                        label = f"WP={latest_wp} REAR LEFT PAUSED ({mode.upper()})"
                    else:
                        label = f"WP={latest_wp} MODE={mode} risk={risk[side]:.2f}"
                    cv2.putText(frame,label,(8,30),cv2.FONT_HERSHEY_SIMPLEX,0.60,(0,255,255),2)
                    cv2.imshow("MORAI MERGE " + side.upper(),frame)
                if cv2.waitKey(1) & 0xff in (ord("q"),27):
                    break

            if not enabled:
                continue
            now = time.monotonic()
            # Roundabout or verified post-merge: left camera only.
            # Before highway merge completion: both cameras must be fresh.
            ready = all(
                last_inferred[side] > 0.0 and now-last_inferred[side] < 1.5
                for side in active_sides
            )
            if ready:
                left = risk["left"]
                rear = risk["left_rear"] if "left_rear" in active_sides else 0.0
                decision = ("MERGE_REJECT" if max(left,rear) >= cfg["risk_stop_threshold"]
                            else "MERGE_ACCEPT")
            else:
                left = 1.0
                rear = 1.0 if "left_rear" in active_sides else 0.0
                decision = "MERGE_REJECT"

            # Prearm computes real camera observations BEFORE zone entry.
            # Main does not apply them as a stop outside the active zone.
            # Only a new inference can confirm a safe gap. Never turn
            # repeated timer ticks into three fake accept confirmations.
            if new_inference or (not ready and now - last_log >= 0.5):
                publisher.sendto(format_result(left,rear,decision),target)
            state = (decision, ready)
            if state != last_state or now-last_log >= 3.0:
                print(f"[MERGE] WP={latest_wp} mode={mode} left={left:.2f} "
                      f"left_rear={rear:.2f} ready={ready} => {decision}")
                last_state, last_log = state, now
    except KeyboardInterrupt:
        print("[MERGE] Interrupted")
    finally:
        for cam in cameras.values():
            cam.close()
        if gate is not None:
            gate.close()
        if publisher is not None:
            publisher.close()
        if args.show:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
