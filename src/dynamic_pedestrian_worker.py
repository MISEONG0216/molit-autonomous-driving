#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Independent pedestrian risk worker, adapted from the user's YOLO/MiDaS code.

Camera packets arrive from traffic_light_detector.py's optional localhost
relay. This module NEVER binds MORAI 9291 or transmits EgoCtrlCmd.

Source risk: MiDaS relative depth 0.40 + birdview area 0.20 +
consecutive-box area-growth 0.40, STOP_THRESHOLD 0.50.
Only COCO person (class 0) can trigger the pedestrian-mission STOP.
Car/bus/truck boxes are retained for optional visualisation only.
"""
import argparse
import json
import socket
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
NETWORK = ROOT / "config" / "integration_network.json"
MISSION = ROOT / "config" / "integration_missions.json"

CONF_THRESHOLD = 0.4
FRONT_CLASSES = (0, 2, 5, 7)
MIDAS_INTERVAL = 3
W_DEPTH = 0.40
W_AREA = 0.20
W_APPROACH = 0.40
BIRDVIEW_WIDTH = 640
BIRDVIEW_HEIGHT = 900
BIRDVIEW_SRC_RATIO = np.float32([
    [0.05, 0.15], [0.95, 0.15], [1.00, 1.00], [0.00, 1.00],
])
AREA_NORM_DENOM_RATIO = 0.25
HOOD_CROP_RATIO = 0.78


def clamp01(value):
    return max(0.0, min(float(value), 1.0))


def get_birdview_matrix(cv2, shape):
    h, w = shape[:2]
    src = BIRDVIEW_SRC_RATIO.copy()
    src[:, 0] *= w
    src[:, 1] *= h
    dst = np.float32([
        [0, 0], [BIRDVIEW_WIDTH - 1, 0],
        [BIRDVIEW_WIDTH - 1, BIRDVIEW_HEIGHT - 1], [0, BIRDVIEW_HEIGHT - 1],
    ])
    return cv2.getPerspectiveTransform(src, dst)


def get_birdview_area_norm(cv2, matrix, x1, y1, x2, y2):
    pts = np.float32([
        [x1, y1], [x2, y1], [x2, y2], [x1, y2],
    ]).reshape(-1, 1, 2)
    warped = cv2.perspectiveTransform(pts, matrix).reshape(-1, 2)
    warped[:, 0] = np.clip(warped[:, 0], 0, BIRDVIEW_WIDTH - 1)
    warped[:, 1] = np.clip(warped[:, 1], 0, BIRDVIEW_HEIGHT - 1)
    area = abs(cv2.contourArea(warped.astype(np.float32)))
    denom = BIRDVIEW_WIDTH * BIRDVIEW_HEIGHT * AREA_NORM_DENOM_RATIO
    return clamp01(area / max(denom, 1e-6))


def get_relative_depth(depth_map, x1, y1, x2, y2):
    h, w = depth_map.shape
    x1, y1 = max(0, int(x1)), max(0, int(y1))
    x2, y2 = min(w, int(x2)), min(h, int(y2))
    d_min, d_max = float(np.min(depth_map)), float(np.max(depth_map))
    if d_max - d_min < 1e-6:
        return 0.0
    normalized = (depth_map - d_min) / (d_max - d_min)
    roi = normalized[y1:y2, x1:x2]
    return clamp01(float(np.mean(roi))) if roi.size else 0.0


class JpegStreamAssembler:
    """Extract latest full JPEG from a stream of MORAI camera datagrams."""

    def __init__(self, max_buffer=3 * 1024 * 1024):
        self.buffer = b""
        self.max_buffer = max_buffer

    def feed(self, packets):
        for packet in packets:
            self.buffer += packet
            if len(self.buffer) > self.max_buffer:
                # Preserve a possible partial latest frame.
                start = self.buffer.rfind(b"\\xff\\xd8")
                self.buffer = self.buffer[start:] if start >= 0 else b""
        end = self.buffer.rfind(b"\\xff\\xd9")
        if end < 0:
            return None
        start = self.buffer.rfind(b"\\xff\\xd8", 0, end)
        if start < 0 or start >= end:
            self.buffer = self.buffer[end + 2:]
            return None
        jpeg = self.buffer[start:end + 2]
        self.buffer = self.buffer[end + 2:]
        return jpeg


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--show", action="store_true", help="show overlay window")
    args = parser.parse_args()

    # Heavy imports are deferred, so pure packet/risk tests do not require
    # torch, ultralytics or a GPU.
    import cv2
    import torch
    from ultralytics import YOLO

    with NETWORK.open("r", encoding="utf-8") as f:
        network = json.load(f)
    with MISSION.open("r", encoding="utf-8") as f:
        mission = json.load(f)["dynamic_pedestrian"]
    in_cfg = network["pedestrian_camera_relay"]
    out_cfg = network["pedestrian_result"]
    if in_cfg["bind_ip"] != "127.0.0.1" or out_cfg["bind_ip"] != "127.0.0.1":
        raise RuntimeError("Pedestrian worker sockets must stay on localhost")
    threshold = float(mission.get("stop_risk_threshold", 0.50))
    if not 0 < threshold <= 1:
        raise ValueError("stop_risk_threshold must be in (0,1]")

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"[PEDESTRIAN] device={device} loading source YOLOv8n + MiDaS_small")
    yolo_model = YOLO("yolov8n.pt")
    midas = torch.hub.load(
        "intel-isl/MiDaS", "MiDaS_small", trust_repo=True
    ).to(device).eval()
    transforms = torch.hub.load(
        "intel-isl/MiDaS", "transforms", trust_repo=True
    )
    transform = transforms.small_transform

    cam = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    cam.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 8 * 1024 * 1024)
    cam.bind(("127.0.0.1", int(in_cfg["port"])))
    cam.settimeout(0.25)
    output = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    target = ("127.0.0.1", int(out_cfg["port"]))
    assembler = JpegStreamAssembler()
    frame_number = 0
    depth_cache = None
    prev_bbox_areas = []
    last_report = None
    last_wait_log = 0.0
    print(f"[PEDESTRIAN] relayed camera 127.0.0.1:{in_cfg['port']}")
    print(f"[PEDESTRIAN] risk decision -> {target}, threshold={threshold:.2f}")
    print("[CONTROL] No EgoCtrlCmd sender in this process.")
    try:
        while True:
            try:
                first, _ = cam.recvfrom(65535)
            except socket.timeout:
                if time.time() - last_wait_log > 3:
                    print("[PEDESTRIAN WAIT] Camera relay packets unavailable")
                    last_wait_log = time.time()
                continue
            packets = [first]
            # Drain all pending packets so heavy YOLO/MiDaS inference always
            # operates on the most recent completed frame, not an old backlog.
            cam.setblocking(False)
            while True:
                try:
                    pkt, _ = cam.recvfrom(65535)
                    packets.append(pkt)
                except BlockingIOError:
                    break
            cam.settimeout(0.25)

            jpeg = assembler.feed(packets)
            if jpeg is None:
                continue
            frame = cv2.imdecode(
                np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR
            )
            if frame is None or frame.size == 0:
                continue

            frame_number += 1
            h, w = frame.shape[:2]
            matrix = get_birdview_matrix(cv2, frame.shape)
            crop_h = int(h * HOOD_CROP_RATIO)
            roi_frame = frame[:crop_h, :]
            results = yolo_model(roi_frame, verbose=False)[0]
            detections = []
            for box in results.boxes:
                cls = int(box.cls[0])
                conf = float(box.conf[0])
                if cls not in FRONT_CLASSES or conf < CONF_THRESHOLD:
                    continue
                x1, y1, x2, y2 = [
                    float(z) for z in box.xyxy[0].cpu().numpy()
                ]
                if y2 >= crop_h - 10 and (x2 - x1) > w * 0.5:
                    continue
                detections.append((x1, y1, x2, y2, conf, cls))

            if (frame_number == 1 or frame_number % MIDAS_INTERVAL == 0) and detections:
                rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                with torch.no_grad():
                    batch = transform(rgb).to(device)
                    pred = midas(batch)
                    pred = torch.nn.functional.interpolate(
                        pred.unsqueeze(1), size=(h, w),
                        mode="bicubic", align_corners=False,
                    ).squeeze()
                    depth_cache = pred.cpu().numpy()

            # The original source computes risk for all detected road users.
            # The user requested a pedestrian-specific STOP module, so only
            # COCO person(0) contributes to the transmitted stopping decision.
            max_person_risk = 0.0
            persons = 0
            curr_bbox_areas = []
            for idx, (x1, y1, x2, y2, conf, cls) in enumerate(detections):
                bbox_area = (x2 - x1) * (y2 - y1)
                curr_bbox_areas.append(bbox_area)
                approach = 0.0
                if idx < len(prev_bbox_areas) and prev_bbox_areas[idx] > 0:
                    area_ratio = (
                        bbox_area - prev_bbox_areas[idx]
                    ) / prev_bbox_areas[idx]
                    approach = clamp01(max(0.0, area_ratio) / 0.15)
                depth = (
                    get_relative_depth(depth_cache, x1, y1, x2, y2)
                    if depth_cache is not None else 0.0
                )
                birdview = get_birdview_area_norm(
                    cv2, matrix, x1, y1, x2, y2
                )
                risk = clamp01(
                    W_DEPTH * depth + W_AREA * birdview + W_APPROACH * approach
                )
                if cls == 0:
                    persons += 1
                    max_person_risk = max(max_person_risk, risk)
                if args.show:
                    color = (0, 0, 255) if cls == 0 and risk >= threshold else (0, 200, 0)
                    cv2.rectangle(
                        frame, (int(x1), int(y1)), (int(x2), int(y2)), color, 2
                    )
                    cv2.putText(
                        frame, f"{yolo_model.names[cls]} risk={risk:.2f}",
                        (int(x1), max(20, int(y1) - 6)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2,
                    )
            prev_bbox_areas = curr_bbox_areas

            stop = max_person_risk >= threshold
            decision = "EMERGENCY_STOP" if stop else "DRIVE_FORWARD"
            packet = {
                "status": "STOP_REQUIRED" if stop else "CLEAR",
                "decision": decision,
                "risk": round(max_person_risk, 4),
                "person_count": persons,
                "frame": frame_number,
            }
            output.sendto(
                json.dumps(packet, separators=(",", ":")).encode("utf-8"),
                target,
            )
            if (decision, persons) != last_report:
                print(
                    f"[PEDESTRIAN] frame={frame_number} person={persons} "
                    f"risk={max_person_risk:.3f} -> {decision}"
                )
                last_report = (decision, persons)
            if args.show:
                cv2.putText(
                    frame, f"{decision} | person={persons} risk={max_person_risk:.2f}",
                    (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                    (0, 0, 255) if stop else (0, 255, 0), 2,
                )
                cv2.imshow("MORAI Pedestrian Risk (relayed camera)", frame)
                if cv2.waitKey(1) & 0xff in (ord("q"), 27):
                    break
    except KeyboardInterrupt:
        print("[PEDESTRIAN] Interrupted")
    finally:
        cam.close()
        output.close()
        if args.show:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
