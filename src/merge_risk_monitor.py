#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Camera-based experimental risk monitor for highway merges and roundabouts.

Adapted from the provided highway_merge_monitor.py.
Reads the team's configured Side (9293) and Rear (1118) cameras.
No vehicle control is transmitted: UDP risk results go to 127.0.0.1:9301.

Output: MERGE|LEFT_RISK|RIGHT_RISK|MAX_RISK|MERGE_ACCEPT/MERGE_REJECT
On missing input or processing failure: MERGE|0|0|0|MERGE_UNKNOWN

NOTE: MiDaS gives relative depth, not absolute range. Bounding-box size
growth is a heuristic rather than tracked object motion or TTC.
The resulting risk score is NOT a validated collision probability.
"""
import argparse
import os
import socket
import time
from collections import deque

import cv2
import numpy as np
import torch
from ultralytics import YOLO


VEHICLE_CLASSES = {2, 5, 7}  # COCO car/bus/truck
CONF_THRESHOLD = 0.40
MIDAS_INTERVAL = 3
W_DEPTH, W_AREA, W_APPROACH = 0.35, 0.25, 0.40
MAX_JPEG_BUFFER = 2 * 1024 * 1024
BIRD_W, BIRD_H = 640, 900
AREA_NORM_RATIO = 0.25
RISK_BRAKE_THRESHOLD = 0.60
RISK_LEVELS = (
    (0.80, "GRADE_4"),
    (0.60, "GRADE_3"),
    (0.40, "GRADE_2"),
    (0.20, "GRADE_1"),
    (0.00, "GRADE_0"),
)


def clamp01(x):
    return max(0.0, min(1.0, float(x)))


def grade_for(risk):
    return next(label for threshold, label in RISK_LEVELS if risk >= threshold)


class MoraiJpegCamera:
    """Reassemble JPEGs split over UDP packets; reuse latest complete frame."""

    def __init__(self, port):
        self.port = int(port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("0.0.0.0", self.port))
        self.sock.setblocking(False)
        self.buffer = b""
        self.last_frame = None
        self.last_frame_at = 0.0

    def poll(self):
        # Matching the existing traffic_light_detector.py JPEG SOI/EOI strategy.
        packets = []
        while True:
            try:
                packet, _ = self.sock.recvfrom(65535)
                packets.append(packet)
            except BlockingIOError:
                break
        for packet in packets:
            self.buffer += packet
        if len(self.buffer) > MAX_JPEG_BUFFER:
            self.buffer = b""
            return
        end = self.buffer.rfind(b"\xff\xd9")
        start = self.buffer.rfind(b"\xff\xd8", 0, end) if end >= 0 else -1
        if start < 0 or end <= start:
            return
        jpg = self.buffer[start:end + 2]
        self.buffer = self.buffer[end + 2:]
        frame = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
        if frame is not None and frame.size:
            self.last_frame = frame
            self.last_frame_at = time.time()

    def close(self):
        self.sock.close()


class CameraRiskAnalyzer:
    def __init__(self, device, yolo, midas, transform):
        self.device = device
        self.yolo = yolo
        self.midas = midas
        self.transform = transform
        self.depth_cache = {}
        self.prev_area = {}
        self.frame_num = {}

    def analyze(self, frame, source):
        h, w = frame.shape[:2]
        num = self.frame_num.get(source, 0) + 1
        self.frame_num[source] = num
        boxes = []
        for box in self.yolo(frame, verbose=False)[0].boxes:
            label = int(box.cls[0].item())
            confidence = float(box.conf[0].item())
            if label in VEHICLE_CLASSES and confidence >= CONF_THRESHOLD:
                x1, y1, x2, y2 = map(float, box.xyxy[0].tolist())
                boxes.append((x1, y1, x2, y2, confidence, label))

        depth = self.depth_cache.get(source)
        if boxes and (depth is None or num % MIDAS_INTERVAL == 0):
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            with torch.no_grad():
                pred = self.midas(self.transform(rgb).to(self.device))
                depth = torch.nn.functional.interpolate(
                    pred.unsqueeze(1), size=(h, w),
                    mode="bicubic", align_corners=False,
                ).squeeze().cpu().numpy()
            self.depth_cache[source] = depth

        # The uploaded code's full-rectangle bird view is just a resize
        # (no calibrated ground-plane projection). We preserve area scoring.
        image_area = max(1.0, w * h * AREA_NORM_RATIO)
        dmin = float(np.min(depth)) if depth is not None else 0.0
        dmax = float(np.max(depth)) if depth is not None else 0.0
        result_risk = 0.0
        output = frame.copy()
        previous_areas = self.prev_area.get(source, [])

        for i, (x1, y1, x2, y2, confidence, label) in enumerate(boxes):
            x1i, y1i = max(0, int(x1)), max(0, int(y1))
            x2i, y2i = min(w, int(x2)), min(h, int(y2))
            bbox_area = max(0.0, (x2-x1)*(y2-y1))
            area_norm = clamp01(bbox_area / image_area)
            depth_norm = 0.0
            if depth is not None and dmax > dmin + 1e-6:
                cropped = depth[y1i:y2i, x1i:x2i]
                if cropped.size:
                    depth_norm = clamp01(float(np.mean(
                        (cropped - dmin) / (dmax-dmin)
                    )))

            approach_norm = 0.0
            if i < len(previous_areas) and previous_areas[i] > 1.0:
                area_change = (bbox_area - previous_areas[i]) / previous_areas[i]
                approach_norm = clamp01(max(0.0, area_change) / 0.15)
            risk = clamp01(
                W_DEPTH * depth_norm + W_AREA * area_norm
                + W_APPROACH * approach_norm
            )
            result_risk = max(result_risk, risk)
            color = (0, 0, 255) if risk >= RISK_BRAKE_THRESHOLD else (0, 255, 0)
            cv2.rectangle(output, (x1i, y1i), (x2i, y2i), color, 2)
            cv2.putText(
                output, f"{grade_for(risk)} {risk:.2f}",
                (x1i, max(20, y1i-6)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2,
            )
        self.prev_area[source] = [
            max(0.0, (box[2]-box[0])*(box[3]-box[1]))
            for box in boxes
        ]
        cv2.putText(
            output, f"{source}: {len(boxes)} vehicles risk={result_risk:.2f}",
            (12, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
            (255, 255, 255), 2,
        )
        return result_risk, output


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--side-port", type=int, default=9293)
    parser.add_argument("--rear-port", type=int, default=1118)
    parser.add_argument("--result-ip", default="127.0.0.1")
    parser.add_argument("--result-port", type=int, default=9301)
    parser.add_argument("--stale-sec", type=float, default=1.2)
    parser.add_argument("--no-gui", action="store_true")
    args = parser.parse_args()
    if args.side_port == args.rear_port:
        parser.error("side/rear camera ports must differ")

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"[MERGE] YOLO + MiDaS loading on {device}...")
    yolo = YOLO("yolov8n.pt")
    midas = torch.hub.load(
        "intel-isl/MiDaS", "MiDaS_small", trust_repo=True
    ).to(device).eval()
    transform = torch.hub.load(
        "intel-isl/MiDaS", "transforms", trust_repo=True
    ).small_transform
    side = MoraiJpegCamera(args.side_port)
    rear = MoraiJpegCamera(args.rear_port)
    udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    analyzer = CameraRiskAnalyzer(device, yolo, midas, transform)
    print(
        f"[MERGE] camera SIDE:{args.side_port} REAR:{args.rear_port} "
        f"-> {args.result_ip}:{args.result_port}"
    )
    last_send = 0.0
    try:
        while True:
            side.poll()
            rear.poll()
            now = time.time()
            fresh_side = (
                side.last_frame is not None
                and now - side.last_frame_at <= args.stale_sec
            )
            fresh_rear = (
                rear.last_frame is not None
                and now - rear.last_frame_at <= args.stale_sec
            )
            frames = []
            if fresh_side and fresh_rear:
                try:
                    risk_l, view_l = analyzer.analyze(side.last_frame, "SIDE")
                    risk_r, view_r = analyzer.analyze(rear.last_frame, "REAR")
                    max_risk = max(risk_l, risk_r)
                    decision = (
                        "MERGE_REJECT"
                        if max_risk >= RISK_BRAKE_THRESHOLD
                        else "MERGE_ACCEPT"
                    )
                    payload = (
                        f"MERGE|{risk_l:.3f}|{risk_r:.3f}|"
                        f"{max_risk:.3f}|{decision}"
                    )
                    frames = [view_l, view_r]
                except Exception as exc:
                    print(f"[MERGE] inference error: {exc}")
                    payload = "MERGE|0|0|0|MERGE_UNKNOWN"
            else:
                payload = "MERGE|0|0|0|MERGE_UNKNOWN"
            udp.sendto(payload.encode("ascii"), (args.result_ip, args.result_port))
            if now - last_send >= 1.0:
                print("[MERGE]", payload)
                last_send = now
            if frames and not args.no_gui:
                height = min(img.shape[0] for img in frames)
                views = [
                    cv2.resize(img, (
                        max(1, int(img.shape[1] * height / img.shape[0])),
                        height,
                    )) for img in frames
                ]
                cv2.imshow("MORAI Camera Merge Risk (Side/Rear)", np.hstack(views))
                if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                    break
            time.sleep(0.01)
    except KeyboardInterrupt:
        pass
    finally:
        side.close()
        rear.close()
        udp.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
