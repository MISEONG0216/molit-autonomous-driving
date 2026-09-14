#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""MORAI camera traffic-light detector.

Receives JPEG frames from MORAI on UDP 9291 and publishes one of
STOP, GO, GO_LEFT, UNKNOWN to localhost UDP 9300.
"""

import os
import socket
import sys
from collections import Counter, deque

import cv2
import numpy as np
import torch
import torch.nn.functional as F
from ultralytics import YOLO

try:
    from pytorch_lightning.callbacks.early_stopping import EarlyStopping
    torch.serialization.add_safe_globals([EarlyStopping])
except Exception:
    pass

TRAFFICLIGHT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "trafficlight")
sys.path.insert(0, TRAFFICLIGHT_DIR)
from module import NIA_SEGNet_module

CAMERA_IP = "127.0.0.1"
CAMERA_PORT = 9291
DECISION_IP = "127.0.0.1"
DECISION_PORT = 9300
MAX_DGRAM = 65535

CLASS_NAMES = ["red", "yellow", "green", "greenleft", "redleft"]
VEHICLE_ROI = (0.20, 0.02, 0.82, 0.58)
TARGET_SIGNAL_POINT = (0.58, 0.27)
MIN_BOX_WIDTH = 16
MIN_BOX_HEIGHT = 10
MIN_ASPECT_RATIO = 1.35
HISTORY_SIZE = 7
MIN_STABLE_VOTES = 4


def preprocess_crop(crop_bgr, device):
    crop_rgb = cv2.cvtColor(crop_bgr, cv2.COLOR_BGR2RGB)
    crop_resized = cv2.resize(crop_rgb, (70, 20), interpolation=cv2.INTER_AREA)
    tensor = torch.from_numpy(crop_resized).permute(2, 0, 1).float() / 255.0
    return tensor.unsqueeze(0).to(device)


def candidate_score(candidate, width, height):
    cx = ((candidate["xmin"] + candidate["xmax"]) * 0.5) / width
    cy = ((candidate["ymin"] + candidate["ymax"]) * 0.5) / height
    tx, ty = TARGET_SIGNAL_POINT
    distance = ((cx - tx) ** 2 + (cy - ty) ** 2) ** 0.5
    position_score = max(0.0, 1.0 - distance / 0.55)
    aspect_score = min(candidate["aspect"] / 3.0, 1.0)
    return (0.30 * candidate["det_conf"] + 0.30 * candidate["cls_conf"]
            + 0.20 * position_score + 0.20 * aspect_score)


def driving_decision(label):
    if label == "green":
        return "GO"
    if label == "greenleft":
        return "GO_LEFT"
    if label in ("red", "redleft", "yellow"):
        return "STOP"
    return "UNKNOWN"


def load_models(device):
    ckpt_path = os.path.join(TRAFFICLIGHT_DIR, "epoch=5-step=113.ckpt")
    original_torch_load = torch.load

    def trusted_checkpoint_load(*args, **kwargs):
        kwargs["weights_only"] = False
        return original_torch_load(*args, **kwargs)

    torch.load = trusted_checkpoint_load
    try:
        classifier = NIA_SEGNet_module.load_from_checkpoint(ckpt_path)
    finally:
        torch.load = original_torch_load

    classifier.to(device)
    classifier.eval()
    detector = YOLO(os.path.join(TRAFFICLIGHT_DIR, "yolov8n.pt"))
    return classifier, detector


def main():
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"[INFO] device={device}")
    classifier, detector = load_models(device)
    print("[OK] traffic-light models loaded")

    camera_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    camera_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    camera_sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 10 * 1024 * 1024)
    camera_sock.bind((CAMERA_IP, CAMERA_PORT))
    camera_sock.setblocking(False)

    decision_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    decision_target = (DECISION_IP, DECISION_PORT)
    print(f"[WAIT] MORAI camera {CAMERA_IP}:{CAMERA_PORT}")
    print(f"[INFO] decision output {DECISION_IP}:{DECISION_PORT}")

    jpeg_buffer = b""
    history = deque(maxlen=HISTORY_SIZE)
    last_reported = None

    try:
        while True:
            packets = []
            while True:
                try:
                    packet, _ = camera_sock.recvfrom(MAX_DGRAM)
                    packets.append(packet)
                except BlockingIOError:
                    break

            if not packets:
                if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                    break
                continue

            for packet in packets:
                jpeg_buffer += packet

            if len(jpeg_buffer) > 2 * 1024 * 1024:
                jpeg_buffer = b""
                continue

            end_index = jpeg_buffer.rfind(b"\xff\xd9")
            start_index = jpeg_buffer.rfind(b"\xff\xd8", 0, end_index) if end_index != -1 else -1
            if start_index == -1 or end_index <= start_index:
                continue

            jpg_data = jpeg_buffer[start_index:end_index + 2]
            jpeg_buffer = jpeg_buffer[end_index + 2:]
            frame = cv2.imdecode(np.frombuffer(jpg_data, dtype=np.uint8), cv2.IMREAD_COLOR)
            if frame is None or frame.size == 0:
                continue

            height, width, _ = frame.shape
            roi_left = int(VEHICLE_ROI[0] * width)
            roi_top = int(VEHICLE_ROI[1] * height)
            roi_right = int(VEHICLE_ROI[2] * width)
            roi_bottom = int(VEHICLE_ROI[3] * height)
            candidates = []

            result = detector(frame, imgsz=1280, conf=0.25, verbose=False)[0]
            for box in result.boxes:
                if int(box.cls[0].item()) != 9:
                    continue
                confidence = float(box.conf[0].item())
                xmin, ymin, xmax, ymax = map(int, box.xyxy[0].tolist())
                box_width, box_height = xmax - xmin, ymax - ymin
                center_x, center_y = (xmin + xmax) // 2, (ymin + ymax) // 2
                aspect = box_width / max(box_height, 1)

                if box_width < MIN_BOX_WIDTH or box_height < MIN_BOX_HEIGHT:
                    continue
                if aspect < MIN_ASPECT_RATIO:
                    continue
                if not (roi_left <= center_x <= roi_right and roi_top <= center_y <= roi_bottom):
                    continue

                x1 = max(0, int(xmin - box_width * 0.15))
                y1 = max(0, int(ymin - box_height * 0.15))
                x2 = min(width, int(xmax + box_width * 0.15))
                y2 = min(height, int(ymax + box_height * 0.15))
                crop = frame[y1:y2, x1:x2]
                if crop.size == 0:
                    continue

                with torch.no_grad():
                    probabilities = F.softmax(classifier(preprocess_crop(crop, device)), dim=1)[0]
                    top_probability, predicted_index = torch.max(probabilities, dim=0)

                candidates.append({
                    "xmin": xmin, "ymin": ymin, "xmax": xmax, "ymax": ymax,
                    "aspect": aspect, "det_conf": confidence,
                    "cls_conf": top_probability.item(),
                    "label": CLASS_NAMES[predicted_index.item()],
                })

            selected = None
            votes = 0
            if candidates:
                selected = max(candidates, key=lambda item: candidate_score(item, width, height))
                history.append(selected["label"])
                stable_label, votes = Counter(history).most_common(1)[0]
                if votes < MIN_STABLE_VOTES:
                    stable_label = "unknown"
            else:
                history.clear()
                stable_label = "unknown"

            decision = driving_decision(stable_label)
            decision_sock.sendto(decision.encode("ascii"), decision_target)
            if (stable_label, decision) != last_reported:
                print(f"[VEHICLE SIGNAL] {stable_label} -> {decision} ({votes}/{len(history)} votes)")
                last_reported = (stable_label, decision)

            if selected:
                color = (0, 255, 0) if decision.startswith("GO") else (0, 0, 255)
                if stable_label == "yellow":
                    color = (0, 255, 255)
                cv2.rectangle(frame, (selected["xmin"], selected["ymin"]),
                              (selected["xmax"], selected["ymax"]), color, 3)
                cv2.putText(frame, f"VEHICLE: {stable_label} / {decision}",
                            (selected["xmin"], max(25, selected["ymin"] - 12)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.65, color, 2)
            else:
                cv2.putText(frame, "VEHICLE: unknown / UNKNOWN", (30, 45),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

            cv2.rectangle(frame, (roi_left, roi_top), (roi_right, roi_bottom), (255, 180, 0), 1)
            cv2.imshow("MORAI Vehicle Traffic Light", frame)
            if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                break
    except KeyboardInterrupt:
        pass
    finally:
        decision_sock.sendto(b"UNKNOWN", decision_target)
        camera_sock.close()
        decision_sock.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
