#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Highway merge risk monitor adapted from highway.zip.

It does NOT control the vehicle. It only publishes:
MERGE|side_risk|rear_risk|max_risk|MERGE_ACCEPT(or MERGE_REJECT)

Default camera ports preserve the teammate code (9293, 1118), but all bind
addresses/ports can be changed with environment variables.
"""

import os
import socket
import time
from pathlib import Path

import cv2
import numpy as np
import torch
from ultralytics import YOLO


CAMERA_BIND_IP = os.getenv("MERGE_CAMERA_BIND_IP", "0.0.0.0")
PORT_SIDE = int(os.getenv("MERGE_SIDE_PORT", "9293"))
PORT_REAR = int(os.getenv("MERGE_REAR_PORT", "1118"))
RESULT_IP = os.getenv("MERGE_RESULT_IP", "127.0.0.1")
RESULT_PORT = int(os.getenv("MERGE_RESULT_PORT", "9301"))

CONF_THRESHOLD = 0.4
VEHICLE_CLASSES = [2, 5, 7]
MIDAS_INTERVAL = 3

W_DEPTH = 0.35
W_AREA = 0.25
W_APPROACH = 0.40

BIRDVIEW_WIDTH = 640
BIRDVIEW_HEIGHT = 900
BIRDVIEW_SRC_RATIO = np.float32([
    [0.00, 0.00],
    [1.00, 0.00],
    [1.00, 1.00],
    [0.00, 1.00],
])
AREA_NORM_DENOM_RATIO = 0.25

RISK_LEVELS = [
    (0.80, "GRADE_4", "REJECT", "CRITICAL", (0, 0, 180)),
    (0.60, "GRADE_3", "REJECT", "HIGH", (0, 0, 255)),
    (0.40, "GRADE_2", "CAUTION", "MEDIUM", (0, 165, 255)),
    (0.20, "GRADE_1", "ACCEPT", "LOW", (0, 255, 255)),
    (0.00, "GRADE_0", "ACCEPT", "SAFE", (0, 255, 0)),
]


def clamp01(value):
    return max(0.0, min(float(value), 1.0))


class MORAICameraReceiver:
    def __init__(self, port):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((CAMERA_BIND_IP, int(port)))
        self.sock.settimeout(0.5)

    def receive_frame(self):
        try:
            data, _ = self.sock.recvfrom(65535)
            header_offset = 0
            if data.startswith(b"#Cam"):
                header_offset = 12
            else:
                soi = data.find(b"\xff\xd8")
                if soi != -1:
                    header_offset = soi

            jpg_data = data[header_offset:]
            frame = cv2.imdecode(
                np.frombuffer(jpg_data, dtype=np.uint8),
                cv2.IMREAD_COLOR,
            )
            return frame
        except socket.timeout:
            return None
        except Exception:
            return None

    def close(self):
        self.sock.close()


def get_birdview_matrix(frame_shape):
    h, w = frame_shape[:2]
    src = BIRDVIEW_SRC_RATIO.copy()
    src[:, 0] *= w
    src[:, 1] *= h
    dst = np.float32([
        [0, 0],
        [BIRDVIEW_WIDTH - 1, 0],
        [BIRDVIEW_WIDTH - 1, BIRDVIEW_HEIGHT - 1],
        [0, BIRDVIEW_HEIGHT - 1],
    ])
    return cv2.getPerspectiveTransform(src.astype(np.float32), dst)


def get_birdview_area_norm(matrix, x1, y1, x2, y2):
    pts = np.float32([
        [x1, y1],
        [x2, y1],
        [x2, y2],
        [x1, y2],
    ]).reshape(-1, 1, 2)

    warped = cv2.perspectiveTransform(pts, matrix).reshape(-1, 2)
    warped[:, 0] = np.clip(warped[:, 0], 0, BIRDVIEW_WIDTH - 1)
    warped[:, 1] = np.clip(warped[:, 1], 0, BIRDVIEW_HEIGHT - 1)

    area = abs(cv2.contourArea(warped.astype(np.float32)))
    denom = (
        BIRDVIEW_WIDTH
        * BIRDVIEW_HEIGHT
        * AREA_NORM_DENOM_RATIO
    )
    return clamp01(area / max(denom, 1e-6))


def get_relative_depth(depth_map, x1, y1, x2, y2):
    h, w = depth_map.shape
    x1 = max(0, int(x1))
    y1 = max(0, int(y1))
    x2 = min(w, int(x2))
    y2 = min(h, int(y2))

    d_min = depth_map.min()
    d_max = depth_map.max()
    if d_max - d_min < 1e-6:
        return 0.0

    norm_map = (depth_map - d_min) / (d_max - d_min)
    roi = norm_map[y1:y2, x1:x2]
    if roi.size == 0:
        return 0.0
    return clamp01(float(np.mean(roi)))


def assess_risk(risk_score):
    for threshold, level, action, desc, color in RISK_LEVELS:
        if risk_score >= threshold:
            return level, action, desc, color
    return "GRADE_0", "ACCEPT", "SAFE", (0, 255, 0)


def process_camera_stream(
    frame,
    cam_name,
    yolo_model,
    midas,
    transform,
    device,
    frame_count,
    depth_map_cache,
    prev_bbox_areas,
):
    h, w = frame.shape[:2]
    matrix = get_birdview_matrix(frame.shape)

    results = yolo_model(frame, verbose=False)[0]
    detections = []

    for box in results.boxes:
        cls = int(box.cls[0])
        conf = float(box.conf[0])
        if cls in VEHICLE_CLASSES and conf >= CONF_THRESHOLD:
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
            detections.append([x1, y1, x2, y2, conf, cls])

    if (
        (frame_count % MIDAS_INTERVAL == 0 or frame_count == 1)
        and detections
    ):
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        with torch.no_grad():
            input_batch = transform(rgb).to(device)
            pred = midas(input_batch)
            pred = torch.nn.functional.interpolate(
                pred.unsqueeze(1),
                size=(h, w),
                mode="bicubic",
                align_corners=False,
            ).squeeze()
            depth_map_cache = pred.cpu().numpy()

    action_status = "ACCEPT"
    max_risk = 0.0
    curr_bbox_areas = []

    for idx, det in enumerate(detections, start=1):
        x1, y1, x2, y2, conf, cls = det
        bbox_area = (x2 - x1) * (y2 - y1)
        curr_bbox_areas.append(bbox_area)

        approach_norm = 0.0
        if (
            len(prev_bbox_areas) >= idx
            and prev_bbox_areas[idx - 1] > 0
        ):
            area_ratio = (
                bbox_area - prev_bbox_areas[idx - 1]
            ) / prev_bbox_areas[idx - 1]
            approach_norm = clamp01(
                max(0.0, area_ratio) / 0.15
            )

        if depth_map_cache is not None:
            depth_norm = get_relative_depth(
                depth_map_cache, x1, y1, x2, y2
            )
        else:
            depth_norm = 0.0

        area_norm = get_birdview_area_norm(
            matrix, x1, y1, x2, y2
        )

        risk_score = clamp01(
            (W_DEPTH * depth_norm)
            + (W_AREA * area_norm)
            + (W_APPROACH * approach_norm)
        )

        risk_level, action, desc, color = assess_risk(risk_score)
        max_risk = max(max_risk, risk_score)

        if action == "REJECT":
            action_status = "REJECT"

        cv2.rectangle(
            frame,
            (int(x1), int(y1)),
            (int(x2), int(y2)),
            color,
            2,
        )
        label = (
            f"[{action}] R:{risk_score:.2f} "
            f"(Dn:{depth_norm:.2f} Delta:{approach_norm:.2f})"
        )
        cv2.putText(
            frame,
            label,
            (int(x1), max(20, int(y1) - 6)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            color,
            2,
        )

    banner_color = (
        (0, 255, 0)
        if action_status == "ACCEPT"
        else (0, 0, 255)
    )
    cv2.rectangle(frame, (0, 0), (w, 30), (30, 30, 30), -1)
    cv2.putText(
        frame,
        f"{cam_name} | Vehicles:{len(detections)} | Max Risk:{max_risk:.2f}",
        (10, 20),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5,
        banner_color,
        1,
    )

    return (
        frame,
        action_status,
        max_risk,
        depth_map_cache,
        curr_bbox_areas,
    )


def main():
    device = torch.device(
        "cuda" if torch.cuda.is_available() else "cpu"
    )
    print(f"[*] Device: {device}")

    script_dir = Path(__file__).resolve().parent
    yolo_path = script_dir / "trafficlight" / "yolov8n.pt"

    print("[*] YOLOv8 & MiDaS loading...")
    yolo_model = YOLO(str(yolo_path))
    midas = torch.hub.load(
        "intel-isl/MiDaS",
        "MiDaS_small",
        trust_repo=True,
    ).to(device).eval()

    transforms = torch.hub.load(
        "intel-isl/MiDaS",
        "transforms",
        trust_repo=True,
    )
    transform = transforms.small_transform

    cam_side = MORAICameraReceiver(PORT_SIDE)
    cam_rear = MORAICameraReceiver(PORT_REAR)

    result_sock = socket.socket(
        socket.AF_INET,
        socket.SOCK_DGRAM,
    )
    result_target = (RESULT_IP, RESULT_PORT)

    print(
        f"[*] Camera ready: side={PORT_SIDE}, rear={PORT_REAR}"
    )
    print(
        f"[*] Merge result output: {RESULT_IP}:{RESULT_PORT}"
    )

    frame_count = 0
    depth_side = None
    depth_rear = None
    prev_areas_side = []
    prev_areas_rear = []
    last_frame_side = None
    last_frame_rear = None
    prev_time = time.time()

    try:
        while True:
            frame_side = cam_side.receive_frame()
            frame_rear = cam_rear.receive_frame()

            if frame_side is not None:
                last_frame_side = frame_side
            if frame_rear is not None:
                last_frame_rear = frame_rear

            if (
                last_frame_side is None
                or last_frame_rear is None
            ):
                continue

            frame_count += 1

            h1, w1 = last_frame_side.shape[:2]
            h2, w2 = last_frame_rear.shape[:2]
            target_h = min(h1, h2)

            f_side = cv2.resize(
                last_frame_side.copy(),
                (int(w1 * target_h / h1), target_h),
            )
            f_rear = cv2.resize(
                last_frame_rear.copy(),
                (int(w2 * target_h / h2), target_h),
            )

            (
                f_side_out,
                status_side,
                risk_side,
                depth_side,
                prev_areas_side,
            ) = process_camera_stream(
                f_side,
                f"SIDE_CAM ({PORT_SIDE})",
                yolo_model,
                midas,
                transform,
                device,
                frame_count,
                depth_side,
                prev_areas_side,
            )

            (
                f_rear_out,
                status_rear,
                risk_rear,
                depth_rear,
                prev_areas_rear,
            ) = process_camera_stream(
                f_rear,
                f"REAR_CAM ({PORT_REAR})",
                yolo_model,
                midas,
                transform,
                device,
                frame_count,
                depth_rear,
                prev_areas_rear,
            )

            if (
                status_side == "REJECT"
                or status_rear == "REJECT"
            ):
                final_decision = "MERGE_REJECT"
                final_color = (0, 0, 255)
            else:
                final_decision = "MERGE_ACCEPT"
                final_color = (0, 255, 0)

            max_risk = max(
                float(risk_side),
                float(risk_rear),
            )
            message = (
                f"MERGE|{risk_side:.4f}|{risk_rear:.4f}|"
                f"{max_risk:.4f}|{final_decision}"
            )
            result_sock.sendto(
                message.encode("ascii"),
                result_target,
            )

            combined = np.hstack(
                (f_side_out, f_rear_out)
            )
            ch, cw = combined.shape[:2]

            now = time.time()
            fps = 1.0 / (now - prev_time + 1e-6)
            prev_time = now

            cv2.rectangle(
                combined,
                (0, ch - 50),
                (cw, ch),
                (15, 15, 15),
                -1,
            )
            cv2.putText(
                combined,
                f"TOTAL DECISION: {final_decision}",
                (20, ch - 18),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                final_color,
                2,
            )
            cv2.putText(
                combined,
                (
                    f"Side Risk: {risk_side:.2f} | "
                    f"Rear Risk: {risk_rear:.2f} | "
                    f"FPS: {fps:.1f}"
                ),
                (max(20, cw - 520), ch - 18),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (220, 220, 220),
                1,
            )

            cv2.imshow(
                "MORAI Highway Merge Risk Monitor",
                combined,
            )
            if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                break

    except KeyboardInterrupt:
        pass

    finally:
        cam_side.close()
        cam_rear.close()
        result_sock.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
