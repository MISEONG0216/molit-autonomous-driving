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

from integration.dynamic_pedestrian_io import load_dynamic_zones, in_dynamic_zone
from integration.highway_follow_io import (
    LANE_ROI_RATIO, in_follow_zone, grade_speed_cap,
    vehicle_overlaps_lane, retained_follow_risk,
)

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
                start = self.buffer.rfind(bytes((255, 216)))
                self.buffer = self.buffer[start:] if start >= 0 else b""
        end = self.buffer.rfind(bytes((255, 217)))
        if end < 0:
            return None
        start = self.buffer.rfind(bytes((255, 216)), 0, end)
        if start < 0 or start >= end:
            self.buffer = self.buffer[end + 2:]
            return None
        jpeg = self.buffer[start:end + 2]
        self.buffer = self.buffer[end + 2:]
        return jpeg


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--show", action="store_true", help="show overlay window")
    parser.add_argument("--follow", action="store_true", help="reuse this model for in-lane highway following")
    args = parser.parse_args()

    # Heavy imports are deferred, so pure packet/risk tests do not require
    # torch, ultralytics or a GPU.
    import cv2
    import torch
    from ultralytics import YOLO

    with NETWORK.open("r", encoding="utf-8") as f:
        network = json.load(f)
    with MISSION.open("r", encoding="utf-8") as f:
        all_missions = json.load(f)
        mission = all_missions["dynamic_pedestrian"]
        follow_cfg = all_missions.get("highway_follow", {})
    in_cfg = network["pedestrian_camera_relay"]
    out_cfg = network["pedestrian_result"]
    follow_out_cfg = network.get("highway_follow_result") if args.follow else None
    if args.follow and (not follow_out_cfg or follow_out_cfg["bind_ip"] != "127.0.0.1"):
        raise RuntimeError("highway_follow_result must be configured on localhost")
    gate_cfg = network["pedestrian_gate"]
    zones = load_dynamic_zones(mission, 4430)
    if any(cfg["bind_ip"] != "127.0.0.1" for cfg in (in_cfg, out_cfg, gate_cfg)):
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
    gate = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    gate.bind(("127.0.0.1", int(gate_cfg["port"])))
    gate.setblocking(False)
    output = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    target = ("127.0.0.1", int(out_cfg["port"]))
    follow_target = (("127.0.0.1", int(follow_out_cfg["port"]))
                     if args.follow else None)
    follow_risk_held = 0.0
    follow_last_seen = float("-inf")
    follow_last_risk = 0.0
    assembler = JpegStreamAssembler()
    frame_number = 0
    depth_cache = None
    prev_bbox_areas = []
    last_report = None
    last_wait_log = 0.0
    last_gate_time = 0.0
    last_gate_wp = None
    previously_enabled = False
    print(f"[PEDESTRIAN ZONE] {zones} -- inference only when inside")
    print(f"[PEDESTRIAN GATE] localhost:{gate_cfg['port']} from main")
    print(f"[PEDESTRIAN] relayed camera 127.0.0.1:{in_cfg['port']}")
    print(f"[PEDESTRIAN] risk decision -> {target}, threshold={threshold:.2f}")
    if args.follow:
        print(f"[HIGHWAY FOLLOW] zones={follow_cfg.get('zones_wp')} result={follow_target}")
        print("[HIGHWAY FOLLOW] one shared front camera, YOLO + MiDaS; lane ROI only")
    print("[CONTROL] No EgoCtrlCmd sender in this process.")
    try:
        while True:
            # Main owns waypoint calculations. The worker may evaluate a
            # pedestrian only inside the designated mission range.
            while True:
                try:
                    raw_gate, _ = gate.recvfrom(512)
                except BlockingIOError:
                    break
                try:
                    payload = json.loads(raw_gate.decode("ascii"))
                    wp = payload.get("waypoint_index")
                    if type(wp) is int and 0 <= wp < 4430:
                        last_gate_wp = wp
                        last_gate_time = time.monotonic()
                except (ValueError, UnicodeError, TypeError, AttributeError):
                    pass
            gate_fresh = (last_gate_wp is not None
                          and time.monotonic() - last_gate_time <= 1.0)
            ped_prearm_wp = max(0, int(mission.get("prewarm_wp", 0)))
            follow_prearm_wp = max(0, int(follow_cfg.get("prewarm_wp", 0)))
            ped_enabled = bool(gate_fresh and (
                in_dynamic_zone(last_gate_wp, zones) or
                any(start-ped_prearm_wp <= last_gate_wp < start
                    for start, end in zones)
            ))
            follow_enabled = bool(args.follow and gate_fresh and (
                in_follow_zone(last_gate_wp, follow_cfg, 4430) or
                any(start-follow_prearm_wp <= last_gate_wp < start
                    for start, end in follow_cfg.get("zones_wp", []))
            ))
            enabled = ped_enabled or follow_enabled
            if enabled != previously_enabled:
                assembler.buffer = b""
                prev_bbox_areas = []
                depth_cache = None
                follow_risk_held = 0.0
                follow_last_seen = float("-inf")
                follow_last_risk = 0.0
                print(
                    f"[PEDESTRIAN/FOLLOW] ZONE {'ON' if enabled else 'OFF'}"
                    f" wp={last_gate_wp}"
                )
            previously_enabled = enabled

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

            # Outside the mission, drop camera packets without inference
            # or a stop request. This also avoids consuming GPU/CPU cycles.
            if not enabled:
                continue
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
            # Highway: keep near-field vehicles, even if their boxes touch
            # the bottom edge. Pedestrian-only path retains the old crop.
            crop_h = int(h * (0.98 if follow_enabled else HOOD_CROP_RATIO))
            roi_frame = frame[:crop_h, :]
            # Avoid spending inference on unrelated classes in highway mode.
            detect_classes = list(FRONT_CLASSES) if ped_enabled else [2, 5, 7]
            results = yolo_model(roi_frame, classes=detect_classes, verbose=False)[0]
            detections = []
            for box in results.boxes:
                cls = int(box.cls[0])
                conf = float(box.conf[0])
                if cls not in FRONT_CLASSES or conf < CONF_THRESHOLD:
                    continue
                x1, y1, x2, y2 = [
                    float(z) for z in box.xyxy[0].cpu().numpy()
                ]
                if not follow_enabled and y2 >= crop_h - 10 and (x2 - x1) > w * 0.5:
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
            max_vehicle_risk = 0.0
            persons = 0
            vehicles = 0
            lane_poly = (np.asarray(LANE_ROI_RATIO, dtype=np.float32)
                         * np.asarray([w, h], dtype=np.float32)).astype(np.int32)
            curr_bbox_areas = []
            for idx, (x1, y1, x2, y2, conf, cls) in enumerate(detections):
                # The supplied trapezoid filters objects by their bounding-box
                # bottom center. Neither risk nor drawing includes adjacent lanes.
                if follow_enabled and (
                    cls not in (2, 5, 7)
                    or not vehicle_overlaps_lane(x1, y1, x2, y2, w, h)
                ):
                    continue
                # Align approach-area history with *filtered* in-lane boxes,
                # not all boxes returned by YOLO.
                idx = len(curr_bbox_areas)
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
                if cls == 0 and ped_enabled:
                    persons += 1
                    max_person_risk = max(max_person_risk, risk)
                if cls in (2, 5, 7) and follow_enabled:
                    vehicles += 1
                    max_vehicle_risk = max(max_vehicle_risk, risk)
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
            if ped_enabled:
                packet = {
                    "status": "STOP_REQUIRED" if stop else "CLEAR",
                    "decision": decision,
                    "risk": round(max_person_risk, 4),
                    "person_count": persons,
                    "frame": frame_number,
                    "waypoint_index": last_gate_wp,
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
            if follow_enabled:
                # Immediate rise, controlled release. Prevent the rapidly
                # alternating 0.1/0.58 scores seen in prior pedestrian logs.
                if vehicles:
                    follow_last_seen = time.monotonic()
                    follow_last_risk = max_vehicle_risk
                risk_with_memory = retained_follow_risk(
                    max_vehicle_risk, follow_last_risk,
                    time.monotonic() - follow_last_seen,
                    hold_sec=float(follow_cfg.get("lost_vehicle_hold_sec", 0.8)),
                )
                follow_risk_held = max(risk_with_memory, 0.80 * follow_risk_held)
                grade, cap = grade_speed_cap(follow_risk_held)
                follow_packet = {
                    "kind": "HIGHWAY_FOLLOW", "waypoint_index": last_gate_wp,
                    "frame": frame_number, "risk": round(follow_risk_held, 4),
                    "vehicles": vehicles,
                }
                output.sendto(json.dumps(
                    follow_packet, separators=(",", ":")).encode("utf-8"),
                    follow_target)
                if frame_number % 15 == 0:
                    print(f"[HIGHWAY FOLLOW] wp={last_gate_wp} vehicles={vehicles}"
                          f" risk={follow_risk_held:.3f} grade={grade} cap={cap:g}km/h")
            if args.show:
                text = (f"FOLLOW wp={last_gate_wp} grade={grade} "
                        f"risk={follow_risk_held:.2f} vehicles={vehicles}"
                        if follow_enabled else
                        f"{decision} | person={persons} risk={max_person_risk:.2f}")
                if follow_enabled:
                    cv2.polylines(frame, [lane_poly], True, (255, 255, 0), 2)
                cv2.putText(frame, text, (20, 35),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.70,
                            (0, 0, 255) if stop and ped_enabled else (0, 255, 0), 2)
                cv2.imshow("MORAI Pedestrian / Highway Follow", frame)
                if cv2.waitKey(1) & 0xff in (ord("q"), 27):
                    break
    except KeyboardInterrupt:
        print("[PEDESTRIAN] Interrupted")
    finally:
        cam.close()
        gate.close()
        output.close()
        if args.show:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
