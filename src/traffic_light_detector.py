#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""MORAI camera: vehicle traffic-light + automatic stop-line detector.

Camera UDP 9291 ->
  - vehicle signal: STOP / GO / GO_LEFT / UNKNOWN
  - stop-line vertical position (0.0~1.0 of image height)

Publishes ASCII UDP to localhost:9300 as:
    SIGNAL|STOPLINE_Y_RATIO|STOPLINE_CONF
Example:
    STOP|0.8124|0.91
If a stop line is not currently detected:
    STOP|-1.0000|0.00

The stop-line target ratio can be calibrated ONCE at a known-good stopping
position (for example the verified waypoint 415). That single camera
calibration is then reused at every intersection; per-intersection waypoint
registration is not required.
"""

import argparse
import json
import os
import socket
import sys
import time
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

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(SCRIPT_DIR)
TRAFFICLIGHT_DIR = os.path.join(SCRIPT_DIR, "trafficlight")
CONFIG_DIR = os.path.join(PROJECT_ROOT, "config")
STOPLINE_CALIBRATION_FILE = os.path.join(CONFIG_DIR, "stopline_calibration.json")

sys.path.insert(0, TRAFFICLIGHT_DIR)
from module import NIA_SEGNet_module

CAMERA_IP = os.getenv("TRAFFIC_CAMERA_BIND_IP", "0.0.0.0")
CAMERA_PORT = int(os.getenv("TRAFFIC_CAMERA_PORT", "9291"))
DECISION_IP = os.getenv("TRAFFIC_DECISION_IP", "127.0.0.1")
DECISION_PORT = int(os.getenv("TRAFFIC_DECISION_PORT", "9300"))
MAX_DGRAM = 65535
WINDOW_NAME = "MORAI Vehicle Signal + Stop Line"

# ---------------- Vehicle traffic-light detector ----------------
CLASS_NAMES = ["red", "yellow", "green", "greenleft", "redleft"]
VEHICLE_ROI = (0.22, 0.02, 0.80, 0.46)
TARGET_SIGNAL_POINT = (0.58, 0.27)
MIN_BOX_WIDTH = 16
MIN_BOX_HEIGHT = 10
MIN_ASPECT_RATIO = 1.70
MAX_ASPECT_RATIO = 6.50
HISTORY_SIZE = 7
MIN_STABLE_VOTES = 5
# Independent stable left-arrow evidence: 5 of the last 7 processed frames.
LEFT_ARROW_MIN_VOTES = 5
STABLE_SIGNAL_HOLD_SEC = float(os.getenv("TRAFFIC_SIGNAL_HOLD_SEC", "0.60"))
YOLO_IMGSZ = int(os.getenv("TRAFFIC_YOLO_IMGSZ", "640"))

# ---------------- Universal stop-reference detector ----------------
# The camera sees a zebra crossing as several long white "fingers" extending
# in the driving direction. The stable landmark is the vehicle-side ends of
# those stripes, not an arbitrary horizontal Hough line.
STOPLINE_ROI = (0.16, 0.46, 0.84, 0.88)
STOPLINE_HISTORY_SIZE = 7
STOPLINE_MIN_STABLE = 3
STOPLINE_MAX_JITTER_RATIO = 0.025

WHITE_MIN_VALUE = 160
WHITE_MAX_SAT = 115

# Zebra stripe component / alignment requirements.
STRIPE_X_MIN = 0.18
STRIPE_X_MAX = 0.82
STRIPE_MIN_AREA = 140
STRIPE_MIN_W = 10
STRIPE_MAX_W = 180
STRIPE_MIN_H = 8
STRIPE_MAX_H = 190
STRIPE_MIN_FILL = 0.28

# Several stripe ends must line up across the road.
STRIPE_END_GROUP_TOL_PX = 22
STRIPE_MIN_COUNT = 4
STRIPE_MIN_X_SPAN_RATIO = 0.24
STRIPE_MIN_TOTAL_WIDTH_RATIO = 0.11
STRIPE_MAX_EDGE_ANGLE_DEG = 16.0

STOPREF_FEATURE_VERSION = "zebra_vehicle_edge_v2"

DEFAULT_STOPLINE_TARGET_RATIO = 0.79
CALIBRATION_SAMPLES = 30


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


def load_stopline_target_ratio():
    try:
        with open(STOPLINE_CALIBRATION_FILE, "r", encoding="utf-8") as f:
            data = json.load(f)
        ratio = float(data["target_y_ratio"])
        feature = str(data.get("feature_version", ""))
        if feature == STOPREF_FEATURE_VERSION and 0.50 <= ratio <= 0.95:
            return ratio, True
    except Exception:
        pass
    return DEFAULT_STOPLINE_TARGET_RATIO, False


def save_stopline_target_ratio(ratio, frame_width, frame_height):
    os.makedirs(CONFIG_DIR, exist_ok=True)
    payload = {
        "target_y_ratio": round(float(ratio), 6),
        "frame_width": int(frame_width),
        "frame_height": int(frame_height),
        "feature_version": STOPREF_FEATURE_VERSION,
        "description": "One-time zebra vehicle-side edge camera calibration at verified stopping position",
        "saved_at_unix": time.time(),
    }
    with open(STOPLINE_CALIBRATION_FILE, "w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2, ensure_ascii=False)


def _zebra_vehicle_edge(frame, white_mask, roi_box):
    """Detect the vehicle-side edge of a zebra crossing.

    Each zebra stripe is a white elongated component.  Near a crossing, the
    vehicle-side ends of several stripes line up across the road.  We group
    those bottom endpoints, fit one line through them, and evaluate the line
    at image center.  Requiring 4+ separate stripes rejects lane dividers,
    arrows and most isolated markings.
    """
    h, w = frame.shape[:2]
    x1, y1, x2, y2 = roi_box

    contours, _ = cv2.findContours(
        white_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )

    parts = []
    for contour in contours:
        x, y, cw, ch = cv2.boundingRect(contour)
        area = float(cv2.contourArea(contour))
        box_area = float(max(cw * ch, 1))
        fill = area / box_area

        gx = x + x1
        gy = y + y1
        cx = gx + 0.5 * cw
        bottom = gy + ch

        if area < STRIPE_MIN_AREA:
            continue
        if not (STRIPE_MIN_W <= cw <= STRIPE_MAX_W):
            continue
        if not (STRIPE_MIN_H <= ch <= STRIPE_MAX_H):
            continue
        if fill < STRIPE_MIN_FILL:
            continue
        if not (STRIPE_X_MIN * w <= cx <= STRIPE_X_MAX * w):
            continue

        parts.append({
            "x": gx, "y": gy, "w": cw, "h": ch,
            "cx": float(cx), "bottom": float(bottom),
            "area": area, "fill": fill,
        })

    if len(parts) < STRIPE_MIN_COUNT:
        return None

    # Cluster components by their vehicle-side (bottom) endpoint.
    parts.sort(key=lambda p: p["bottom"])
    groups = []
    for part in parts:
        best = None
        best_d = float("inf")
        for g in groups:
            d = abs(part["bottom"] - g["center"])
            if d <= STRIPE_END_GROUP_TOL_PX and d < best_d:
                best = g
                best_d = d
        if best is None:
            groups.append({"center": part["bottom"], "parts": [part]})
        else:
            best["parts"].append(part)
            best["center"] = float(np.median(
                [p["bottom"] for p in best["parts"]]
            ))

    candidates = []
    for g in groups:
        ps = g["parts"]
        if len(ps) < STRIPE_MIN_COUNT:
            continue

        left = min(p["x"] for p in ps)
        right = max(p["x"] + p["w"] for p in ps)
        x_span_ratio = (right - left) / float(w)
        total_width_ratio = sum(p["w"] for p in ps) / float(w)

        if x_span_ratio < STRIPE_MIN_X_SPAN_RATIO:
            continue
        if total_width_ratio < STRIPE_MIN_TOTAL_WIDTH_RATIO:
            continue

        xs = np.array([p["cx"] for p in ps], dtype=float)
        ys = np.array([p["bottom"] for p in ps], dtype=float)

        # Fit the crossing edge so slightly slanted / curved-camera views work.
        if len(ps) >= 2 and float(np.ptp(xs)) > 1.0:
            slope, intercept = np.polyfit(xs, ys, 1)
        else:
            slope = 0.0
            intercept = float(np.median(ys))

        angle = abs(np.degrees(np.arctan(float(slope))))
        if angle > STRIPE_MAX_EDGE_ANGLE_DEG:
            continue

        center_y = float(slope * (0.5 * w) + intercept)
        y_ratio = center_y / float(h)

        # Reject implausible landmarks outside the road/near-field zone.
        if not (0.50 <= y_ratio <= 0.90):
            continue

        residual = float(np.median(np.abs(
            ys - (slope * xs + intercept)
        )))
        alignment_score = max(
            0.0, 1.0 - residual / float(STRIPE_END_GROUP_TOL_PX)
        )
        confidence = min(
            1.0,
            0.36 * min(len(ps) / 6.0, 1.0)
            + 0.24 * min(x_span_ratio / 0.45, 1.0)
            + 0.18 * min(total_width_ratio / 0.22, 1.0)
            + 0.22 * alignment_score
        )

        candidates.append({
            "x1": int(left),
            "x2": int(right),
            "y1": int(round(slope * left + intercept)),
            "y2": int(round(slope * right + intercept)),
            "y_ratio": y_ratio,
            "confidence": confidence,
            "kind": "zebra_vehicle_edge",
            "parts": ps,
            "slope": float(slope),
            "intercept": float(intercept),
        })

    if not candidates:
        return None

    # The nearest valid zebra edge is the one with greatest image y.
    return max(candidates, key=lambda c: (c["y_ratio"], c["confidence"]))


def detect_stopline(frame):
    """Return the vehicle-side edge of a zebra crossing or None."""
    h, w = frame.shape[:2]
    x1 = int(STOPLINE_ROI[0] * w)
    y1 = int(STOPLINE_ROI[1] * h)
    x2 = int(STOPLINE_ROI[2] * w)
    y2 = int(STOPLINE_ROI[3] * h)

    roi = frame[y1:y2, x1:x2]
    if roi.size == 0:
        return None, (x1, y1, x2, y2), None

    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    white_mask = cv2.inRange(
        hsv,
        np.array([0, 0, WHITE_MIN_VALUE], dtype=np.uint8),
        np.array([180, WHITE_MAX_SAT, 255], dtype=np.uint8),
    )

    # Preserve the large zebra blocks while removing isolated noise.
    white_mask = cv2.medianBlur(white_mask, 5)
    white_mask = cv2.morphologyEx(
        white_mask, cv2.MORPH_OPEN,
        np.ones((3, 3), np.uint8), iterations=1
    )
    white_mask = cv2.morphologyEx(
        white_mask, cv2.MORPH_CLOSE,
        np.ones((5, 5), np.uint8), iterations=1
    )

    candidate = _zebra_vehicle_edge(
        frame, white_mask, (x1, y1, x2, y2)
    )
    return candidate, (x1, y1, x2, y2), white_mask


def stable_stopline_measurement(history):
    if len(history) < STOPLINE_MIN_STABLE:
        return None
    values = np.array(history, dtype=float)
    median = float(np.median(values))
    recent = values[-STOPLINE_MIN_STABLE:]
    if float(np.max(recent) - np.min(recent)) > STOPLINE_MAX_JITTER_RATIO:
        return None
    return median


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--calibrate-stopline",
        action="store_true",
        help="verified stopping position에서 정지선 화면 y비율을 1회 보정/저장",
    )
    args = parser.parse_args()

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"[INFO] device={device}")
    print(f"[INFO] YOLO mode=vehicle-ROI only, imgsz={YOLO_IMGSZ}")
    classifier, detector = load_models(device)
    print("[OK] traffic-light models loaded")

    stop_target_ratio, calibrated = load_stopline_target_ratio()
    if calibrated:
        print(f"[STOP REF] calibrated target y-ratio={stop_target_ratio:.4f}")
    else:
        print(f"[STOP REF] calibration missing/old; default target={stop_target_ratio:.4f}")
        print("[STOP REF] run stopline-calibrate at verified index 415 position")

    if args.calibrate_stopline:
        print("[CALIBRATE] Keep the vehicle stationary at the verified index-415 stop position.")
        print(f"[CALIBRATE] Collecting {CALIBRATION_SAMPLES} stable zebra-near-edge samples...")

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
    signal_history = deque(maxlen=HISTORY_SIZE)
    left_arrow_history = deque(maxlen=HISTORY_SIZE)
    last_stable_label = "unknown"
    last_stable_time = 0.0
    stopline_history = deque(maxlen=STOPLINE_HISTORY_SIZE)
    calibration_values = []
    last_reported = None
    last_stopline_report = None
    first_frame_reported = False

    # Create the debug window explicitly on Ubuntu/Linux.  Calling imshow()
    # alone can leave the auto-created Qt window hidden or placed awkwardly.
    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WINDOW_NAME, 960, 540)
    cv2.moveWindow(WINDOW_NAME, 60, 60)
    print(f"[GUI] debug window ready: {WINDOW_NAME}")

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
                # Camera frame has not arrived yet. Avoid cv2.waitKey here because
                # some Windows/OpenCV builds error before any GUI window exists.
                time.sleep(0.01)
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
            if not first_frame_reported:
                print(f"[CAMERA] decoded frame {width}x{height}")
                first_frame_reported = True

            # ========================================================
            # 1) Vehicle signal detection
            # ========================================================
            roi_left = int(VEHICLE_ROI[0] * width)
            roi_top = int(VEHICLE_ROI[1] * height)
            roi_right = int(VEHICLE_ROI[2] * width)
            roi_bottom = int(VEHICLE_ROI[3] * height)
            candidates = []
            raw_signal_boxes = []

            # YOLO is the CPU bottleneck.  Run it only on the already-defined
            # vehicle-signal ROI, then map detections back to full-frame coords.
            # This preserves the classifier/voting logic while reducing inference
            # area substantially compared with full-frame imgsz=1280.
            vehicle_roi_frame = frame[
                roi_top:roi_bottom,
                roi_left:roi_right,
            ]
            inference_started = time.time()
            result = detector(
                vehicle_roi_frame,
                imgsz=YOLO_IMGSZ,
                conf=0.25,
                verbose=False,
            )[0]
            inference_ms = (time.time() - inference_started) * 1000.0

            for box in result.boxes:
                if int(box.cls[0].item()) != 9:
                    continue
                confidence = float(box.conf[0].item())
                rxmin, rymin, rxmax, rymax = map(int, box.xyxy[0].tolist())
                xmin = rxmin + roi_left
                ymin = rymin + roi_top
                xmax = rxmax + roi_left
                ymax = rymax + roi_top
                box_width, box_height = xmax - xmin, ymax - ymin
                center_x, center_y = (xmin + xmax) // 2, (ymin + ymax) // 2
                aspect = box_width / max(box_height, 1)

                reject_reason = None
                if box_width < MIN_BOX_WIDTH or box_height < MIN_BOX_HEIGHT:
                    reject_reason = "SIZE"
                elif aspect < MIN_ASPECT_RATIO or aspect > MAX_ASPECT_RATIO:
                    reject_reason = "ASPECT"
                elif not (
                    roi_left <= center_x <= roi_right
                    and roi_top <= center_y <= roi_bottom
                ):
                    reject_reason = "ROI"

                raw_signal_boxes.append({
                    "xmin": xmin, "ymin": ymin, "xmax": xmax, "ymax": ymax,
                    "w": box_width, "h": box_height,
                    "aspect": aspect, "conf": confidence,
                    "reject": reject_reason,
                })

                if reject_reason is not None:
                    continue

                cx1 = max(0, int(xmin - box_width * 0.15))
                cy1 = max(0, int(ymin - box_height * 0.15))
                cx2 = min(width, int(xmax + box_width * 0.15))
                cy2 = min(height, int(ymax + box_height * 0.15))
                crop = frame[cy1:cy2, cx1:cx2]
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

            # Separate per-frame left-arrow evidence from the top-scoring box.
            # A red lamp can score higher in the same traffic signal image.
            # The NIA classifier has two left-bearing classes:
            # greenleft (green arrow) and redleft (red + left-arrow lamp).
            # redleft remains STOP as the ordinary signal; independent
            # arrow evidence is gated to configured left-only intersections
            # by MissionManager. Require >=5 / 7 processed frames to avoid
            # releasing on a single misclassification.
            left_arrow_history.append(any(
                item["label"] in ("greenleft", "redleft")
                and item["cls_conf"] >= 0.60
                for item in candidates
            ))
            stable_left_arrow = (
                sum(left_arrow_history) >= LEFT_ARROW_MIN_VOTES
            )

            selected = None
            votes = 0
            now_signal = time.time()
            voted_label = "unknown"

            if candidates:
                selected = max(
                    candidates,
                    key=lambda item: candidate_score(item, width, height),
                )
                signal_history.append(selected["label"])
                voted_label, votes = Counter(signal_history).most_common(1)[0]

                if votes >= MIN_STABLE_VOTES:
                    stable_label = voted_label
                    last_stable_label = stable_label
                    last_stable_time = now_signal
                elif (
                    last_stable_label != "unknown"
                    and now_signal - last_stable_time <= STABLE_SIGNAL_HOLD_SEC
                ):
                    # Short YOLO dropouts or a few transition frames should not
                    # make a previously stable signal flicker to UNKNOWN.
                    stable_label = last_stable_label
                else:
                    stable_label = "unknown"

            else:
                # Keep the original gradual history decay so a stale class does
                # not dominate once detections resume.
                if signal_history:
                    signal_history.popleft()

                if signal_history:
                    voted_label, votes = Counter(signal_history).most_common(1)[0]

                if (
                    last_stable_label != "unknown"
                    and now_signal - last_stable_time <= STABLE_SIGNAL_HOLD_SEC
                ):
                    stable_label = last_stable_label
                else:
                    stable_label = "unknown"

            decision = driving_decision(stable_label)

            # ========================================================
            # 2) Stop-line detection
            # ========================================================
            stop_candidate, stop_roi, _ = detect_stopline(frame)
            if stop_candidate is not None:
                stopline_history.append(stop_candidate["y_ratio"])
                stable_y_ratio = stable_stopline_measurement(stopline_history)
                stop_conf = stop_candidate["confidence"]
            else:
                stopline_history.clear()
                stable_y_ratio = None
                stop_conf = 0.0

            # Calibration is independent of current traffic-light color,
            # but ONLY the zebra vehicle-side edge may define the target.
            if (
                args.calibrate_stopline
                and stable_y_ratio is not None
                and stop_candidate is not None
                and stop_candidate.get("kind") == "zebra_vehicle_edge"
            ):
                calibration_values.append(stable_y_ratio)
                print(
                    f"\r[CALIBRATE] {len(calibration_values):02d}/{CALIBRATION_SAMPLES} "
                    f"y={stable_y_ratio:.4f}      ",
                    end="",
                    flush=True,
                )
                if len(calibration_values) >= CALIBRATION_SAMPLES:
                    final_ratio = float(np.median(calibration_values))
                    save_stopline_target_ratio(final_ratio, width, height)
                    print()
                    print(f"[CALIBRATE OK] target y-ratio={final_ratio:.4f}")
                    print(f"[SAVED] {STOPLINE_CALIBRATION_FILE}")
                    print("You can now close this window and run auto-intersection-drive.")
                    # Keep the debug window visible; no more sample accumulation.
                    args.calibrate_stopline = False
                    stop_target_ratio = final_ratio

            # Send one compact message every processed frame.
            send_ratio = stable_y_ratio if stable_y_ratio is not None else -1.0
            # Fourth field is independent left-arrow evidence, NOT a global GO.
            # Only an explicitly mapped left-turn intersection may prioritize it.
            message = (
                f"{decision}|{send_ratio:.4f}|{stop_conf:.2f}|"
                f"{int(stable_left_arrow)}"
            )
            decision_sock.sendto(message.encode("ascii"), decision_target)

            if (stable_label, decision, stable_left_arrow) != last_reported:
                print(
                    f"[VEHICLE SIGNAL] {stable_label} -> {decision} "
                    f"({votes}/{len(signal_history)} votes), "
                    f"LEFT_ARROW={int(stable_left_arrow)} "
                    f"({sum(left_arrow_history)}/{len(left_arrow_history)})"
                )
                last_reported = (stable_label, decision, stable_left_arrow)

            report_bucket = None if stable_y_ratio is None else round(stable_y_ratio, 2)
            if report_bucket != last_stopline_report:
                if stable_y_ratio is None:
                    print("[STOP REF] not detected")
                else:
                    delta = stable_y_ratio - stop_target_ratio
                    print(
                        f"[STOP REF] y={stable_y_ratio:.4f} target={stop_target_ratio:.4f} "
                        f"delta={delta:+.4f}"
                    )
                last_stopline_report = report_bucket

            # ========================================================
            # 3) Debug overlay
            # ========================================================
            # Raw COCO class-9 detections are drawn before our vehicle-signal
            # filters so we can distinguish YOLO misses from filter rejection.
            for raw_box in raw_signal_boxes:
                if raw_box["reject"] is None:
                    continue
                rx1, ry1 = raw_box["xmin"], raw_box["ymin"]
                rx2, ry2 = raw_box["xmax"], raw_box["ymax"]
                cv2.rectangle(frame, (rx1, ry1), (rx2, ry2), (0, 255, 255), 2)
                raw_text = (
                    f"RAW TL {raw_box['conf']:.2f} "
                    f"{raw_box['w']}x{raw_box['h']} "
                    f"{raw_box['reject']}"
                )
                cv2.putText(
                    frame, raw_text,
                    (rx1, max(18, ry1 - 5)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1
                )

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
            cv2.putText(
                frame,
                (
                    f"YOLO TL raw:{len(raw_signal_boxes)} "
                    f"accepted:{len(candidates)} {inference_ms:.0f}ms"
                ),
                (roi_left + 5, max(20, roi_top + 20)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 180, 0), 1
            )

            sx1, sy1, sx2, sy2 = stop_roi
            cv2.rectangle(frame, (sx1, sy1), (sx2, sy2), (255, 0, 255), 1)
            target_y_px = int(stop_target_ratio * height)
            cv2.line(frame, (sx1, target_y_px), (sx2, target_y_px), (0, 255, 255), 2)
            cv2.putText(frame, "STOP TARGET", (sx1 + 8, max(25, target_y_px - 8)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)

            if stop_candidate is not None:
                c = stop_candidate

                # Show detected zebra stripe components.
                for p in c.get("parts", []):
                    cv2.rectangle(
                        frame,
                        (int(p["x"]), int(p["y"])),
                        (int(p["x"] + p["w"]), int(p["y"] + p["h"])),
                        (80, 180, 80), 1
                    )

                cv2.line(
                    frame,
                    (c["x1"], c["y1"]),
                    (c["x2"], c["y2"]),
                    (0, 255, 0), 4
                )
                line_text = "STOP REF (ZEBRA NEAR EDGE)"
                if stable_y_ratio is not None:
                    line_text += f" y={stable_y_ratio:.3f}"
                cv2.putText(
                    frame, line_text,
                    (max(20, c["x1"]), max(30, min(c["y1"], c["y2"]) - 12)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2
                )

            cv2.putText(frame, f"Target y-ratio: {stop_target_ratio:.3f}",
                        (30, height - 25), cv2.FONT_HERSHEY_SIMPLEX, 0.65,
                        (0, 255, 255), 2)

            cv2.imshow(WINDOW_NAME, frame)
            if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                break

    except KeyboardInterrupt:
        pass
    finally:
        decision_sock.sendto(b"UNKNOWN|-1.0000|0.00", decision_target)
        camera_sock.close()
        decision_sock.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
