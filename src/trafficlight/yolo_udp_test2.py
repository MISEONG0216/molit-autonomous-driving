import os
import sys
import socket
from collections import Counter, deque
import cv2
import numpy as np
import torch
import torch.nn.functional as F
from ultralytics import YOLO

# PyTorch Lightning EarlyStopping 肄쒕갚 ?덉쟾 媛앹껜 ?깅줉
try:
    from pytorch_lightning.callbacks.early_stopping import EarlyStopping
    torch.serialization.add_safe_globals([EarlyStopping])
except Exception:
    pass

from module import NIA_SEGNet_module

# ----------------------------------------------------
# ?ㅼ젙 諛?珥덇린??
# ----------------------------------------------------
CKPT_NAME = "epoch=5-step=113.ckpt"  
UDP_IP = "127.0.0.1"
UDP_PORT = 9291  # MORAI 移대찓???쇱꽌??Destination Port? ?쇱튂?쒗궗 寃?                  
MAX_DGRAM = 65535

# module.py???먮옒 ?쇰꺼 ?쒖꽌 ?ы솗??(0: red, 1: yellow, 2: green, 3: greenleft, 4: redleft)
CLASS_NAMES = ["red", "yellow", "green", "greenleft", "redleft"]

# 李⑤웾 吏꾪뻾 ?좏샇 ?좏깮 ?ㅼ젙(1280x720 湲곗??쇰줈 ?뺢퇋?붾맂 媛?
# MORAI 移대찓?쇨? 怨좎젙?섏뼱 ?덉쑝硫?二쇳뻾 李⑤줈 ?좏샇媛 ?ㅼ뼱?ㅻ뒗 踰붿쐞留?醫곹?
# 蹂댄뻾???좏샇媛 ????좏샇濡??좏깮??媛?μ꽦????텧 ???덈떎.
VEHICLE_ROI = (0.20, 0.02, 0.82, 0.58)  # left, top, right, bottom
TARGET_SIGNAL_POINT = (0.58, 0.27)       # 二쇳뻾 李⑤줈 ????좏샇???덉긽 ?꾩튂
MIN_BOX_WIDTH = 16
MIN_BOX_HEIGHT = 10
MIN_ASPECT_RATIO = 1.35                  # 李⑤웾??媛濡??좏샇???곗꽑
HISTORY_SIZE = 7
MIN_STABLE_VOTES = 4

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
print(f"[INFO] ?곗궛 ?μ튂: {device}")

current_dir = os.path.dirname(os.path.abspath(__file__))

# 1. ?좏샇??遺꾨쪟 紐⑤뜽 濡쒕뱶
ckpt_path = os.path.join(current_dir, CKPT_NAME)
try:
    # PyTorch 2.6+??torch.load??湲곕낯媛믪씠 weights_only=True濡?諛붾뚯뿀??
    # ??泥댄겕?ъ씤?몃뒗 援ы삎 PyTorch Lightning ?꾩껜 泥댄겕?ъ씤?몄씠誘濡?
    # 異쒖쿂媛 ?뺤씤???숇큺 ?뚯씪???쒗빐 湲곗〈 諛⑹떇(weights_only=False)?쇰줈 ?쎈뒗??
    original_torch_load = torch.load

    def trusted_checkpoint_load(*args, **kwargs):
        kwargs["weights_only"] = False
        return original_torch_load(*args, **kwargs)

    torch.load = trusted_checkpoint_load
    try:
        classifier_model = NIA_SEGNet_module.load_from_checkpoint(ckpt_path)
    finally:
        torch.load = original_torch_load

    classifier_model.to(device)
    classifier_model.eval()
    print(f"[SUCCESS] ?좏샇??遺꾨쪟 紐⑤뜽({CKPT_NAME}) 濡쒕뱶 ?깃났!")
except Exception as e:
    print(f"[ERROR] 遺꾨쪟 紐⑤뜽 濡쒕뱶 ?ㅽ뙣: {e}")
    sys.exit(1)

# 2. YOLO 媛앹껜 ?먯? 紐⑤뜽 濡쒕뱶
detector_model = YOLO(os.path.join(current_dir, "yolov8n.pt"))
TRAFFIC_LIGHT_CLASS_ID = 9           


def preprocess_crop(crop_bgr):
    """?숈뒿 ?곗씠?곗뀑怨??꾩쟾??媛숈? ?꾩쿂由щ줈 遺꾨쪟 ?낅젰??留뚮뱺??

    datasets.py???숈뒿 ?낅젰? RGB, (?믪씠 20, ?덈퉬 70), [0, 1] 踰붿쐞?대ŉ
    ImageNet ?뺢퇋?붾? ?ъ슜?섏? ?딅뒗?? ??洹쒖튃???ㅻⅤ寃??곸슜?섎㈃ 紐⑤뜽??    red 媛숈? ???대옒?ㅼ뿉 ?명뼢?????덈떎.
    """
    crop_rgb = cv2.cvtColor(crop_bgr, cv2.COLOR_BGR2RGB)

    # cv2.resize ?몄옄??(?덈퉬, ?믪씠). ?숈뒿 ?먯꽌 shape? (3, 20, 70).
    crop_resized = cv2.resize(crop_rgb, (70, 20), interpolation=cv2.INTER_AREA)

    # [0, 255] -> [0.0, 1.0] ?먯꽌 蹂??(H, W, C) -> (C, H, W)
    tensor = torch.from_numpy(crop_resized).permute(2, 0, 1).float() / 255.0

    return tensor.unsqueeze(0).to(device)


def candidate_score(candidate, w_img, h_img):
    """二쇳뻾 李⑤줈??李⑤웾 ?좏샇??媛?μ꽦???대━?ㅽ떛 ?먯닔濡?怨꾩궛?쒕떎."""
    cx = ((candidate["xmin"] + candidate["xmax"]) * 0.5) / w_img
    cy = ((candidate["ymin"] + candidate["ymax"]) * 0.5) / h_img
    tx, ty = TARGET_SIGNAL_POINT
    distance = ((cx - tx) ** 2 + (cy - ty) ** 2) ** 0.5
    position_score = max(0.0, 1.0 - distance / 0.55)
    aspect_score = min(candidate["aspect"] / 3.0, 1.0)
    return (
        0.30 * candidate["det_conf"]
        + 0.30 * candidate["cls_conf"]
        + 0.20 * position_score
        + 0.20 * aspect_score
    )


def driving_decision(label):
    """???李⑤웾 ?좏샇瑜?蹂댁닔?곸씤 二쇳뻾 ?먮떒媛믪쑝濡?蹂?섑븳??"""
    if label == "green":
        return "GO"
    if label == "greenleft":
        return "GO_LEFT"
    if label in ("red", "redleft", "yellow"):
        return "STOP"
    return "UNKNOWN"


def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1024 * 1024 * 10)
    sock.bind((UDP_IP, UDP_PORT))

    # Send traffic-light decision to Pure Pursuit
    signal_sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    signal_target = ("127.0.0.1", 9300)
    
    # ?뚮툝濡쒗궧 紐⑤뱶 ?ㅼ젙 (?ㅻ옒??踰꾪띁 鍮꾩슦湲곗슜)
    sock.setblocking(False)
    
    print(f"[INFO] MORAI UDP 移대찓???섏떊 ?湲?以?.. ({UDP_IP}:{UDP_PORT})")

    buffer = b""
    signal_history = deque(maxlen=HISTORY_SIZE)
    last_reported = None

    while True:
        try:
            # 1. ?뚯폆 踰꾪띁???볦뿬 ?덈뒗 諛由??⑦궥?ㅼ쓣 ?쎌뼱??踰꾨━怨? 媛??留덉?留?理쒖떊) ?⑦궥留??섏쭛
            packets = []
            while True:
                try:
                    packet, _ = sock.recvfrom(MAX_DGRAM)
                    packets.append(packet)
                except BlockingIOError:
                    break

            if not packets:
                # ?쎌쓣 ???덈뒗 ?⑦궥???꾩쭅 ?놁쑝硫??좎떆 ?湲?
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
                continue

            # ?쎌뼱???⑦궥?ㅼ쓣 踰꾪띁???댁뼱 遺숈엫
            for pkt in packets:
                buffer += pkt

            # 踰꾪띁媛 ?덈Т ?щ㈃ ?ㅻ옒???곗씠?곗씠誘濡?由ъ뀑
            if len(buffer) > 2 * 1024 * 1024:
                buffer = b""
                continue

            # 2. 踰꾪띁?먯꽌 '媛??理쒓렐???꾩꽦?? JPEG留?異붿텧?쒕떎.
            # YOLO 異붾줎 以??щ윭 ?꾨젅?꾩씠 ?볦씠嫄곕굹 UDP ?⑦궥 ?섎굹媛 鍮좎?硫?
            # ?ㅻ옒??SOI? ?ㅼ쓬 ?꾨젅?꾩쓽 EOI媛 ?⑹퀜???붾㈃ ?쇰?媛 源⑥쭏 ???덈떎.
            # 留덉?留?EOI? 洹?諛붾줈 ?욎쓽 SOI瑜?吏앹??쇰㈃ ?먯긽?????꾨젅?꾩? 踰꾨━怨?            # 吏?곕룄 以꾩씪 ???덈떎.
            end_idx = buffer.rfind(b'\xff\xd9')
            start_idx = buffer.rfind(b'\xff\xd8', 0, end_idx) if end_idx != -1 else -1

            if start_idx != -1 and end_idx != -1 and end_idx > start_idx:
                jpg_data = buffer[start_idx : end_idx + 2]
                buffer = buffer[end_idx + 2:]  # 理쒖떊 JPEG ?댄썑??誘몄셿???곗씠?곕쭔 蹂닿?

                np_arr = np.frombuffer(jpg_data, dtype=np.uint8)
                frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

                # ?꾨젅?꾩씠 ?뺤긽?곸쑝濡??붿퐫?⑸릺?덉쓣 ?뚮쭔 泥섎━ 吏꾪뻾
                if frame is not None and frame.size > 0:
                    h_img, w_img, _ = frame.shape

                    # YOLO 異붾줎 (怨좏빐?곷룄 1280, Conf 0.25)
                    results = detector_model(frame, imgsz=1280, conf=0.25, verbose=False)[0]

                    candidates = []
                    roi_left = int(VEHICLE_ROI[0] * w_img)
                    roi_top = int(VEHICLE_ROI[1] * h_img)
                    roi_right = int(VEHICLE_ROI[2] * w_img)
                    roi_bottom = int(VEHICLE_ROI[3] * h_img)

                    for box in results.boxes:
                        cls_id = int(box.cls[0].item())
                        conf = float(box.conf[0].item())

                        if cls_id == TRAFFIC_LIGHT_CLASS_ID:
                            xmin, ymin, xmax, ymax = map(int, box.xyxy[0].tolist())
                            bw = xmax - xmin
                            bh = ymax - ymin
                            cx = (xmin + xmax) // 2
                            cy = (ymin + ymax) // 2
                            aspect = bw / max(bh, 1)

                            # 蹂댄뻾???좏샇? ?묒? ?먭굅由??ㅺ?異쒖쓣 1李??쒖쇅?쒕떎.
                            if bw < MIN_BOX_WIDTH or bh < MIN_BOX_HEIGHT:
                                continue
                            if aspect < MIN_ASPECT_RATIO:
                                continue
                            if not (roi_left <= cx <= roi_right and roi_top <= cy <= roi_bottom):
                                continue

                            # Bounding Box Padding (15%)
                            xmin_pad = max(0, int(xmin - bw * 0.15))
                            ymin_pad = max(0, int(ymin - bh * 0.15))
                            xmax_pad = min(w_img, int(xmax + bw * 0.15))
                            ymax_pad = min(h_img, int(ymax + bh * 0.15))

                            crop_img = frame[ymin_pad:ymax_pad, xmin_pad:xmax_pad]

                            if crop_img.size != 0:
                                input_tensor = preprocess_crop(crop_img)

                                with torch.no_grad():
                                    outputs = classifier_model(input_tensor)
                                    probs = F.softmax(outputs, dim=1)[0]
                                    
                                    top_prob, pred_idx = torch.max(probs, dim=0)
                                    p1 = top_prob.item()
                                    idx1 = pred_idx.item()

                                    predicted_label = CLASS_NAMES[idx1]

                                candidates.append({
                                    "xmin": xmin, "ymin": ymin,
                                    "xmax": xmax, "ymax": ymax,
                                    "aspect": aspect,
                                    "det_conf": conf,
                                    "cls_conf": p1,
                                    "label": predicted_label,
                                })

                    # ?꾨낫媛 ?щ윭 媛쒖뿬??二쇳뻾 ?먮떒?먮뒗 ???李⑤웾 ?좏샇 ?섎굹留??ъ슜?쒕떎.
                    if candidates:
                        selected = max(candidates, key=lambda c: candidate_score(c, w_img, h_img))
                        signal_history.append(selected["label"])
                        counts = Counter(signal_history)
                        stable_label, votes = counts.most_common(1)[0]
                        if votes < MIN_STABLE_VOTES:
                            stable_label = "unknown"

                        decision = driving_decision(stable_label)
                        signal_sender.sendto(
                            decision.encode("ascii"),
                            signal_target
                        )
                        if stable_label != last_reported:
                            print(f"[VEHICLE SIGNAL] {stable_label} -> {decision} ({votes}/{len(signal_history)} votes)")
                            last_reported = stable_label

                        if decision in ("GO", "GO_LEFT"):
                            box_color = (0, 255, 0)
                        elif stable_label == "yellow":
                            box_color = (0, 255, 255)
                        else:
                            box_color = (0, 0, 255)

                        cv2.rectangle(
                            frame,
                            (selected["xmin"], selected["ymin"]),
                            (selected["xmax"], selected["ymax"]),
                            box_color,
                            3,
                        )
                        cv2.putText(
                            frame,
                            f"VEHICLE: {stable_label} / {decision}",
                            (selected["xmin"], max(25, selected["ymin"] - 12)),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.65,
                            box_color,
                            2,
                        )
                    else:
                        # ?좏샇瑜??볦낀?????댁쟾 GO瑜??좎??섏? ?딅룄濡?利됱떆 UNKNOWN 泥섎━.
                        signal_history.clear()
                        signal_history.append("unknown")
                        signal_sender.sendto(
                            b"UNKNOWN",
                            signal_target
                        )
                        if last_reported != "unknown":
                            print("[VEHICLE SIGNAL] unknown -> UNKNOWN (no candidate)")
                            last_reported = "unknown"
                        cv2.putText(
                            frame,
                            "VEHICLE: unknown / UNKNOWN",
                            (30, 45),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.8,
                            (0, 0, 255),
                            2,
                        )

                    # ?ㅼ젣 ?곸슜?섎뒗 李⑤웾 ?좏샇 ?먯깋 ?곸뿭???붾㈃???쒖떆?쒕떎.
                    cv2.rectangle(frame, (roi_left, roi_top), (roi_right, roi_bottom), (255, 180, 0), 1)

                    cv2.imshow("MORAI Dynamic Traffic Light Detection", frame)

        except Exception as e:
            # ?ㅻ쪟瑜??꾩쟾???④린硫??ы듃/?붿퐫??紐⑤뜽 臾몄젣瑜?援щ텇?????녿떎.
            print(f"[WARN] ?꾨젅??泥섎━ ?ㅻ쪟: {type(e).__name__}: {e}")
            buffer = b""

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()
    sock.close()


if __name__ == '__main__':
    main()

