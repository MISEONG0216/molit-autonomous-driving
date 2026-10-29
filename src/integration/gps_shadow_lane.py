"""Adaptation of user's MORAI HSV+Canny + BEV + EMA lane-center script.

Lane-only steering request; no UDP bindings, global path, or EgoCtrlCmd.
The original morai_lanenet_viz.py does NOT run a TensorFlow LaneNet model.
"""
import json
import math

ALPHA = 0.25


def parse_lane_result(raw):
    invalid = {"valid": False, "steering": 0.0, "confidence": 0.0,
               "frame": -1, "center_error": 0.0}
    if not raw or len(raw) > 2048:
        return invalid
    try:
        p = json.loads(raw.decode("utf-8"))
        if p.get("kind") != "LANE_CENTER" or type(p.get("valid")) is not bool:
            return invalid
        if type(p.get("frame")) is not int or p["frame"] < 1:
            return invalid
        values = tuple(float(p[k]) for k in ("steering", "confidence", "center_error"))
        if not all(map(math.isfinite, values)):
            return invalid
        if not (abs(values[0]) <= 1.0 and 0 <= values[1] <= 1
                and abs(values[2]) <= 2.0):
            return invalid
        return {"valid": p["valid"], "steering": values[0],
                "confidence": values[1], "center_error": values[2],
                "frame": p["frame"]}
    except (ValueError, TypeError, KeyError, UnicodeError, AttributeError):
        return invalid


class LaneCenterDetector:
    def __init__(self, cfg):
        self.cfg = dict(cfg)
        self.left_prev = None
        self.right_prev = None

    def reset(self):
        self.left_prev = None
        self.right_prev = None

    def detect(self, frame):
        import cv2
        import numpy as np
        if frame is None or frame.size == 0:
            self.reset()
            return {"valid": False, "steering": 0.0, "confidence": 0.0,
                    "center_error": 0.0}, None
        h, w = frame.shape[:2]
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        yellow = cv2.inRange(hsv, np.array([15, 70, 70]),
                            np.array([35, 255, 255]))
        white = cv2.inRange(hsv, np.array([0, 0, 190]),
                           np.array([255, 30, 255]))
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(cv2.GaussianBlur(gray, (5, 5), 0), 50, 150)
        binary = cv2.bitwise_or(cv2.bitwise_or(yellow, white), edges)

        src = np.float32([
            [int(w*.35),int(h*.60)], [int(w*.65),int(h*.60)],
            [int(w*.05),int(h*.95)], [int(w*.95),int(h*.95)]
        ])
        dst = np.float32([
            [int(w*.20),0], [int(w*.80),0],
            [int(w*.20),h], [int(w*.80),h]
        ])
        transform = cv2.getPerspectiveTransform(src, dst)
        warped = cv2.warpPerspective(binary, transform, (w,h))
        # Use the user's histogram and polynomial method, but do not accept
        # np.argmax=0 on empty histograms or reuse invalid EMA indefinitely.
        histogram = np.sum(warped[h//2:, :], axis=0)
        mid = w//2
        left_base = int(np.argmax(histogram[:mid]))
        right_base = int(np.argmax(histogram[mid:]))+mid
        valid_peaks = (
            histogram[left_base] >= 255*8
            and histogram[right_base] >= 255*8
        )
        yy, xx = warped.nonzero()
        margin = int(self.cfg.get("window_margin_px", 80))
        left_idx = np.abs(xx-left_base) < margin
        right_idx = np.abs(xx-right_base) < margin
        min_points = int(self.cfg.get("min_lane_pixels", 80))
        if (not valid_peaks or np.count_nonzero(left_idx) < min_points
                or np.count_nonzero(right_idx) < min_points):
            self.reset()
            return {"valid":False, "steering":0.0, "confidence":0.0,
                    "center_error":0.0}, warped
        try:
            left = np.polyfit(yy[left_idx], xx[left_idx], 2)
            right = np.polyfit(yy[right_idx], xx[right_idx], 2)
        except (ValueError, np.linalg.LinAlgError):
            self.reset()
            return {"valid":False, "steering":0.0, "confidence":0.0,
                    "center_error":0.0}, warped
        y_near, y_far = 0.88*h, 0.60*h
        x_l_near, x_r_near = np.polyval(left,y_near),np.polyval(right,y_near)
        x_l_far, x_r_far = np.polyval(left,y_far),np.polyval(right,y_far)
        width_near = (x_r_near-x_l_near)/w
        width_far = (x_r_far-x_l_far)/w
        min_width = float(self.cfg.get("min_lane_width_ratio", 0.16))
        max_width = float(self.cfg.get("max_lane_width_ratio", 0.78))
        if (not min_width <= width_near <= max_width
                or not min_width <= width_far <= max_width
                or abs(width_near-width_far) > 0.25):
            self.reset()
            return {"valid":False, "steering":0.0, "confidence":0.0,
                    "center_error":0.0}, warped
        alpha = float(self.cfg.get("ema_alpha", ALPHA))
        self.left_prev = left if self.left_prev is None else (
            alpha*left+(1-alpha)*self.left_prev)
        self.right_prev = right if self.right_prev is None else (
            alpha*right+(1-alpha)*self.right_prev)
        near = (np.polyval(self.left_prev,y_near)+
                np.polyval(self.right_prev,y_near))*0.5
        far = (np.polyval(self.left_prev,y_far)+
               np.polyval(self.right_prev,y_far))*0.5
        center_err = (0.35*(near-w/2)+0.65*(far-w/2))/(w/2)
        gain = float(self.cfg.get("steer_gain_rad", 0.20))
        steering = max(-float(self.cfg.get("max_steer_rad",0.22)),
                       min(float(self.cfg.get("max_steer_rad",0.22)),
                           -gain*float(center_err)))
        confidence = min(1.0, min(
            int(np.count_nonzero(left_idx)),
            int(np.count_nonzero(right_idx))
        ) / float(self.cfg.get("confidence_pixel_scale", 700.0)))
        return {"valid":confidence >= float(self.cfg.get("min_confidence",0.35)),
                "steering":steering, "confidence":confidence,
                "center_error":float(center_err)}, warped
