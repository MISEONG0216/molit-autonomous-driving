#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import socket
import time


class LatestUdpTextReceiver:
    """Non-blocking UDP receiver that keeps only the latest datagram."""

    def __init__(self, bind_ip, port, max_size=512):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((bind_ip, int(port)))
        self.sock.setblocking(False)
        self.max_size = int(max_size)
        self.last_raw = None
        self.last_update = 0.0

    def poll(self):
        updated = False
        while True:
            try:
                raw, _ = self.sock.recvfrom(self.max_size)
                self.last_raw = raw
                self.last_update = time.time()
                updated = True
            except BlockingIOError:
                break
        return self.last_raw, self.last_update, updated

    def close(self):
        self.sock.close()


def parse_traffic_message(raw):
    if not raw:
        return {
            "signal": "UNKNOWN",
            "stopline_y": None,
            "stopline_conf": 0.0,
            "left_arrow": False,
        }

    text = raw.decode("ascii", errors="ignore").strip().upper()
    parts = text.split("|")
    signal = parts[0] if parts else "UNKNOWN"
    if signal not in ("STOP", "GO", "GO_LEFT", "UNKNOWN"):
        signal = "UNKNOWN"

    stopline_y = None
    stopline_conf = 0.0
    if len(parts) >= 2:
        try:
            value = float(parts[1])
            if 0.0 <= value <= 1.0:
                stopline_y = value
        except Exception:
            pass
    if len(parts) >= 3:
        try:
            stopline_conf = max(0.0, min(1.0, float(parts[2])))
        except Exception:
            pass

    # Optional V11 field; older three-field messages remain supported.
    left_arrow = len(parts) >= 4 and parts[3].strip() == "1"
    return {
        "signal": signal,
        "stopline_y": stopline_y,
        "stopline_conf": stopline_conf,
        "left_arrow": left_arrow,
    }


def parse_merge_message(raw):
    result = {
        "valid": False,
        "side_risk": 0.0,
        "rear_risk": 0.0,
        "max_risk": 0.0,
        "decision": "UNKNOWN",
    }
    if not raw:
        return result

    try:
        text = raw.decode("ascii", errors="ignore").strip().upper()
        parts = text.split("|")
        if len(parts) < 5 or parts[0] != "MERGE":
            return result
        side = max(0.0, min(1.0, float(parts[1])))
        rear = max(0.0, min(1.0, float(parts[2])))
        max_risk = max(0.0, min(1.0, float(parts[3])))
        decision = parts[4]
        if decision not in ("MERGE_ACCEPT", "MERGE_REJECT"):
            decision = "UNKNOWN"
        result.update({
            "valid": decision != "UNKNOWN",
            "side_risk": side,
            "rear_risk": rear,
            "max_risk": max_risk,
            "decision": decision,
        })
    except Exception:
        pass
    return result
