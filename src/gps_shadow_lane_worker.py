#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""MORAI front camera relay -> user's HSV/Canny lane-center observation.

Reads only localhost 9311 from the traffic detector's camera relay.
Does NOT bind 9291 and NEVER transmits EgoCtrlCmd.
"""
import argparse
import json
import socket
import time
from pathlib import Path

import numpy as np
from integration.merge_side_io import JpegAssembler
from integration.gps_shadow_lane import LaneCenterDetector

ROOT=Path(__file__).resolve().parents[1]


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument("--show", action="store_true")
    args=parser.parse_args()
    import cv2
    net=json.loads((ROOT/"config/integration_network.json").read_text(encoding="utf-8"))
    missions=json.loads((ROOT/"config/integration_missions.json").read_text(encoding="utf-8"))
    cfg=missions.get("gps_shadow", {})
    relay=net["gps_shadow_camera_relay"]
    result=net["gps_shadow_lane_result"]
    for x in (relay,result):
        if x["bind_ip"] != "127.0.0.1":
            raise RuntimeError("Lane worker UDP must remain localhost-only")
    rx=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
    rx.setsockopt(socket.SOL_SOCKET,socket.SO_RCVBUF,8*1024*1024)
    rx.bind(("127.0.0.1", int(relay["port"])))
    rx.settimeout(0.20)
    tx=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
    target=("127.0.0.1",int(result["port"]))
    assembler=JpegAssembler()
    detector=LaneCenterDetector(cfg)
    frame_count=0
    last_frame_at=0.0
    last_status=None
    print(f"[GPS SHADOW LANE] relay=127.0.0.1:{relay['port']} -> {target}",flush=True)
    print("[GPS SHADOW LANE] HSV/Canny BEV+EMA; NO EgoCtrlCmd",flush=True)
    try:
        while True:
            try:
                packet,_=rx.recvfrom(65535)
            except socket.timeout:
                continue
            jpeg=assembler.feed(packet)
            # Drain the queue without leaving too many unprocessed JPEG frames.
            rx.setblocking(False)
            for _ in range(512):
                try:
                    packet,_=rx.recvfrom(65535)
                except BlockingIOError:
                    break
                newest=assembler.feed(packet)
                if newest is not None:
                    jpeg=newest
            rx.settimeout(0.20)
            if jpeg is None:
                continue
            now=time.monotonic()
            if now-last_frame_at < 1.0/max(1.0,float(cfg.get("lane_max_hz",12.0))):
                continue
            last_frame_at=now
            frame=cv2.imdecode(np.frombuffer(jpeg,dtype=np.uint8),cv2.IMREAD_COLOR)
            if frame is None or frame.size == 0:
                continue
            frame_count+=1
            lane, warped=detector.detect(frame)
            packet={"kind":"LANE_CENTER","valid":bool(lane["valid"]),
                    "steering":round(lane["steering"],6),
                    "confidence":round(lane["confidence"],4),
                    "center_error":round(lane["center_error"],5),
                    "frame":frame_count}
            tx.sendto(json.dumps(packet,separators=(",",":")).encode("ascii"),target)
            status=(packet["valid"],round(packet["confidence"],1))
            if status!=last_status and (frame_count%3==0 or last_status is None):
                print(f"[GPS SHADOW LANE] frame={frame_count} valid={packet['valid']}"
                      f" conf={packet['confidence']:.2f} steer={packet['steering']:.3f}",flush=True)
                last_status=status
            if args.show:
                h,w=frame.shape[:2]
                cv2.putText(frame,f"LANE={packet['valid']} steer={packet['steering']:.3f}"
                            f" conf={packet['confidence']:.2f}",(12,35),
                            cv2.FONT_HERSHEY_SIMPLEX,.72,(0,255,0),2)
                cv2.imshow("MORAI GPS SHADOW LANE",frame)
                if warped is not None:
                    cv2.imshow("MORAI LANE BEV MASK",warped)
                if cv2.waitKey(1)&0xff in (ord("q"),27):
                    break
    except KeyboardInterrupt:
        pass
    finally:
        rx.close()
        tx.close()
        if args.show:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
