#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Raw MORAI GPS UDP diagnostic.

Receives UDP datagrams directly with Python socket.
It intentionally does NOT use MORAI-NetworkModule Receiver, multiprocessing,
ctypes.memmove, GPS parsing, IMU, LiDAR, or vehicle control.

Goal:
- If packets keep arriving while the vehicle moves, MORAI GPS UDP transmission
  is alive and the NetworkModule Receiver path is the likely problem.
- If packets stop arriving here too, investigate MORAI GPS/network settings.
"""

import socket
import time

IP = "127.0.0.1"
PORT = 9201
TIMEOUT_S = 1.0
REPORT_INTERVAL_S = 1.0


def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((IP, PORT))
    sock.settimeout(TIMEOUT_S)

    print("=" * 78)
    print(" MORAI GPS RAW UDP TEST")
    print("=" * 78)
    print(f"[LISTEN] {IP}:{PORT}")
    print("[SAFE] MORAI-NetworkModule Receiver is NOT used.")
    print("[SAFE] No IMU / LiDAR / EgoCtrlCmd is used.")
    print("[TEST] Keep vehicle still for 3-5 s, then move it slowly.")
    print("[IMPORTANT] Close other Python programs using GPS port 9201 first.")
    print()

    total_packets = 0
    interval_packets = 0
    last_report = time.monotonic()
    last_packet_time = None
    previous_packet = None
    changed_packets = 0

    try:
        while True:
            try:
                raw_data, sender = sock.recvfrom(65535)
                now = time.monotonic()
                total_packets += 1
                interval_packets += 1
                last_packet_time = now

                changed = previous_packet is None or raw_data != previous_packet
                if changed:
                    changed_packets += 1
                previous_packet = raw_data

                if now - last_report >= REPORT_INTERVAL_S:
                    elapsed = now - last_report
                    hz = interval_packets / elapsed if elapsed > 0 else 0.0
                    print(
                        f"[RX] total={total_packets:6d} "
                        f"rate={hz:5.1f} Hz "
                        f"size={len(raw_data):4d} bytes "
                        f"changed_total={changed_packets:6d} "
                        f"sender={sender[0]}:{sender[1]}"
                    )
                    interval_packets = 0
                    last_report = now

            except socket.timeout:
                now = time.monotonic()
                if last_packet_time is None:
                    print("[TIMEOUT] No GPS UDP packet received yet.")
                else:
                    silent = now - last_packet_time
                    print(f"[TIMEOUT] No GPS UDP packet for {silent:.1f} s.")

    except KeyboardInterrupt:
        print()
        print("[STOP] Raw GPS UDP test finished.")
        print(f"[SUMMARY] total_packets={total_packets}, changed_packets={changed_packets}")
    finally:
        sock.close()


if __name__ == "__main__":
    main()
