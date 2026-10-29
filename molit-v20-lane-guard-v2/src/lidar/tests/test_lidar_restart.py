import sys
import time
from pathlib import Path

# tests/에서 실행해도 프로젝트 루트의 모듈을 import할 수 있도록 설정
PROJECT_ROOT = Path(__file__).resolve().parent.parent

if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from lidar_receiver import LidarReceiver


def wait_for_fresh(receiver, timeout=15.0):

    start_time = time.monotonic()

    while True:

        points, stats = receiver.get_latest()

        if (
            points is not None
            and stats["fresh"]
        ):

            return points, stats

        if (
            time.monotonic()
            - start_time
        ) > timeout:

            return None, None

        time.sleep(0.1)


receiver = LidarReceiver()


print()
print("======================================")
print(" LiDAR Restart Test")
print("======================================")
print()


# ============================================================
# TEST 1
# ============================================================

print("[TEST 1] First start")

receiver.start()


points, stats = wait_for_fresh(
    receiver
)


if points is None:

    print(
        "[FAIL] First start: "
        "fresh LiDAR frame not received"
    )

    receiver.stop()
    raise SystemExit(1)


print(
    f"[PASS] First start | "
    f"scan={stats['scan']} | "
    f"points={len(points)} | "
    f"quality={stats['quality']} | "
    f"age={stats['age_sec']:.2f}s"
)


# ============================================================
# STOP
# ============================================================

print()
print("[TEST 2] Stop receiver")

receiver.stop()


status = receiver.get_status()


print(
    f"running={status['running']}"
)


if status["running"]:

    print(
        "[FAIL] Receiver did not stop"
    )

    raise SystemExit(1)


print(
    "[PASS] Receiver stopped"
)


# 잠깐 대기
time.sleep(2.0)


# ============================================================
# RESTART
# ============================================================

print()
print("[TEST 3] Restart receiver")

receiver.start()


points, stats = wait_for_fresh(
    receiver
)


if points is None:

    print(
        "[FAIL] Restart: "
        "fresh LiDAR frame not received"
    )

    receiver.stop()
    raise SystemExit(1)


print(
    f"[PASS] Restart | "
    f"scan={stats['scan']} | "
    f"points={len(points)} | "
    f"quality={stats['quality']} | "
    f"age={stats['age_sec']:.2f}s"
)


# ============================================================
# FINAL STOP
# ============================================================

receiver.stop()


status = receiver.get_status()


print()
print("========== FINAL STATUS ==========")

print(
    f"Running         : "
    f"{status['running']}"
)

print(
    f"Total scans     : "
    f"{status['scan_count']}"
)

print(
    f"Valid scans     : "
    f"{status['valid_scan_count']}"
)

print(
    f"Dropped scans   : "
    f"{status['dropped_scan_count']}"
)

print(
    f"Invalid packets : "
    f"{status['invalid_packet_count']}"
)

print()
print(
    "Restart test finished."
)
