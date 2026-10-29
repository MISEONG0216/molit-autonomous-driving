import sys
import time
from pathlib import Path

# tests/에서 실행해도 프로젝트 루트의 모듈을 import할 수 있도록 설정
PROJECT_ROOT = Path(__file__).resolve().parent.parent

if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from lidar_receiver import LidarReceiver


receiver = LidarReceiver()

receiver.start()


print()
print("======================================")
print(" LiDAR Freshness Test")
print("======================================")
print()
print("1. MORAI LiDAR를 정상 실행한 상태로 기다리세요.")
print("2. FRESH가 확인되면 MORAI 시뮬레이션을 일시정지하거나")
print("   LiDAR 데이터 송신을 중단하세요.")
print("3. 약 3초 후 STALE로 바뀌는지 확인합니다.")
print()
print("Ctrl+C로 종료")
print()


last_state = None


try:

    while True:

        points, stats = receiver.get_latest()


        if points is None:

            state = "WAITING"

            if state != last_state:

                print(
                    "[STATUS] WAITING - "
                    "아직 완성된 LiDAR scan 없음"
                )

                last_state = state


        else:

            if stats["fresh"]:
                state = "FRESH"
            else:
                state = "STALE"


            # 상태가 바뀌었을 때 출력
            if state != last_state:

                print()
                print(
                    f"[STATUS CHANGE] "
                    f"{last_state} -> {state}"
                )

                last_state = state


            print(
                f"\r"
                f"[{state:7s}] "
                f"scan={stats['scan']:4d} | "
                f"age={stats['age_sec']:5.2f}s | "
                f"points={len(points):5d} | "
                f"quality={stats['quality']:10s}",
                end="",
                flush=True
            )


        time.sleep(0.2)


except KeyboardInterrupt:

    print()


finally:

    receiver.stop()

    print()
    print("Freshness test finished.")
