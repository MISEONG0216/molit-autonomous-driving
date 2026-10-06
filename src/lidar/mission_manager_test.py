#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""DRY-RUN sanity test for obstacle MissionManager.

No MORAI connection and no EgoCtrlCmd transmission are used here.  The goal is
to verify mission priority before wiring the manager into the live LiDAR loop.
"""

from dataclasses import dataclass
from mission_manager import MissionManager


@dataclass
class StaticResult:
    status: str
    obstacle_id: int | None = None


@dataclass
class DynamicState:
    status: str
    confirmed_obstacle_id: int | None = None


def check(manager, static_status, dynamic_status, expected):
    static = StaticResult(static_status, 101)
    dynamic = DynamicState(dynamic_status, 202)
    decision = manager.decide(static, dynamic)

    ok = decision.mode == expected
    mark = "PASS" if ok else "FAIL"
    print(
        f"[{mark}] static={static_status:<13} "
        f"dynamic={dynamic_status:<14} "
        f"=> {decision.mode:<12} expected={expected}"
    )
    if not ok:
        raise AssertionError(
            f"{static_status} + {dynamic_status}: "
            f"expected {expected}, got {decision.mode}"
        )


def main():
    manager = MissionManager()

    print("=" * 72)
    print(" OBSTACLE MISSION MANAGER DRY-RUN")
    print("=" * 72)
    print("Priority: dynamic STOP > static STOP > static AVOID > GLOBAL")
    print()

    check(manager, "CLEAR", "CLEAR", "GLOBAL_PATH")
    check(manager, "AVOIDING", "CLEAR", "STATIC_AVOID")
    check(manager, "STOP_REQUIRED", "CLEAR", "STATIC_STOP")

    # Dynamic risk must override every static state.
    check(manager, "CLEAR", "RISK_CONFIRMED", "STOP")
    check(manager, "AVOIDING", "RISK_CONFIRMED", "STOP")
    check(manager, "STOP_REQUIRED", "RISK_CONFIRMED", "STOP")

    print()
    print("[OK] Mission priority table passed.")
    print("[SAFE] This test sends no vehicle-control command.")


if __name__ == "__main__":
    main()
