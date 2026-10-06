#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Mission-level arbitration between static and dynamic obstacle planners.

This module does NOT send vehicle-control commands.  It only selects which
behavior should have priority:
    STOP         : confirmed dynamic obstacle risk (highest priority)
    STATIC_STOP  : static planner cannot produce a safe avoidance path
    STATIC_AVOID : follow the static planner's local avoidance path
    GLOBAL_PATH  : normal global-path driving
"""

from dataclasses import dataclass
from typing import Optional


@dataclass
class MissionDecision:
    mode: str
    reason: str
    static_status: str
    dynamic_status: str
    static_obstacle_id: Optional[int] = None
    dynamic_obstacle_id: Optional[int] = None


class MissionManager:
    """Priority arbiter. Dynamic safety stop > static stop/avoid > global."""

    def decide(self, static_result, dynamic_state):
        static_status = getattr(static_result, "status", "CLEAR")
        dynamic_status = getattr(dynamic_state, "status", "CLEAR")

        static_obstacle_id = getattr(static_result, "obstacle_id", None)
        dynamic_obstacle_id = getattr(
            dynamic_state, "confirmed_obstacle_id", None
        )

        if dynamic_status == "RISK_CONFIRMED":
            return MissionDecision(
                mode="STOP",
                reason="Confirmed dynamic-obstacle risk has highest priority.",
                static_status=static_status,
                dynamic_status=dynamic_status,
                static_obstacle_id=static_obstacle_id,
                dynamic_obstacle_id=dynamic_obstacle_id,
            )

        if static_status == "STOP_REQUIRED":
            return MissionDecision(
                mode="STATIC_STOP",
                reason="Static planner reports no safe avoidance candidate.",
                static_status=static_status,
                dynamic_status=dynamic_status,
                static_obstacle_id=static_obstacle_id,
                dynamic_obstacle_id=dynamic_obstacle_id,
            )

        if static_status == "AVOIDING":
            return MissionDecision(
                mode="STATIC_AVOID",
                reason="Static planner selected a local avoidance path.",
                static_status=static_status,
                dynamic_status=dynamic_status,
                static_obstacle_id=static_obstacle_id,
                dynamic_obstacle_id=dynamic_obstacle_id,
            )

        return MissionDecision(
            mode="GLOBAL_PATH",
            reason="No obstacle mission currently overrides global driving.",
            static_status=static_status,
            dynamic_status=dynamic_status,
            static_obstacle_id=static_obstacle_id,
            dynamic_obstacle_id=dynamic_obstacle_id,
        )
