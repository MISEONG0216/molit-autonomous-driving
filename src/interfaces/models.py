"""Shared data contracts between autonomous-driving modules.

Mission modules exchange data through these models instead of sending CtrlCmd
directly. Fields can be extended as each mission is implemented.
"""
from dataclasses import dataclass
from enum import Enum
from typing import Optional, Tuple


class TrafficLightState(str, Enum):
    UNKNOWN = "UNKNOWN"
    RED = "RED"
    YELLOW = "YELLOW"
    GREEN = "GREEN"
    LEFT = "LEFT"
    RED_LEFT = "RED_LEFT"
    GREEN_LEFT = "GREEN_LEFT"


class MissionAction(str, Enum):
    NORMAL = "NORMAL"
    LANE_ASSIST = "LANE_ASSIST"
    SLOW_DOWN = "SLOW_DOWN"
    STOP = "STOP"
    STATIC_AVOID = "STATIC_AVOID"
    DYNAMIC_AVOID = "DYNAMIC_AVOID"
    LANE_CHANGE_REQUEST = "LANE_CHANGE_REQUEST"
    EMERGENCY_STOP = "EMERGENCY_STOP"


@dataclass
class LaneResult:
    offset_m: float
    confidence: float


@dataclass
class Obstacle:
    obstacle_id: Optional[int]
    x: float
    y: float
    width: float
    length: float
    distance: float
    velocity_x: float = 0.0
    velocity_y: float = 0.0
    is_dynamic: bool = False
    confidence: float = 1.0


@dataclass
class MissionDecision:
    action: MissionAction
    target_speed_kmh: Optional[float] = None
    local_path: Optional[Tuple[Tuple[float, float], ...]] = None
    reason: str = ""
