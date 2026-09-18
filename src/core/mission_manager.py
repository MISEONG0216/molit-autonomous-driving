"""Mission arbitration skeleton.

Mission modules provide perception/planning results. This module decides which
behavior has priority. It intentionally does not send MORAI CtrlCmd directly.
"""
from interfaces.models import MissionAction, MissionDecision, TrafficLightState


class MissionManager:
    def decide(self, traffic_light: TrafficLightState = TrafficLightState.UNKNOWN) -> MissionDecision:
        # Safety-first placeholder. Mission-specific priority rules will be
        # implemented and validated before integration.
        if traffic_light == TrafficLightState.RED:
            return MissionDecision(
                action=MissionAction.STOP,
                target_speed_kmh=0.0,
                reason="traffic_light_red",
            )
        return MissionDecision(action=MissionAction.NORMAL, reason="no_active_mission")
