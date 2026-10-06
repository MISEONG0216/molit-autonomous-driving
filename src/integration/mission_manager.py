#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time


class MissionManager:
    """Combine V7 base driving with traffic-light and merge-risk constraints.

    Missions may constrain target speed/steering, but they never send EgoCtrlCmd
    directly. The final command remains owned by the integrated drive loop.
    """

    def __init__(self, traffic_cfg, merge_cfg, stop_indices):
        self.traffic_cfg = dict(traffic_cfg or {})
        self.merge_cfg = dict(merge_cfg or {})
        self.stop_indices = [int(x) for x in stop_indices]

        self.signal = "UNKNOWN"
        self.signal_last_update = 0.0
        self.red_latched = False
        self.go_release_count = 0
        self.intersection_stop_hold = False
        self.hold_stop_index = None

    def update_traffic_input(self, traffic, updated_at):
        if traffic is None:
            return
        self.signal = str(traffic.get("signal", "UNKNOWN")).upper()
        self.signal_last_update = float(updated_at or 0.0)

    def _signal_fresh(self, now):
        timeout = float(self.traffic_cfg.get("signal_timeout_sec", 1.2))
        return (
            self.signal_last_update > 0.0
            and now - self.signal_last_update <= timeout
        )

    def _update_signal_latch(self, now):
        # Preserve the validated branch behavior: GO confirmation increments
        # per control-loop while the latest GO/GO_LEFT result is still fresh.
        if not self._signal_fresh(now):
            return

        if self.signal == "STOP":
            self.red_latched = True
            self.go_release_count = 0
            return

        if self.signal in ("GO", "GO_LEFT"):
            if self.red_latched:
                self.go_release_count += 1
                needed = int(self.traffic_cfg.get("go_release_confirm_loops", 8))
                if self.go_release_count >= needed:
                    self.red_latched = False
                    self.intersection_stop_hold = False
                    self.hold_stop_index = None
                    self.go_release_count = 0
            else:
                self.go_release_count = 0
            return

        self.go_release_count = 0

    def _active_stop_index(self, nearest_index):
        start_offset = int(self.traffic_cfg.get("mission_start_offset", 25))
        end_offset = int(self.traffic_cfg.get("mission_end_offset", 15))
        for stop_idx in self.stop_indices:
            if stop_idx - start_offset <= nearest_index <= stop_idx + end_offset:
                return stop_idx
        return None

    @staticmethod
    def _in_any_zone(index, zones):
        for zone in zones or []:
            try:
                start = int(zone[0])
                end = int(zone[1])
            except Exception:
                continue
            if start <= end:
                if start <= index <= end:
                    return True
            else:
                # closed-path wrap zone
                if index >= start or index <= end:
                    return True
        return False

    def apply(
        self,
        nearest_index,
        base_target_speed_kmh,
        base_steering,
        traffic_enabled=False,
        merge_enabled=False,
        merge_control_enabled=False,
        merge_result=None,
        merge_updated_at=0.0,
        merge_all=False,
        now=None,
    ):
        now = time.time() if now is None else float(now)
        target_speed = float(base_target_speed_kmh)
        steering = float(base_steering)
        reasons = []

        traffic_state = "OFF"
        active_stop = None

        if traffic_enabled:
            self._update_signal_latch(now)
            active_stop = self._active_stop_index(int(nearest_index))
            fresh = self._signal_fresh(now)
            traffic_state = self.signal if fresh else "VISION_WAIT"

            if active_stop is not None:
                slowdown_offset = int(self.traffic_cfg.get("slowdown_offset", 20))
                approach_speed = float(
                    self.traffic_cfg.get("approach_speed_kmh", 3.0)
                )
                slowdown_index = active_stop - slowdown_offset

                if self.intersection_stop_hold:
                    target_speed = 0.0
                    steering = 0.0
                    if self.red_latched:
                        traffic_state = f"HOLD_RED_{active_stop}"
                    else:
                        traffic_state = "HOLD_RELEASE_WAIT"
                    reasons.append("traffic_hold")

                elif self.red_latched:
                    if nearest_index >= active_stop:
                        self.intersection_stop_hold = True
                        self.hold_stop_index = active_stop
                        target_speed = 0.0
                        steering = 0.0
                        traffic_state = f"STOP_GPS_{active_stop}"
                        reasons.append("traffic_stop")

                    elif nearest_index >= slowdown_index:
                        target_speed = min(target_speed, approach_speed)
                        traffic_state = f"APPROACH_RED_{active_stop}"
                        reasons.append("traffic_slow")

                    else:
                        traffic_state = f"RED_FAR_{active_stop}"

                elif not fresh:
                    traffic_state = "VISION_WAIT"
                elif self.signal == "UNKNOWN":
                    traffic_state = "UNKNOWN"
                else:
                    traffic_state = self.signal
            else:
                traffic_state = "CRUISE"

        merge_state = "OFF"
        merge_risk = 0.0
        merge_fresh = False

        if merge_enabled and merge_result:
            timeout = float(self.merge_cfg.get("result_timeout_sec", 0.8))
            merge_fresh = (
                bool(merge_result.get("valid", False))
                and merge_updated_at > 0.0
                and now - float(merge_updated_at) <= timeout
            )
            if merge_fresh:
                merge_risk = float(merge_result.get("max_risk", 0.0))
                merge_state = str(
                    merge_result.get("decision", "UNKNOWN")
                )
            else:
                merge_state = "STALE"

        # Clarified strategy: always keep the global path. Merge perception
        # only imposes a speed cap in a configured elevated/highway zone.
        if (
            merge_enabled
            and merge_control_enabled
            and merge_fresh
        ):
            zones = self.merge_cfg.get("zones", [])
            zone_active = bool(merge_all) or self._in_any_zone(
                int(nearest_index), zones
            )

            if zone_active:
                for item in self.merge_cfg.get("risk_speed_caps", []):
                    try:
                        threshold = float(item["min_risk"])
                        cap = float(item["cap_kmh"])
                    except Exception:
                        continue

                    if merge_risk >= threshold:
                        if cap < target_speed:
                            target_speed = cap
                            reasons.append(f"merge_risk_{merge_risk:.2f}")
                        break

        return {
            "target_speed_kmh": max(0.0, target_speed),
            "steering": max(-1.0, min(1.0, steering)),
            "reasons": reasons,
            "traffic_state": traffic_state,
            "active_stop_index": active_stop,
            "red_latched": self.red_latched,
            "go_release_count": self.go_release_count,
            "merge_state": merge_state,
            "merge_risk": merge_risk,
            "merge_fresh": merge_fresh,
        }
