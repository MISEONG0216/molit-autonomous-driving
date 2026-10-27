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
        self._last_go_counted_update = 0.0

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
        # STOP latches immediately. UNKNOWN/stale frames never clear a RED stop.
        # GO must be confirmed by distinct, fresh UDP updates (not repeated
        # control-loop reads of a single GO packet).
        if not self._signal_fresh(now):
            return

        if self.signal == "STOP":
            self.red_latched = True
            self.go_release_count = 0
            self._last_go_counted_update = 0.0
            return

        if self.signal in ("GO", "GO_LEFT"):
            if self.red_latched:
                if self.signal_last_update > self._last_go_counted_update:
                    self._last_go_counted_update = self.signal_last_update
                    self.go_release_count += 1
                    needed = int(
                        self.traffic_cfg.get("go_release_confirm_loops", 8)
                    )
                    if self.go_release_count >= needed:
                        self.red_latched = False
                        self.go_release_count = 0
                        self._last_go_counted_update = 0.0
            else:
                self.go_release_count = 0
            return

        self.go_release_count = 0
        self._last_go_counted_update = 0.0

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

            # Full-brake test strategy: STOP applies anywhere on the path.
            # Do not require a waypoint/stopline window; stay stopped until
            # GO/GO_LEFT has been confirmed on fresh camera packets.
            if self.red_latched:
                target_speed = 0.0
                traffic_state = "FULL_BRAKE_RED"
                reasons.append("traffic_full_brake")
            elif not fresh:
                traffic_state = "VISION_WAIT"
            else:
                traffic_state = self.signal

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
