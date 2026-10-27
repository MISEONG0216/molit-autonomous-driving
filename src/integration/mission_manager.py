#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import math


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
        self.hold_stop_index = None
        self.completed_stop_indices = set()
        self.path_s = None
        self.route_length_m = 0.0

    def set_global_path(self, path_points):
        """Precompute actual along-path metres instead of waypoint offsets."""
        if path_points is None or len(path_points) < 2:
            raise ValueError("global path must have at least two points")
        s = [0.0]
        for i in range(1, len(path_points)):
            x0, y0 = float(path_points[i - 1][0]), float(path_points[i - 1][1])
            x1, y1 = float(path_points[i][0]), float(path_points[i][1])
            s.append(s[-1] + math.hypot(x1 - x0, y1 - y0))
        self.path_s = s
        self.route_length_m = s[-1]

    def _next_stop(self, nearest_index):
        """Return next unserved stop index and forward path distance (metres)."""
        if self.path_s is None:
            return None, None
        if not 0 <= nearest_index < len(self.path_s):
            return None, None
        current_s = self.path_s[nearest_index]
        candidates = []
        for stop in self.stop_indices:
            if stop in self.completed_stop_indices:
                continue
            if not 0 <= stop < len(self.path_s):
                continue
            d = self.path_s[stop] - current_s
            if d < 0.0:
                d += self.route_length_m
            candidates.append((d, stop))
        if not candidates:
            return None, None
        d, stop = min(candidates)
        return stop, d

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

    def _update_signal_latch(self, now, active_stop, distance_m):
        """Never release on UNKNOWN; require fresh, distinct GO UDP packets."""
        fresh = self._signal_fresh(now)
        if not fresh:
            self.go_release_count = 0
            return

        if self.signal == "STOP":
            # With a loaded path, ignore traffic lights outside the upcoming
            # intersection's approach zone. Without a path, retain V8 test
            # compatibility (the main integration always supplies a path).
            approach_m = float(
                self.traffic_cfg.get("approach_distance_m", 60.0)
            )
            in_approach = (
                self.path_s is None
                or (
                    active_stop is not None
                    and distance_m is not None
                    and distance_m <= approach_m
                )
            )
            if in_approach:
                self.red_latched = True
                if self.hold_stop_index is None:
                    self.hold_stop_index = active_stop
            self.go_release_count = 0
            self._last_go_counted_update = 0.0
            return

        if self.signal in ("GO", "GO_LEFT"):
            holding = self.red_latched
            if holding and self.signal_last_update > self._last_go_counted_update:
                self._last_go_counted_update = self.signal_last_update
                self.go_release_count += 1
                needed = int(
                    self.traffic_cfg.get("go_release_confirm_loops", 8)
                )
                if self.go_release_count >= needed:
                    completed_stop = self.hold_stop_index
                    if completed_stop is not None:
                        self.completed_stop_indices.add(completed_stop)
                    self.red_latched = False
                    self.hold_stop_index = None
                    self.go_release_count = 0
                    self._last_go_counted_update = 0.0
            elif not holding:
                self.go_release_count = 0
            return

        self.go_release_count = 0
        self._last_go_counted_update = 0.0

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

        stop_distance_m = None
        full_brake = False
        traffic_approach = False
        if traffic_enabled:
            active_stop, stop_distance_m = self._next_stop(
                int(nearest_index)
            )
            self._update_signal_latch(now, active_stop, stop_distance_m)
            fresh = self._signal_fresh(now)
            is_green = fresh and self.signal in ("GO", "GO_LEFT")

            # Limit speed near each traffic light, regardless of its colour.
            # Only a confirmed STOP triggers full braking; UNKNOWN alone does not.
            if self.red_latched:
                target_speed = 0.0
                full_brake = True
                traffic_state = "FULL_BRAKE_RED"
                reasons.append("traffic_full_brake")
            elif stop_distance_m is not None and stop_distance_m <= float(
                self.traffic_cfg.get("approach_distance_m", 60.0)
            ):
                traffic_approach = True
                target_speed = min(
                    target_speed,
                    float(self.traffic_cfg.get("approach_speed_kmh", 30.0))
                )
                if not fresh:
                    traffic_state = f"APPROACH_VISION_WAIT_{active_stop}"
                elif is_green:
                    traffic_state = f"APPROACH_GO_{active_stop}"
                else:
                    traffic_state = f"APPROACH_{active_stop}"
                reasons.append("traffic_approach")
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
            "stop_distance_m": stop_distance_m,
            "traffic_full_brake": full_brake,
            "traffic_approach": traffic_approach,
            "red_latched": self.red_latched,
            "go_release_count": self.go_release_count,
            "merge_state": merge_state,
            "merge_risk": merge_risk,
            "merge_fresh": merge_fresh,
        }
