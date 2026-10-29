"""Bounded GPS-loss pose bridge: lane vision steers; DR serves LiDAR only.

NO MORAI control and NO global-path steering are implemented here.
When NMEA repeats an old valid fix while the status velocity indicates motion,
treat that position as stale. A stale GPS fix must not silently keep V7 active.
"""
import math


class GpsShadowPose:
    def __init__(self, cfg):
        self.cfg = dict(cfg or {})
        self.pose = None
        self.last_sample = None
        self.last_change_at = None
        self.last_update = None
        self.shadow_started_at = None
        self.shadow_distance_m = 0.0
        self.shadow = False
        self.recovery_count = 0
        self.last_recovery_sample = None
        self.last_mode = "NO_FIX"

    @staticmethod
    def _valid_position(raw):
        if raw is None:
            return None
        try:
            x, y = float(raw[0]), float(raw[1])
            if not (math.isfinite(x) and math.isfinite(y)):
                return None
            return (x, y)
        except (ValueError, TypeError, IndexError):
            return None

    @staticmethod
    def _distance(a, b):
        return math.hypot(a[0]-b[0], a[1]-b[1])

    def update(self, raw_position, yaw, signed_speed_kmh, now):
        now = float(now)
        speed = float(signed_speed_kmh)
        yaw = float(yaw)
        if not all(map(math.isfinite, (now, speed, yaw))):
            return "STOP", self.pose, float("inf")
        raw = self._valid_position(raw_position)
        dt = 0.0 if self.last_update is None else now-self.last_update
        max_gap = float(self.cfg.get("max_update_gap_sec", 0.8))
        gap_bad = self.last_update is not None and (dt < 0 or dt > max_gap)
        if self.pose is not None and not gap_bad and dt > 0:
            move = (speed / 3.6) * dt
            self.pose = (
                self.pose[0] + move * math.cos(yaw),
                self.pose[1] + move * math.sin(yaw),
            )
            if self.shadow:
                self.shadow_distance_m += abs(move)
        self.last_update = now

        changed = (
            raw is not None
            and (self.last_sample is None
                 or self._distance(raw, self.last_sample) >=
                 float(self.cfg.get("gps_change_threshold_m", 0.08)))
        )
        if changed:
            self.last_change_at = now
            self.last_sample = raw

        if self.pose is None:
            if raw is None:
                self.last_mode = "NO_FIX"
                return self.last_mode, None, float("inf")
            self.pose = raw
            self.last_change_at = now
            self.last_sample = raw
            self.last_mode = "GPS"
            return self.last_mode, self.pose, 0.0

        age = now-(self.last_change_at if self.last_change_at is not None else now)
        moving = abs(speed) >= float(self.cfg.get("moving_threshold_kmh", 2.0))
        gps_missing = raw is None
        gps_stale = moving and age > float(self.cfg.get("stale_moving_sec", 1.7))
        if not self.shadow and not (gps_missing or gps_stale):
            # Only an actually CHANGED GPS position can correct dead-reckoning
            # drift; reusing the cached coordinate would conceal a lost fix.
            if changed:
                self.pose = raw
            self.last_mode = "GPS"
            return self.last_mode, self.pose, 0.0

        if not self.shadow:
            self.shadow = True
            self.shadow_started_at = (
                min(now, self.last_change_at)
                if self.last_change_at is not None else now
            )
            self.shadow_distance_m = 0.0
            self.recovery_count = 0
            self.last_recovery_sample = None

        # Require three distinct and geometrically plausible GPS updates
        # before resuming global Pure Pursuit. One cached fix is not a recovery.
        if changed and raw is not None and self._distance(
                raw, self.pose) <= float(self.cfg.get("gps_recovery_max_error_m", 6.0)):
            if self.last_recovery_sample is None or self._distance(
                    raw, self.last_recovery_sample) >= 0.08:
                self.last_recovery_sample = raw
                self.recovery_count += 1
            if self.recovery_count >= int(self.cfg.get("gps_recovery_samples", 3)):
                self.pose = raw
                self.shadow = False
                self.shadow_started_at = None
                self.shadow_distance_m = 0.0
                self.recovery_count = 0
                self.last_mode = "GPS"
                return self.last_mode, self.pose, 0.0
        elif changed:
            self.recovery_count = 0
            self.last_recovery_sample = None

        duration = now-self.shadow_started_at
        # Long tunnels cannot be cleared using a hard 12-second timer.
        # In sensor-guided mode, main requires fresh lane + LiDAR every cycle
        # and does not use this drifting pose for steering or obstacle planning.
        extended = self.cfg.get("allow_sensor_guided_extended", False) is True
        expired = (
            gap_bad
            or (not extended and (
                duration > float(self.cfg.get("max_shadow_sec", 12.0))
                or self.shadow_distance_m > float(
                    self.cfg.get("max_shadow_distance_m", 35.0))
            ))
        )
        self.last_mode = "STOP" if expired else "SHADOW"
        return self.last_mode, self.pose, duration
