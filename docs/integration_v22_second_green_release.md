# V22 — WP620 confirmed-green approach speed release

Base: `integration-v21-second-traffic-approach`. V21 is unchanged.

## Why

At the second traffic light (stop waypoint 620), V21 limits the V7 target
to 35km/h within 65m of the stop, then 20km/h within 25m, irrespective
of the observed light color. V22 keeps this conservative fallback but
allows normal V7 cruise after confirming green.

## Conditions (WP620 only)

`config/integration_missions.json`:

```json
"620": {
  "approach_distance_m": 65,
  "approach_speed_kmh": 35,
  "near_distance_m": 25,
  "near_speed_kmh": 20,
  "green_release_confirm_packets": 3,
  "green_release_start_distance_m": 55
}
```

1. From 65m: V21 approach speed caps still apply (35 / 20 km/h).
2. Inside 55m: require **three new, distinct, consecutive fresh** camera
   `GO` updates. Re-reading the same timestamp does not increment count.
3. When confirmed: traffic approach caps for **WP620 only** are
   removed. The final speed follows normal V7 longitudinal policy and
   V7 path speed limits, **not forced maximum throttle or 57km/h**.
4. `UNKNOWN`, `STOP`, `GO_LEFT`, stale signal, or leaving the WP620
   approach all reset the green confirmation counter and reapply
   approach limits as applicable.
5. Any valid STOP in the approach still uses the existing RED latch:
   target=0, immediate brake=1. After a RED stop, the existing
   **eight distinct GO** packets must release it. V22's three-GO
   shortcut NEVER bypasses a latched RED.
6. A static LiDAR avoidance route still caps speed at 25km/h; 
   LiDAR `STOP_REQUIRED` or stale in-zone result still causes
   full brake. These priorities are applied in the main controller
   **after** the traffic manager's target calculation.
7. Other traffic lights, WP1224 left-turn logic, merge zones and
   original V7 profile are unchanged.

## Diagnostics

Main display:
- `TRAFFIC CTRL: APPROACH_GREEN_RELEASE_620`
- `GREEN_BYPASS_620=True`, `GREEN_COUNT_620=3`

Added CSV fields (at the end):
- `traffic_green_bypass_620` (0 or 1)
- `traffic_green_count_620`

Before three GO packets, the V21 WP620 speeds still apply.

## Run (Ubuntu / MORAI)

```bash
cd ~/molit-autonomous-driving
git fetch origin
git switch integration-v22-green-release-second
git pull --ff-only origin integration-v22-green-release-second
source .venv/bin/activate

python3 -m unittest discover -s tests -p 'test_second_green_release_v22.py' -v
python3 -m unittest discover -s tests -p 'test_second_traffic_approach_v21.py' -v
python3 -m unittest discover -s tests -p 'test_static_speed_25_v20.py' -v
```

In separate terminal sessions (same repository and virtualenv):

```bash
python3 src/traffic_light_detector.py
python3 src/lidar/static_avoidance_module.py
python3 src/main_integrated_v1.py --traffic --static --max-speed 57
```

The main line above is DRY-RUN. For a supervised simulation **after**
unit tests and mode checks:

```bash
python3 src/main_integrated_v1.py --drive --traffic --static --static-control --max-speed 57
```

Only main transmits EgoCtrlCmd.

## Known limitations

- Camera results have no cryptographic association with a particular
  traffic-light fixture. The gate restricts the GO bypass to WP620's
  final 55m, but false positives from nearby lights remain possible.
- If STOP is first recognized too late, lifting the 20km/h approach
  limit may cause a stop-line overrun despite full braking.
- The V19 WP590 entry ZONE_MISMATCH transient remains unmodified as
  requested. This experiment changes WP620 traffic control only.
- Unit tests check logic but do not establish MORAI or real-road
  safety. Test the green and red signal states in controlled simulation.
