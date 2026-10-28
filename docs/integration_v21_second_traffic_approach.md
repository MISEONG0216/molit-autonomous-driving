# V21: second traffic light (WP620) approach tuning

Base: `integration-v20-static-avoid-25kph`. Existing V20 obstacle
avoidance, V7 global speed profile, static zone 590-760, highway merge
and traffic signal logic are unchanged.

## Requested adjustment

Config-only modification in `config/integration_missions.json`:
- WP620 previous: start slowing 100m ahead, cap at 25km/h; below 35m,
  cap further at 10km/h.
- WP620 V21: start slowing **65m** ahead, cap at **35km/h**; below **25m**,
  cap further at **20km/h**.

The `MissionManager` uses remaining *along-path metres* to WP620,
not a count of waypoints. All caps are upper bounds; the V7 speed
profile, other active missions, or full red-light braking can impose
a lower target.

For the Oct 28 V20 log, the old 10km/h cap engaged at WP550 (35m
before WP620) and actual speed fell from ~24.7 to ~9.5km/h over ~1.5s.
This explains slow approach even when a green lamp was visible.

## Red stop remains active

This change does **not** disable or weaken the `STOP` latch.
A sufficiently fresh STOP packet in the approach still commands
`traffic_full_brake` (target 0km/h, brake 1.0).

The run has seen a STOP result only around **10m before WP620**,
so 20km/h approaching this point is an additional stop-line overrun
risk, particularly if camera detection is late. V21 is an experimental
comfort/speed change, not a certified stopping-distance solution.

## Verification

```bash
cd ~/molit-autonomous-driving
git fetch origin
git switch integration-v21-second-traffic-approach
git pull --ff-only origin integration-v21-second-traffic-approach
source .venv/bin/activate
python3 -m unittest discover -s tests -p 'test_second_traffic_approach_v21.py' -v
```

The synthetic 0.5m waypoint tests verify exact threshold entry, normal
`GO` approach, STOP full-brake priority, unchanged WP1224 left-only
override, unchanged static avoidance 25km/h and unchanged static WP zones.

If the current Ubuntu checkout has local modifications to
`config/integration_missions.json`, preserve them before switching
branches. Do not forcibly reset or discard local settings.

On the real MORAI scenario, inspect `traffic_stop_index=620`,
`traffic_stop_distance_m`, `comp_speed_kmh`,
`final_target_kmh`, `traffic_raw`, `brake_cmd` around 65m and
25m, and check whether STOP actually brings the car to rest ahead of
the stopline. A red-light overrun requires restoring earlier slowdown
or improving signal-detection/stopping logic.
