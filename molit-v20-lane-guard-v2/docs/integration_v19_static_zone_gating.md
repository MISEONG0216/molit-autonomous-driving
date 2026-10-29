# V19: Static-obstacle avoidance only within designated waypoint zone

Base: V18 path-locked implementation. V18 and older branches are unchanged.

## Why

The 2026-10-28 MORAI logs demonstrate:
- Successful static avoidance: main waypoint **635–697**, worker obstacle ID 138.
- Unexpected avoidance: **862–921** (obstacle ID 170).
- Unexpected avoidance: **1270–1323** (obstacle ID 329).

The uploaded competition map shows a designated "신호등 & 정적 장애물" region,
distinct from the "신호등 & 동적 장애물" and other route segments. Since the map
does not give exact waypoint indices, V19 uses **provisional** experiment-grounded
static-only waypoint limits **590–760**. Adjust these after checking MORAI
scenario placement, not merely from the image. The user-supplied logs do not
prove that all valid static-obstacle placements lie in these bounds.

## Configuration

File: `config/integration_missions.json`

```json
"static": {
  "avoidance_zones_wp": [[590, 760]]
}
```

Both the LiDAR worker and main controller load this exact configuration.
Indices are inclusive. Invalid or overlapping ranges cause startup failure.

## Behaviour

**Outside zone, no active local route**
- LiDAR worker still receives sensor packets and pose, but does not run
  static obstacle detection/tracking/planning. It sends
  `status=CLEAR, latch_state=ZONE_OFF, zone_active=false` to main.
- Main displays `STATIC: ZONE_OFF` and follows original V7 global path
  (and any enabled traffic/merge behaviour), without activating static
  STOP or stale-data fail-stop.
- A detected object here cannot generate a NEW static local path.

**Inside zone**
- Full V18 static detector/tracker/planner/path latch remains active.
- `AVOIDING` switches to the existing limited-speed local path.
- `STOP_REQUIRED`, stale/missing LiDAR data, worker zone mismatch, or
  missing forward path require main to issue a full brake.
- `CLEAR` keeps V7 global profile.

**Started an avoidance path inside zone, then crossed zone boundary**
- LiDAR worker continues the existing locked path until the geometric
  release index, without starting a new avoidance path after exiting.
- Main continues enforcing static safety and route tracking while the
  worker's `latch_active=true`; it does not silently return to V7.
- Missing data during this still-active phase triggers fail-stop.
- Once the worker reports `latch_active=false` outside zone, main
  returns to V7 and ignores new static detections.

## How to run

```bash
cd ~/molit-autonomous-driving
git fetch origin
git switch integration-v19-static-zone-gated
git pull --ff-only origin integration-v19-static-zone-gated
source .venv/bin/activate

python3 -m unittest discover -s tests -p 'test_static_zone_v19.py' -v
python3 -m unittest discover -s tests -p 'test_static_path_lock_v18.py' -v
python3 -m unittest discover -s tests -p 'test_static_integration_v17.py' -v
```

Start worker in separate terminal:
```bash
python3 src/lidar/static_avoidance_module.py
```
If traffic is needed, start its detector in a third terminal:
```bash
python3 src/traffic_light_detector.py
```
Main DRY-RUN first:
```bash
python3 src/main_integrated_v1.py --traffic --static --max-speed 10
```

Only after checking terminal output and mission-zone alignment, test
a supervised MORAI simulation drive. Outside zone at WP862 / WP1270:
`STATIC: ZONE_OFF`, no static avoidance path. Inside zone (e.g., WP635),
`STATIC: CLEAR` or `AVOIDING` while actively receiving LiDAR.

## Caveats

This is **mission-area gating**, not a fix for false object classification.
It does not make dynamic obstacles safe to ignore and cannot cover static
obstacles placed outside the configured zone. Any unmodelled static obstruction
outside the zone can be struck, because static detection there is bypassed.

The V7 maximum-speed profile (up to 57km/h) remains intact outside avoidance.
The legacy planner only starts local avoidance when close to the obstacle
and no long-range approach speed controller was added. A 57km/h-to-local
low-speed transition is NOT proven safe by these changes.

V19 adds these diagnostic fields:
- Worker CSV: `waypoint_index`, `in_static_zone`, `latch_active`,
  `static_zone_active`.
- Main CSV: `static_in_zone`, `static_route_locked`,
  `static_worker_wp`, `static_latch_state`.

Only main sends `EgoCtrlCmd`. The worker never sends vehicle commands.
