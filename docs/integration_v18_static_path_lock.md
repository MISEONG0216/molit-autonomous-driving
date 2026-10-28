# V18: Static LiDAR path lock (experimental)

Parent: `integration-v17-lidar-static-modular`. No changes to V17 are required.

## Purpose

When V17 is already avoiding a box, a tracking-ID change (e.g. 184 -> 195)
must not automatically discard the previously accepted local path. V18 keeps
the **first accepted** avoidance path until its calculated return index.

A new or re-identified object is **not ignored blindly**. On every LiDAR scan,
V18 checks every eligible tracked obstacle against the untraversed portion of
the locked path. If the obstacle intersects the corridor, V18 publishes
`STOP_REQUIRED`, with latch diagnostic `STOP_REQUIRED_PATH_BLOCKED`.

The path remains latched during a stop, and can resume when the corridor
clears. An ordinary `CLEAR` from the raw planner cannot cause early release.

## Changes

- `src/lidar/static_avoidance_path_latch.py`:
  - `LOCKED_NEW_PATH`: first path accepted.
  - `LOCKED ...`: follow unchanged path regardless of raw CLEAR or AVOIDING.
  - `LOCKED_RAW_STOP_IGNORED ...`: raw STOP from a newly numbered object,
    while the remaining locked corridor is collision-free.
  - `STOP_REQUIRED_PATH_BLOCKED`: a tracked object overlaps the corridor.
  - `RELEASED_BY_PATH`: reached original avoidance return index.
  - A new `AVOIDING` path is checked against all tracked obstacles even
    on the **first** scan (the planner ordinarily considers one selected box).
- `src/lidar/static_avoidance_module.py`: passes all tracked
  global-coordinate obstacles into the latch; uses vehicle width, safety
  margin, minimum obstacle width and height from the existing planner config.
- `tests/test_static_path_lock_v18.py`: no-MORAI tests for ID changes,
  genuine blockers, point-between-samples collision, dynamic obstacles,
  suppression of premature CLEAR releases, resuming after blockage, and
  reaching the locked path's original release index.

## Important limitations

- Geometry uses the same **width-based circular collision approximation**
  as the source planner. It is not a verified drivable-area or lane-boundary
  checker. LiDAR tracker misses can also hide an obstruction.
- The source planner still assumes a **7m drivable corridor**. Check actual
  permitted lane limits separately.
- V17's `CLEAR` state retains V7 global-path speed profile up to
  `--max-speed 57`. V18 does **not** add advance braking to 25km/h.
  The local avoiding speed is still capped by V17 configuration
  (`test_max_speed_kmh`, usually 5 or 10) and the planner's 8km/h request.
  A 57km/h approach to a 15m trigger is **not validated** and may be unsafe
  even in simulation.
- LiDAR result timeout, STOP_REQUIRED braking, traffic priority and the
  single EgoCtrlCmd sender remain unchanged.

## Ubuntu checkout

```bash
cd ~/molit-autonomous-driving
git fetch origin
git switch integration-v18-static-path-lock
git pull --ff-only origin integration-v18-static-path-lock
source .venv/bin/activate
python3 -m unittest discover -s tests -p 'test_static_path_lock_v18.py' -v
python3 -m unittest discover -s tests -p 'test_static_integration_v17.py' -v
```

If Git refuses to switch due to local config edits, save them outside the
repo or stash only those specific files. Do not reset or discard them.

## Functional test (start with DRY-RUN)

Terminal A:
```bash
python3 src/lidar/static_avoidance_module.py
```

Terminal B:
```bash
python3 src/main_integrated_v1.py --static --traffic --max-speed 10
```

For traffic integration run `src/traffic_light_detector.py` in a separate
terminal. DRY-RUN never sends EgoCtrlCmd.

Examine `logs/static_worker_*.csv` fields `raw_status`, `status`,
`latch_state`, `obstacle_id`, `path_points` and the matching
`integrated_v1_*.csv` from the main process. If the worker reports
`STOP_REQUIRED_PATH_BLOCKED`, this is a deliberate geometric stop, not
the old tracking-ID stop.

Once dry-run and safe low-speed driving both pass, compare with the
previously failing WP 635-665 scenario; inspect the lateral route visually.
Never run the legacy LiDAR drive sender concurrently with main.
