# V17 — Modular MORAI LiDAR static obstacle integration

**Integration base:** `integration-v16-fifth-left-combined` (traffic #5 verified).
**Source:** `lidar_object_detection/src/lidar/lidar_static_avoidance_drive_5kph.py`.
**Hardware:** Windows MORAI at `172.20.10.2`, Ubuntu at `172.20.10.5` for the current session.

## Data flow (single vehicle command owner)

```
Windows MORAI
  GPS:9201 ─────────────────────────┐
  IMU:1111 ─────────────────────────┤
  Competition Status:1909 ──────────┤
                                   ▼
                        main_integrated_v1.py
                        V7 path + traffic + merge
                          │                 ▲
          pose x,y,yaw:9302                 │ static plan:9303
                          ▼                 │
                    static_avoidance_module.py
                    LiDAR:1116 -> DBSCAN
                    -> tracker + ego motion
                    -> static planner + original path latch
                    -> CLEAR / AVOIDING / STOP_REQUIRED
                          │
                 NEVER sends EgoCtrlCmd
                                   ▲
                        main: only CtrlCmd sender
                        Windows:9093
```

All pose/plan worker IPC uses localhost UDP; no extra network messages to MORAI.
Other V16 traffic/merge sensor channels remain unchanged.

## Branch contents

- `src/lidar/static_avoidance_module.py`: separate worker, no GPS/IMU bind, no control sender.
- `src/lidar/static_avoidance_path_latch.py`: original local-path latch extracted from 5-km/h program.
- `src/lidar/static_obstacle_planner.py`, `src/lidar/obstacle_adapter.py`, `src/lidar/object_detection/{object_detector,obstacle,tracker}.py`: copied from original source branch, unchanged.
- `src/integration/static_avoidance_io.py`: validates local paths and calculates original 2.5m local Pure Pursuit steering.
- `src/main_integrated_v1.py`: `--static` observe-only, `--static-control` apply planned steering/speed/stop.
- `tests/test_static_integration_v17.py`: offline regression tests for parser, local steering, main syntax.

## First-start protocol

Check Windows MORAI LiDAR Publisher **destination `172.20.10.5:1116`** and competition-network settings. Avoid running the old `lidar_static_avoidance_drive_5kph.py --drive` simultaneously: it is another vehicle command sender.

```bash
cd ~/molit-autonomous-driving
git fetch origin
git switch integration-v17-lidar-static-modular
git pull --ff-only origin integration-v17-lidar-static-modular
source .venv/bin/activate
python3 -m unittest discover -s tests -p 'test_static_integration_v17.py' -v
```

Start these **four Ubuntu terminal sessions**:

1. `python3 src/traffic_light_detector.py`
2. `python3 src/merge_risk_monitor.py`
3. `python3 src/lidar/static_avoidance_module.py`
4. **First observe without vehicle control:** `python3 src/main_integrated_v1.py --traffic --merge --static --max-speed 5`

Only after the LiDAR worker outputs fresh `CLEAR` or `AVOIDING` and main reports `STATIC ... age<3.5s`, start the main test **in a closed MORAI test scene, with vehicle stationary**:

```bash
python3 src/main_integrated_v1.py --drive --traffic --merge \
  --merge-control --static --static-control --max-speed 5
```

**Important:** The full course is capped to 5 km/h while `--static-control` is set, even if `--max-speed` is accidentally set to 57. This is **not** a production/high-speed version.

`--static` by itself never changes the V16 global steering or target speed; it only logs worker results. The worker can run with the main in DRY-RUN, as long as MORAI sends LiDAR scans.

## Arbitration and fail-safe

1. V7 computes the global nearest waypoint and global steering.
2. MissionManager applies traffic/merge speed restrictions or stop.
3. Fresh LiDAR `CLEAR`: retain global steering; cap at 5 km/h if control enabled.
4. Fresh LiDAR `AVOIDING`: compute steering along supplied local global-XY path; cap at 5 km/h and recommended speed.
5. Fresh `STOP_REQUIRED`, missing/stale (>3.5 seconds) result, invalid local path or no forward Pure Pursuit lookahead: command full brake, no acceleration.
6. Traffic RED or merge STOP retain priority over any local path.
7. `main_integrated_v1.py` is the **only** process transmitting `EgoCtrlCmd`.

The original planner's configured drivable width is **7m** and candidate lateral offsets reach **±2.2m**. **This does not prove single-lane compliance**. Check visually that no simulated lane boundary is crossed before expanding scenarios/speed. The first low-speed test should use a controlled single static box and open course rather than a full 57km/h competition lap.

## What to verify

- MORAI Windows → Ubuntu: raw UDP 1116 actually arriving.
- LiDAR worker: `[STATIC] ... CLEAR` then `AVOIDING` with path > 0.
- Main: `STATIC : AVOIDING ... AVOID=True STOP=False`; steering changes.
- After obstacle: `CLEAR` and `AVOID=False`; global steering resumes.
- If local path impossible: `STOP_REQUIRED` → `STATIC_FULL_BRAKE`.
- Remove LiDAR packets: after timeout → `VISION_WAIT` → `STATIC_FULL_BRAKE`.
- Terminal Ctrl+C sends stop through the main loop.
- Logs: `logs/integrated_v1_*.csv` and `logs/static_worker_*.csv`.
- Leave the 5th signal in the same RED/ordinary-green/left-green sequence; `GO_LEFT` should remain functional.

**Status:** committed integration candidate; unit tests and actual MORAI low-speed drive must still be run on the user's Ubuntu + Windows simulator setup.
