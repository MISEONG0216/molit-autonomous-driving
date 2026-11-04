# V43 — GPS-shadow path alignment by line segments

Base: `competition-v42-shadow-offset-diagnostics-20261104`.

## Observed failure
At the second tunnel obstacle, v42 worker output:
- `scan=208 EGO_LIDAR_NEW_RIGHT`
- `scan=209 EGO_LIDAR_LOCKED_RIGHT`
- `scan=215 EGO_LIDAR_BLOCKED_PATH_NOT_ALIGNED`

This proves the locked path's *alignment check* rejected continued travel. It does not prove the vehicle was geometrically safe or that a bigger lateral offset is needed.

## Changes
- `shadow_wall_guard.path_alignment_metrics`: project the current vehicle pose (origin of ego frame) onto each **line segment** instead of only measuring waypoint/vertex distances. This avoids a false error when the car is between spaced waypoints.
- `static_avoidance_module.py`: on subsequent scans, revalidate from `max(0, nearest-1)`, retaining the segment immediately preceding the nearest waypoint. Otherwise slicing from `nearest` can discard the segment beneath the car.
- `[SHADOW ALIGN]` logs show segment distance, vertex distance, nearest segment and closest point in the current vehicle frame, and reason for rejection.
- **No increase** to `max_path_pose_error_m=1.8`, no changes to 5km/h shadow maneuver speed or LiDAR swept collision checks. Candidate offsets 2.9–3.2m remain diagnostic-only. No modifications to single-laptop network IP or Windows `.venv38` launcher.
- This logged alignment error uses the **existing dead-reckoning pose**; it is a *tracking error in that pose frame*, NOT an independent estimate of absolute localization drift. True pose uncertainty needs independent ground truth / other sensors.

## Tests
From Windows CMD at repo root:
```cmd
.venv38\Scripts\python.exe -m unittest discover -s tests -p "test_shadow_alignment_v43.py" -v
.venv38\Scripts\python.exe -m unittest discover -s tests -p "test_shadow_wall_v34.py" -v
.venv38\Scripts\python.exe -m unittest discover -s tests -p "test_shadow_diagnostics_v42.py" -v
```
These tests are committed but **not yet verified in the user's Windows MORAI installation**. Test first and inspect failures before driving.

## Validation
Dry-run `run_integrated_windows.bat` after ensuring all old workers are closed. To inspect live alignment near the second obstacle, first collect the `[SHADOW ALIGN]` messages (particularly just before a STOP_REQUIRED), alongside the main CSV and MORAI recording. Do not disable safety-stop checks or assume a zero-point candidate is legally safe to drive; wall/road boundary and pose drift remain unverified.
