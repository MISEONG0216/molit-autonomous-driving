# V45 — Five seconds after the first LEFT tunnel obstacle detour, attempt RIGHT lane shift

Base: `competition-v44-shadow-adaptive-return-20261104`.

## Evidence from uploaded 2026-11-04 run (v44)
`integrated_v1_20261104_172639.csv` shows:
- First LEFT path released: ~42.781s, WP3865 (`EGO_LIDAR_TAIL_RELEASED`).
- 5.0s after release: ~47.781s, before the next obstacle region.
- Second hazard STOP started at ~61.931s, WP3987, with `SIDE_WALL_NOT_OBSERVED`, later dynamic-track and no-free-side stops.

This suggests an earlier lane transition could help, but **does not prove the right lane is physically open**.

## What V45 does
1. When the *first* LEFT GPS-shadow LiDAR detour safely releases, mark the current monotonic time once. This is not measured from entering the tunnel or from starting the first detour.
2. Wait 5 seconds. Within the scenario-specific approximate waypoint window WP3850–3970, if the current LiDAR planning result is `CLEAR`, try a one-time RIGHT lane-change.
3. Generate smooth forward-local paths with **RIGHT** lateral offsets 2.4 and 2.8m, over 10m lateral-shift length, and hold offset until the end of an 18m local path. The path does **not** immediately return to the previous lane.
4. Before sending any local path, check independent forward 1.5–6m emergency obstacle gate, *all observed* LiDAR obstacle/wall returns along the entire shifted vehicle swept path, current path alignment and both lateral wall sectors, with existing 0-hit and 1.35m safety-margin defaults.
5. If no candidate is verified, **do not force a lane change**; continue existing normal CLEAR/STOP_REQUIRED logic. Recheck while eligible. If verified, start one locked RIGHT path (`EGO_LIDAR_NEW_RIGHT_TIMED`); all following scans use original v44 wall and alignment revalidation. Dynamic-track stops remain active.
6. Keep vehicle command output only in the integrated main, and retain GPS shadow detour target 5km/h. Do not increase speed. The timer does not bypass any STOP request.

## Important limitations
- LiDAR observed clearance does NOT independently show that the destination is a legally permitted right lane. No dedicated camera lane-boundary legality check was added. It is a MORAI scenario-specific experiment.
- Dead-reckoning waypoints and heading can drift in the tunnel. WP3850–3970 is a guard, not reliable global localization.
- The integrated camera lane-centering controller may resume at the new position after the local path is released, but its target lane choice remains unverified.
- If sensors cannot verify the maneuver, the vehicle should *not* move right at exactly five seconds.
- **Do not** bypass wall checks, change the collision margin or disable emergency stop. Official event rules governing lane crossings and tunnel maneuvers have not been independently verified.

## Tests and run (Windows CMD, Python 3.8.10)
```cmd
git fetch origin
git switch --track origin/competition-v45-shadow-right-shift-5s-20261104
.venv38\Scripts\python.exe -m unittest discover -s tests -p "test_shadow_right_shift_v45.py" -v
.venv38\Scripts\python.exe -m unittest discover -s tests -p "test_shadow_return_v44.py" -v
run_integrated_windows.bat
```
Stop all old V44 workers first, ensure GPS/IMU localhost settings are preserved and use Dry-run first. After tests and sensor logs are verified, a supervised low-speed experiment can be started with `run_integrated_windows.bat --drive`; halt immediately if the path approaches a wall or obstacle.

Expected console events:
```text
[RIGHT SHIFT] First LEFT detour released; waiting 5.0s
[RIGHT SHIFT] elapsed=5.1s wp=... status=AVOIDING reason=RIGHT_SHIFT_VERIFIED candidates=...
[STATIC] ... latch=EGO_LIDAR_NEW_RIGHT_TIMED ... side=RIGHT
```

The exact event times and success are NOT yet verified by MORAI; synthetic tests are committed but have not been run on the user's Windows environment.
