# V41: GPS-shadow replanning after first avoidance

Base: `competition-v40-shadow-bidirectional-adaptive-20261104`

## Reproduced evidence (uploaded V40 integrated CSV, Nov 4, 2026)
- 744 control rows from WP3684 to WP4033.
- First GPS-shadow avoidance begins at ~18.10 s / WP3824: `EGO_LIDAR_NEW_LEFT`.
- The selected locked detour holds a **5 km/h** target: intended shadow-specific safety cap, not a sensor failure.
- At ~30.03 s / WP3852 `EGO_LIDAR_TAIL_RELEASED` appears. It remains the reported state for the subsequent **370** CSV rows all the way to WP4033, while `static_state=CLEAR`, path length=0 and LiDAR scan IDs still advance.
- This is strong evidence of a state-machine defect: `latch_state` was not reinitialized each LiDAR scan. `if shadow_locked_path is None and latch_state != "EGO_LIDAR_TAIL_RELEASED"` therefore skipped all new planning after the first release.
- CSV alone does not timestamp the exact physical collision or establish obstacle geometry; do not claim second car detection or successful clearance.

## Changes
1. Reset the `latch_state` result at the start of each GPS-shadow scan; a genuine tail release still skips replanning **that same scan**, but the next fresh scan may begin another obstacle avoidance.
2. Independently check short-range raw LiDAR returns (1.5-6.0 m, configured vehicle half-width + safety margin) while about to report `CLEAR`. If blocked, publish `STOP_REQUIRED`, even if the planner output erroneously says clear.
3. Preserve V40 adaptive left/right alternatives, locked-maneuver wall sweeps, normal GPS planning, 5 km/h detour speed and Windows `.venv38` launcher.
4. Add isolated regression test `tests/test_shadow_replan_v41.py`.

## Test (Windows CMD)
```cmd
.venv38\Scripts\python.exe -m unittest discover -s tests -p "test_shadow_replan_v41.py" -v
.venv38\Scripts\python.exe -m unittest discover -s tests -p "test_shadow_adaptive_v40.py" -v
```

Test commands are published, but not yet executed in the user's installed MORAI environment. Run `run_integrated_windows.bat` (dry-run) before any supervised test drive.

## Limitations
- The LiDAR emergency stop is only a near-field fallback, not proof of obstacle detection, full safety coverage or stopping-distance adequacy.
- The second obstacle may also be poorly represented in LiDAR scan/ROI; enable `STATIC_CSV_LOG=1` and inspect worker logs/point clouds if it still approaches without a safe route.
- GPS dead-reckoning drift and road-marking legality remain unverified. Do not increase detour speed until repeated low-speed tests succeed.
