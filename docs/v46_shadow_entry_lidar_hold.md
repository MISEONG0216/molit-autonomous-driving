# V46 — Stop at GPS-shadow entry and check LiDAR observations

Base: **v44**, not v45. Preserves the successful first-LEFT and subsequent RIGHT planning logic from V44 and V43's segment alignment. V45 timer-driven early right lane change is intentionally NOT included.

## Behavior
- First valid LiDAR frame after `gps_shadow` becomes true: start one-time entry observation state.
- While observation is pending, worker reports `STOP_REQUIRED` with `ENTRY_OBSERVE_*` reason; integrated main applies its existing full-brake policy. Do not generate and latch an avoidance path while waiting.
- Require >=2s elapsed; pose nearly stationary for >=0.8s; >=5 consecutive fresh LiDAR frames with 40+ forward non-ground points, >=8 returns on EACH lateral side, and no returns within 8m in current 1.35m corridor.
- Only when ready resume original V44 GPS-shadow planning, with full sweep, endpoint visibility and emergency stops. No unsafe bypass. Driver must still verify first/second obstacle behavior visually.
- If close obstacle or side visibility remains poor, STOP_REQUIRED persists. Stationary sampling cannot recover physically occluded walls; this step detects insufficient scans, not reconstructs an unknown free-space map.
- The "stop" inference uses estimated pose displacement per LiDAR scan (0.08m and 3 degrees thresholds) because the worker's pose packet has no independent wheel speed. GPS-shadow dead reckoning can drift. Actual stopped state should be cross-checked with simulator speed in integrated CSV.
- This triggers on GPS-shadow activation, which may not coincide exactly with physical tunnel entrance. If the car enters GPS shadow too near an obstacle to stop safely, the simulated run must be stopped; do not remove close-object conditions.

## Windows CMD, Python 3.8.10
```cmd
git fetch origin
git switch --track origin/competition-v46-shadow-entry-lidar-hold-20261104
.venv38\Scripts\python.exe -m unittest discover -s tests -p "test_shadow_entry_hold_v46.py" -v
.venv38\Scripts\python.exe -m unittest discover -s tests -p "test_shadow_return_v44.py" -v
run_integrated_windows.bat
```

Tests are committed, not yet run in the user's MORAI Windows environment. Do not do driving test if failed.

Expected console:
```
[SHADOW ENTRY] Begin LiDAR stop-and-observe at GPS shadow activation
[SHADOW ENTRY] scan=... ready=False reason=WAIT_STOP {...}
[SHADOW ENTRY] scan=... ready=False reason=WAIT_TIMER {...}
[SHADOW ENTRY] scan=... ready=True reason=READY {...}
```

Only after verifying stationary behavior and sensor reception, supervised low-speed test can be requested via `run_integrated_windows.bat --drive`. Keep and compare MORAI recording, `integrated_v1_*.csv`, and LiDAR logs. No assumption of compliance with unverified competition rules.
