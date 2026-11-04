# V42: diagnostic-only tunnel offset experiments

Base: `competition-v41-shadow-replan-after-release-20261104`.

## Reason for change

At the first tunnel obstacle near WP3865, V41's LiDAR planner reported these **per-candidate scan hits**:
- LEFT 1.6m: 69
- LEFT 2.0m: 34
- LEFT 2.4m: 6
- LEFT 2.8m: 5

These are not proof that 2.9m or 3.0m clears the obstacle. Also some scans rejected planning because the distant return corridor was unobserved (LEFT=0 points, RIGHT=68 or 69 near ~19m).

## Behavior

- Existing **drivable** GPS-shadow offsets stay `[1.6, 2.0, 2.4, 2.8]` metres.
- New `diagnostic_offsets_m = [2.9, 3.0, 3.1, 3.2]` on LEFT **and** RIGHT are evaluated and printed with `[DIAGNOSTIC ONLY]`, but **never added to the selectable steering paths**.
- All candidates use the previous full swept-footprint check, half vehicle width 0.9m + 0.45m margin, no allowed collision points.
- For each return, the nearest path segment determines whether the collision is on `ENTER` (lateral shift), `PASS` (passing obstacle) or `RETURN` (merging back). Logs show each segment's hit count and minimum distance.
- When `RETURN_CORRIDOR_UNOBSERVED`, calculate collision diagnostics **but still stop**. Candidate results do not override missing scan coverage.
- Previous V41 tail-release fix and close-forward emergency check retained.
- Main controller and `run_integrated_windows.bat` unchanged. Current GitHub network JSON explicitly uses localhost for GPS, IMU and control, matching the user's successful one-laptop configuration.

## CMD usage

Stop all existing V41 workers first. Preserve any unrelated uncommitted local edits before switching.

```cmd
cd /d "%USERPROFILE%\Desktop\molit-autonomous-driving"
git fetch origin
git switch --track origin/competition-v42-shadow-offset-diagnostics-20261104
.venv38\Scripts\python.exe -m unittest discover -s tests -p "test_shadow_diagnostics_v42.py" -v
.venv38\Scripts\python.exe -m unittest discover -s tests -p "test_shadow_adaptive_v40.py" -v
run_integrated_windows.bat
```

The launcher defaults to Dry-run. Keep any subsequent real driving supervised in MORAI, and record the V42 Static LiDAR console logs (including `[SHADOW CANDIDATE]`) and main CSV. Compare collision points by segment and actual LiDAR visibility before enabling any larger offset.

## Limitations

- Offline regression tests are committed but not yet executed against the user's Windows 3.8 environment.
- A low collision hit count is not proof of safety: blind spots, tunnel wall, lane boundaries, heading changes and motion dynamics remain unverified.
- The 2.9-3.2m paths are strictly diagnostic and not approved for automatic vehicle motion.
