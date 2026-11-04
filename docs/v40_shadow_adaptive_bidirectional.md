# V40 GPS-shadow adaptive left/right avoidance

Base branch: `competition-v39-bidirectional-static-avoid-20261104`.

## Motivation
In the user's 2026-11-04 MORAI tunnel test around WP3867, the main controller reported `MODE=SHADOW`, `STATIC=STOP_REQUIRED`, and zero local-path points. In V39 the **regular GPS** static planner already checks left and right, but GPS shadow uses `src/lidar/shadow_ego_planner.py` with a fixed 1.6 m side shift and fixed 10–15 m return. V40 modifies **only the GPS-shadow local candidate generation**, not the GPS-active static planner, GPS/IMU UDP settings or the vehicle controller.

## Algorithm
- Input: fresh vehicle-frame LiDAR points (`+x` forward, `+y` left), as in V39.
- Fail-stop for close obstacle, insufficient scan or missing side observations.
- For each detected frontal obstacle, calculate shift-end before obstacle and return-start after observed obstacle extent; cap path horizon at 24 m.
- Try both `LEFT` and `RIGHT` lateral shifts of 1.6, 2.0, 2.4, 2.8 m.
- For every proposal check *all* LiDAR returns (not just narrow DBSCAN clusters) against the entire path swept by vehicle half-width 0.9 m plus margin 0.45 m.
- A delayed return must be supported by side-wall observations near its endpoint; otherwise fail-stop (`RETURN_CORRIDOR_UNOBSERVED`).
- Choose the smallest safe shift (LEFT on equal scores) without asserting that observed LiDAR clearance is sufficient to cross road markings.
- No safe candidates => `STOP_REQUIRED`. One accepted detour is **locked** and checked anew every scan by the existing V39 wall guard; no unsafe side-switching mid-detour.
- Existing shadow avoidance speed cap remains **5 km/h**, and no LiDAR worker sends vehicle control commands.

## Diagnostics
The `V40 Static LiDAR` CMD prints records such as:

```text
[SHADOW V40] ... result=STOP_REQUIRED reason=NO_FREE_SIDE
[SHADOW CANDIDATE] LEFT 1.6m SWEEP_COLLISION hits=... clearance_min=...
[SHADOW CANDIDATE] LEFT 2.0m SAFE hits=0 ...
```

The lines above are examples, NOT evidence from the user's prior actual scan.

## Test (Windows CMD, Python 3.8 virtualenv)
```cmd
.venv38\Scripts\python.exe -m unittest discover -s tests -p "test_shadow_adaptive_v40.py" -v
.venv38\Scripts\python.exe -m unittest discover -s tests -p "test_shadow_ego_v35.py" -v
```

## Launch
Switch branch while keeping uncommitted local IP changes safe. The repo's V39 Windows batch launcher and Python 3.8 virtualenv are unchanged.
```cmd
git fetch origin
git switch --track origin/competition-v40-shadow-bidirectional-adaptive-20261104
run_integrated_windows.bat
```
This launches a **dry run**. Stop all child worker windows before restarting; verify camera, LiDAR, and IMU are live. Use `run_integrated_windows.bat --drive` only after static checks and under MORAI operator supervision. The launcher itself caps the main maximum speed to 20 km/h; actual GPS-shadow avoidance is capped at 5 km/h.

## Limits
A screenshot cannot establish drivable road width, and LiDAR point proximity is not a legal lane boundary or a collision-proof free-space map. Camera lane boundaries, sensor blind spots, pose drift, and steering reachability are not yet explicitly validated. A 2.8 m shift may require the opposing lane and therefore MUST be validated against the competition's latest official rules and on MORAI before use. A candidate may still be rejected, which is correct when safety cannot be shown. Offline synthetic tests do not establish real driving safety.
