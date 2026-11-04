# V44 — GPS-shadow adaptive return-point selection

Base: `competition-v43-shadow-path-alignment-20261104`.

## Why
In the WP3988 second-obstacle LiDAR diagnostics, rightward shifts >=2.9m produced 0 collisions in the PASS phase but still 8 collisions in RETURN, and some scans reported SIDE_WALL_NOT_OBSERVED. A shift increase alone does not solve premature merge-back. Version 44 compares alternate return start/end locations **without weakening current collision/visibility gates**.

## Changes
- Normal GPS-region planning and the first-obstacle V43 segment-alignment improvement remain unchanged.
- For each LEFT and RIGHT lateral candidate, check three merge-back start alternatives: **base, base+1.5m, base+3.0m**. Endpoints follow at least 5m later and cannot exceed the 24m planning horizon.
- Normal selectable lateral shifts remain **1.6 / 2.0 / 2.4 / 2.8m**. **2.9 / 3.0 / 3.1 / 3.2m remain diagnostic-only**, even if their geometry appears clear.
- Every (side, lateral offset, return delay) combination checks *all* current non-ground LiDAR returns against the complete 1.35m-radius swept corridor. Zero hits (or existing configured tolerance) is required.
- Each individual return endpoint must have at least three side LiDAR observations **on both sides** and fall inside the horizon. Visibility failure never authorizes movement.
- Candidate selection minimizes lateral displacement, then return delay, then prefers left on ties.
- Console prints `[SHADOW CANDIDATE]` with ENTER/PASS/RETURN hit counts, `delay`, `observed`; and `[SHADOW SELECTED]` displays the chosen offset and return delay.
- Existing emergency LiDAR STOP, 5km/h shadow maneuver cap, v41 replan-after-release correction, V43 continuous segment alignment, Windows Python 3.8.10 virtualenv, localhost IP settings and integrated main controller are preserved.
- Note that missing SIDE_WALL_NOT_OBSERVED observations can still stop the entire planner *before* candidate generation. This update does not eliminate that safety gate.

## Tests from Windows CMD

```cmd
.venv38\Scripts\python.exe -m unittest discover -s tests -p "test_shadow_return_v44.py" -v
.venv38\Scripts\python.exe -m unittest discover -s tests -p "test_shadow_diagnostics_v42.py" -v
.venv38\Scripts\python.exe -m unittest discover -s tests -p "test_shadow_alignment_v43.py" -v
```

Synthetic regression tests have been committed, but real MORAI driving and the user's local Windows tests have not yet been verified. If any test fails, do not begin a drive.

## Safe observation plan
1. Close all old worker windows to avoid UDP port conflicts.
2. With MORAI running on the same laptop, start `run_integrated_windows.bat` (DRY-RUN). Review LiDAR logs near the blocked second obstacle.
3. Compare which offset/delay has zero ENTER/PASS/RETURN hits **and** a verified return corridor. Preserve logs, screen capture and main integrated CSV. Only then consider a supervised low-speed `--drive` test.
4. A zero count does not prove legality or safety: tunnel walls, sensor occlusion, heading/pose drift, road markings and maneuver feasibility still need actual verification.

The latest official competition rules on crossing markings have not been independently verified for this change; no assumption of permission is made.
