# V49 — Second stationary-survey match triggers RIGHT HOLD, then independent return

Base: v48. This version changes **ONLY the second obstacle** behavior and the second-stage worker state. FIRST_LEFT templates, camera GPS-shadow lane tracking, main vehicle controller, emergency dynamic pedestrian tracking, 5km/h shadow speed, and localhost sensor/control IPs remain unchanged.

## Prior failure
At the second obstacle, v48 produced RIGHT 2.4m/2.8m candidate `BEYOND_HORIZON`, with predicted return end 32.5–35.5m, above the 24m LiDAR planning limit. A simple max-horizon increase would trust unseen wall and road.

## New second obstacle trigger
- Normal GPS-shadow steering continues with existing camera lane control.
- Only after FIRST_LEFT route is *released*, evaluate SECOND_RIGHT, using each new complete LiDAR scan in vehicle frame `X forward, Y left`.
- Match the second saved stationary-survey pattern: object forward 17.5m (allow ±2.5m), lateral centroid ~+1.9m (allow ±1.6m), at least 8 returns with ≥1.1m lateral extent, a compact longitudinal group ≤2.5m, inside observation window X=15–20m and Y=0–4m.
- Require **two consecutive scan matches** before choosing a path; never trigger on a GPS/DR waypoint or a fixed delay.
- When pattern is absent, do not create the detour; continue camera lane tracking only while existing independent forward-emergency LiDAR guard allows. A close central obstacle (<8m and enough returns) triggers STOP rather than blind continue.
- Before choosing a route require side-sector scans and observed far-end walls. For both existing offsets **2.4m/2.8m RIGHT**, verify a complete smooth lateral shift (from 1.5m to 9m forward) and HOLD the right offset until **23m forward**. Every known point within the 1.35m footprint clearance rejects the candidate. 23m is within 24m maximum plan horizon; this is NOT permission to drive blind to 23m.

## Return phase
- At the end of a verified right-hold path, only after original fresh forward LiDAR safety gate, generate a short 12m return beginning from the *current dead-reckoned pose*, leftward by the **same offset actually chosen** on outbound route.
- Verify both side visibility, return-end observation and the original `check_wall_clearance` swept-footprint and path-alignment checks. If rejected, report STOP_REQUIRED, remain in the second stage and retry using fresh scans. Do NOT jump straight to camera lane steering or mark the mission completed.
- After the verified return finishes and the original release safety test passes, mark second stage complete and switch to camera lane steering. GPS recovery still restores global path tracking.
- Dead reckoning may drift and LiDAR absence is not proof of a free lane. Driver must verify legal lane boundary crossings and walls in MORAI.

## Test first (Windows CMD)
```cmd
git fetch origin
git switch --track origin/competition-v49-second-survey-right-hold-20261104
.venv38\Scripts\python.exe -m unittest discover -s tests -p "test_second_survey_v49.py" -v
.venv38\Scripts\python.exe -m unittest discover -s tests -p "test_shadow_predefined_v48.py" -v
run_integrated_windows.bat
```
Tests committed to GitHub but NOT yet proven to pass on the user's Windows Python3.8.10 environment. Do not drive until these are green. Full simulator and safety tests still required.

Expected logs:
- `[TEMPLATE COMPLETED] FIRST_LEFT completed=1`
- `[SHADOW V49] ... reason=WAITING_SECOND_SURVEY_MATCH` (camera lane)
- `[V49 MATCH] confirmed=2 ...`
- `[TEMPLATE SELECTED] SECOND_RIGHT side=RIGHT ...` (LiDAR verified hold)
- `[V49 RETURN] LiDAR-verified LEFT merge-back selected`, or `STOP ...`.
- `[TEMPLATE COMPLETED] SECOND_RETURN completed=2` only after success.

This phase is for low-speed supervised MORAI exploration, not a validated production lane change. No changes to competition-rules assumptions.
