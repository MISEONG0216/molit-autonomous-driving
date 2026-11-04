# V53 — unmatched second obstacle means CAMERA lane tracking, not premature stop

Based on v52. FIRST_LEFT entry/hold and camera lane steering are unchanged.

## The observed problem
Previously, after FIRST_LEFT the second-stage template returned STOP_REQUIRED for SECOND_SURVEY_UNMATCHED_CLOSE when an observed object at ~8–15m did not match the saved second survey from a different stopped-ego pose. This was contrary to the intended behavior: stay in CAMERA lane tracking until a LiDAR match occurs.

## Changes
- SECOND_RIGHT stage returns CLEAR / WAITING_SECOND_SURVEY_MATCH or CLEAR / SECOND_SURVEY_PATTERN_MISMATCH when the expected 17.5m survey object is not recognized. The integrated main uses original CAMERA steering in that state.
- After one safe candidate scan, the first confirmation is CLEAR / SECOND_MATCH_CONFIRMING_CAMERA, not STOP_REQUIRED. Require 2 successive *fresh* matches before switching to the verified LiDAR RIGHT path.
- Position match uses a wall-relative lateral gap (approximately 3.5m from LEFT wall to the candidate object), since the second survey was recorded while ego was in a different lane. Require sufficient left wall returns; missing wall observation is a fail-stop, not a free-space assumption.
- The existing closer-than-8m current-lane obstacle STOP remains, as do full candidate swept-path collision rejection and missing-boundary STOP when proposing a right maneuver. Do not blindly continue into an object merely because a survey comparison failed.
- FIRST_LEFT geometry and FIRST_LEFT completion logic from v52 are unchanged. **Camera lane-following can still pull the car into a different lane after first path release**. Fixing selected lane identity requires checking the camera processor's target center separately and is NOT addressed by this patch.
- No change to speed limits, GPS/IMU target IP, main steering arbitration, or Python version.

## Windows test sequence
```cmd
git fetch origin
git switch --track origin/competition-v53-second-match-camera-continue-20261104
.venv38\Scripts\python.exe -m unittest discover -s tests -p "test_second_camera_continue_v53.py" -v
.venv38\Scripts\python.exe -m unittest discover -s tests -p "test_first_left_hold_v52.py" -v
.venv38\Scripts\python.exe -m unittest discover -s tests -p "test_second_survey_v49.py" -v
run_integrated_windows.bat
```

Tests committed to GitHub, not yet executed on user's Windows machine. Do not drive if failed. For supervised MORAI motion after verification, use `run_integrated_windows.bat --drive`, stop on unexpected lane selection/obstacle approach, and send fresh integrated CSV plus worker LiDAR console.

Expected new log examples:
```text
[SHADOW V53] result=CLEAR reason=WAITING_SECOND_SURVEY_MATCH
[SHADOW V53] result=CLEAR reason=SECOND_SURVEY_PATTERN_MISMATCH
[SHADOW V53] result=CLEAR reason=SECOND_MATCH_CONFIRMING_CAMERA
[TEMPLATE SELECTED] SECOND_RIGHT side=RIGHT
```

The last line is allowed only after two consecutive survey matches AND clearance validation. A nearby central obstacle, unmatched wall data, or blocked trajectory is still STOP_REQUIRED.

## Known limitations
The survey relative gap (+X forward, +Y left) is approximate, with assumptions about which returns are the left wall. Other walls/vehicles may contaminate the reference; inspect per-scan logs in MORAI before treating it as a reliable trigger. The static first obstacle and second obstacle were measured at distinct stopped vehicle poses. This is not a verified route for real-road use.
