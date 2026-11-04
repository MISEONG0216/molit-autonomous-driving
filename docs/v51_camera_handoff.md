# V51 — No forced return after SECOND_RIGHT; diagnose the post-FIRST camera handoff

Base: V50, whose main controller and the first-left template are preserved.

## Evidence from user-supplied 18:49 run
`integrated_v1_20261104_184903.csv`:
- WP3866 t=34.698: FIRST_LEFT LiDAR AVOIDING started (44 points).
- WP3900 t=48.307: FIRST_LEFT released. At t=48.23 the steering was ~-0.503rad from LiDAR local PP; immediately afterward steering returned to camera ~-0.05rad.
- WP3901..3924: worker state CLEAR (not AVOIDING RIGHT), and `steering_rad` matched `lane_steering_rad` exactly. Thus visible movement to the right during this interval was **camera lane output, not SECOND_RIGHT template**.
- WP3924 t=52.539: STOP_REQUIRED / SECOND_SURVEY_UNMATCHED_CLOSE.
- WP3925 t=53.245: STOP_REQUIRED / SECOND_APPROACH_TOO_CLOSE.
- **No `EGO_LIDAR_NEW_RIGHT` appeared in this run**.

This matters: simply delaying the SECOND_RIGHT template cannot prevent the CAMERA lane follower from moving right after FIRST_LEFT. The camera processor selects a lane center; proving a specific intended lane identity/marking requires new perception diagnostics. Do not claim this patch solves that separate issue.

## Changes
- Preserve FIRST_LEFT geometry, GPS flicker tolerance, obstacle survey match around 17.5m, two fresh matching scans, and LiDAR candidate clearance.
- After RIGHT HOLD reaches its endpoint, only once existing tail-end pose test and current LiDAR 6m forward-clear test pass, release to original camera lane tracking. Remove V49's forced 12m LEFT merge-back.
- If fresh LiDAR near the endpoint cannot clear, **STOP_REQUIRED persists**. The planner still refuses an unverified right-side path. No fixed-time steering.
- Leave main integrated steering logic unchanged to preserve the measured FIRST_LEFT performance. Its camera lane follower already resumes at LEFT release. Assess lane center selection as separate issue.

## CMD (Python 3.8.10)
```cmd
git fetch origin
git switch --track origin/competition-v51-camera-handoff-after-second-20261104
.venv38\Scripts\python.exe -m unittest discover -s tests -p "test_camera_handoff_v51.py" -v
.venv38\Scripts\python.exe -m unittest discover -s tests -p "test_second_survey_v49.py" -v
run_integrated_windows.bat
```

Tests are committed but not verified in Windows yet. Do not commence driving if they fail. Observe dry-run logs. For any supervised slow MORAI test, stop at unexpected steering and retain CSV / LiDAR worker log.

**Next independent investigation needed:** Near WP3900, compare camera detected left/right lane lines and target lane center before and after LiDAR path release, and compare `lane_steering_rad` with `steering_rad`. The measured rightward movement was not a LiDAR SECOND_RIGHT action.
