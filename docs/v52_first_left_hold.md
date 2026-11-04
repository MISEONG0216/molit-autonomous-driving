# V52 — FIRST_LEFT passes and holds LEFT; SECOND_RIGHT only at stored LiDAR survey match

Base V51. The user-provided `integrated_v1_20261104_185641.csv` shows the first obstacle detour at WP3867, strong RIGHT steering as part of the *first locked path* well before its release, around -0.51 rad at WP3900, and first release at WP3901, followed by CAMERA lane steering ~-0.06 rad. The second template had **not activated** then; at WP3919 it returned SECOND_SURVEY_UNMATCHED_CLOSE.

## Minimal targeted code change
- `config/tunnel_predefined_templates.json` now includes `first_left_hold.enabled=true`.
- In `shadow_predefined_templates.choose_template_detour`, for `FIRST_LEFT` only, retain original LEFT entrance smoothstep but *hold positive LEFT offset* through observed obstacle plus at least 3m. End is between 17m and 20m (never past 24m horizon). There is **no built-in return to y=0/right in the first template**.
- Before selecting a path, it still checks full 1.35m footprint against ALL observed LiDAR points and confirms BOTH lateral sectors at the hold endpoint. Existing per-scan wall checks, emergency stops and control arbitration stay on.
- After safe tail release, the *existing* GPS-shadow camera lane tracker resumes. It can still choose a different lane if camera lane identity is wrong, which is a separate known limitation; no right steering suppression or blind control override is added.
- The second template continues using the 17.5m stationary-survey geometry match (15–20m forward observation window, lateral pattern, two independent consecutive scans) before the RIGHT move. A mismatched or too-close obstruction still means STOP_REQUIRED rather than a forced lane change.
- Original V48 / V49 / V50 / V51 branches remain unchanged.

## Offline point-cloud check (first survey, points_vehicle_frame(2).csv)
Using first independent scan with X-forward / Y-left filtered range Z [-1.45,1.25]:
- Nearest central object cluster X 11.40988–13.99923m, LEFT -3.15111..0.8822m.
- FIRST_LEFT shift completes ~9.40988m forward, hold ends 17.0m forward.
- Lateral 2.4m: **0** LiDAR sweep hits inside 1.35m corridor; closest observed point ~1.518m.
- Lateral 2.8m: **0** LiDAR sweep hits; closest observed point ~1.918m.
- BOTH side sectors near 17m endpoint observed (~102 LEFT points and 90 RIGHT points).
- **This is a stationary-scan, geometric-only diagnostic, NOT a moving MORAI simulation result or proof of legal lane changes**.

## Windows Python 3.8.10 test instructions

```cmd
git fetch origin
git switch --track origin/competition-v52-left-hold-20261104
.venv38\Scripts\python.exe -m unittest discover -s tests -p "test_first_left_hold_v52.py" -v
.venv38\Scripts\python.exe -m unittest discover -s tests -p "test_shadow_predefined_v48.py" -v
.venv38\Scripts\python.exe -m unittest discover -s tests -p "test_camera_handoff_v51.py" -v
run_integrated_windows.bat
```

Tests are committed but have not yet been executed on the user's Windows machine. Run a supervised `--drive` only after tests, dry run, safe stop/markings checks, and operator readiness. Save integrated CSV plus LiDAR logs. Watch first `EGO_LIDAR_NEW_LEFT`, `EGO_LIDAR_LOCKED_LEFT`, release, camera steering, and confirm `SECOND_RIGHT` begins only after the 17.5m survey pattern is matched. If the camera lane tracker itself moves across the lane before SECOND_RIGHT, investigate lane identity rather than adding a blind delay or steering lock.
