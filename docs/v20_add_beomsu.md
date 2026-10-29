# V20 Add Beomsu — conservative lane-clearance experiment

Base: `integration-v20-lane-tuning` (unchanged V7, traffic-light, merge, LiDAR static-zone logic). Only `v20-add-beomsu` has these modifications. **Not validated in MORAI.**

## What changed
- `src/integration/beomsu_lane_guard.py`: generate a separate smooth Pure Pursuit tracking path; original official global path, waypoint mission indexes, speed profile and LiDAR reference remain unchanged.
- Known first video area: WP1480–1715 with taper WP1480–1530 and WP1655–1715. Positive offsets move right of travel, away from the left yellow median at WP1533/1578/1655. This first-zone mapping is from existing V22 source comments, **not independently measured from a CSV in this session**.
- Optional manually mapped extra ranges: `--beomsu-extra-zone START:END:SIGNED_METRES`, where negative means left; intended for tunnel approach/interior **only after the corresponding waypoint ranges and safe free space are measured**. No unverified tunnel bias is enabled.
- Offset size bounded to 0.50m, smoothed over each ramp, and target speed capped at 20km/h during nonzero offset. A 20km/h cap does **not** guarantee line or wall clearance.
- Extra CSV fields: `beomsu_offset_m`, `beomsu_tracking_dist_m`, `beomsu_zone_active`; existing lane diagnostics remain relative to official path.
- New offline test: `tests/test_beomsu_lane_guard.py`.

## Missing run CSV
No actual run `.csv` file was present in the examined V20 repository tree. **GitHub cannot recover a CSV that was only written on a team's laptop and never committed or uploaded.** The existing V20 controller creates `logs/integrated_v1_YYYYMMDD_HHMMSS.csv` locally when run.

## Windows 11 installation and supervised test (30 km/h configuration)

`config/integration_network.json` in **this branch** sets MORAI CtrlCmd to `127.0.0.1:9093`. This is correct ONLY when MORAI and the integrated Python controller run on the same computer, with the simulator configured to receive on port 9093. If the MORAI host is elsewhere, change the address to that host's actual IP.

Use Windows CMD. Close previous driving-control processes before starting this branch. **Only one process may send EgoCtrlCmd**. Keep MORAI stopped/paused until ready.

### CMD 1: get a clean copy (if a copy is already present, skip cloning and use the pull commands)
```bat
cd /d "%USERPROFILE%"
git clone --branch v20-add-beomsu --single-branch https://github.com/MISEONG0216/molit-autonomous-driving.git molit-beomsu-test
cd /d "%USERPROFILE%\molit-beomsu-test"
git pull --ff-only origin v20-add-beomsu
powershell -NoProfile -ExecutionPolicy Bypass -File setup_windows.ps1
.\.venv\Scripts\python.exe -m unittest discover -s tests -p "test_beomsu_lane_guard.py" -v
```
If `molit-beomsu-test` is already cloned, omit the `git clone` command and instead run `cd /d "%USERPROFILE%\molit-beomsu-test"` followed by `git pull --ff-only origin v20-add-beomsu`.

Check MORAI GPS UDP9201, IMU UDP1111, Competition Status UDP1909, front camera UDP9291, LiDAR UDP1116 and controller UDP9093. Set vehicle Ego mode AV / ExternalCtrl. Make sure the appropriate sensors are enabled. The setup script installs Python dependencies and MORAI NetworkModule; it does NOT configure the MORAI simulator itself.

### CMD 1: traffic worker (leave running)
```bat
cd /d "%USERPROFILE%\molit-beomsu-test"
.\.venv\Scripts\python.exe src\traffic_light_detector.py
```

### CMD 2: static LiDAR worker (leave running)
```bat
cd /d "%USERPROFILE%\molit-beomsu-test"
.\.venv\Scripts\python.exe src\lidar\static_avoidance_module.py
```

### CMD 3: main DRY-RUN (does not transmit vehicle control)
```bat
cd /d "%USERPROFILE%\molit-beomsu-test"
.\.venv\Scripts\python.exe src\main_integrated_v1.py --traffic --static --static-control --max-speed 30 --beomsu-first-right-m 0.25 --beomsu-roundabout-right-m 0.20 --beomsu-roundabout-kmh 12
```
Ensure GPS, IMU, speed, camera and LiDAR feeds are valid. Ctrl+C to end **only CMD 3** after checking logs; leave the two workers running.

### CMD 3: controlled MORAI DRIVE (after verifying inputs and safety)
```bat
.\.venv\Scripts\python.exe src\main_integrated_v1.py --drive --traffic --static --static-control --max-speed 30 --beomsu-first-right-m 0.25 --beomsu-roundabout-right-m 0.20 --beomsu-roundabout-kmh 12
```
No other driving code may be running. Supervise and stop on any guardrail/lane issue. `--max-speed 30` caps normal driving; the **first modified waypoint zone** is capped at 20 km/h whenever its right offset is nonzero. The V7 curve/mission speed may be lower. This does not detect yellow lane lines in real time. The tunnel area is **NOT automatically corrected**.

Log: `logs/integrated_v1_YYYYMMDD_HHMMSS.csv`. Video + CSV WP comparison is needed to evaluate the fix. Set 0.25m only as the first experiment, not as a verified clearance value.

## New experimental roundabout fine-tuning (2026-10-30)

**IMG_5046.mp4** displays crossing of roundabout-exit painted guide/hatch markings around video t=54s and left yellow line close contact around t=57s. The visual recording is NOT timestamp-synchronized with a controller CSV; these video seconds are not waypoint indexes.

This branch ports the **optional** previous V22 modules, based on V22 CSV runs `20261029_222543` and `20261029_225509`, which recorded repeatable left-of-reference-path deviation near WP1870-1885.

- `--beomsu-roundabout-right-m 0.20`: a smooth 0→0.20m **rightward tracking-only shift** across WP1859→1870, held WP1870→1885, fades out by WP1902. This is a test hypothesis, not a verified marking clearance. The WP1480-1715 guardrail fix is untouched.
- `--beomsu-roundabout-kmh 12`: **prebrake before** WP1835 using route-distance preview, hold up to 12km/h through WP1892, gradually release thereafter. The controller must actually track this target; verify real Competition Status speed.
- Both switches **default OFF** so pre-existing commands and normal 30km/h path remain unchanged until explicitly opted in.
- CSV appends `beomsu_roundabout_right_m`, `beomsu_roundabout_speed_cap_kmh` and `beomsu_roundabout_speed_zone` for review. Run offline geometry test:
```bat
.\.venv\Scripts\python.exe -m unittest discover -s tests -p "test_beomsu_roundabout.py" -v
```

### Tunnel approach remains UNMODIFIED

**IMG_5047.mp4** shows left yellow marking close to the vehicle around t=15-18s, near the tunnel entrance. The video does not establish the exact WP corresponding to line contact nor reliable lane/wall clearance. No automatic lateral correction was enabled in the tunnel. V20 has **no safe GPS-shadow lane/LiDAR navigation**; earlier V34/V35 branches are experiments and must not be silently merged into V20. If GPS becomes invalid or steering behaves abnormally on entry, abort the drive, do not try to force passage with a guessed offset.

Upload the matching local `logs/integrated_v1_*.csv` to map roundabout and tunnel video to actual WP and diagnose signed CTE, heading error, V7 lookahead and speed. GPS path CTE is not a physical tire-to-marking measurement.

## Critical limitations
1. The camera and LiDAR do **not** currently provide real-time lane-line or guardrail bounds to this new offset function. It cannot enforce the competition's 3-second yellow-line rule. Absolute wheel-to-line distance must be measured from video or calibrated sensing.
2. GPS/reference-path tracking may be offset even if measured `signed_cte_m=0`; zero CTE does not prove lane center.
3. Offsets near walls/tunnel may worsen collision. No automatic extra-zone offset is enabled until video/CSV waypoint correspondence is established.
4. Existing V20 code does not include V35 GPS-shadow/tunnel navigation. This branch intentionally preserves V20 rather than silently merging unrelated experimental changes.
5. The full integrated stack and `unittest` suite were not executed in MORAI by the assistant. Hardware/UDP settings must be verified.

## Compare and tune
Record video including terminal WP/VIDEO SYNC and save local `logs/integrated_v1_*.csv`. Compare near WP1533/1578/1655 before/after. Only then decide whether to change 0.25m to 0.15 or 0.35m. At tunnel entry record the WP where the tire approaches the yellow line and where the wall becomes too close, and only apply a safe signed offset after determining which side has room.
