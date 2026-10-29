# V38 + Beomsu full-course experiment (Windows)

**Correct parent:** `integration-v38-shadow-clear-corridor` at `0bc21714d1a959a7cf4ed2e98a7b12723e50dccf`. Do NOT use `v-add-finish-beomsu`: it is based on V39. The V39 variable-offset tunnel-detour changes are **NOT** incorporated into this branch.

## What changed from V38

- Optional previously successful median/guardrail path adjustment WP1480–1715.
- Optional roundabout exit path shift WP1859–1902 and 12km/h distance-preview speed cap.
- Read-only CSV diagnostics for official-path signed CTE and heading error on all logged GPS positions.
- One CtrlCmd sender in `src/main_integrated_v1.py`; V38 traffic, pedestrian, static cone avoidance, merge, following, GPS shadow, and tunnel LiDAR modules unchanged.
- Local single-PC MORAI control target `127.0.0.1:9093` and missing `scikit-learn` requirement.
- Windows 3-CMD helper supervisors: group vision launches traffic+pedestrian+lane, group safety launches LiDAR+merge; **supervisors never send controls**.

## Install once (new directory, to avoid reusing V39 checkout)

```bat
cd /d "%USERPROFILE%"
if not exist "molit-finish-v38\.git" git clone --branch v38-add-finish-beomsu --single-branch https://github.com/MISEONG0216/molit-autonomous-driving.git molit-finish-v38
cd /d "%USERPROFILE%\molit-finish-v38"
git pull --ff-only origin v38-add-finish-beomsu
powershell -NoProfile -ExecutionPolicy Bypass -File setup_windows.ps1
```

Run tests before first drive:

```bat
.\.venv\Scripts\python.exe -m unittest discover -s tests -p "test_beomsu*.py" -v
```

MORAI must be AV/ExternalCtrl and on this same PC, with GPS 9201, IMU 1111, vehicle status 1909, front camera 9291, LEFT camera 9293, **verified LEFT-REAR camera 1118**, LiDAR 1116. This port 1118 is in config, but its presence must be checked in actual simulator settings. Do not substitute RIGHT 9295.

## Three Windows CMD windows

**CMD 1: Vision** (traffic, pedestrian/follow, lane shadow).
```bat
cd /d "%USERPROFILE%\molit-finish-v38"
.\.venv\Scripts\python.exe scripts\run_beomsu_workers.py vision
```

**CMD 2: Safety** (LiDAR/static cone, left & left-rear merge).
```bat
cd /d "%USERPROFILE%\molit-finish-v38"
.\.venv\Scripts\python.exe scripts\run_beomsu_workers.py safety
```

**CMD 3: Main dry-run** (no CtrlCmd transmission).
```bat
cd /d "%USERPROFILE%\molit-finish-v38"
.\.venv\Scripts\python.exe src\main_integrated_v1.py --traffic --static --static-control --pedestrian --pedestrian-control --merge --merge-control --follow --follow-control --gps-shadow --max-speed 30 --beomsu-first-right-m 0.25 --beomsu-roundabout-right-m 0.20 --beomsu-roundabout-kmh 12
```

After verifying sensor packets, terminate **only** CMD 3 with Ctrl+C, then for supervised simulator drive use same command **plus `--drive`**:
```bat
.\.venv\Scripts\python.exe src\main_integrated_v1.py --drive --traffic --static --static-control --pedestrian --pedestrian-control --merge --merge-control --follow --follow-control --gps-shadow --max-speed 30 --beomsu-first-right-m 0.25 --beomsu-roundabout-right-m 0.20 --beomsu-roundabout-kmh 12
```

Only one main controller at a time. Never leave the earlier V20 or V39 `--drive` process running. Stop immediately if the simulator shows unsafe steering or sensor loss. For logs collect matching `logs/integrated_v1_*.csv`, `logs/static_worker_*.csv`, `logs/beomsu_*.log`, and video. Main CSV uses **official** path as error reference, which is not lane-boundary clearance. GPS-shadow dead reckoning can drift.

**Status:** GitHub source and V38 differences checked, **offline tests and MORAI full-course execution not yet run**.
