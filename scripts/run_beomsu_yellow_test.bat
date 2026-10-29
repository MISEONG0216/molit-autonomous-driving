@echo off
setlocal
cd /d "%~dp0.."
if not exist ".venv\Scripts\python.exe" (
  echo [ERROR] Missing .venv\Scripts\python.exe
  exit /b 1
)
set "DRIVE_FLAG="
if /I "%~1"=="drive" (
  set "DRIVE_FLAG=--drive"
) else (
  if not "%~1"=="" (
    echo [ERROR] Usage: scripts\run_beomsu_yellow_test.bat [drive]
    exit /b 2
  )
)
echo ==========================================================
echo  V20 Beomsu YELLOW LINE - EXPERIMENTAL TEST ONLY
echo ==========================================================
echo [INFO] Only this process sends EgoCtrlCmd when 'drive' is selected.
echo [INFO] Start traffic detector and LiDAR worker in separate CMDs.
echo [INFO] Confirm MORAI AV ExternalCtrl, GPS/IMU/status/vision/LiDAR.
echo [INFO] Median: WP1480-1715 RIGHT +0.25m
echo [INFO] Roundabout exit: WP1859-1902 RIGHT +0.25m, 12kmh hold
echo [INFO] Tunnel entrance gore: WP3640-3820 RIGHT +0.12m
echo [INFO] Post-tunnel curve: WP4050-4300 RIGHT +0.25m
echo [CAUTION] Offsets are UNVALIDATED. Stop if lane/wall clearance decreases.
echo [MODE] %DRIVE_FLAG%
".venv\Scripts\python.exe" src\main_integrated_v1.py %DRIVE_FLAG% ^
  --traffic --static --static-control --max-speed 30 ^
  --beomsu-first-right-m 0.25 ^
  --beomsu-roundabout-right-m 0.25 ^
  --beomsu-roundabout-kmh 12 ^
  --beomsu-extra-zone "3640:3820:0.12" ^
  --beomsu-extra-zone "4050:4300:0.25"
exit /b %ERRORLEVEL%
