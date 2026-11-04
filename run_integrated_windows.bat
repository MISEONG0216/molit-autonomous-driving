@echo off
setlocal EnableExtensions
cd /d "%~dp0"
set "PYTHON=%CD%\.venv38\Scripts\python.exe"
if not exist "%PYTHON%" (
  echo [ERROR] Python 3.8 virtual environment missing: .venv38\Scripts\python.exe
  exit /b 1
)
set "DRIVE_FLAG="
if /I "%~1"=="--drive" set "DRIVE_FLAG=--drive"
if not "%~1"=="" if /I not "%~1"=="--drive" (
  echo Usage: run_integrated_windows.bat [--drive]
  exit /b 2
)
echo [V52] Start MORAI simulator with local UDP sensors first.
echo [V52] Only integrated main process sends vehicle control.
echo [V52] Starting traffic, pedestrian, static LiDAR, merge and GPS-shadow lane workers...
start "V52 Traffic Camera" /D "%CD%" "%ComSpec%" /k ""%PYTHON%" -u src\traffic_light_detector.py --relay-pedestrian --relay-lane"
start "V52 Pedestrian Follow" /D "%CD%" "%ComSpec%" /k ""%PYTHON%" -u src\dynamic_pedestrian_worker.py --follow"
start "V52 Static LiDAR" /D "%CD%" "%ComSpec%" /k ""%PYTHON%" -u src\lidar\static_avoidance_module.py"
start "V52 Merge Camera" /D "%CD%" "%ComSpec%" /k ""%PYTHON%" -u src\merge_side_camera_worker.py"
start "V52 GPS Shadow Lane" /D "%CD%" "%ComSpec%" /k ""%PYTHON%" -u src\gps_shadow_lane_worker.py"
echo [V52] Launching integrated control with 20 km/h max. Default is DRY-RUN.
echo [V52] Use --drive only after verifying all sensor workers.
"%PYTHON%" src\main_integrated_v1.py --traffic --static --static-control --pedestrian --pedestrian-control --merge --merge-control --follow --follow-control --gps-shadow --max-speed 20 %DRIVE_FLAG%
echo [V52] Main exited. Close remaining V52 worker windows to release UDP ports.
endlocal
