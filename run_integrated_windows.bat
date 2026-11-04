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
echo [V54] Start MORAI simulator with local UDP sensors first.
echo [V54] Only integrated main process sends vehicle control.
echo [V54] Starting traffic, pedestrian, static LiDAR, merge and GPS-shadow lane workers...
start "V54 Traffic Camera" /D "%CD%" "%ComSpec%" /k ""%PYTHON%" -u src\traffic_light_detector.py --relay-pedestrian --relay-lane"
start "V54 Pedestrian Follow" /D "%CD%" "%ComSpec%" /k ""%PYTHON%" -u src\dynamic_pedestrian_worker.py --follow"
start "V54 Static LiDAR" /D "%CD%" "%ComSpec%" /k ""%PYTHON%" -u src\lidar\static_avoidance_module.py"
start "V54 Merge Camera" /D "%CD%" "%ComSpec%" /k ""%PYTHON%" -u src\merge_side_camera_worker.py"
start "V54 GPS Shadow Lane" /D "%CD%" "%ComSpec%" /k ""%PYTHON%" -u src\gps_shadow_lane_worker.py"
echo [V54] Launching integrated control with 40 km/h max (mission-specific lower caps remain). Default is DRY-RUN.
echo [V54] Use --drive only after verifying all sensor workers.
"%PYTHON%" src\main_integrated_v1.py --traffic --static --static-control --pedestrian --pedestrian-control --merge --merge-control --follow --follow-control --gps-shadow --max-speed 40 %DRIVE_FLAG%
echo [V54] Main exited. Close remaining V54 worker windows to release UDP ports.
endlocal
