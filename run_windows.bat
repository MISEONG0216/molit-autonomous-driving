@echo off
setlocal
cd /d "%~dp0"

if not exist ".venv\Scripts\python.exe" (
    echo [FAIL] .venv not found. Run setup_windows.bat first.
    exit /b 1
)

if "%~1"=="" goto usage

if /I "%~1"=="gps" (
    .venv\Scripts\python.exe src\gps_test.py
    exit /b %errorlevel%
)

if /I "%~1"=="imu" (
    .venv\Scripts\python.exe src\imu_test.py
    exit /b %errorlevel%
)

if /I "%~1"=="ctrl" (
    .venv\Scripts\python.exe src\ctrl_test.py
    exit /b %errorlevel%
)

if /I "%~1"=="pure-pursuit" (
    .venv\Scripts\python.exe src\pure_pursuit_udp.py
    exit /b %errorlevel%
)

if /I "%~1"=="pure-pursuit-drive" (
    .venv\Scripts\python.exe src\pure_pursuit_udp.py --drive
    exit /b %errorlevel%
)

if /I "%~1"=="traffic-light" (
    .venv\Scripts\python.exe src\traffic_light_detector.py
    exit /b %errorlevel%
)

if /I "%~1"=="traffic-light-dry" (
    .venv\Scripts\python.exe src\pure_pursuit_udp.py --traffic-light
    exit /b %errorlevel%
)

if /I "%~1"=="traffic-light-drive" (
    .venv\Scripts\python.exe src\pure_pursuit_udp.py --drive --traffic-light
    exit /b %errorlevel%
)

:usage
echo Usage:
echo   run_windows.bat gps                 - GPS UDP receive test
echo   run_windows.bat imu                 - IMU UDP receive test
echo   run_windows.bat ctrl                - low-speed control test
echo   run_windows.bat pure-pursuit        - Pure Pursuit dry run
echo   run_windows.bat pure-pursuit-drive  - Pure Pursuit actual drive
echo   run_windows.bat traffic-light       - camera vehicle-signal detector
echo   run_windows.bat traffic-light-dry   - signal integration dry run
echo   run_windows.bat traffic-light-drive - signal-aware actual drive
exit /b 1
