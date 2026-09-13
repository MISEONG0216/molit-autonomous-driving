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

:usage
echo Usage:
echo   run_windows.bat gps                 - GPS UDP receive test
echo   run_windows.bat imu                 - IMU UDP receive test
echo   run_windows.bat ctrl                - low-speed control test
echo   run_windows.bat pure-pursuit        - Pure Pursuit dry run
echo   run_windows.bat pure-pursuit-drive  - Pure Pursuit actual drive
exit /b 1
