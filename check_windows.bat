@echo off
setlocal EnableDelayedExpansion
cd /d "%~dp0"

set FAIL=0

echo ==============================================
echo  MOLIT Autonomous Driving - Windows Check
echo ==============================================
echo.

where python >nul 2>&1
if errorlevel 1 (
    echo [FAIL] python
    set FAIL=1
) else (
    echo [OK] python
)

where git >nul 2>&1
if errorlevel 1 (
    echo [FAIL] git
    set FAIL=1
) else (
    echo [OK] git
)

if exist ".venv\Scripts\python.exe" (
    echo [OK] .venv
) else (
    echo [FAIL] .venv - run setup_windows.bat first
    set FAIL=1
)

if exist "config\network.json" (
    echo [OK] config\network.json
) else (
    echo [FAIL] config\network.json
    set FAIL=1
)

if exist "requirements.txt" (
    echo [OK] requirements.txt
) else (
    echo [FAIL] requirements.txt
    set FAIL=1
)

set MORAI_OK=1
for %%F in (
    "third_party\MORAI-NetworkModule\lib\network\UDP.py"
    "third_party\MORAI-NetworkModule\lib\define\GPS.py"
    "third_party\MORAI-NetworkModule\lib\define\IMU.py"
    "third_party\MORAI-NetworkModule\lib\define\EgoCtrlCmd.py"
) do (
    if not exist %%F set MORAI_OK=0
)

if "!MORAI_OK!"=="1" (
    echo [OK] MORAI Network Module 24.R2.0 structure
) else (
    echo [FAIL] MORAI Network Module missing or incomplete
    set FAIL=1
)

if exist ".venv\Scripts\python.exe" (
    echo.
    echo Checking Python modules...
    .venv\Scripts\python.exe -c "import numpy; print('[OK] numpy')"
    if errorlevel 1 set FAIL=1
    .venv\Scripts\python.exe -c "import matplotlib; print('[OK] matplotlib')"
    if errorlevel 1 set FAIL=1
    .venv\Scripts\python.exe -c "import cv2; print('[OK] cv2')"
    if errorlevel 1 set FAIL=1
    .venv\Scripts\python.exe -c "import pyproj; print('[OK] pyproj')"
    if errorlevel 1 set FAIL=1
)

if exist ".venv\Scripts\python.exe" if "!MORAI_OK!"=="1" (
    .venv\Scripts\python.exe -c "import sys; from pathlib import Path; p=Path.cwd()/'third_party'/'MORAI-NetworkModule'; sys.path.insert(0,str(p)); from lib.network.UDP import Receiver, Sender; from lib.define.GPS import GPS; from lib.define.IMU import IMU; from lib.define.EgoCtrlCmd import EgoCtrlCmd; print('[OK] MORAI UDP Python import')"
    if errorlevel 1 set FAIL=1
)

echo.
if "!FAIL!"=="0" (
    echo Environment check passed.
    exit /b 0
) else (
    echo Environment check failed. Check the [FAIL] items above.
    exit /b 1
)
