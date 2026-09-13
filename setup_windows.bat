@echo off
setlocal
cd /d "%~dp0"

echo ==============================================
echo  MOLIT Autonomous Driving - Windows Setup
echo ==============================================
echo.

where python >nul 2>&1
if errorlevel 1 (
    echo [FAIL] Python not found.
    echo Install Python 3.8+ and check "Add Python to PATH".
    exit /b 1
)

where git >nul 2>&1
if errorlevel 1 (
    echo [FAIL] Git not found.
    echo Install Git for Windows first.
    exit /b 1
)

echo [1/6] Python version
python --version

echo.
echo [2/6] Create virtual environment
if not exist ".venv\Scripts\python.exe" (
    python -m venv .venv
    if errorlevel 1 (
        echo [FAIL] Failed to create .venv
        exit /b 1
    )
) else (
    echo .venv already exists.
)

echo.
echo [3/6] Upgrade pip
.venv\Scripts\python.exe -m pip install --upgrade pip
if errorlevel 1 exit /b 1

echo.
echo [4/6] Install Python requirements
.venv\Scripts\python.exe -m pip install -r requirements.txt
if errorlevel 1 exit /b 1

echo.
echo [5/6] Prepare MORAI Network Module 24.R2.0
if not exist "third_party" mkdir third_party
if not exist "third_party\MORAI-NetworkModule\.git" (
    if exist "third_party\MORAI-NetworkModule" rmdir /s /q "third_party\MORAI-NetworkModule"
    git clone --depth 1 --branch 24.R2.0 https://github.com/MORAI-Autonomous/MORAI-NetworkModule.git third_party\MORAI-NetworkModule
    if errorlevel 1 (
        echo [FAIL] Failed to clone MORAI Network Module.
        exit /b 1
    )
) else (
    echo MORAI Network Module already exists.
)

echo.
echo [6/6] Setup complete
echo Run: check_windows.bat
exit /b 0
