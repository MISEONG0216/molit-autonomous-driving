$ErrorActionPreference = "Stop"

Set-Location $PSScriptRoot

$MoraiDir = "third_party/MORAI-NetworkModule"
$MoraiRepo = "https://github.com/MORAI-Autonomous/MORAI-NetworkModule.git"
$MoraiRef = "24.R2.0"

Write-Host "[1/5] Python 확인"
$pythonCmd = $null
if (Get-Command py -ErrorAction SilentlyContinue) {
    $pythonCmd = "py"
} elseif (Get-Command python -ErrorAction SilentlyContinue) {
    $pythonCmd = "python"
} else {
    throw "Python을 찾을 수 없습니다. Python 3.8.x를 설치하고 PATH에 추가한 뒤 다시 실행하세요."
}

& $pythonCmd --version

Write-Host "[2/5] 가상환경 생성"
if (-not (Test-Path ".venv/Scripts/python.exe")) {
    if (Test-Path ".venv") {
        Remove-Item -Recurse -Force ".venv"
    }
    & $pythonCmd -m venv .venv
}

$VenvPython = Join-Path $PSScriptRoot ".venv/Scripts/python.exe"

Write-Host "[3/5] Python 라이브러리 설치"
& $VenvPython -m pip install --upgrade pip
& $VenvPython -m pip install -r requirements.txt

Write-Host "[4/5] MORAI 공식 Network Module 준비"
New-Item -ItemType Directory -Force -Path "third_party" | Out-Null
if (-not (Test-Path "$MoraiDir/.git")) {
    if (Test-Path $MoraiDir) {
        Remove-Item -Recurse -Force $MoraiDir
    }
    git clone --depth 1 --branch $MoraiRef $MoraiRepo $MoraiDir
} else {
    Write-Host "MORAI Network Module이 이미 존재합니다: $MoraiDir"
}

Write-Host "[5/5] 설치 확인"
& $VenvPython -c "import numpy, pyproj; print('Python packages OK')"

Write-Host ""
Write-Host "Windows 초기 설정 완료"
Write-Host "다음 단계: MORAI UDP 설정 후 .\run_windows.ps1 gps"
Write-Host "PowerShell 실행 정책 오류가 나면 현재 창에서 다음을 실행하세요:"
Write-Host "  Set-ExecutionPolicy -Scope Process Bypass"
