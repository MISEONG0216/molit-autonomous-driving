param(
    [ValidateSet("main", "gps", "imu", "ctrl")]
    [string]$Mode = "main",
    [switch]$Drive,
    [double]$MaxSpeed = 57.0
)

$ErrorActionPreference = "Stop"
Set-Location $PSScriptRoot

$Python = Join-Path $PSScriptRoot ".venv/Scripts/python.exe"
if (-not (Test-Path $Python)) {
    throw ".venv가 없습니다. 먼저 .\setup_windows.ps1 을 실행하세요."
}

switch ($Mode) {
    "gps" {
        & $Python "src/gps_test.py"
        break
    }
    "imu" {
        & $Python "src/imu_test.py"
        break
    }
    "ctrl" {
        Write-Host "[CAUTION] 차량이 움직일 수 있습니다. MORAI Ego가 AV / ExternalCtrl인지 확인하세요."
        & $Python "src/ctrl_test.py"
        break
    }
    "main" {
        $ArgsList = @("src/main.py", "--max-speed", $MaxSpeed.ToString([System.Globalization.CultureInfo]::InvariantCulture))
        if ($Drive) {
            $ArgsList += "--drive"
        }
        & $Python @ArgsList
        break
    }
}
