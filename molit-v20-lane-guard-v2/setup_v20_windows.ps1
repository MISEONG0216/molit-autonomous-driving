param(
    [string]$ControlIP = '127.0.0.1'
)
$ErrorActionPreference = 'Stop'
Set-Location -LiteralPath $PSScriptRoot

if (-not (Get-Command git.exe -ErrorAction SilentlyContinue)) {
    throw 'Git for Windows is required. Install Git and reopen PowerShell.'
}
if (-not (Get-Command py.exe -ErrorAction SilentlyContinue)) {
    throw 'Python Launcher (py.exe) is required. Install Python 3.11 64-bit with launcher.'
}
& py.exe -3.11 -c "import sys; assert sys.version_info[:2] == (3,11); print(sys.version)"
if ($LASTEXITCODE -ne 0) { throw 'Python 3.11 not found. Install Python 3.11.9 (64-bit).' }

$VenvPython = Join-Path $PSScriptRoot '.venv\Scripts\python.exe'
if (Test-Path -LiteralPath $VenvPython) {
    $ver = (& $VenvPython -c "import sys;print(str(sys.version_info.major)+'.'+str(sys.version_info.minor))").Trim()
    if ($ver -ne '3.11') {
        throw ('Existing .venv uses Python ' + $ver + '. Rename .venv and rerun this setup; do not delete it without backup.')
    }
} else {
    if (Test-Path -LiteralPath (Join-Path $PSScriptRoot '.venv')) {
        throw 'Incomplete .venv folder exists. Rename it first.'
    }
    Write-Host '[1] Creating Python 3.11 virtual environment'
    & py.exe -3.11 -m venv '.venv'
    if ($LASTEXITCODE -ne 0) { throw 'Could not create .venv' }
}

Write-Host '[2] Running provided Windows installer (packages + MORAI network module)'
& (Join-Path $PSScriptRoot 'setup_windows.ps1')
if ($LASTEXITCODE -ne 0) { throw 'Setup failed. See previous error.' }

Write-Host '[3] Verifying extra LiDAR clustering dependency'
& $VenvPython -m pip install 'scikit-learn'
if ($LASTEXITCODE -ne 0) { throw 'scikit-learn installation failed.' }

Write-Host '[4] Setting MORAI control target address'
$NetworkFile = Join-Path $PSScriptRoot 'config\integration_network.json'
$cfg = Get-Content -LiteralPath $NetworkFile -Raw -Encoding UTF8 | ConvertFrom-Json
$cfg.control.target_ip = $ControlIP
$NetworkJson = $cfg | ConvertTo-Json -Depth 20
[System.IO.File]::WriteAllText($NetworkFile, $NetworkJson + [Environment]::NewLine, (New-Object System.Text.UTF8Encoding($false)))
Write-Host ('[CONTROL] target=' + $ControlIP + ':' + $cfg.control.port)

Write-Host '[5] Running environment check'
& (Join-Path $PSScriptRoot 'check_v20_windows.ps1')
if ($LASTEXITCODE -ne 0) { throw 'Environment validation failed.' }

Write-Host ''
Write-Host '[SETUP DONE] Start MORAI, configure UDP sensors, then use run_v20_windows.ps1 in separate terminals.'
