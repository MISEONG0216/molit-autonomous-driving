$ErrorActionPreference = 'Stop'

Set-Location -LiteralPath $PSScriptRoot

$MoraiDir = Join-Path $PSScriptRoot 'third_party\MORAI-NetworkModule'
$MoraiRepo = 'https://github.com/MORAI-Autonomous/MORAI-NetworkModule.git'
$MoraiRef = '24.R2.0'
$VenvDir = Join-Path $PSScriptRoot '.venv'
$VenvPython = Join-Path $VenvDir 'Scripts\python.exe'
$Requirements = Join-Path $PSScriptRoot 'requirements.txt'

Write-Host '[1/6] Checking Git'
if (-not (Get-Command git.exe -ErrorAction SilentlyContinue)) {
    throw 'Git was not found. Install Git for Windows and reopen PowerShell.'
}
git --version

Write-Host '[2/6] Checking Python'
$PythonExe = $null
$PythonPrefix = @()

if (Get-Command py.exe -ErrorAction SilentlyContinue) {
    $PythonExe = 'py.exe'
    $PythonPrefix = @('-3')
}
elseif (Get-Command python.exe -ErrorAction SilentlyContinue) {
    $PythonExe = 'python.exe'
}
else {
    throw 'Python 3 was not found. Install Python and enable Add Python to PATH.'
}

& $PythonExe @PythonPrefix --version
if ($LASTEXITCODE -ne 0) {
    throw 'Python 3 could not be started.'
}

Write-Host '[3/6] Creating virtual environment'
if (-not (Test-Path -LiteralPath $VenvPython)) {
    if (Test-Path -LiteralPath $VenvDir) {
        Remove-Item -LiteralPath $VenvDir -Recurse -Force
    }
    & $PythonExe @PythonPrefix -m venv $VenvDir
    if ($LASTEXITCODE -ne 0) {
        throw 'Failed to create .venv.'
    }
}

if (-not (Test-Path -LiteralPath $VenvPython)) {
    throw '.venv\Scripts\python.exe was not created.'
}

Write-Host '[4/6] Installing Python packages'
& $VenvPython -m pip install --upgrade pip
if ($LASTEXITCODE -ne 0) { throw 'pip upgrade failed.' }
& $VenvPython -m pip install -r $Requirements
if ($LASTEXITCODE -ne 0) { throw 'requirements installation failed.' }

Write-Host '[5/6] Preparing MORAI Network Module 24.R2.0'
$ThirdPartyDir = Join-Path $PSScriptRoot 'third_party'
New-Item -ItemType Directory -Force -Path $ThirdPartyDir | Out-Null
$MoraiGitDir = Join-Path $MoraiDir '.git'

if (-not (Test-Path -LiteralPath $MoraiGitDir)) {
    if (Test-Path -LiteralPath $MoraiDir) {
        Remove-Item -LiteralPath $MoraiDir -Recurse -Force
    }
    git clone --depth 1 --branch $MoraiRef $MoraiRepo $MoraiDir
    if ($LASTEXITCODE -ne 0) {
        throw 'Failed to clone MORAI-NetworkModule.'
    }
}
else {
    Write-Host ('MORAI Network Module already exists: ' + $MoraiDir)
}

Write-Host '[6/6] Verifying installation'
& $VenvPython -c "import numpy, pyproj; print('Python packages OK')"
if ($LASTEXITCODE -ne 0) {
    throw 'Python package verification failed.'
}

Write-Host ''
Write-Host 'Windows setup completed.'
Write-Host 'Next: configure MORAI UDP, then run:'
Write-Host '  .\run_windows.ps1 gps'
Write-Host ''
Write-Host 'If PowerShell blocks scripts, run this once in the current window:'
Write-Host '  Set-ExecutionPolicy -Scope Process -ExecutionPolicy Bypass'
