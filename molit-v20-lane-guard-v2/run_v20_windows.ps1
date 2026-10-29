param(
    [ValidateSet('gps','imu','status','traffic','lidar','merge','dry','drive','tests')]
    [string]$Mode = 'dry',
    [switch]$Traffic,
    [switch]$Static,
    [switch]$StaticControl,
    [switch]$Merge,
    [switch]$MergeControl,
    [switch]$MergeAll,
    [double]$MaxSpeed = 10.0,
    [switch]$ConfirmDrive
)
$ErrorActionPreference = 'Stop'
Set-Location -LiteralPath $PSScriptRoot
$Python = Join-Path $PSScriptRoot '.venv\Scripts\python.exe'
if (-not (Test-Path -LiteralPath $Python)) { throw 'Run setup_v20_windows.ps1 first.' }
if ($StaticControl -and -not $Static) { throw '-StaticControl requires -Static.' }
if ($MergeControl -and -not $Merge) { throw '-MergeControl requires -Merge.' }
if ($MaxSpeed -le 0 -or $MaxSpeed -gt 57) { throw '-MaxSpeed must be >0 and <=57.' }

switch ($Mode) {
  'gps'    { & $Python 'src/gps_test.py'; break }
  'imu'    { & $Python 'src/imu_test.py'; break }
  'status' { & $Python 'src/competition_status_speed_test.py'; break }
  'traffic'{ & $Python 'src/traffic_light_detector.py'; break }
  'lidar'  { & $Python 'src/lidar/static_avoidance_module.py'; break }
  'merge'  { & $Python 'src/highway_merge_monitor_integration.py'; break }
  'tests'  { & $Python -m unittest discover -s tests -p 'test_*.py' -q; break }
  default {
    if ($Mode -eq 'drive' -and -not $ConfirmDrive) {
        throw 'Vehicle commands are disabled without -ConfirmDrive. First run -Mode dry.'
    }
    $Arguments = @('src/main_integrated_v1.py','--max-speed',$MaxSpeed.ToString([System.Globalization.CultureInfo]::InvariantCulture))
    if ($Traffic) { $Arguments += '--traffic' }
    if ($Static) { $Arguments += '--static' }
    if ($StaticControl) { $Arguments += '--static-control' }
    if ($Merge) { $Arguments += '--merge' }
    if ($MergeControl) { $Arguments += '--merge-control' }
    if ($MergeAll) { $Arguments += '--merge-all' }
    if ($Mode -eq 'drive') { $Arguments += '--drive' }
    Write-Host ('[V20] ' + ($Arguments -join ' '))
    & $Python @Arguments
  }
}
if ($LASTEXITCODE -ne 0) { throw ('Command failed with exit code ' + $LASTEXITCODE) }
