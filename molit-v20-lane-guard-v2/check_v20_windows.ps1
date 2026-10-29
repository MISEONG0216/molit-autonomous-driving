$ErrorActionPreference = 'Stop'
Set-Location -LiteralPath $PSScriptRoot
$Python = Join-Path $PSScriptRoot '.venv\Scripts\python.exe'
if (-not (Test-Path -LiteralPath $Python)) { throw 'Run setup_v20_windows.ps1 first.' }

Write-Host '===== v20 Python packages ====='
& $Python -c "import sys, numpy, cv2, pyproj, torch, torchvision, pytorch_lightning, ultralytics, sklearn, PIL, matplotlib; print('Python:', sys.version.split()[0]); print('Torch:',torch.__version__); print('CUDA available:',torch.cuda.is_available()); print('All Python imports OK')"
if ($LASTEXITCODE -ne 0) { throw 'Python import check failed.' }

Write-Host '===== Model / path / network module files ====='
$paths = @(
    'third_party\MORAI-NetworkModule\lib\network\UDP.py',
    'third_party\MORAI-NetworkModule\lib\define\GPS.py',
    'third_party\MORAI-NetworkModule\lib\define\IMU.py',
    'src\trafficlight\epoch=5-step=113.ckpt',
    'src\trafficlight\yolov8n.pt',
    'path\2026_molit_comp_global_path.txt',
    'config\integration_network.json',
    'config\integration_missions.json'
)
foreach ($p in $paths) {
    if (-not (Test-Path -LiteralPath $p)) { throw ('Missing file: ' + $p) }
    Write-Host ('OK: ' + $p)
}
Write-Host '===== Offline regression tests ====='
& $Python -m unittest discover -s tests -p 'test_*.py' -q
if ($LASTEXITCODE -ne 0) { throw 'v20 unit tests failed.' }
Write-Host '===== Network config ====='
$cfg = Get-Content -LiteralPath 'config\integration_network.json' -Raw -Encoding UTF8 | ConvertFrom-Json
Write-Host ('MORAI control destination: ' + $cfg.control.target_ip + ':' + $cfg.control.port)
Write-Host ('LiDAR listen: ' + $cfg.lidar.bind_ip + ':' + $cfg.lidar.port)
Write-Host 'Check MORAI UDP destinations separately. These tests do NOT confirm live camera/LiDAR/vehicle operation.'
