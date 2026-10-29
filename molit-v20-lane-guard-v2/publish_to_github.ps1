param(
    [string]$RepoUrl = 'https://github.com/MISEONG0216/molit-autonomous-driving.git',
    [string]$BaseBranch = 'lidar_object_detection',
    [string]$NewBranch = 'v20-lane-guard-v2-20261030',
    [string]$CloneDir = 'C:\MORAI\v20-github-publish'
)
$ErrorActionPreference = 'Stop'
$SourceDir = $PSScriptRoot

if (-not (Get-Command git -ErrorAction SilentlyContinue)) {
    throw 'Git for Windows is required.'
}
if (-not (Test-Path -LiteralPath (Join-Path $SourceDir 'src\main_integrated_v1.py'))) {
    throw 'Run this script from the extracted molit-v20-lane-guard-v2 folder.'
}
if (Test-Path -LiteralPath $CloneDir) {
    throw "CloneDir already exists. To avoid overwriting data, choose a different -CloneDir: $CloneDir"
}
Write-Host '[1/6] Cloning team repository into a NEW folder (original repo and source untouched)'
& git clone $RepoUrl $CloneDir
if ($LASTEXITCODE -ne 0) { throw 'Git clone failed. Verify URL, account access, and internet.' }

Write-Host '[2/6] Creating a separate v20 branch'
& git -C $CloneDir show-ref --verify --quiet "refs/remotes/origin/$BaseBranch"
if ($LASTEXITCODE -ne 0) { throw "Base branch origin/$BaseBranch not found. Check git branch -r and choose -BaseBranch." }
& git -C $CloneDir show-ref --verify --quiet "refs/remotes/origin/$NewBranch"
if ($LASTEXITCODE -eq 0) { throw "Branch $NewBranch already exists on the remote. Choose a new -NewBranch." }
& git -C $CloneDir switch -c $NewBranch "origin/$BaseBranch"
if ($LASTEXITCODE -ne 0) { throw 'Could not create local branch.' }

Write-Host '[3/6] Replacing ONLY the newly cloned branch working tree with complete portable v20 V2 snapshot'
& git -C $CloneDir rm -r -q --ignore-unmatch .
if ($LASTEXITCODE -ne 0) { throw 'Could not clear the clone working tree.' }
# No original project folder is deleted. All changes are only in the fresh clone directory.
& robocopy $SourceDir $CloneDir /E /XD '.git' '.venv' 'third_party' 'logs' '__pycache__' 'backup_*' '.pytest_cache' /XF '*.pyc' '*.pyo' '*.log' '*.tmp' | Out-Null
if ($LASTEXITCODE -ge 8) { throw "File copy failed. Robocopy exit code: $LASTEXITCODE" }

Write-Host '[4/6] Checking and staging files'
& git -C $CloneDir add -A
if ($LASTEXITCODE -ne 0) { throw 'git add failed.' }
& git -C $CloneDir status --short
Write-Host 'Check that no passwords, tokens, personal data, or unlicensed model weights are being published.'
Write-Host 'Models included: checkpoint and yolov8n.pt; check distribution rights before pushing to a public repository.'

Write-Host '[5/6] Committing the new branch snapshot'
& git -C $CloneDir commit -m 'Add standalone MORAI v20 integration with lane guard V2 and Windows setup'
if ($LASTEXITCODE -ne 0) {
    throw 'git commit failed. If author identity is missing, set git config --global user.name and user.email; rerun commit/push in CloneDir.'
}

Write-Host '[6/6] Pushing ONLY the new branch (does not push to main or lidar_object_detection)'
& git -C $CloneDir push -u origin $NewBranch
if ($LASTEXITCODE -ne 0) {
    throw "Push failed. Check GitHub login and permission. Commit is in $CloneDir; rerun: git -C `"$CloneDir`" push -u origin $NewBranch"
}
Write-Host ''
Write-Host "PUBLISHED: $RepoUrl / branch $NewBranch"
Write-Host 'From other laptop, use: git clone --single-branch --branch ' $NewBranch $RepoUrl
