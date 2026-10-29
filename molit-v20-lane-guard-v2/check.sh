#!/usr/bin/env bash
set -u

cd "$(dirname "$0")"

fail=0
ok() { echo "[OK] $1"; }
bad() { echo "[FAIL] $1"; fail=1; }

command -v python3 >/dev/null 2>&1 && ok "python3" || bad "python3 없음"
command -v git >/dev/null 2>&1 && ok "git" || bad "git 없음"

venv_ready=0
if [ -x ".venv/bin/python" ] && [ -f ".venv/bin/activate" ]; then
  ok ".venv 정상"
  venv_ready=1
elif [ -d ".venv" ]; then
  bad ".venv가 불완전함 - python3-venv 설치 후 ./setup.sh 재실행 필요"
else
  bad ".venv 없음 - ./setup.sh 실행 필요"
fi

[ -f "config/network.json" ] && ok "config/network.json" || bad "config/network.json 없음"
[ -f "requirements.txt" ] && ok "requirements.txt" || bad "requirements.txt 없음"

MORAI_DIR="third_party/MORAI-NetworkModule"
required_morai_files=(
  "$MORAI_DIR/lib/network/UDP.py"
  "$MORAI_DIR/lib/define/GPS.py"
  "$MORAI_DIR/lib/define/IMU.py"
  "$MORAI_DIR/lib/define/EgoCtrlCmd.py"
)

morai_ready=1
for f in "${required_morai_files[@]}"; do
  if [ ! -f "$f" ]; then
    morai_ready=0
  fi
done

if [ "$morai_ready" -eq 1 ]; then
  ok "MORAI Network Module 24.R2.0 구조"
else
  bad "MORAI Network Module 없음/불완전 - ./setup.sh 재실행 필요"
fi

if [ "$venv_ready" -eq 1 ]; then
  .venv/bin/python - <<'PY'
mods = ["numpy", "matplotlib", "cv2", "pyproj"]
missing = []
for m in mods:
    try:
        __import__(m)
        print(f"[OK] Python module: {m}")
    except Exception:
        print(f"[FAIL] Python module: {m}")
        missing.append(m)
raise SystemExit(1 if missing else 0)
PY
  [ $? -eq 0 ] || fail=1
fi

if [ "$venv_ready" -eq 1 ] && [ "$morai_ready" -eq 1 ]; then
  .venv/bin/python - <<'PY'
import sys
from pathlib import Path
root = Path.cwd()
morai = root / "third_party" / "MORAI-NetworkModule"
sys.path.insert(0, str(morai))
from lib.network.UDP import Receiver, Sender
from lib.define.GPS import GPS
from lib.define.IMU import IMU
from lib.define.EgoCtrlCmd import EgoCtrlCmd
print("[OK] MORAI UDP Python import")
PY
  [ $? -eq 0 ] || fail=1
fi

if [ $fail -eq 0 ]; then
  echo
  echo "환경 점검 통과"
  exit 0
else
  echo
  echo "환경 점검 실패 - 위 FAIL 항목을 확인하세요."
  exit 1
fi
