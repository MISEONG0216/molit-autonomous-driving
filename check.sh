#!/usr/bin/env bash
set -u

cd "$(dirname "$0")"

fail=0

ok() { echo "[OK] $1"; }
bad() { echo "[FAIL] $1"; fail=1; }

command -v python3 >/dev/null 2>&1 && ok "python3" || bad "python3 없음"
command -v git >/dev/null 2>&1 && ok "git" || bad "git 없음"

if [ -d ".venv" ]; then
  ok ".venv 존재"
else
  bad ".venv 없음 - ./setup.sh 실행 필요"
fi

if [ -f "config/network.json" ]; then
  ok "config/network.json"
else
  bad "config/network.json 없음"
fi

if [ -f "requirements.txt" ]; then
  ok "requirements.txt"
else
  bad "requirements.txt 없음"
fi

if [ -d ".venv" ]; then
  source .venv/bin/activate
  python - <<'PY'
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

if [ $fail -eq 0 ]; then
  echo
  echo "환경 점검 통과"
  exit 0
else
  echo
  echo "환경 점검 실패 - 위 FAIL 항목을 확인하세요."
  exit 1
fi
