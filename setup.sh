#!/usr/bin/env bash
set -e

cd "$(dirname "$0")"

echo "[1/5] Python 확인"
python3 --version

echo "[2/5] venv 지원 확인"
if ! python3 -m venv --help >/dev/null 2>&1; then
  echo "python3-venv 패키지가 필요합니다."
  echo "다음 명령을 실행하세요:"
  echo "  sudo apt update"
  echo "  sudo apt install -y python3-venv"
  exit 1
fi

echo "[3/5] 가상환경 생성"
if [ -d ".venv" ] && [ ! -x ".venv/bin/python" ]; then
  echo "불완전한 .venv를 삭제하고 다시 생성합니다."
  rm -rf .venv
fi

if [ ! -x ".venv/bin/python" ]; then
  if ! python3 -m venv .venv; then
    echo
    echo "가상환경 생성에 실패했습니다. Ubuntu에서는 python3-venv가 필요합니다."
    echo "다음 명령을 실행한 뒤 ./setup.sh를 다시 실행하세요:"
    echo "  sudo apt update"
    echo "  sudo apt install -y python3-venv"
    exit 1
  fi
fi

echo "[4/5] 라이브러리 설치"
.venv/bin/python -m pip install --upgrade pip
.venv/bin/python -m pip install -r requirements.txt

echo "[5/5] 실행 권한 설정"
chmod +x setup.sh check.sh run.sh

echo
echo "설치 완료"
echo "다음 명령을 실행하세요: ./check.sh"
