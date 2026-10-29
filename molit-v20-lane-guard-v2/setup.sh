#!/usr/bin/env bash
set -e

cd "$(dirname "$0")"

MORAI_DIR="third_party/MORAI-NetworkModule"
MORAI_REPO="https://github.com/MORAI-Autonomous/MORAI-NetworkModule.git"
MORAI_REF="24.R2.0"

echo "[1/6] Python 확인"
python3 --version

echo "[2/6] venv 지원 확인"
if ! python3 -m venv --help >/dev/null 2>&1; then
  echo "python3-venv 패키지가 필요합니다."
  echo "다음 명령을 실행하세요:"
  echo "  sudo apt update"
  echo "  sudo apt install -y python3-venv"
  exit 1
fi

echo "[3/6] 가상환경 생성"
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

echo "[4/6] Python 라이브러리 설치"
.venv/bin/python -m pip install --upgrade pip
.venv/bin/python -m pip install -r requirements.txt

echo "[5/6] MORAI 공식 Network Module 준비"
mkdir -p third_party
if [ ! -d "$MORAI_DIR/.git" ]; then
  rm -rf "$MORAI_DIR"
  git clone --depth 1 --branch "$MORAI_REF" "$MORAI_REPO" "$MORAI_DIR"
else
  echo "MORAI Network Module이 이미 존재합니다: $MORAI_DIR"
fi

echo "[6/6] 실행 권한 설정"
chmod +x setup.sh check.sh run.sh

echo
echo "설치 완료"
echo "다음 명령을 실행하세요: ./check.sh"
