#!/usr/bin/env bash
set -e

cd "$(dirname "$0")"

echo "[1/4] Python 확인"
python3 --version

echo "[2/4] 가상환경 생성"
if [ ! -d ".venv" ]; then
  python3 -m venv .venv
fi

echo "[3/4] 라이브러리 설치"
source .venv/bin/activate
python -m pip install --upgrade pip
pip install -r requirements.txt

echo "[4/4] 실행 권한 설정"
chmod +x setup.sh check.sh run.sh

echo
echo "설치 완료"
echo "다음 명령을 실행하세요: ./check.sh"
