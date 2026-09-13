#!/usr/bin/env bash
set -e

cd "$(dirname "$0")"

if [ ! -d ".venv" ]; then
  echo ".venv가 없습니다. 먼저 ./setup.sh 를 실행하세요."
  exit 1
fi

source .venv/bin/activate

if [ -f "src/main.py" ]; then
  python src/main.py
else
  echo "src/main.py가 아직 없습니다. Pure Pursuit 통합 코드를 추가한 뒤 여기서 실행됩니다."
fi
