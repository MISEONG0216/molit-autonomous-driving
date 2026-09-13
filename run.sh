#!/usr/bin/env bash
set -e

cd "$(dirname "$0")"

if [ ! -x ".venv/bin/python" ]; then
  echo ".venv가 없습니다. 먼저 ./setup.sh 를 실행하세요."
  exit 1
fi

case "${1:-}" in
  gps)
    exec .venv/bin/python src/gps_test.py
    ;;
  imu)
    exec .venv/bin/python src/imu_test.py
    ;;
  ctrl)
    exec .venv/bin/python src/ctrl_test.py
    ;;
  pure-pursuit)
    if [ -f "src/pure_pursuit_udp.py" ]; then
      exec .venv/bin/python src/pure_pursuit_udp.py
    else
      echo "Pure Pursuit 통합 코드는 아직 준비 중입니다."
      exit 1
    fi
    ;;
  *)
    echo "사용법:"
    echo "  ./run.sh gps          # GPS UDP 수신 테스트"
    echo "  ./run.sh imu          # IMU UDP 수신 테스트"
    echo "  ./run.sh ctrl         # 5 km/h 직진 3초 후 정지 테스트"
    echo "  ./run.sh pure-pursuit # 추후 통합 Pure Pursuit 실행"
    exit 1
    ;;
esac
