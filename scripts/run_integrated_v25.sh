#!/usr/bin/env bash
# Start all V25 perception workers plus the one and only EgoCtrlCmd publisher.
# Default: DRY-RUN (no vehicle control). Use --drive for supervised MORAI test.
set -euo pipefail
cd "$(dirname "$0")/.."
if [[ ! -f .venv/bin/activate ]]; then
  echo "[ERROR] .venv not found; activate/install project virtualenv first" >&2
  exit 1
fi
source .venv/bin/activate
mkdir -p logs
PIDS=()
cleanup() {
  trap - EXIT INT TERM
  echo "[V25] stopping perception workers..."
  for pid in "${PIDS[@]}"; do
    kill "$pid" 2>/dev/null || true
  done
  wait 2>/dev/null || true
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
STAMP="$(date +%Y%m%d_%H%M%S)"
echo "[V25] perception logs: logs/v25_*_${STAMP}.log"
python3 -u src/traffic_light_detector.py --relay-pedestrian >"logs/v25_traffic_${STAMP}.log" 2>&1 &
PIDS+=("$!")
python3 -u src/dynamic_pedestrian_worker.py --show >"logs/v25_pedestrian_${STAMP}.log" 2>&1 &
PIDS+=("$!")
python3 -u src/lidar/static_avoidance_module.py >"logs/v25_static_${STAMP}.log" 2>&1 &
PIDS+=("$!")
python3 -u src/merge_side_camera_worker.py --show >"logs/v25_merge_${STAMP}.log" 2>&1 &
PIDS+=("$!")
sleep 2
echo "[V25] main starting: --traffic --static --pedestrian --merge (all mission controls enabled)"
echo "[V25] additional args: $*"
echo "[V25] check logs/v25_merge_${STAMP}.log if left/right previews do not open."
python3 src/main_integrated_v1.py \
  --traffic --static --static-control \
  --pedestrian --pedestrian-control \
  --merge --merge-control --max-speed 57 "$@"
