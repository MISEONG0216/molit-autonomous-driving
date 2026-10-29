#!/usr/bin/env bash
# Start all V26 corrected left + left-rear camera workers plus the one and only EgoCtrlCmd publisher.
# Default: DRY-RUN (no vehicle control). Use --drive for supervised MORAI test.
set -euo pipefail
cd "$(dirname "$0")/.."
if [[ ! -f .venv/bin/activate ]]; then
  echo "[ERROR] .venv not found; activate/install project virtualenv first" >&2
  exit 1
fi
source .venv/bin/activate
# A rear-left camera is NOT included in the supplied official front/left/right set.
# Never substitute the RIGHT camera (9295) for the missing left-rear view.
python3 - <<'PY'
import json
from pathlib import Path
p = Path("config/integration_missions.json")
cfg = json.loads(p.read_text(encoding="utf-8"))["merge"]
port = cfg.get("rear_left_camera_port")
reserved = {9291, 9293, 9295, 9300, 9301, 9302, 9303, 9304, 9305, 9306, 9307}
if type(port) is not int or not (1 <= port <= 65535) or port in reserved:
    raise SystemExit(
        "[ERROR] Left-REAR camera port must be verified and set in "
        "config/integration_missions.json -> merge.rear_left_camera_port. "
        "Right camera UDP9295 is forbidden. Launcher stopped before any vehicle control."
    )
print(f"[MERGE CAM] LEFT:9293 + LEFT_REAR:{port}; RIGHT:DISABLED")
PY
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
# Default: ONE main CSV and ONE shared worker log per run.
# Optional --detailed-logs: per-worker logs plus LiDAR per-scan CSV.
DETAILED_LOGS=0
MAIN_ARGS=()
for arg in "$@"; do
  if [[ "$arg" == "--detailed-logs" ]]; then
    DETAILED_LOGS=1
  else
    MAIN_ARGS+=("$arg")
  fi
done
STAMP="$(date +%Y%m%d_%H%M%S)"
if [[ "$DETAILED_LOGS" == "1" ]]; then
  export STATIC_CSV_LOG=1
  echo "[LOG] detailed mode: main CSV, LiDAR CSV and 4 worker .log files"
  python3 -u src/traffic_light_detector.py --relay-pedestrian >"logs/v25_traffic_${STAMP}.log" 2>&1 &
  PIDS+=("$!")
  python3 -u src/dynamic_pedestrian_worker.py --show --follow >"logs/v25_pedestrian_${STAMP}.log" 2>&1 &
  PIDS+=("$!")
  python3 -u src/lidar/static_avoidance_module.py >"logs/v25_static_${STAMP}.log" 2>&1 &
  PIDS+=("$!")
  python3 -u src/merge_side_camera_worker.py --show >"logs/v25_merge_${STAMP}.log" 2>&1 &
  PIDS+=("$!")
else
  export STATIC_CSV_LOG=0
  SESSION_LOG="logs/session_${STAMP}.log"
  : > "$SESSION_LOG"
  echo "[LOG] compact mode: integrated CSV + $SESSION_LOG (all worker outputs)"
  python3 -u src/traffic_light_detector.py --relay-pedestrian >>"$SESSION_LOG" 2>&1 &
  PIDS+=("$!")
  python3 -u src/dynamic_pedestrian_worker.py --show --follow >>"$SESSION_LOG" 2>&1 &
  PIDS+=("$!")
  python3 -u src/lidar/static_avoidance_module.py >>"$SESSION_LOG" 2>&1 &
  PIDS+=("$!")
  python3 -u src/merge_side_camera_worker.py --show >>"$SESSION_LOG" 2>&1 &
  PIDS+=("$!")
fi
sleep 2
echo "[V27] main starting: traffic, static, pedestrian, merge, highway-follow controls ON"
echo "[V25] additional args: ${MAIN_ARGS[*]}"
echo "[V25] if camera previews do not open, check session log or detailed logs."
python3 src/main_integrated_v1.py \
  --traffic --static --static-control \
  --pedestrian --pedestrian-control \
  --merge --merge-control \
  --follow --follow-control --max-speed 57 "${MAIN_ARGS[@]}"
