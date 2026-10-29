# V28 — Narrow merge-only fix (based on V27)

V27's **main control, V7 Pure Pursuit, 57/experimental 120 speed profile, traffic light, pedestrian WP720–1050, LiDAR, highway-follow and compact logging are unchanged**. No MORAI EgoCtrlCmd is ever transmitted by the merge worker.

## Issue 1: roundabout rear-left falsely stops after joining
- In **WP1770–1820**, active risk camera is **LEFT UDP9293 only**. **LEFT_REAR UDP1118** packets are drained but **NOT decoded, inferred, used as risk, or required to be fresh**.
- In highway merge **WP2480–2560 / WP3050–3190**, **LEFT9293 + LEFT_REAR1118** both resume full inference and both must be fresh. RIGHT9295 remains unused.
- The rear-left display window may show **REAR LEFT PAUSED (ROUNDABOUT)** but its image processing is off.
- On any mode change, stale JPEG fragments, bounding-box growth and previous-zone risks are reset. Rear-left risk field is **0.0** in the roundabout (excluded; not an actual sensor measurement). Missing/stale **left** image still generates MERGE_REJECT, not an unsafe accept.
- Existing main STOP latch and 3 distinct MERGE_ACCEPT confirmation packets are unchanged. Roundabout vehicle conflicts visible to the LEFT camera can still require stopping. Releasing after passing the real conflict point may require specifying a separate verified waypoint; none was invented.

## Issue 2: far / 2–3-lane vehicle false positives
- Comparison V23 merge code against V27: **the core `vehicle_risk()`, YOLO classes/confidence and stop threshold 0.60 were unchanged**. V27 was not the source of increased perception sensitivity.
- Keep original `vehicle_risk()` and **nearby bbox risk unchanged**; for **highway merge only**, apply a configurable factor to small-area bounding boxes:
  - area ratio <= **0.006** → multiply original risk by **0.45**
  - area ratio >= **0.030** → multiply by **1.00** (exact original behavior)
  - in between → linearly interpolate
- Parameters are in `config/integration_missions.json` merge section:
  `distance_sensitivity_enabled`, `far_area_ratio`, `near_full_risk_area_ratio`, `far_risk_weight`.
- **This does not measure actual distance or classify adjacent lanes.** A physically dangerous car far away could be downweighted; a large vehicle in an adjacent lane might still be counted. Use MORAI recordings and lane overlays to tune; do not claim guaranteed merge safety. To restore the old highway risk, set `distance_sensitivity_enabled` to `false`.

## Reproduction / checks
```bash
cd ~/molit-autonomous-driving
git fetch origin
git switch integration-v28-merge-roundabout-camera-filter
git pull --ff-only origin integration-v28-merge-roundabout-camera-filter
source .venv/bin/activate

python3 -m unittest discover -s tests -p 'test_merge_roundabout_v28.py' -v
python3 -m unittest discover -s tests -p 'test_merge_side_v25.py' -v

# Dry run (no EgoCtrlCmd)
bash scripts/run_integrated_v25.sh
# Supervised low-speed MORAI evaluation only after results are checked
bash scripts/run_integrated_v25.sh --drive --max-speed 15
```
Use `--detailed-logs` only for diagnosis. Normal run produces one integrated CSV and one session log. Expected `session_...` log: `[MERGE ZONE] ROUNDABOUT ... rear_left=OFF` and later `[MERGE ZONE] HIGHWAY_MERGE ... rear_left=ON`.

Do not assume any real MORAI test was run by this code commit.
