# V30: mission prewarm + roundabout completion + near-field highway following

Based strictly on V29. The three reported issues from 2026-10-29 MORAI runs are addressed without modifying V7 Pure Pursuit, traffic-light policy, static LiDAR planner, network ports, or the single EgoCtrlCmd publisher.

## 1. Abrupt STOP at waypoint zone boundaries
Evidence:
- 22:59:11 roundabout at WP1770: `MERGE_VISION_WAIT` then `MERGE_STOP_HOLD`, because side worker was previously gated off and first inference was unavailable.
- 22:54:56 pedestrian WP720: `pedestrian_stop_or_stale` while first pedestrian inference was starting.
- WP590 and WP1201 long STOPs are mostly `traffic_full_brake`, which is **not** a camera startup fault. V30 does **not** override real RED traffic-light stops or unsafe LiDAR results.

Fix:
- Start pedestrian, merge and follow inference **70 WP before** each mission's active zone (typically ~35m on 0.5m waypoint spacing).
- Inference before zone is observation-only; only main activates corresponding mission control inside its original WP range.
- At zone entry accept only **recent prewarm packets <=1.0s old**. Missing/stale/unsafe perception still retains existing fail-stop (or follow's 15km/h cap).
- Do NOT reset side-camera inference on prearm -> active transition.
- A fresh prewarmed pedestrian STOP_REQUIRED must still be latched at WP720 even if the packet arrived before entering the zone.

## 2. Roundabout no repeat stop after verified entry
- WP1770–1820 continues to monitor **LEFT9293** only; rear-left1118 remains disabled.
- State machine: (1) STOP/vision-wait actually occurred, (2) three distinct fresh MERGE_ACCEPT updates release the stop, (3) advance >=10 WP after that release, (4) reach at least **WP1800**, (5) camera still reports fresh MERGE_ACCEPT at completion. THEN transition to `ROUNDABOUT_DONE`.
- Once done, both merge camera inference channels are disabled and the merge mission no longer stops the vehicle until WP1820. **Traffic-light, pedestrian, LiDAR, and other safety constraints remain active**.
- Any new danger before completion resets safe-release progress and keeps the stop latch.
- **WP1800 is provisional and NOT an independently verified physical conflict-point boundary.** Validate geometry in MORAI before higher speeds. Do not equate “once stopped” with “safe to depart”.

## 3. Highway following misses close vehicles
- Shared front camera remains UDP9291 -> localhost relay; no duplicate camera bindings.
- Highway mode now processes image up to **98% height** instead of the prior 78% hood crop. Pedestrian-only mode retains original 78%.
- Replace narrow `LANE_ROI_RATIO` bottom-center-only gate with near-widening trapezoid and horizontal bbox-overlap test. The near edge spans normalized x 0.12–0.88 at y 0.97; far edge x 0.44–0.56 at y 0.38.
- Do not discard a large near-field detection merely because its bounding box touches the bottom of the crop.
- If a tracked in-lane vehicle disappears from detection, retain its last score for up to 0.8s and fade until 1.6s. After that, return to the current camera score. This is NOT a calibrated range/headway/TTC estimate and can still miss vehicles.

## Files modified
- `src/integration/merge_side_io.py`
- `src/merge_side_camera_worker.py`
- `src/integration/mission_manager.py`
- `src/main_integrated_v1.py`
- `src/integration/highway_follow_io.py`
- `src/dynamic_pedestrian_worker.py`
- `config/integration_missions.json`
- Regression tests `test_v30_prewarm_roundabout_follow.py` plus compatibility adjustments to V28/V29 tests.

## Apply and supervised MORAI DRIVE run
```bash
cd ~/molit-autonomous-driving
git fetch origin
git switch integration-v30-prewarm-roundabout-near-follow
git pull --ff-only origin integration-v30-prewarm-roundabout-near-follow
source .venv/bin/activate
python3 -m unittest discover -s tests -p 'test_v30_prewarm_roundabout_follow.py' -v
python3 -m unittest discover -s tests -p 'test_merge_roundabout_v28.py' -v
python3 -m unittest discover -s tests -p 'test_merge_complete_v29.py' -v
python3 -m unittest discover -s tests -p 'test_merge_side_v25.py' -v
bash scripts/run_integrated_v25.sh --drive --max-speed 15
```
DRY-RUN is intentionally omitted per user preference. Do not test 120km/h until near-ROI false positives and missed vehicles are evaluated under low-speed supervision.

Expected worker log: `ROUNDABOUT_PREARM` before WP1770, `ROUNDABOUT` at WP1770, `ROUNDABOUT_DONE` only after confirmed safe entry; `HIGHWAY_PREARM` before WP2480 and WP3050. Main display: `MERGE: ROUNDABOUT_DONE` after completion.

**Status**: committed on GitHub. Tests and MORAI simulation have NOT been executed in the user's Ubuntu environment by the assistant.
