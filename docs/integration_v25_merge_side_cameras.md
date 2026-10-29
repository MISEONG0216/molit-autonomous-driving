# V25: Side-camera merge perception integrated with V24

Branch: `integration-v25-merge-side-cameras` (V24 kept unchanged).

## V25 behavior
- Main remains **the only** MORAI EgoCtrlCmd publisher; uses existing allowed cmd_type=1 and ctrl_mode=2.
- Existing V24 traffic, V22 green release, LiDAR static avoidance, and pedestrian WP720–1050 remain enabled and unchanged.
- Left MORAI camera UDP9293 + right MORAI camera UDP9295 are captured by `src/merge_side_camera_worker.py`.
- `--show` shows **both side-camera windows even before the merge zone** (raw previews outside zone).
- Main publishes current nearest WP to localhost UDP9307; worker only runs YOLO on these cameras within configured merge zones (WP2480–2560, WP3050–3190) and roundabout WP1770–1820.
- YOLO COCO classes 2/3/5/7 = car/motorcycle/bus/truck, model yolov8n.pt.
- Side-image apparent object size and bounding-box growth produce a **heuristic** 0..1 risk score per side. **It is NOT calibrated metric distance, closing speed, gap, or time-to-collision.** This is a simulation prototype needing scenario validation; it cannot guarantee every moving car is detected.
- Worker reports `MERGE|left|right|max|MERGE_REJECT` or `MERGE_ACCEPT` to localhost UDP9301. Main already parses these messages.
- Any side camera missing, stale inference, or a high-risk car produces a REJECT; main also fails closed when UDP result missing/stale in the configured zone. Hold releases after three new safe camera result packets. A timer-only repeated result does **not** produce fake safe confirmations.
- Side worker never adjusts steering, writes controls or alters the global path; global V7 path remains.
- When entering a merge zone, main ignores an old last-zone acceptance until a fresh merge packet arrives.

## Run on Ubuntu
Ensure MORAI Windows `172.20.10.2` delivers camera packets to Ubuntu `172.20.10.5` ports 9291, 9293, 9295; Competition Status UDP1909, LiDAR1116, GPS9201, IMU1111, control target Windows9093.

```bash
cd ~/molit-autonomous-driving
git fetch origin
git switch integration-v25-merge-side-cameras
git pull --ff-only origin integration-v25-merge-side-cameras
source .venv/bin/activate
python3 -m unittest discover -s tests -p 'test_merge_side_v25.py' -v
# First run: DRY-RUN; no EgoCtrlCmd sent
bash scripts/run_integrated_v25.sh
# Then, only after verifying camera preview and risk/stop logs:
bash scripts/run_integrated_v25.sh --drive
```

The launcher starts four worker processes (front traffic + camera relay, pedestrian, LiDAR, left/right side camera) and the single main process. All logs are under `logs/v25_*_<timestamp>.log`, main creates `logs/integrated_v1_*.csv`. It terminates workers when main stops. Do not run other processes binding UDP9291/9293/9295/1116 or another EgoCtrlCmd sender.

With MORAI near WP2480–2560 or 3050–3190, the merge log should show `[MERGE ZONE] ON` and `MERGE_REJECT` while a nearby car is observed; main display shows `MERGE_STOP_HOLD` / `MERGE_FULL_BRAKE`. Only an empty/low-risk **fresh** observation from both cameras can release the stop. The worker also treats WP1770–1820 as merge-monitoring because that roundabout zone already existed in config.

## Important unresolved validation
1. The original user-provided source material does not specify a ground-truth calibrated merge detector, lane-relative ROI, side camera depth or velocity. Vision risk uses apparent object size and growth only. It can miss distant fast vehicles, occluded vehicles, or confuse traffic in adjacent lanes. **Do not claim verified merge safety.**
2. A 57 km/h V7 target before the designated merge zones can require pre-merge braking; current code limits speed once inside zone but does not validate stopping distance. Confirm the exact beginning of ramp/gap zones and add a safe approach distance if required.
3. Three GPU-active detectors (traffic, pedestrian, merge) can overload available compute. Measure inference rate, UDP freshness and commanded vs actual speed in a reproducible MORAI run.
4. V24 WP720–760 overlaps the static module WP590–760. The old second-signal and static module issues remain separate and have **not** been automatically changed by V25.
5. If YOLO weights are missing, worker exits. Until it is running and sending valid information, merge-control must not be assumed to perform safe merging.
6. Never use this experimental heuristic on a real road without full validation and safety architecture.
