# V27 — Forward in-lane highway following + experimental speed extension

Base: `integration-v23-rebased-ped720-1050-merge-left-rear`. Existing traffic, static LiDAR, pedestrian WP720–1050, left+rear-left merge detection and **compact logging** remain.

## Mission boundaries (as provided)
- **Following/graded speed**: WP **2570–3040**; WP **3200–tollgate exit**.
- Tollgate-exit WP is **unknown** in the provided configuration. `zones_wp=[[2570,3040],[3200,null]]` means following remains armed through the end of the global path until the exact tollgate exit is identified. This avoids removing safety protection early.
- **Experimental 120 km/h permission**: currently **only WP2570–3040**, because those exact start/end points were supplied. The second fast zone remains **<=57 km/h** until `highway_follow.tollgate_exit_wp` is set to the actual waypoint in MORAI. There is **no invented tollgate WP**.
- Existing merge-risk STOP remains WP2480–2560, WP3050–3190, plus roundabout1770–1820. Overlaps may request the lower speed or STOP.

## Source code adaptation and improvements
- Original source: user-provided front ROI vehicle risk monitor, **trapezoid** `((.48,.48),(.52,.48),(.66,.78),(.34,.78))`, bbox-bottom-center lane test; YOLOv8n car/bus/truck; MiDaS_small relative depth(0.4), warped area(0.2), bbox growth(0.4).
- The original source's independent raw camera `bind(9291)` would race with the existing traffic signal process. Instead the **existing `traffic_light_detector.py` is the only raw front-camera listener**, sending packets to the preexisting localhost relay9305. `dynamic_pedestrian_worker.py --follow` now shares its ONE GPU YOLO/MiDaS inference pipeline with the forward-following task.
- Pedestrian outputs go to UDP9304 **only in WP720–1050**. Front vehicle following goes to UDP9308 **only in its designated zones**. Model never sends EgoCtrlCmd.
- Main `src/main_integrated_v1.py --follow --follow-control` receives recent messages and converts grades into **speed caps** rather than image-score-induced full brake. Raw score is an **uncalibrated image-space proxy**, NOT distance, relative speed or verified time-to-collision.
- Increase risk immediately; use 0.80 per inference decay for release to reduce frame-by-frame oscillation. Result stale/missing or wrong waypoint within zone => **15 km/h cap**. Other traffic/pedestrian/static/merge emergency STOP commands retain precedence.

| Risk | Grade | Maximum target km/h |
|---|---:|---:|
| <0.20 | 0 | 120 |
| 0.20–<0.40 | 1 | 90 |
| 0.40–<0.60 | 2 | 65 |
| 0.60–<0.80 | 3 | 40 |
| >=0.80 | 4 | 20 |
| No fresh camera observation | VISION_WAIT | 15 |

The commanded speed is the **minimum** of V7/experimental curvature profile, grade cap, static/traffic/merge/pedestrian caps, and the requested global maximum. The main alone outputs legal MORAI EgoCtrlCmd (cmd_type=1, ctrl_mode=2).

## About 120 km/h
- V7 Pure Pursuit is actually validated only up to **57 km/h**. This new feature is strictly an **experimental simulation-only** target-profile generator, **not a proven safe 120-km/h controller**. Existing V7 pure pursuit steering and LFD remain unchanged, which may be unsuitable at 120.
- The fast speed profile uses capped curvature-derived targets `v<=sqrt(2.0 m/s² * radius) * 3.6`, forward acceleration feasibility `3.0 m/s²`, and braking preview `2.0 m/s²`. These are **assumptions**, not experimentally verified vehicle braking or lateral grip values.
- A short fast segment may **not physically reach 120** because the approach/deceleration distance must be respected.
- Beyond configured fast zones normal profile remains <=57. An unspecified tollgate endpoint is **not silently considered a high-speed permission**.
- The user's original algorithm cannot verify whether **ego speed exceeds lead-car speed**. Bbox-area growth is only a crude approaching proxy. Missed detections, occlusions, camera decode errors and heavy GPU workloads remain potential hazards. **Do not operate unsupervised**.

## Run
```bash
cd ~/molit-autonomous-driving
git fetch origin
git switch integration-v27-highway-follow-risk-speed
git pull --ff-only origin integration-v27-highway-follow-risk-speed
source .venv/bin/activate
python3 -m unittest discover -s tests -p 'test_highway_follow_v27.py' -v

# Sim dry run, previous validated <=57 profile:
bash scripts/run_integrated_v25.sh

# Supervised slow driving validation first:
bash scripts/run_integrated_v25.sh --drive --max-speed 15

# Then staged acceleration, only after inspecting speed tracking, yaw/steering,
# camera freshness and braking in MORAI:
bash scripts/run_integrated_v25.sh --drive --max-speed 57
bash scripts/run_integrated_v25.sh --drive --max-speed 70
bash scripts/run_integrated_v25.sh --drive --max-speed 90
# Only after stable staged tests:
bash scripts/run_integrated_v25.sh --drive --max-speed 120
```

Default remains 57; `--max-speed 120` is an explicit opt-in. The launcher still writes **one integrated CSV and one session log** unless `--detailed-logs` is supplied. Main CSV now includes `follow_zone,state,risk,grade,speed_cap,age,vehicles`. Check `HIGHWAY` terminal line and these CSV fields.

## Unverified / operator checks
1. **Find exact WP at which tollgate is passed**, and enter `highway_follow.tollgate_exit_wp` in `config/integration_missions.json` only after verifying. Review road geometry and speed regulations before enabling 120 for second zone.
2. Confirm UDP9291 raw -> traffic light -> relayed9305 -> shared worker -> forward result9308 -> main and both merge side cameras (9293 & 1118). No right-side camera 9295.
3. Verify at least 1.5s freshness and YOLO/MiDaS performance, and compare actual Competition Vehicle Status speed against target under each grade.
4. Check highway 120 prohibition/permission in the competition rulebook and MORAI vehicle physical limits. They are not validated by this commit.
5. Test with leading vehicles of different speeds, near/far, other-lane vehicles, camera faults, and merging interactions. No real MORAI integration test has been run by the assistant.
