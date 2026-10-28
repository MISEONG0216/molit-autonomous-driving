# V23 — camera pedestrian STOP mission, WP430–580 only (experimental)

Based on `integration-v22-green-release-second`; earlier branches remain unchanged.

## Source and architecture

The uploaded front-camera monitoring source uses COCO YOLOv8n, MiDaS_small,
a bird's-eye transformed box area, relative depth and apparent box growth.
The original only displays `EMERGENCY_STOP` and does not send a main
control request. V23 integrates its weighted risk calculation as a
standalone decision worker and makes **only main** transmit EgoCtrlCmd.

**Do not start the uploaded raw script alongside the traffic detector.**
Both would bind MORAI camera port 9291 and compete for UDP data.

Instead:
- `src/traffic_light_detector.py --relay-pedestrian` is the **single**
  MORAI front-camera UDP 9291 listener. It keeps its original traffic
  decision port 9300 and additionally relays *raw camera packets* on
  localhost port 9305.
- `src/dynamic_pedestrian_worker.py` receives the relayed camera UDP
  9305, runs supplied YOLO/MiDaS risk logic **only inside the configured
  pedestrian mission**, sends local JSON risk/status on port 9304, and
  never sends EgoCtrlCmd.
- `src/main_integrated_v1.py` sends the current nearest waypoint on
  localhost gate UDP 9306, receives and checks worker decisions on
  port 9304, and applies any STOP with full brake only in the dynamic
  pedestrian zone. It still owns the sole EgoCtrlCmd sender (9093).

## Dynamic-only waypoint gate

`config/integration_missions.json`:

```json
"dynamic_pedestrian": {
  "zones_wp": [[430, 580]],
  "result_timeout_sec": 3.5,
  "stop_risk_threshold": 0.50,
  "clear_confirm_frames": 3
}
```

**WP430–580 is PROVISIONAL**, based on the mission diagram and earlier
logs showing the first signal/stop near WP414 and second near WP620.
The provided diagram does **not** establish exact pedestrian spawn
boundaries. The team must inspect MORAI scenario and adjust the range
before relying on it in competition.

* Outside WP430–580, the pedestrian worker drops relayed frames
  without YOLO/MiDaS inference and emits no stop decisions.
  Main also ignores all pedestrian status outside this zone and follows
  pre-existing V22 control. The static LiDAR mission remains at WP590–760.
* Inside WP430–580, a COCO `person` (class 0) with the supplied
  weighted risk score >= 0.50 requests `STOP_REQUIRED`.
  Source car/bus/truck classes remain detectable for display but **do not
  cause pedestrian-only STOP**.
* A risk stop latches until 3 distinct fresh `CLEAR` frames. Stale or
  missing pedestrian results inside the zone also request STOP.
* Exiting WP580 disables this mission and clears its latch. This is a
  strict competition-area gate, **not** a safe default for unknown
  pedestrian placements outside the designated zone.
* LiDAR static STOP, traffic RED STOP and merge STOP retain their original
  full-brake priority. An approved WP620 green release cannot override
  a pedestrian STOP.

## Run on Ubuntu

Fetch branch and test:

```bash
cd ~/molit-autonomous-driving
git fetch origin
git switch integration-v23-pedestrian-camera-stop
git pull --ff-only origin integration-v23-pedestrian-camera-stop
source .venv/bin/activate
python3 -m unittest discover -s tests -p 'test_dynamic_pedestrian_v23.py' -v
python3 -m unittest discover -s tests -p 'test_second_green_release_v22.py' -v
python3 -m unittest discover -s tests -p 'test_static_zone_v19.py' -v
```

Start **four separate terminals**, all in the repo directory and active
virtual environment:

Terminal 1 — the existing traffic detector is the **only** MORAI 9291
camera receiver. The relay flag is mandatory for pedestrian integration:

```bash
python3 src/traffic_light_detector.py --relay-pedestrian
```

Terminal 2 — pedestrian YOLO+MiDaS worker:

```bash
python3 src/dynamic_pedestrian_worker.py --show
```

The `--show` GUI flag is optional. The first start may require downloading
the original YOLO and MiDaS model weights. Check your environment for
`torch`, `torchvision`, `ultralytics`, `opencv-python` and dependency
compatibility. Do not run a second script binding port 9291.

Terminal 3 — previous successful LiDAR module:

```bash
python3 src/lidar/static_avoidance_module.py
```

Terminal 4 — first verify DRY-RUN (no control packets sent):

```bash
python3 src/main_integrated_v1.py \
  --traffic --static --pedestrian --max-speed 57
```

When the worker, GO/STOP traffic signal and dynamic zone are verified,
run a closely supervised MORAI simulation-only control test:

```bash
python3 src/main_integrated_v1.py \
  --drive --traffic --static --static-control \
  --pedestrian --pedestrian-control --max-speed 57
```

Main terminal should show:
- Outside zone: `PEDESTRIAN: ZONE_OFF`, no emergency stop.
- Upon WP430 entry: `VISION_WAIT` may briefly brake while waiting for
  fresh camera results. `CLEAR` means no risk above threshold.
- When pedestrian high-risk: `STOP_REQUIRED` and
  `PEDESTRIAN_FULL_BRAKE` (unless another stop priority is active).
- After 3 distinct CLEAR frames: releases stop, resuming the other missions.
- At WP581: pedestrian inference and stop requests become inactive.

## Verify and tune

At WP430–580, test a crossing pedestrian plus a no-pedestrian drive,
then inspect main `logs/integrated_v1_*.csv` new columns:
`pedestrian_zone`, `pedestrian_state`, `pedestrian_risk`,
`pedestrian_count`, `pedestrian_stop`, `pedestrian_age_sec`,
`pedestrian_frame` and `pedestrian_clear_count`.

The source MiDaS output is **relative depth**, not a calibrated metric
distance or trustworthy time-to-collision estimate. The source's
`prev_bbox_areas` matched by detection index rather than persistent track
identity can also spike. This is an initial experimental stop policy, not
validated pedestrian collision avoidance. Camera heavy inference can slow
both processes on CPU. Preserve source constraints and compare actual MORAI
collision and brake-to-stop behaviour before competition use.
