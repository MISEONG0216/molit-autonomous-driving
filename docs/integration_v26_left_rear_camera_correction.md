# V26: merge camera orientation correction (LEFT + LEFT_REAR ONLY)

The operator clarified that the two merge cameras are (1) LEFT-facing side and (2) LEFT-REAR-facing, **not** LEFT+RIGHT. V25 incorrectly subscribed to the RIGHT camera UDP9295. V26 never binds that port.

## Configuration required before starting
- Existing LEFT camera UDP9293 is known.
- **LEFT-REAR camera UDP port is not known from currently provided verified sensor settings.** The official supplied 3-camera sensor set documents FRONT9291, LEFT9293, RIGHT9295; it does not document the extra rear-left-facing camera. Do not guess a port or repurpose RIGHT9295.
- In MORAI's actual sensor/network settings, verify the independently configured rear-left camera **receiver / destination port**, and ensure its destination IP is Ubuntu 172.20.10.5.
- Edit `config/integration_missions.json` -> `merge.rear_left_camera_port` from `null` to the verified integer. Valid value must not overlap FRONT9291 / LEFT9293 / RIGHT9295 or local integration ports 9300–9307. Set correct camera orientation separately in MORAI.
- If RIGHT camera must be physically OFF, disable that sensor or its UDP publisher in MORAI; code only stops consuming it.
- Until rear-left port is configured, `scripts/run_integrated_v25.sh` **refuses to start** (no control command). This is deliberate so incomplete perception cannot be mistaken for validated merging.

## Dataflow
```
MORAI LEFT UDP9293 -------\
                           > merge_side_camera_worker.py -> MERGE|side_risk|rear_risk|max_risk|decision -> UDP9301 -> main
MORAI LEFT_REAR UDP<port> /
MORAI RIGHT UDP9295 (NOT USED)
```

- The worker shows two windows: MORAI MERGE LEFT and MORAI MERGE LEFT_REAR (`--show`) even outside the merge waypoint zones.
- YOLO is run inside configured merge zones only (WP2480–2560, WP3050–3190, configured roundabout WP1770–1820).
- Image-space bounding box size/growth are uncalibrated surrogates, **not** metric range, speed or verified time-to-collision. Three clear results in the main module release a latched stop. Invalid/missing outputs fail closed in active merge zones.
- Existing GPS, V7 steering, traffic, pedestrian WP720–1050, static 25km/h logic are untouched. The merge worker never sends EgoCtrlCmd.

## Apply
```bash
cd ~/molit-autonomous-driving
git fetch origin
git switch integration-v26-left-rear-merge-cameras
git pull --ff-only origin integration-v26-left-rear-merge-cameras
source .venv/bin/activate
python3 -m unittest discover -s tests -p 'test_merge_side_v25.py' -v
# First configure actual rear-left port in config/integration_missions.json.
bash scripts/run_integrated_v25.sh          # DRY-RUN
bash scripts/run_integrated_v25.sh --drive  # only after live simulation validation
```

No successful real MORAI run has been claimed. Verify UDP receipt, both decoded views, car recognition, STOP command and release in a safe controlled simulation before drive testing.
