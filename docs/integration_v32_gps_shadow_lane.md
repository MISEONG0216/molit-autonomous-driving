# V32 — GPS blackout: user's lane center + existing LiDAR/pedestrian controls

**Source**: builds on V31 `integration-v31-roundabout-right-half-roi`. Original uploaded `morai_lanenet_viz.py` is HSV yellow/white segmentation + Canny + perspective BEV + polynomial fitting + EMA 0.25; it does **not** load or use TensorFlow LaneNet pretrained weights. The accompanying LaneNet source ZIPs/README are therefore not required to run this particular visual lane routine.

## Arbitration and architecture

Only `src/main_integrated_v1.py` sends MORAI EgoCtrlCmd (cmd_type=1, ctrl_mode=2).

| Condition | Steering owner | Speed/stop owner |
|---|---|---|
| Live GPS | Existing V7 Pure Pursuit | Existing missions and V7 longitudinal policy |
| GPS invalid (0/0), or cached coordinate >1s while Competition Status reports movement | Camera lane center (user's HSV/Canny BEV/EMA method) | 12km/h upper cap, plus existing traffic/merge/highway/obstacle limits |
| Fresh LiDAR static `AVOIDING` path during blackout | **Original** LiDAR local-path Pure Pursuit has priority over lane steering (even if obstacle occludes lane) | Original avoidance speed cap <=25; blackout cap <=12 |
| Dynamic pedestrian risk / STOP_REQUIRED | No steering override | Original YOLO(person)+MiDaS latch: FULL STOP |
| LiDAR STOP_REQUIRED, stale LiDAR, invalid lane with no LiDAR avoidance, IMU/status invalid, GPS uninitialized, blackout expiry / excess drift | None | FULL STOP |

**GPS shadow guidance does NOT steer along guessed global waypoints.** The IMU yaw + Competition Vehicle Status signed speed integrate an approximate pose only to preserve the original LiDAR detector/tracker/global-XY local-path planner and waypoint mission gating. This bounded dead-reckoning is **not** an independent absolute localization system.

**Important limitation:** The original dynamic module covers **pedestrians (COCO person)**, not all moving cars. The original LiDAR static planner still references the global waypoint file for local detours and may become inaccurate with dead-reckoning drift. Completely GPS-independent, long-duration obstacle avoidance would require an ego-centric LiDAR path planner and calibrated lane geometry that these sources do not provide.

## Camera port ownership

Traffic light worker remains **the only process binding front camera UDP9291**. It fans out each camera datagram via localhost UDP to:
- pedestrian/highway YOLO+MiDaS worker: 9305 (unchanged)
- new lane-center worker: **9311**

New lane worker publishes validated `LANE_CENTER` JSON to **127.0.0.1:9312**. It neither binds 9291 nor sends vehicle control. Side cameras 9293/1118 and LiDAR1116 are unchanged.

## Invalid/stale GPS and recovery
- GPS position 0/0 -> immediate shadow if a prior good fix exists; otherwise stop until first fix.
- A nonzero GPS coordinate held unchanged for >1.0s **while moving >=2km/h** is considered stale (a cached Receiver object is not valid new GPS evidence).
- During shadow, speed <=12km/h, max 12s or 35m without GPS, max 7m deviation of short-term estimated pose from global path, control-loop timestamp gap <=0.8s.
- Lane frame must be valid and <=0.8s old; otherwise stop unless an original fresh LiDAR avoidance path has been accepted.
- Restore V7 only after **three distinct plausible** GPS coordinate updates within 6m of the short-term integrated pose. This avoids a single cached GPS fix causing an unsafe steering mode flip.
- These thresholds are **conservative trial settings**, not calibrated guarantees. Long blackout -> FULL STOP.

## Existing obstacle code reused

`src/lidar/static_avoidance_module.py`: original LiDAR UDP1116 receiver -> DBSCAN detection -> tracker -> StaticObstaclePlanner -> AvoidancePathLatch -> result UDP9303. During GPS shadow, the module is enabled irrespective of global static mission zone and receives a bounded DR pose with `gps_shadow=true` from main. It does **not** send EgoCtrlCmd. A new local path overrides lane steering in main.

`src/dynamic_pedestrian_worker.py`: original camera YOLO(person)+MiDaS risk, STOP_REQUIRED / CLEAR UDP9304. During GPS shadow, main sends `gps_shadow=true` on existing gate9306, so detection operates even outside WP720-1050. The original 3-clear-frame stop latch remains unchanged.

All V31 signal, roundabout, highway merge, front following and static controls are retained. No new unauthorized EgoVehicleStatus position fields are read.

## Ubuntu setup and supervised MORAI real DRIVE

```bash
cd ~/molit-autonomous-driving
git fetch origin
git switch integration-v32-gps-shadow-lane-obstacle
git pull --ff-only origin integration-v32-gps-shadow-lane-obstacle
source .venv/bin/activate

python3 -m unittest discover -s tests -p 'test_gps_shadow_v32.py' -v

bash scripts/run_integrated_v25.sh --drive --max-speed 10
```

V32 adds `--gps-shadow` to the existing launch script automatically; it also runs the lane worker. Do **not** bind the user's original standalone `morai_lanenet_viz.py` directly to UDP9291 while this integrated launcher is running (it would compete with traffic camera ownership). Stop any previous V31/V30 launcher first.

CSV now adds `gps_mode`, `gps_shadow_sec`, `lane_valid`, `lane_confidence`, `lane_steering_rad`, and `shadow_stop`, preserving old columns. Session log shows `[GPS SHADOW LANE]` and camera relay startup.

## What still needs validation

1. MORAI 9291 forward video actually delivers ~12Hz usable JPEG frames to the lane worker; compare BEV lane segmentation with road markings.
2. Steering **sign**, 0.20 gain, 0.22rad cap and 12km/h cap need supervised low-speed track testing.
3. Initial GPS must be valid before entering shadow. Test both zero GPS and nonzero stale cached NMEA during movement.
4. LiDAR detour using DR pose should be compared against MORAI positions in the blackout; abort if path geometry is inconsistent.
5. Existing dynamic obstacle module **does not** prove vehicle/pedestrian collision avoidance; STOP behavior and hold/release require staged tests.
6. A run lasting >12 seconds in GPS blackout intentionally stops. Do not remove this limit without new independent localization evidence.

**Validation status**: Code committed to V32 branch. Offline test cases added but not executed in the user's Ubuntu or MORAI by the assistant. No claim of verified real drive performance.
