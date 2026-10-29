# V34 — GPS-shadow LiDAR wall clearance before static obstacle avoidance

Builds on `integration-v33-shadow-tunnel-avoidance-guard`; V33 only STOPped on every static `AVOIDING` request during GPS loss. V34 allows movement ONLY if raw LiDAR evidence confirms a candidate path has clearance.

## Why the V32 vehicle hit the tunnel wall

- The original `src/lidar/static_obstacle_config.json` uses `drivable_width_m: 7` and offsets `[-2.2, -1.8, 1.8, 2.2]`m. These are assumptions about path corridor width, **not measurements of tunnel walls**.
- `StaticObstaclePlanner._candidate_safety_diagnostic` checks the selected object's footprint and the assumed 7m corridor; a tunnel wall can be excluded from its DBSCAN static object set due to width/length filtering.
- On GPS loss the original V32 planner receives short-term IMU+vehicle-speed dead-reckoning, which can drift relative to the global route. Camera-lane steering normally dominates but the accepted local avoidance path overrides it, even beside the tunnel wall.

## V34 behavior

**With live GPS:** unchanged V31/V32 original static LiDAR planner, local Pure Pursuit and speed control.

**GPS lost and original static planner reports AVOIDING:**
1. Use the original planner's obstacle selection, path templates and candidate offsets, generating both LEFT and RIGHT alternatives.
2. Reject candidates that already fail the original selected-obstacle collision check.
3. Transform each candidate route into vehicle-frame coordinates using the bounded estimated pose and yaw.
4. Compare the swept candidate against **ALL fresh non-ground LiDAR returns**, not only accepted DBSCAN objects. Require observed left/right sectors and check for tunnel walls.
5. Include vehicle half-width **0.9m** plus **0.45m margin** around the entire evaluated path. Refuse every candidate whose swept corridor intersects a return, or whose sensing coverage is incomplete.
6. Choose a surviving candidate using the original planner's opposite-side/small-offset preference, otherwise send `STOP_REQUIRED`.
7. Continue original `AvoidancePathLatch`, but **revalidate the locked path at every new LiDAR scan**. Any loss of clearance => STOP.
8. Main accepts `AVOIDING` in shadow only when worker reports validated boolean `shadow_wall_verified: true` in UDP9303, with packet age <=**2.4s**. Reuse original main's local-path Pure Pursuit; original main retains sole EgoCtrlCmd control.
9. If no avoidance is active, LiDAR still stops for a near forward wall/object inside the ego footprint. The original LiDAR tracker also STOPs for `is_dynamic` tracked objects within 12m ahead and 1.8m half-width. Existing pedestrian YOLO/MiDaS emergency stop stays active.

**GPS shadow lane driving** is unchanged from V32: user-provided HSV+Canny + BEV + EMA lane-center steering at <=12km/h; **verified obstacle detours capped at 6km/h**. Recovery to V7 Pure Pursuit after valid GPS samples is unchanged.

## Important limitations

- The fixed original candidate offsets may leave **no feasible passage in a narrow tunnel**. In that case STOP is correct; the system will not attempt a fabricated clearance.
- The LiDAR wall checker has conservative but **uncalibrated** Z ROI (-1.45m to +1.25m), image-to-LiDAR alignment, and body footprint assumptions. It does not prove collision-free navigation, and cannot confirm unseen/occluded structures.
- The new scan gating requires points in both lateral sectors. Missing returns -> STOP, not an assumption that the side is empty.
- Global-XY route candidates still use bounded dead reckoning while GPS is missing. This is only an interim approach; fully independent long tunnel navigation would require local-frame trajectory planning using measured lane/wall boundaries.
- Unlike a full dynamic-object avoidance planner, LiDAR dynamic tracks STOP rather than overtake moving vehicles.

## Apply, test, then supervised MORAI DRIVE

```bash
cd ~/molit-autonomous-driving
git fetch origin
git switch integration-v34-shadow-lidar-wall-clearance
git pull --ff-only origin integration-v34-shadow-lidar-wall-clearance
source .venv/bin/activate
python3 -m unittest discover -s tests -p 'test_shadow_wall_v34.py' -v
python3 -m unittest discover -s tests -p 'test_shadow_tunnel_guard_v33.py' -v
python3 -m unittest discover -s tests -p 'test_gps_shadow_v32.py' -v
bash scripts/run_integrated_v25.sh --drive --max-speed 5 --detailed-logs
```

The script name remains V25; it launches **V34 branch contents** and all existing modules, plus the GPS-shadow lane worker.

**Test order:** start MORAI at a known valid GPS fix; approach tunnel slowly. Inspect worker status `SAFE_LEFT` or `SAFE_RIGHT` and `shadow_wall_verified` and monitor clearance. Do not test higher speed before confirming the actual LiDAR coverage and steering sign. If status `NO_SAFE_SIDE`, `SCAN_SPARSE`, or `WALL_OR_OBSTACLE_IN_SWEPT_PATH`, the vehicle **must stop**.

**Validation status:** V34 changes committed. Synthetic tests added; neither the tests nor actual MORAI simulation have been executed in the user's machine by the assistant.
