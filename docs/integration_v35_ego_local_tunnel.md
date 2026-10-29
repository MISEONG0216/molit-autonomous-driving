# V35 GPS-shadow tunnel: ego-frame LiDAR path + lane camera

**Problem established from 2026-10-30 00:58 run:** V34 at WP3827 reported `SHADOW_WALL_OK_RIGHT` despite right tunnel wall; by scan431 it changed to `SHADOW_WALL_UNVERIFIED:WALL_OR_OBSTACLE_IN_SWEPT_PATH`, and the main displayed `GPS MODE=STOP outage=25.7s` because V32's old GPS blackout cutoff was 12s. Lane detector was fresh with confidence 1.00; pedestrian was clear. Simply increasing the 7m global path width or removing STOP is unsafe.

## What changed
- `src/lidar/shadow_ego_planner.py`: completely separate vehicle-local candidate generation for GPS shadow using the ORIGINAL LiDAR receiver point coordinate (+forward,+left,+up). Detect frontal obstacle inside ego corridor; candidate LEFT and RIGHT lateral offsets are swept against **ALL fresh non-ground LiDAR returns**, including tunnel walls excluded by width-limited static-object clustering. If no frontal obstacle, return CLEAR and follow camera lane. If either side lacks LiDAR coverage or neither candidate clears vehicle 1.8m + 0.45m margin, STOP.
- `src/lidar/static_avoidance_module.py`: uses ego-local planner ONLY when `gps_shadow=true`. It **locks the first accepted maneuver** instead of shifting laterally an additional 1.6m on every scan. Every scan verifies the remaining locked route via current point-cloud wall check. Its globally expressed path is only an interface to the already existing local Pure Pursuit controller, using IMU+vehicle velocity integrated short-term pose; **never generated from global waypoints**. GPS normal runs continue to use original static planner and latch.
- `src/main_integrated_v1.py`: GPS-shadow mode relies on camera lane-center and local LiDAR, not global-WP navigation/traffic/merge/follow gates. Requires camera lane freshness and static LiDAR result <=1.6s old; stale or STOP_REQUIRED => full brake; confirmed lateral detour temporarily overrides lane steering at <=5km/h. Pedestrian YOLO+MiDaS and LiDAR dynamic track STOP remain active.
- `src/integration/gps_shadow_io.py`: eliminate fixed 12s / 35m blackout cutoff **ONLY when `allow_sensor_guided_extended=true`**, while the main enforces live perception. Invalid IMU/vehicle status/control-loop gap still stop. DR global pose is for short-term local maneuver tracking, not navigating a global waypoint route.
- `config/integration_missions.json`: new `gps_shadow_ego_planner` candidate geometry and scan age limits. All thresholds are preliminary and MUST be checked against MORAI sensor mounting and tunnel dimensions.

## Important limitations
1. **No simulated drive verification yet**. The synthetic tests are in GitHub, but cannot prove tunnel clearance.
2. Camera HSV/Canny/BEV detector may report high confidence on tunnel-edge lines; ensure visible lane fit actually follows the lane.
3. LiDAR blind spots/occlusions can prevent a safe passage or cause false clearance; only frontal sector and two side-return counts are checked here, NOT formal visibility-ray free-space proof.
4. LiDAR scan may be only ~0.5–1Hz on limited hardware, while vehicle moves continuously; V35 caps detour at 5km/h and result age at 1.6s. If results are stale, STOP.
5. Original static planner is preserved for GPS-active sections. GPS shadow uses separate new ego-local planner and original LiDAR detection/tracking. Dynamic LiDAR tracks near vehicle stop rather than overtake.
6. If tunnel too narrow for the preset 1.6m sideways detour, the correct result is STOP. Do not disable safety gates to force passage.
7. In exceptionally long blackouts, even the relative pose used for a **locked 15m detour** can drift. No camera/LiDAR SLAM or absolute localization was added.

## How to run
```bash
cd ~/molit-autonomous-driving
git fetch origin
git switch integration-v35-shadow-ego-local-avoid
git pull --ff-only origin integration-v35-shadow-ego-local-avoid
source .venv/bin/activate

python3 -m unittest discover -s tests -p 'test_shadow_ego_v35.py' -v
python3 -m unittest discover -s tests -p 'test_gps_shadow_v32.py' -v
python3 -m unittest discover -s tests -p 'test_shadow_wall_v34.py' -v

# Low-speed supervised MORAI real drive, with full LiDAR log
bash scripts/run_integrated_v25.sh --drive --max-speed 5 --detailed-logs
```
No DRY-RUN required. Stop previous control sender first and supervise collision risk.

## Expected logs
- Shadow lane unobstructed: `EGO_LIDAR_CLEAR`, `GPS MODE=SHADOW`, lane valid, <=12km/h if --max-speed allows.
- Shadow vehicle-frame detour selected: `EGO_LIDAR_NEW_LEFT/RIGHT`, then `EGO_LIDAR_LOCKED_LEFT/RIGHT`, final target <=5km/h.
- Missing lateral observation, blocked both sides: `EGO_LIDAR_STOP_SIDE_WALL_NOT_OBSERVED` / `EGO_LIDAR_STOP_NO_FREE_SIDE`, full brake.
- No fresh lane, LiDAR, IMU or vehicle status: main full brake.
- No unconditional 12-second countdown stop anymore with live camera+LiDAR.

## Status
Committed to V35 branch. Added offline regression tests but they have NOT been run against MORAI or in the user's Ubuntu environment.
