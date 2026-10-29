# V20 Add Beomsu — conservative lane-clearance experiment

Base: `integration-v20-lane-tuning` (unchanged V7, traffic-light, merge, LiDAR static-zone logic). Only `v20-add-beomsu` has these modifications. **Not validated in MORAI.**

## What changed
- `src/integration/beomsu_lane_guard.py`: generate a separate smooth Pure Pursuit tracking path; original official global path, waypoint mission indexes, speed profile and LiDAR reference remain unchanged.
- Known first video area: WP1480–1715 with taper WP1480–1530 and WP1655–1715. Positive offsets move right of travel, away from the left yellow median at WP1533/1578/1655. This first-zone mapping is from existing V22 source comments, **not independently measured from a CSV in this session**.
- Optional manually mapped extra ranges: `--beomsu-extra-zone START:END:SIGNED_METRES`, where negative means left; intended for tunnel approach/interior **only after the corresponding waypoint ranges and safe free space are measured**. No unverified tunnel bias is enabled.
- Offset size bounded to 0.50m, smoothed over each ramp, and target speed capped at 20km/h during nonzero offset. A 20km/h cap does **not** guarantee line or wall clearance.
- Extra CSV fields: `beomsu_offset_m`, `beomsu_tracking_dist_m`, `beomsu_zone_active`; existing lane diagnostics remain relative to official path.
- New offline test: `tests/test_beomsu_lane_guard.py`.

## Missing run CSV
No actual run `.csv` file was present in the examined V20 repository tree. **GitHub cannot recover a CSV that was only written on a team's laptop and never committed or uploaded.** The existing V20 controller creates `logs/integrated_v1_YYYYMMDD_HHMMSS.csv` locally when run.

## Low-speed test (after existing sensor/UDP settings and workers are verified)
Use Windows CMD in the project root:
```bat
git fetch origin
git switch --track origin/v20-add-beomsu
python -m unittest discover -s tests -p "test_beomsu_lane_guard.py" -v
python src\main_integrated_v1.py --traffic --static --max-speed 15 --beomsu-first-right-m 0.25
```
Above is DRY-RUN: it does not send vehicle control. After confirming proper receipt of all sensors, traffic and static workers, stopping all other EgoCtrlCmd senders and checking `config/integration_network.json` target:
```bat
python src\main_integrated_v1.py --drive --traffic --static --static-control --max-speed 15 --beomsu-first-right-m 0.25
```
The existing standalone detector/worker must be launched separately using their V20 instructions. Do not start multiple main control senders. Monitor the beginning roadway and abort/reset if wall clearance shrinks. The first 0.25m adjustment is a test hypothesis, not a guaranteed fix. Always compare to an unchanged baseline video and CSV.

## Critical limitations
1. The camera and LiDAR do **not** currently provide real-time lane-line or guardrail bounds to this new offset function. It cannot enforce the competition's 3-second yellow-line rule. Absolute wheel-to-line distance must be measured from video or calibrated sensing.
2. GPS/reference-path tracking may be offset even if measured `signed_cte_m=0`; zero CTE does not prove lane center.
3. Offsets near walls/tunnel may worsen collision. No automatic extra-zone offset is enabled until video/CSV waypoint correspondence is established.
4. Existing V20 code does not include V35 GPS-shadow/tunnel navigation. This branch intentionally preserves V20 rather than silently merging unrelated experimental changes.
5. The full integrated stack and `unittest` suite were not executed in MORAI by the assistant. Hardware/UDP settings must be verified.

## Compare and tune
Record video including terminal WP/VIDEO SYNC and save local `logs/integrated_v1_*.csv`. Compare near WP1533/1578/1655 before/after. Only then decide whether to change 0.25m to 0.15 or 0.35m. At tunnel entry record the WP where the tire approaches the yellow line and where the wall becomes too close, and only apply a safe signed offset after determining which side has room.
