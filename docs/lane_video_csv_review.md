# V20 lane video / CSV measurement branch

Branch: `integration-v20-lane-tuning`
Base: `integration-v20-static-avoid-25kph`

## What changed

This is a **read-only measurement experiment**, not a new steering controller.
The V7 curve speed profile, target selection, lookahead, steering gain, traffic
and static obstacle logic, and transmitted EgoCtrlCmd remain unchanged.
The only controller change is extra CSV columns and terminal diagnostics.

Files:
- `src/integration/lane_diagnostics.py`: projection onto a nearby global path
  segment; signed lateral distance (+left, -right relative to path travel);
  signed heading error.
- `src/main_integrated_v1.py`: terminal video sync marker and telemetry columns.
- `scripts/review_lane_csv.py`: offline CSV inspection after video review.
- `tests/test_lane_diagnostics.py`: geometry checks without simulator.

## How to capture one comparable run (Windows CMD)

In project folder, after resolving any uncommitted local changes:

```bat
git fetch origin
git switch --track origin/integration-v20-lane-tuning
python -m unittest discover -s tests -p "test_lane_diagnostics.py" -v
```

If the branch is already local, use `git switch integration-v20-lane-tuning`
instead of `--track`.

Check `config/integration_network.json` control target IP and all sensor ports
against the actual MORAI setup. **Do not assume the checked-in target IP is your PC**.
Ensure only the main integrated process sends EgoCtrlCmd.
Use the same MORAI scenario, start pose, camera angle and recording layout for
successive comparison runs.

Launch MORAI, then start separate CMD terminals:

Terminal A (LiDAR worker; no control sender):
```bat
python src\lidar\static_avoidance_module.py
```

Terminal B (traffic-light detector, if using --traffic):
```bat
python src\traffic_light_detector.py
```

Terminal C (safe dry-run first):
```bat
python src\main_integrated_v1.py --traffic --static --max-speed 30
```

If MORAI sensors, traffic detection and static worker output are verified,
stop dry-run, then perform a supervised simulator-only drive at a conservative
speed; this sends control commands:

```bat
python src\main_integrated_v1.py --drive --traffic --static --static-control --max-speed 30
```

Do not run legacy driving scripts or multiple main controller instances in
parallel. Do not increase to 57 km/h until lower speed runs are stable.

## Video capture

Record the MORAI driving view and the main CMD terminal in **one video frame**
(e.g., OBS display capture, with both windows tiled). Use 720p / 30 fps first
to reduce simulator GPU load. Start recording **before** launching Terminal C.
Keep these terminal lines visible whenever possible:

```text
NEAREST     : 1234
VIDEO SYNC  : 2026-10-28T21:15:08.123+09:00 / elapsed=44.12s
GLOBAL CTE  : +0.423 m (+LEFT/-RIGHT) / heading_err=-3.15deg
TRACK PATH  : GLOBAL_V7 (CTE always vs global path)
```

The CSV's `elapsed_sec`, `wall_time_local` and `waypoint_index` correspond
to these printed values. A video player's 00:00 is NOT automatically the CSV
elapsed=0; read the displayed elapsed time to sync instead.
The print happens after sensor and control computations and may have a small
delay. Use a couple of reference frames if sub-second accuracy is needed.

If a lane marking is touched, note:
- video time and **visible CSV elapsed time**
- start/end when wheel/tire contacts line and contact duration (video evidence)
- left/right contact, WP shown on terminal
- global or local avoidance track mode
- whether video was paused/recording dropped frames.

## Analyze CSV

Controller writes `logs/integrated_v1_YYYYMMDD_HHMMSS.csv`.
The new fields are **appended** to the existing V20 fields to retain compatibility:

```text
wall_time_local,wall_time_epoch_s,signed_cte_m,abs_cte_m,
path_heading_deg,heading_error_deg,nearest_segment_wp,curve_radius_m,
v7_effective_lfd_m,v7_steering_gain,v7_target_wp,v7_switch_bias,
tracking_path_source
```

A sample CSV review (replace filename):

```bat
python scripts\review_lane_csv.py --csv logs\integrated_v1_20261028_211500.csv
python scripts\review_lane_csv.py --csv logs\integrated_v1_20261028_211500.csv --start 44.0 --end 48.0
```

Default 0.7 m **global path error** is only a candidate REVIEW threshold:
it is not the lane boundary or a 3-second penalty trigger. To change it:

```bat
python scripts\review_lane_csv.py --csv logs\integrated_v1_20261028_211500.csv --threshold 0.5
```

`signed_cte_m` always measures deviation from the supplied global path,
including when the controller intentionally follows a LOCAL_AVOIDANCE path.
Do not call those samples a lane violation without video evidence.
GPS/IMU errors and path not centered in lane can also create discrepancies.

## Comparison workflow

1. Preserve video + CSV together for baseline run, same seed/scenario.
2. Locate actual tire-line contact spans in the video.
3. Inspect the matching CSV elapsed times / waypoint ranges.
4. Check speed, radius, signed CTE, heading error, LFD, gain, and track mode.
5. Make **one** tuning change in a further branch, then rerun same scenario.
6. Compare time touching lane lines, affected WP ranges, maximum deviation,
   completion, and whether other mission behavior regressed.

This branch does not detect lane lines, record camera video directly, or assert
2026 rule compliance. The 3-second penalty condition should be checked against
the most recent official event rules.
