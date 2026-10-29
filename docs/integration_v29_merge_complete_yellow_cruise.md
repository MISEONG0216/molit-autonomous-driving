# V29 — Fix WP2538 rear-camera merge deadlock and yellow-zone speed cap

Based on V28. Exactly these components change:
- `src/integration/merge_side_io.py` (camera modes)
- `src/merge_side_camera_worker.py` (pause rear inference after specified WP)
- `src/integration/mission_manager.py` (remove the *merge-only* 20km/h speed cap for safe/yellow highway observations)
- `config/integration_missions.json` (config)
- corresponding tests.

All existing signals, pedestrian, LiDAR, waypoint steering, highway-following, main UDP control, CSV schema and launcher remain unchanged.

## Evidence from user's MORAI run on 2026-10-29
- In `integrated_v1_20261029_223537.csv`, WP2538 was occupied for ~51.7 sec; speed fell to zero, brake=1, `MERGE_STOP_HOLD`, risk ~0.80.
- Session shows `left=0.00 left_rear=0.80 => MERGE_REJECT` repeated at WP2538.
- Camera screenshot at WP2538 shows a very large/close rear-left vehicle box; a single image cannot prove it is stationary or collision-free.

## Change
- First highway merge: WP2480–2537 uses both LEFT9293+LEFT_REAR1118. At **WP2538 through WP2560** the user reports the merge already completed. The V29 **post_merge** mode stops decoding/inferencing LEFT_REAR1118, excludes it from risk and freshness, and **keeps LEFT9293 active**. Existing 3 distinct MERGE_ACCEPT camera updates must still clear a previously latched stop. If LEFT is dangerously close or stale, the stop remains. This is a geometry assumption based on the reported run, not an independent physical validation.
- Second highway merge WP3050–3190 continues using BOTH cameras. No post-merge WP is guessed for the second merge.
- Roundabout WP1770–1820 still uses LEFT only.
- Yellow/clear highway merge result (`risk < 0.60` and MERGE_ACCEPT): `merge.zone_speed_kmh=20` **no longer caps** the V7 target. Red `MERGE_REJECT` or score >=0.60 still STOPs, stale camera still STOPs, and the existing 3-confirmation latch remains. Roundabout still keeps its 20km/h zone cap. Other traffic/pedestrian/LiDAR/curvature/highway-follow limits retain precedence.
- Right camera9295 remains disabled; worker never sends EgoCtrlCmd. Existing log remains single integrated CSV + session log.

## Why not auto-release on a stationary-looking rear car?
Stationary camera imagery cannot reliably distinguish a stopped vehicle behind from a vehicle alongside or slowly approaching; image-area growth is also noisy. A time-based forced departure can cause a collision. Here the rear camera is ignored **only after the user-observed WP**, while left-side conflict monitoring stays active. The user must independently verify that WP2538 is indeed past the merge conflict point in the actual scenario.

## Test and run
```bash
cd ~/molit-autonomous-driving
git fetch origin
git switch integration-v29-merge-complete-yellow-cruise
git pull --ff-only origin integration-v29-merge-complete-yellow-cruise
source .venv/bin/activate
python3 -m unittest discover -s tests -p 'test_merge_complete_v29.py' -v
python3 -m unittest discover -s tests -p 'test_merge_roundabout_v28.py' -v
python3 -m unittest discover -s tests -p 'test_merge_side_v25.py' -v
bash scripts/run_integrated_v25.sh  # DRY-RUN; no EgoCtrlCmd
# Only after observing both camera gate outputs:
bash scripts/run_integrated_v25.sh --drive --max-speed 15
```

Expected session log at WP2538:
`[MERGE ZONE] POST_MERGE WP=2538 active=('left',) rear_left=OFF`.
When left perception is fresh and score below threshold, `MERGE_ACCEPT` is expected. Main still requires 3 distinct packets after a latched stop.

**Do not jump to unsupervised 57/120km/h driving without verifying that the merge path is geometrically complete and that the forward lane is clear.**
