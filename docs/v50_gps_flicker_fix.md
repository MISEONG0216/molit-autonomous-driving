# V50 — fix GPS fix-flicker resetting first-LEFT state

Base: V49. Uploaded integrated CSV `integrated_v1_20261104_184208.csv` demonstrates:
- WP3822: FIRST_LEFT accepted and started (39 path points).
- WP3823–3840: LEFT locked/actively followed.
- WP3839 (29.22s): GPS briefly declared valid.
- WP3848 (31.57s): GPS shadow returned after ~2.3s.
- WP3851 (33.61s): LEFT tail was released.
- WP3860 (35.33s): STOP_REQUIRED / NO_VERIFIED_TEMPLATE_PATH, no second RIGHT path.

Root cause in V49 source: on **any** `gps_shadow=True -> False` transition, `template_completed_count = 0` and `template_active_id = None` executed, while the GPS branch also deleted `shadow_locked_path` immediately. A brief tunnel GPS fix therefore erases progress and can cause first-LEFT to re-trigger rather than continuing with second stage. This behavior is separate from Python installation, UDP receipt and optimizer settings.

V50 changes ONLY the GPS-transition mission-state logic:
- Preserve completed template count, active template identity and shadow locked path across a short normal-GPS period.
- Only reset all of those after **8 continuous seconds** of normal GPS (or on process restart).
- The existing integrated main still uses GPS tracking while GPS is declared valid, and LiDAR shadow mode only overrides when re-entering SHADOW and passing fresh path/wall checks; do not bypass any safety stop.
- FIRST_LEFT geometry and detector thresholds are unchanged. V49's second 17.5m pattern and staged return are unchanged.

## Run Windows CMD
```cmd
git fetch origin
git switch --track origin/competition-v50-shadow-stage-flicker-fix-20261104
.venv38\Scripts\python.exe -m unittest discover -s tests -p "test_shadow_gps_flicker_v50.py" -v
.venv38\Scripts\python.exe -m unittest discover -s tests -p "test_second_survey_v49.py" -v
.venv38\Scripts\python.exe -m unittest discover -s tests -p "test_shadow_predefined_v48.py" -v
run_integrated_windows.bat
```
These regression tests have been committed, not executed on the user's Windows installation. Inspect failures before driving. Use supervised MORAI only after passing, and save main CSV and worker LiDAR logs.

Expected diagnostics:
```text
[V50 GPS FLICKER] shadow resumed after 2.3s; template_stage=0 active=FIRST_LEFT
[TEMPLATE COMPLETED] FIRST_LEFT completed=1
[SHADOW V49] ... reason=WAITING_SECOND_SURVEY_MATCH
```
V49 log tag remains, because the second template planning itself is unchanged.

**Known limitation:** The main controller may temporarily track GPS global path during this short recovered fix and may deviate from the latched shadow path; on return the original alignment and LiDAR wall gates may reject the route. This patch fixes an identified state-reset defect, but cannot guarantee full first-LEFT motion. If log reports `PATH_NOT_ALIGNED` or wall clearance, stop and examine pose error rather than increasing thresholds.
