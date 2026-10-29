# IMG_5049: V20 yellow-line supervised experiment (NOT proven safe)

## Evidence / scope
The provided `IMG_5049.mp4` is ~93.63 seconds at 2x playback, representing ~187.3 seconds of original playback. The footage appears to begin during an already-running drive, not from the original WP0; it is a recording of a screen recording. Exact wheel contact, camera-lane boundaries and continuous penalty duration **cannot** be measured reliably from these pixels.

- 16-34s displayed (32-68s original): median/guardrail road; close to yellow boundary in some frames, but no reliably established continuous contact. Previously successful +0.25m right shift WP1480-1715 **preserved**.
- 36-43s (72-86s original): roundabout left boundary and yellow triangle, especially 40-43s. Cmd overlay showed WP1855 near 40s and WP1872 near 42s. **Experimental** exit shift increases +0.20 -> **+0.25m RIGHT** while keeping 12km/h speed profile.
- 69-74s (138-148s original): yellow gore at highway exit; vehicle comes close; no indisputable multi-second wheel overlap. Observe and don't alter that segment aggressively.
- 75-78s (150-156s original): yellow divergence leading into tunnel; video frame ~76s shows WP3795. **Small provisional** right shift +0.12m with smooth ramps WP3640-3820. This needs right-lane/tunnel-wall clearance validation; do NOT blindly increase.
- 83-88s (166-176s original): post-tunnel curve, yellow center line visibly under/near vehicle footprint repeatedly. ~84s CMD shows WP4120 and signed CTE ~+1.13m relative to official path, ~86s WP4172. **Highest-priority experimental** +0.25m RIGHT WP4050-4300, smooth ramps. CTE is not boundary clearance, and entire time under paint is not necessarily tire contact.
- 89-93s: terminal split/gore. Re-check video; avoid assuming every yellow-hatched triangle is a wheel crossing.

## Execution

Switch to this new branch `v20-yellow-line-test-beomsu` cloned into a **separate folder** from the stable V20 checkout. Windows:
```bat
cd /d "%USERPROFILE%"
git clone --branch v20-yellow-line-test-beomsu --single-branch https://github.com/MISEONG0216/molit-autonomous-driving.git molit-yellow-beomsu
cd /d "%USERPROFILE%\molit-yellow-beomsu"
powershell -NoProfile -ExecutionPolicy Bypass -File setup_windows.ps1
.\.venv\Scripts\python.exe -m unittest discover -s tests -p "test_beomsu_yellow_profile.py" -v
```

Three separate CMD windows, keep original worker terminals running:
1. `cd /d "%USERPROFILE%\molit-yellow-beomsu" && .\.venv\Scripts\python.exe src\traffic_light_detector.py`
2. `cd /d "%USERPROFILE%\molit-yellow-beomsu" && .\.venv\Scripts\python.exe src\lidar\static_avoidance_module.py`
3. First `scripts\run_beomsu_yellow_test.bat` (dry-run), then Ctrl+C and `scripts\run_beomsu_yellow_test.bat drive` (simulator supervised drive). **Only ONE EgoCtrlCmd sender.**

The batch launcher is a wrapper around unchanged `src/main_integrated_v1.py` flags:
```bat
--traffic --static --static-control --max-speed 30 ^
--beomsu-first-right-m 0.25 ^
--beomsu-roundabout-right-m 0.25 ^
--beomsu-roundabout-kmh 12 ^
--beomsu-extra-zone "3640:3820:0.12" ^
--beomsu-extra-zone "4050:4300:0.25"
```

All experimental extra zones use `beomsu_lane_guard.py` smooth 35-WP ramps and already present 20km/h speed cap while shifted. The official global path, traffic stops, static cone planner, LiDAR ports, GPS and the original V20 branch are unchanged. GPS/IMU/vehicle status should be fresh and MORAI AV ExternalCtrl enabled on the same PC. **Do not test in live traffic or on a real car.** If right-edge/white-hatch clearance decreases, STOP, back out to previously proven V20.

## Verification needed
Please upload the **matching** `logs/integrated_v1_YYYYMMDD_HHMMSS.csv` from this run and 2x video, preferably with beginning and ending synchronized. Look for `nearest_index`, `signed_cte_m`, `heading_error_deg`, `beomsu_offset_m`, `comp_speed_kmh`, `steering_rad`. Analyze consecutive frames near WP1855-1902, WP3640-3820, WP4050-4300. The 3-second penalty requires ground-truth tire-to-yellow segmentation or collision data, not just global path CTE or a sparse video estimate.

Status: GitHub changes created; unit tests and MORAI run must be executed by the user. No claim of zero yellow-line contact.
