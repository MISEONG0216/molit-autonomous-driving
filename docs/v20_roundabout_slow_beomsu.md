# V20 Beomsu roundabout slow-speed experiment — keep verified baseline unchanged

- **Parent:** `v20-add-beomsu` (the preserved video-successful V20 baseline).
- **Experimental branch:** `v20-roundabout-slow-beomsu`.
- **Goal:** minimize contact with yellow markings in the roundabout, aiming for zero. 8 km/h is a *target cap*; this does not itself prove zero tire contact.
- **Only speed profile change:** slow from **12 km/h to 8 km/h**, and hold this cap over **WP1770..1902** instead of WP1835..1892. Continue to use the same distance-preview brake formula before WP1770 and smooth release after WP1902; actual speed may lag the target.
- **Roundabout exit steering adjustment unchanged:** same +0.25 m RIGHT (via `--beomsu-roundabout-right-m 0.25`) at WP1859..1902.
- **All other settings unchanged from previous successful test:** 30 km/h general cap, WP1480..1715 +0.25m, WP3640..3820 +0.12m, WP4050..4300 +0.25m, V20 traffic/static LiDAR worker, GPS/IMU, official global route, stops and vehicle control.
- **Known limitation:** local perception of paint edges/wheel overlap not built into V20. Reducing speed can decrease corner-cutting/overshoot but cannot guarantee no line overlap; if the underlying path geometrically intersects a marking, speed alone is insufficient. **Observe roundabout WP1770..1902 in MORAI and STOP upon yellow-line or road-edge approach.**
- **No V38/V39 tunnel modules were merged.** Original V20 branch is untouched. The V20 tunnel-GPS limitation remains.

## Windows three-CMD test

Clone into an independent folder rather than overwriting the known-good V20:

```bat
cd /d "%USERPROFILE%"
git clone --branch v20-roundabout-slow-beomsu --single-branch https://github.com/MISEONG0216/molit-autonomous-driving.git molit-roundabout-slow
cd /d "%USERPROFILE%\molit-roundabout-slow"
powershell -NoProfile -ExecutionPolicy Bypass -File setup_windows.ps1
.\.venv\Scripts\python.exe -m unittest discover -s tests -p "test_beomsu_roundabout.py" -v
```

1. **CMD 1:** `cd /d "%USERPROFILE%\molit-roundabout-slow"` then `.\.venv\Scripts\python.exe src\traffic_light_detector.py`.
2. **CMD 2:** same folder then `.\.venv\Scripts\python.exe src\lidar\static_avoidance_module.py`.
3. **CMD 3 dry-run:** `scripts\run_beomsu_yellow_test.bat`, then Ctrl+C and **supervised simulator-only** `scripts\run_beomsu_yellow_test.bat drive`.

Only one controller may send EgoCtrlCmd. MORAI must be AV ExternalCtrl, all sensors live, and controller at 127.0.0.1:9093 (single-PC test). STOP if misaligned, if LiDAR/vision/pose missing, or if steering approaches the opposite road edge. Worker and integrated CSV should be uploaded with the run video to verify `comp_speed_kmh`, `beomsu_roundabout_speed_cap_kmh`, signed CTE, steering, and mission state.

**Status:** Source updated on GitHub; unit tests and MORAI road clearance are *not independently executed/verified*.
