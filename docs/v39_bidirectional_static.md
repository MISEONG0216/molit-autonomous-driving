# V39 two-way static obstacle avoidance

Source: `competition-v38-lan-192-168-0-1-20261030`

The ordinary GPS-valid static mission (WP590–760) already had both positive (left) and negative (right) offsets. This revision makes checking both candidates explicit and performs a continuous line-segment-to-obstacle collision test against **all** eligible LiDAR tracks, rather than considering only the selected primary obstacle. It also adds ±2.0m alternatives and regression tests.

- Coordinates: route tangent left normal is +d (LEFT); -d is RIGHT. Not based on screen left/right.
- Candidate offsets: -2.2, -2.0, -1.8, +1.8, +2.0, +2.2 m.
- Both candidates blocked / outside configured road boundary => STOP_REQUIRED. No fallback to a potentially unsafe road shoulder.
- Locked maneuver stays on its initial side until release. The path latch still stops for a new object blocking the remaining route; it does **not** swap sides while moving.
- The 7.0m drivable width and width-based obstacle footprint are existing trial assumptions, **not verified road markings**. A side path can be geometrically clear by these assumptions but cross a painted line. Validate on MORAI at low speed with camera/ground truth before raising speed.
- No changes to GPS-shadow path generation, lane/GPS recovery, MORAI LAN IP/ports or integrated controller.

Test from the repository root with Python 3.8.10 and NumPy:

```bash
python -m unittest discover -s tests -p 'test_static_bidirectional_v39.py' -v
python -m unittest discover -s tests -p 'test_static_speed_25_v20.py' -v
```

Run via the existing `run_windows.ps1` or `run.sh` command after switching to this branch; don't run both vehicle controllers simultaneously. If STOP_REQUIRED persists, enable `STATIC_CSV_LOG=1`, inspect the status and candidate diagnostics, and review LiDAR obstacle width/height and the drivable boundary before modifying clearances.
