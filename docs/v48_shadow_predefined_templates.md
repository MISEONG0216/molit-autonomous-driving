# V48 — camera lane tracking with two predefined obstacle detour templates

**Base:** V47 LiDAR survey (which itself is based on V44). The original main controller and all traffic/camera/GPS shadow logic are **unchanged**. This is an experimental new shadow-only planner.

## Inputs and coordinate frames
- LiDAR VLP16 receiver outputs `[right,forward,up,intensity]`; existing static worker converts to vehicle local `[forward,left,up]`.
- Measurements recorded at *different* stopped vehicle poses:
  - First survey CSV (2): obstruction group forward **11.41–14.00 m**, lateral **−3.15 to +0.88 m**. Zero hits were found in an offline swept-corridor check with left-shift 2.4 or 2.8 m, using the survey scan and observed side walls. **Not a driving validation.**
  - Second survey CSV (3): obstruction group forward **~17.4–17.7 m**, lateral **+0.4 to +3.4 m**. At the surveyed stop its nearest point is **outside** the moving trigger interval of 7–12 m; it must become closer before the second path can be evaluated. Never combine both survey coordinates as if they share a global origin.
- X forward, Y left. Offline preview uses reversed horizontal plotting axis so vehicle-left appears visually left.

## New config
`config/tunnel_predefined_templates.json` defines two sequential fixed shape choices:
1. `FIRST_LEFT`, 2.4 m / 2.8 m left offsets.
2. `SECOND_RIGHT`, 2.4 m / 2.8 m right offsets.

These are NOT guaranteed lane widths or authorized lane changes. They are prototype target offsets to be individually verified.

## State machine
- Before a detected obstacle in the CURRENT vehicle frame, the worker sends `CLEAR`. Main therefore continues existing GPS-shadow camera lane-centering control.
- New obstacle trigger is **7–12 m forward and |lateral| ≤3.5 m**, with at least five non-ground LiDAR returns. Obstacles too close trigger **STOP_REQUIRED**.
- A template's lateral shift starts ~1.5m forward and finishes ~2m before the nearest observed obstacle. It stays offset until after its observed far extent, then returns smoothly 5m later. A small set of predefined hold delays (0, 1.5, 3m) is evaluated.
- Before accepting a template, require currently observed side boundaries on BOTH sides, the entire swept path to be free of all filtered observed returns at 1.35m half-corridor clearance, and adequate BOTH-side visibility near the return endpoint. Do not choose an unobserved/blocked path.
- Accepted path is frozen in the existing LiDAR worker in the current *estimated* pose frame and published to the main's **existing** local Pure Pursuit override. Main still sends vehicle commands. Existing wall-sweep revalidation, dynamic-track stop and stale-sensor stop are preserved.
- Only when the current locked path releases after fresh forward LiDAR clearance does the worker increment from template 1 to template 2. After the second, continue camera lane tracking until GPS returns.
- Reentering GPS-normal mode resets the template state.
- **Never** drive blindly on a timer or a stale point cloud. Be aware there is no independently verified localization or lane-boundary legality during the GPS shadow.

## Test (Windows CMD; Python 3.8.10)

```cmd
git fetch origin
git switch --track origin/competition-v48-shadow-predefined-templates-20261104
.venv38\Scripts\python.exe -m unittest discover -s tests -p "test_shadow_predefined_v48.py" -v
.venv38\Scripts\python.exe -m unittest discover -s tests -p "test_shadow_alignment_v43.py" -v
```

**Offline preview** (insert your actual survey directory; do not type the ellipsis literally):

```cmd
.venv38\Scripts\python.exe src\lidar\preview_tunnel_templates.py --csv "logs\tunnel_surveys\survey_YYYYMMDD_HHMMSS\points_vehicle_frame.csv" --stage first
```

For survey 3 use `--stage second`. Expect `WAITING_FOR_TEMPLATE_OBSTACLE` there if the obstacle remains ~17.4m away; this is **not** automatically a failure. The LiDAR trigger window is 7–12m.

Use `run_integrated_windows.bat` for DRY-RUN sensor/decision logging. Actual motion requires operator-supervised, low-speed `run_integrated_windows.bat --drive` **only after** tests pass, visual track review, side-wall/legal lane validation and fresh LiDAR scan diagnostics. Keep previous working branches untouched.

## Important known limitation
Current camera system follows visible lane markings but does not independently prove permission to cross them. No direct GPS pose exists in the shadow, so world-frame XY inferred from IMU and speed can drift; operator must verify whether that error stays small enough for a 5km/h local maneuver.

No full-run MORAI validation is claimed. Synthesis and test code are committed, not executed on the user's machine.
