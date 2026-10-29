# V7 + Traffic Light + Highway Merge Risk Integration V1

This branch starts integration without rewriting the already validated V7 controller.
`src/main_integrated_v1.py` imports the existing V7 chain and adds mission constraints before longitudinal control.
Only the integrated drive process sends `EgoCtrlCmd`.

## Control ownership

- V7 global-path tracker: base steering + waypoint target speed.
- Traffic-light detector: publishes `STOP / GO / GO_LEFT / UNKNOWN` only.
- Highway merge monitor: publishes risk only.
- Mission manager: applies traffic stop/slow and optional merge-risk speed caps.
- Integrated drive: calculates accel/brake and is the only `EgoCtrlCmd` sender.

## Traffic regression values

Validated stop indices:

`414, 620, 879, 1104, 1224`

Traffic behavior is preserved from the validated branch:
- mission start: 25 waypoint indices before the stop
- slow-down start: 20 waypoint indices before the stop
- approach speed: 3 km/h
- RED release: 8 control-loop GO/GO_LEFT confirmations while the result is fresh

The camera stop-line y-ratio is still produced for diagnostics, but V1 stopping uses the validated GPS waypoint indices.

## Merge strategy

The merge module does not generate a new path. V7 continues following the global path, including the natural lane change in the global path.
The merge module only publishes risk. When merge control is enabled, the MissionManager may lower the target speed in configured elevated/highway waypoint zones.

`config/integration_missions.json` intentionally starts with an empty `zones` list because the exact elevated-road waypoint range has not yet been measured.

The current risk speed caps (35 / 20 / 10 km/h) are integration placeholders, not validated tuning values. Keep merge control OFF until the waypoint zone and caps are tested.

## Camera setup

The merge monitor preserves the teammate implementation defaults:
- side camera: UDP 9293
- rear camera: UDP 1118

They are configurable through environment variables, so the MORAI camera layout can be changed without editing the algorithm:
- `MERGE_SIDE_PORT`
- `MERGE_REAR_PORT`
- `MERGE_CAMERA_BIND_IP`

Traffic camera:
- `TRAFFIC_CAMERA_BIND_IP` default `0.0.0.0`
- `TRAFFIC_CAMERA_PORT` default `9291`

## Run order

Activate the project virtual environment first.

### 1. V7 regression only

Dry-run:

```bash
python3 src/main_integrated_v1.py
```

Drive:

```bash
python3 src/main_integrated_v1.py --drive
```

This must reproduce the current V7 behavior before enabling missions.

### 2. Traffic detector

Terminal 1:

```bash
python3 src/traffic_light_detector.py
```

Terminal 2 dry-run:

```bash
python3 src/main_integrated_v1.py --traffic
```

Then real drive:

```bash
python3 src/main_integrated_v1.py --drive --traffic
```

### 3. Merge observation

Terminal 1:

```bash
python3 src/highway_merge_monitor_integration.py
```

Terminal 2:

```bash
python3 src/main_integrated_v1.py --merge
```

This logs merge risk but does not alter driving.

### 4. Merge speed control

Do not use this until `config/integration_missions.json` has the actual elevated-road waypoint zone and tuned speed caps.

```bash
python3 src/main_integrated_v1.py --drive --merge --merge-control
```

For a short isolated simulator test only:

```bash
python3 src/main_integrated_v1.py --merge --merge-control --merge-all
```

## Network

Changeable IP/port values are in:

`config/integration_network.json`

When Wi-Fi changes, update the MORAI control target IP there instead of modifying V7 source files.

Current config uses the MORAI PC IP from the successful 2026-10-06 test:
`192.168.203.252:9093`.

## Important V1 limitation

The merge code still loads MiDaS with `torch.hub.load`, as in the teammate implementation.
Before competition deployment, make MiDaS fully local/offline and verify the model cache.
