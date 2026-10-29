# v20 선택형 차선 보정 패치 (2026-10-29)

**원본 v20 실행 방식은 그대로 유지**됩니다. 기존 `run_v20_windows.ps1`, GPS/IMU,
카메라, LiDAR 및 mission 기능은 그대로입니다. `config/lane_guard.json`에
`"enabled": false`인 동안 보정은 작동하지 않으며 주행 경로는 원본과 같습니다.

## 영상 관찰과 한계

영상 00:58~01:19에서 차량이 왼쪽 노란 차선에 매우 가까워지거나 걸치는 장면을 확인했습니다.
기존 코드의 `main_integrated_v1.py`는 `global_path`를 순수 추종하고 차선 픽셀을
직접 인식하지 않습니다. **경로가 차선 위인 경우 PP gain만 변경해도 해결되지 않습니다.**
실제 차선 경계는 이 패치가 알 수 없으며, 녹화 구간의 정확한 waypoint_index는
CSV가 있어야 확정 가능합니다. 진입로/교차로에서 일괄 우측 이동은 위험합니다.

## 보정이 적용되는 방식

1. 지정 WP 구간에만 원본 전역 경로의 **오른쪽**으로 최대 `right_offset_m` 이동한
   사본 경로를 생성합니다. 시작/종료 15m 구간에서 점진적으로 0->설정값->0으로 이동합니다.
2. 지정 구간 65m 앞에서 목표 속도를 최대 20km/h로 제한하고, 보정 구간에선
   이탈 거리가 커지면 10km/h로 제한하거나(0.7m 초과) full brake(1.5m 이상)합니다.
3. 카메라·LiDAR·GPS의 송수신 포트와 기존 신호등/장애물 회피 기능은 유지합니다.
4. 웨이포인트 번호는 원본 경로 기준이며 원본 경로 파일은 수정하지 않습니다.

## 적용 방법

1. 현재 사용 중인 프로젝트 폴더 **전체를 먼저 백업**하세요.
2. ZIP 안의 `src/main_integrated_v1.py`, `src/integration/lane_guard.py`,
   `config/lane_guard.json`, `tests/test_lane_guard_optional.py`를
   해당 프로젝트의 같은 상대경로에 복사합니다. 기존 파일은 백업하세요.
3. `config/lane_guard.json`에서 `enabled`를 `true`로 바꾸고 `zones`를 예를 들어
   `[ {"start_wp":900, "end_wp":1100, "right_offset_m":0.35} ]`로 채웁니다.
   **900~1100은 설명을 위한 가상 예시입니다. 절대 실제 경로에 그대로 넣지 마세요!**
   `start_wp`, `end_wp`는 실주행 CSV를 보고 특정해야 합니다.
4. 먼저 DRY-RUN을 실행하고 `LANE GUARD` 표기와 `STOP TARGET`/`STATIC` 상태를 확인하세요.
5. 안전한 MORAI 시뮬레이션에서 10km/h 이내로 실험하고 전후 영상/CSV를 비교하세요.
   보정과 실제 차선이 맞지 않으면 `enabled: false`로 즉시 되돌립니다.

## CSV 확인

`C:\MORAI\molit-autonomous-driving-integration-v20-static-avoid-25kph\logs\integrated_v1_날짜_시간.csv`

`elapsed_sec`, `waypoint_index`, `path_dist_m`, `comp_speed_kmh`, `steering_rad`을
확인하세요. 패치 적용 후 CSV에는 lane_guard 필드가 6개 추가됩니다.
녹화 시작과 CSV 기록 시작 시점이 다르면 초 단위로 직접 매칭할 수 없으므로
동일한 실제 사건(예: 신호등 정지, 곡선 진입)을 기준으로 시간을 맞춰야 합니다.

## 한계

이 보정은 **원본 GPS 경로를 기준으로 한 지정 구간 위치 보정**이지 차선 감지
또는 차선이탈 방지 보증이 아닙니다. 도로 폭, 주변 차량, 표지판, 실제 차량
폭은 판단하지 않습니다. 좌우 차선이 명확하지 않은 곳에는 적용하지 마세요.
LiDAR 회피 임무 구간과 보정 구간이 겹칠 경우 LiDAR가 조향을 덮어쓸 수 있습니다.

## CSV에서 문제 구간 웨이포인트 찾기

동일한 시점에 녹화와 CSV 기록을 시작했다면 아래처럼 실행하세요.

```powershell
.\.venv\Scripts\python.exe .\tools\find_lane_wp_from_csv.py --start 58 --end 79
```

`waypoint range=...`를 확인해 주세요. 녹화 시작과 CSV 시작이 다른 경우
`--offset` 값을 조정해야 하며, 자동 시간 동기화 기능은 없습니다.

**이 결과만으로 무조건 보정 구간을 확정하지 마세요.** 영상에서 실제로
차선을 넘는 시점과 CSV의 속도·경로 오차·위치를 함께 대조해야 합니다.
