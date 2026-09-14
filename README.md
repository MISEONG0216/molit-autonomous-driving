# MOLIT Autonomous Driving

2026 대학생 AI/SW 모빌리티 경진대회 AI융합자율주행 부문용 MORAI 기반 자율주행 프로젝트입니다.

현재 `main` 기준 주행 코드는 GPS + IMU + Competition Vehicle Status + 공식 전역경로를 사용하여 Pure Pursuit 주행을 수행합니다. 차량 제어는 대회 규정에 맞게 `ctrl_mode=2`, `cmd_type=1 (accel, brake, steer)`을 사용합니다.

## 현재 기준 주행 설정

- 일반구간 테스트 최고속도: **57 km/h**
- 위치: GPS UDP
- 자세: IMU UDP
- 실제 속도: Competition Vehicle Status `signed_vel`
- 경로: `path/2026_molit_comp_global_path.txt`
- 횡제어: Pure Pursuit
- Look Ahead Distance: 속도 기반 Dynamic LFD + 곡률 기반 상한
- 감속 프로파일 Preview: `2.0 m/s^2`
- 최대 Brake command: `0.10`
- Ego Ctrl Cmd: `ctrl_mode=2`, `cmd_type=1`

현재 57 km/h 세팅은 정상 GPS 구간에서 WP 약 1437까지 테스트했습니다. 고속 직선에서 큰 좌우 hunting은 나타나지 않았고, 이후 세밀한 차선 추종 튜닝을 진행할 예정입니다.

**주의: GPS 음영구간(WP 약 4049 부근)은 아직 별도 해결 대상입니다. 현재 코드를 완성된 본선 코드로 간주하면 안 됩니다.**

---

# 1. 기준 환경

현재 개발/검증 기준 환경은 다음과 같습니다.

- Ubuntu **20.04 LTS**
- Python **3.8.x**
- MORAI 대회용 Simulator
- MORAI Network Module `24.R2.0`
- ROS는 현재 메인 주행 코드 실행에 필요하지 않음

`setup.sh`를 실행하면 MORAI 공식 Network Module `24.R2.0`을 `third_party/MORAI-NetworkModule`에 자동으로 내려받습니다.

---

# 2. 처음 사용하는 팀원: 저장소 받기

GitHub 저장소 접근 권한이 있어야 합니다.

```bash
git clone https://github.com/MISEONG0216/molit-autonomous-driving.git
cd molit-autonomous-driving
```

이미 저장소가 있는 경우에는 다음처럼 최신 `main`을 받습니다.

```bash
cd ~/molit-autonomous-driving
git switch main
git pull
```

---

# 3. 필요한 Ubuntu 패키지 설치

먼저 다음 패키지를 설치합니다.

```bash
sudo apt update
sudo apt install -y git python3 python3-pip python3-venv
```

설치 확인:

```bash
python3 --version
git --version
```

Python 3.8.x 환경을 권장합니다.

---

# 4. 프로젝트 초기 설정

저장소 폴더에서 실행합니다.

```bash
cd ~/molit-autonomous-driving
chmod +x setup.sh check.sh run.sh
./setup.sh
```

`setup.sh`가 수행하는 작업:

1. Python 확인
2. `.venv` 가상환경 생성
3. `requirements.txt` 라이브러리 설치
4. MORAI 공식 Network Module `24.R2.0` 다운로드
5. 실행 권한 설정

설치 후 확인:

```bash
./check.sh
```

가상환경을 직접 활성화할 때는:

```bash
source .venv/bin/activate
```

---

# 5. MORAI에서 사용할 시나리오

대회용 샘플 시나리오를 사용합니다.

개발 PC에서는 Competition Vehicle Status 포트를 저장한 별도 시나리오 이름을 다음과 같이 사용하고 있습니다.

```text
2026_molit_comp_sample_scene_ubuntu
```

팀원 PC에 이 이름의 저장 시나리오가 없다면 공식 샘플 시나리오를 불러온 뒤 아래 UDP 설정을 직접 입력하면 됩니다.

MORAI Simulator 실행 파일 자체는 이 저장소에 포함되어 있지 않습니다.

---

# 6. MORAI UDP 설정

현재 코드는 MORAI와 Python을 **같은 Ubuntu PC에서 실행**하는 것을 기준으로 하며 모든 IP는 `127.0.0.1`입니다.

## GPS

MORAI GPS Sensor 설정:

```text
Protocol          UDP
Host Sensor IP    127.0.0.1
Host Sensor Port  9200
Destination IP    127.0.0.1
Destination Port  9201
```

현재 Python 수신 포트:

```text
127.0.0.1:9201
```

## IMU

MORAI IMU Sensor 설정:

```text
Host Sensor IP    127.0.0.1
Host Sensor Port  9090
Destination IP    127.0.0.1
Destination Port  1111
```

현재 Python 수신 포트:

```text
127.0.0.1:1111
```

## Competition Vehicle Status

MORAI Ego-0 Network Settings에서 Competition Vehicle Status를 사용합니다.

Ubuntu에서 현재 검증된 포트:

```text
Host Port         1908
Destination Port  1909
```

Python은 다음 포트에서 수신합니다.

```text
127.0.0.1:1909
```

현재 실제 속도 제어에는 Competition Vehicle Status의 `signed_vel`을 사용합니다.

## Ego Ctrl Cmd

MORAI Ego-0 > Cmd Control:

```text
Protocol          UDP
Host IP           127.0.0.1
Destination IP    127.0.0.1
Host Port         9093
Destination Port  9094
```

Python은 MORAI의 다음 포트로 제어 명령을 전송합니다.

```text
127.0.0.1:9093
```

제어 방식:

```text
ctrl_mode = 2
cmd_type  = 1
```

즉 `accel`, `brake`, `steer`, `gear` 방식으로 제어합니다.

**Simulator Network의 MultiEgo용 7604/7504 연결은 현재 주행 코드에 필요하지 않습니다.**

---

# 7. MORAI 차량 상태 확인

주행 전에 Ego 차량이 외부 제어를 받을 수 있도록 설정합니다.

```text
Ego Controller = AV / ExternalCtrl
```

그리고 다음을 확인합니다.

1. 대회 시나리오가 정상 로드됨
2. GPS Destination Port = `9201`
3. IMU Destination Port = `1111`
4. Competition Vehicle Status Host/Destination = `1908 / 1909`
5. Ego Ctrl Cmd Host Port = `9093`
6. Ego가 `AV / ExternalCtrl`

---

# 8. 센서 통신부터 확인

바로 차량을 움직이지 말고 먼저 GPS와 IMU를 확인합니다.

```bash
cd ~/molit-autonomous-driving
source .venv/bin/activate

./run.sh gps
```

GPS 좌표가 계속 출력되는지 확인한 뒤 `Ctrl+C`.

IMU 확인:

```bash
./run.sh imu
```

IMU 값이 계속 들어오는지 확인한 뒤 `Ctrl+C`.

필요하면 저속 CtrlCmd 테스트도 할 수 있습니다.

```bash
./run.sh ctrl
```

---

# 9. 메인 주행 코드 실행

현재 팀 공용 메인 진입점은 다음 파일입니다.

```text
src/main.py
```

## 9-1. 먼저 Dry-run

차량을 실제로 움직이지 않고 센서/계산 상태만 확인합니다.

```bash
cd ~/molit-autonomous-driving
source .venv/bin/activate
python3 src/main.py
```

기본 최고속도 설정은 57 km/h이지만 `--drive`가 없으면 CtrlCmd를 MORAI에 보내지 않습니다.

## 9-2. 실제 주행

**MORAI 설정을 모두 확인한 뒤에만 실행합니다.**

```bash
python3 src/main.py --drive
```

현재 `src/main.py`의 기본 최고속도는 **57 km/h**입니다.

더 낮은 속도로 먼저 확인하고 싶으면 예를 들어:

```bash
python3 src/main.py --drive --max-speed 20
```

또는:

```bash
python3 src/main.py --drive --max-speed 40
```

57 km/h 실행을 명시하고 싶으면:

```bash
python3 src/main.py --drive --max-speed 57
```

**57 km/h보다 큰 값은 현재 메인 코드에서 허용하지 않습니다.**

주행 중 중지:

```text
Ctrl+C
```

프로그램 종료 시 정지 명령을 전송하도록 구성되어 있습니다.

---

# 10. 주행 로그

주행을 실행하면 CSV 로그가 자동으로 `logs/` 폴더에 저장됩니다.

예:

```text
logs/run_YYYYMMDD_HHMMSS_57kph_drive.csv
```

주요 기록 항목:

- elapsed time
- waypoint index
- 현재 x/y
- yaw
- path distance error
- curve radius
- target speed
- Competition Vehicle Status speed
- steering
- accel
- brake
- Dynamic LFD 관련 값

튜닝 테스트 후에는 해당 CSV를 공유합니다.

---

# 11. 현재 메인 제어 구조

```text
MORAI GPS UDP (9201) ┐
                     ├─> Python Controller
MORAI IMU UDP (1111) ┤      ├─ GPS -> UTM Zone 52 -> MORAI local x/y
                     │      ├─ IMU quaternion -> yaw
Competition Status   ┘      ├─ signed_vel -> current speed
       (1909)               ├─ official global path
                            ├─ speed profile + braking preview
                            ├─ Dynamic/Curvature LFD Pure Pursuit
                            └─ accel/brake/steer
                                      │
                                      v
                           Ego Ctrl Cmd UDP (9093)
                                      │
                                      v
                                    MORAI
```

좌표 변환 기준:

```text
UTM Zone 52 / WGS84
East Offset  = 302595.0
North Offset = 4124145.0
```

---

# 12. 공식 전역경로

현재 사용하는 경로:

```text
path/2026_molit_comp_global_path.txt
```

이 경로는 대회에서 제공한 공식 전역경로를 저장소에 포함한 것입니다.

별도의 임의 경로를 만들어 메인 주행에 사용하지 않습니다.

---

# 13. 현재 튜닝 기준값

현재 메인 코드의 기준 상태:

```text
Max staged speed    57 km/h
Preview decel       2.0 m/s^2
Max brake cmd       0.10
LFD gain            0.65
Minimum LFD         2.5 m
Maximum LFD         9.0 m
Control mode        2
Command type        1
```

곡률이 큰 구간에서는 Dynamic LFD에 추가로 곡률 기반 LFD cap을 적용합니다.

현재 곡률별 LFD cap은 팀 튜닝 값이며 MORAI 공식 규정값이 아닙니다.

---

# 14. 현재 알고 있는 제한사항

1. GPS 음영구간 처리가 아직 구현되지 않았습니다.
2. 57 km/h 세팅은 현재 WP 약 1437까지 단계 검증한 상태입니다.
3. 차선침범 여부는 영상과 CSV를 함께 보면서 추가 튜닝해야 합니다.
4. 고속구간 예외 규정에 따른 60 km/h 초과 주행은 아직 메인 코드에 적용하지 않았습니다.
5. 카메라/신호등/장애물 대응은 현재 메인 주행 코드에 통합되지 않았습니다.

---

# 15. 팀원이 가장 빠르게 실행하는 순서

처음 한 번:

```bash
git clone https://github.com/MISEONG0216/molit-autonomous-driving.git
cd molit-autonomous-driving
sudo apt update
sudo apt install -y git python3 python3-pip python3-venv
chmod +x setup.sh check.sh run.sh
./setup.sh
./check.sh
```

MORAI를 실행하고 UDP 포트를 설정한 뒤:

```bash
cd ~/molit-autonomous-driving
source .venv/bin/activate
./run.sh gps
```

`Ctrl+C` 후:

```bash
./run.sh imu
```

`Ctrl+C` 후 Dry-run:

```bash
python3 src/main.py
```

모든 값이 정상이고 Ego가 `AV / ExternalCtrl`이면 실제 주행:

```bash
python3 src/main.py --drive
```

---

# 16. 기존 팀원 업데이트 방법

```bash
cd ~/molit-autonomous-driving
git switch main
git pull
source .venv/bin/activate
python3 src/main.py
```

문제 없으면:

```bash
python3 src/main.py --drive
```

---

# 17. 개발 규칙

새 Python 라이브러리를 추가했다면 `requirements.txt`에도 반드시 반영합니다.

작업 전:

```bash
git switch main
git pull
```

개별 기능 개발은 가능하면 별도 branch에서 진행하고, 검증 후 `main`에 반영합니다.

현재 `main`은 다른 팀원이 바로 테스트할 수 있는 기준 주행 코드를 유지하는 용도로 사용합니다.
