# MOLIT Autonomous Driving

2026 대학생 AI/SW 모빌리티 경진대회 AI융합자율주행 부문용 MORAI 기반 자율주행 프로젝트입니다.

현재 `main` 기준 주행 코드는 GPS + IMU + Competition Vehicle Status + 공식 전역경로를 사용하여 Pure Pursuit 주행을 수행합니다. 차량 제어는 대회 규정에 맞게 `ctrl_mode=2`, `cmd_type=1 (accel, brake, steer)`을 사용합니다.

## 현재 기준 주행 설정

- 일반구간 기준 최고속도: **57 km/h**
- 위치: GPS UDP
- 자세: IMU UDP
- 실제 속도: Competition Vehicle Status `signed_vel`
- 경로: `path/2026_molit_comp_global_path.txt`
- 횡제어: Pure Pursuit
- LFD: 속도 기반 Dynamic LFD + 곡률 기반 상한
- braking preview: `2.0 m/s^2`
- 최대 Brake command: `0.10`
- Ego Ctrl Cmd: `ctrl_mode=2`, `cmd_type=1`

> 주의: GPS 음영구간은 아직 별도 해결 대상입니다. 현재 코드를 완성된 본선 코드로 간주하면 안 됩니다.

---

# 1. 공통 준비

필요한 것:

- 대회용 MORAI Simulator
- Git
- Python 3.8.x 권장
- GitHub 저장소 접근 권한

ROS는 현재 `src/main.py` 실행에 필요하지 않습니다.

저장소:

```text
https://github.com/MISEONG0216/molit-autonomous-driving.git
```

현재 팀 공용 메인 진입점:

```text
src/main.py
```

---

# 2. Ubuntu 20.04 설치 및 실행

## 2-1. 저장소 받기

```bash
git clone https://github.com/MISEONG0216/molit-autonomous-driving.git
cd molit-autonomous-driving
```

기존 팀원은:

```bash
cd ~/molit-autonomous-driving
git switch main
git pull
```

## 2-2. 필수 패키지 설치

```bash
sudo apt update
sudo apt install -y git python3 python3-pip python3-venv
```

## 2-3. 프로젝트 설정

```bash
chmod +x setup.sh check.sh run.sh
./setup.sh
./check.sh
```

`setup.sh`가 자동으로 수행하는 작업:

1. `.venv` 생성
2. `requirements.txt` 설치
3. MORAI 공식 Network Module `24.R2.0` 다운로드
4. 실행 권한 설정

가상환경 활성화:

```bash
source .venv/bin/activate
```

## 2-4. Ubuntu 센서 확인

MORAI를 실행하고 아래 UDP 설정을 맞춘 뒤:

```bash
./run.sh gps
```

GPS가 정상 출력되면 `Ctrl+C`.

```bash
./run.sh imu
```

IMU가 정상 출력되면 `Ctrl+C`.

## 2-5. Ubuntu Dry-run

```bash
python3 src/main.py
```

`--drive`가 없으므로 차량 제어 명령은 전송하지 않습니다.

## 2-6. Ubuntu 실제 주행

낮은 속도로 먼저 확인:

```bash
python3 src/main.py --drive --max-speed 20
```

현재 기준 57 km/h:

```bash
python3 src/main.py --drive
```

또는 명시적으로:

```bash
python3 src/main.py --drive --max-speed 57
```

---

# 3. Windows 10/11 설치 및 실행

현재 메인 제어 코드는 ROS가 아닌 Python + UDP 구조이므로 Windows에서도 같은 제어 로직을 사용할 수 있습니다.

Windows에서는 Ubuntu용 `setup.sh`, `run.sh` 대신 다음 PowerShell 스크립트를 사용합니다.

```text
setup_windows.ps1
run_windows.ps1
```

## 3-1. 필요한 프로그램

먼저 다음을 설치합니다.

- Git for Windows
- Python 3.8.x 권장

설치 시 Python은 가능하면 **Add Python to PATH**를 체크합니다.

PowerShell에서 확인:

```powershell
git --version
python --version
```

`python` 대신 `py`만 동작해도 `setup_windows.ps1`이 자동으로 처리합니다.

## 3-2. 저장소 받기

PowerShell:

```powershell
git clone https://github.com/MISEONG0216/molit-autonomous-driving.git
cd molit-autonomous-driving
```

기존 팀원은:

```powershell
cd molit-autonomous-driving
git switch main
git pull
```

## 3-3. PowerShell 실행 정책

스크립트 실행이 차단되는 경우 **현재 PowerShell 창에서만** 다음을 실행합니다.

```powershell
Set-ExecutionPolicy -Scope Process Bypass
```

그 뒤 같은 창에서 계속 진행합니다.

## 3-4. Windows 프로젝트 초기 설정

```powershell
.\setup_windows.ps1
```

이 스크립트가 자동으로 수행하는 작업:

1. Python 확인
2. `.venv` 생성
3. `requirements.txt` 설치
4. MORAI 공식 Network Module `24.R2.0` 다운로드
5. 필수 Python 패키지 import 확인

가상환경을 직접 활성화하고 싶으면:

```powershell
.\.venv\Scripts\Activate.ps1
```

하지만 `run_windows.ps1`은 `.venv`의 Python을 직접 사용하므로 매번 Activate할 필요는 없습니다.

## 3-5. Windows 센서 확인

GPS:

```powershell
.\run_windows.ps1 gps
```

정상 출력 후 `Ctrl+C`.

IMU:

```powershell
.\run_windows.ps1 imu
```

정상 출력 후 `Ctrl+C`.

저속 CtrlCmd 테스트가 필요하면:

```powershell
.\run_windows.ps1 ctrl
```

> `ctrl`은 차량을 움직일 수 있으므로 MORAI Ego가 `AV / ExternalCtrl`인지 먼저 확인합니다.

## 3-6. Windows Dry-run

```powershell
.\run_windows.ps1 main
```

기본 `MaxSpeed` 값은 57이지만 `-Drive`가 없으므로 실제 CtrlCmd는 전송하지 않습니다.

## 3-7. Windows 실제 주행

처음에는 5~20 km/h로 확인하는 것을 권장합니다.

```powershell
.\run_windows.ps1 main -Drive -MaxSpeed 5
```

그다음:

```powershell
.\run_windows.ps1 main -Drive -MaxSpeed 20
```

현재 기준 57 km/h:

```powershell
.\run_windows.ps1 main -Drive -MaxSpeed 57
```

또는 Python을 직접 실행해도 됩니다.

```powershell
.\.venv\Scripts\python.exe src\main.py --drive --max-speed 57
```

---

# 4. MORAI UDP 설정

MORAI와 Python을 **같은 PC에서 실행하는 경우** 현재 코드는 `127.0.0.1`을 사용합니다.

## GPS

```text
Protocol          UDP
Host Sensor IP    127.0.0.1
Host Sensor Port  9200
Destination IP    127.0.0.1
Destination Port  9201
```

Python 수신:

```text
127.0.0.1:9201
```

## IMU

```text
Host Sensor IP    127.0.0.1
Host Sensor Port  9090
Destination IP    127.0.0.1
Destination Port  1111
```

Python 수신:

```text
127.0.0.1:1111
```

## Competition Vehicle Status

현재 코드의 Python 수신 포트:

```text
127.0.0.1:1909
```

현재 개발 설정:

```text
Host Port         1908
Destination Port  1909
```

실제 속도 제어에는 `signed_vel`을 사용합니다.

## Ego Ctrl Cmd

```text
Protocol          UDP
Host IP           127.0.0.1
Destination IP    127.0.0.1
Host Port         9093
Destination Port  9094
```

Python 제어 송신 대상:

```text
127.0.0.1:9093
```

제어 방식:

```text
ctrl_mode = 2
cmd_type  = 1
```

즉 `accel`, `brake`, `steer`, `gear` 방식입니다.

`Simulator Network`의 MultiEgo용 연결은 현재 메인 주행 코드에 필요하지 않습니다.

---

# 5. MORAI 실행 전 공통 체크리스트

1. 대회 시나리오 로드
2. GPS Destination Port = `9201`
3. IMU Destination Port = `1111`
4. Competition Vehicle Status Destination Port = `1909`
5. Ego Ctrl Cmd Host Port = `9093`
6. Ego Controller = `AV / ExternalCtrl`
7. GPS 수신 정상
8. IMU 수신 정상
9. Dry-run 정상
10. 그 뒤 `--drive` 또는 `-Drive` 사용

Windows에서 UDP 수신이 되지 않으면 Windows Defender 방화벽에서 MORAI와 Python의 네트워크 접근 허용 여부도 확인합니다.

---

# 6. 현재 제어 구조

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

# 7. 공식 전역경로

현재 사용하는 경로:

```text
path/2026_molit_comp_global_path.txt
```

대회에서 제공한 공식 전역경로이며 별도의 임의 경로를 메인 주행에 사용하지 않습니다.

---

# 8. 현재 튜닝 기준값

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

곡률이 큰 구간에서는 Dynamic LFD에 곡률 기반 LFD cap을 추가 적용합니다.

곡률별 LFD cap은 팀 튜닝 값이며 MORAI 공식 규정값이 아닙니다.

---

# 9. 주행 로그

주행 시 CSV가 자동으로 `logs/`에 저장됩니다.

예:

```text
logs/run_YYYYMMDD_HHMMSS_57kph_drive.csv
```

주요 항목:

- elapsed time
- waypoint index
- x/y
- yaw
- path error
- curve radius
- target speed
- Competition Vehicle Status speed
- steering
- accel
- brake
- Dynamic LFD 관련 값

튜닝 시에는 MORAI 주행 영상 + 터미널 화면 + 같은 주행의 CSV를 함께 확인합니다.

---

# 10. 현재 알려진 제한사항

1. GPS 음영구간 처리가 아직 구현되지 않았습니다.
2. 57 km/h 세팅은 정상 GPS 구간에서 단계 검증 중입니다.
3. 차선침범 여부는 영상과 CSV를 함께 보면서 추가 튜닝해야 합니다.
4. 고속구간 예외 규정에 따른 60 km/h 초과 주행은 아직 메인 코드에 적용하지 않았습니다.
5. 카메라/신호등/장애물 대응은 현재 메인 코드에 통합되지 않았습니다.
6. Windows에서는 각 MORAI 빌드의 UDP UI/방화벽 상태에 따라 포트 통신을 별도로 확인해야 합니다.

---

# 11. 가장 빠른 실행 요약

## Ubuntu

```bash
git clone https://github.com/MISEONG0216/molit-autonomous-driving.git
cd molit-autonomous-driving
sudo apt update
sudo apt install -y git python3 python3-pip python3-venv
chmod +x setup.sh check.sh run.sh
./setup.sh
./check.sh
./run.sh gps
./run.sh imu
python3 src/main.py
python3 src/main.py --drive --max-speed 20
```

## Windows PowerShell

```powershell
git clone https://github.com/MISEONG0216/molit-autonomous-driving.git
cd molit-autonomous-driving
Set-ExecutionPolicy -Scope Process Bypass
.\setup_windows.ps1
.\run_windows.ps1 gps
.\run_windows.ps1 imu
.\run_windows.ps1 main
.\run_windows.ps1 main -Drive -MaxSpeed 20
```

57 km/h까지 확인 후:

```powershell
.\run_windows.ps1 main -Drive -MaxSpeed 57
```

---

# 12. 개발 규칙

작업 전:

```bash
git switch main
git pull
```

개별 기능 개발은 가능하면 별도 branch에서 진행하고 검증 후 `main`에 반영합니다.

현재 `main`은 다른 팀원이 바로 테스트할 수 있는 기준 주행 코드를 유지합니다.
