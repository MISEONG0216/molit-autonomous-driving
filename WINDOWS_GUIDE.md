# Windows 실행 가이드

이 문서는 `molit-autonomous-driving` 프로젝트를 **Windows 10/11에서 처음 내려받아 MORAI Pure Pursuit 실제 주행까지 실행하는 전체 과정**을 정리한 문서입니다.

> 검증 상태
>
> - Ubuntu: MORAI 환경에서 전역경로 1바퀴 완주 검증 완료
> - Windows: MORAI 환경에서 GPS/IMU 수신, Pure Pursuit Dry Run, 실제 주행 및 전역경로 1바퀴 완주 검증 완료
> - Windows 검증 PC에서는 Python `3.13.14`, Git `2.55.0.windows.4` 사용
>
> Ubuntu와 Windows 모두 같은 `src/pure_pursuit_udp.py`와 대회 전역경로를 사용합니다.

---

## 1. 필요한 프로그램 확인

Windows PC에 다음 프로그램이 필요합니다.

- Git for Windows
- Python
- 대회용 MORAI Simulator

CMD를 열고 확인합니다.

```cmd
git --version
python --version
```

실제 Windows 검증에서는 다음 환경으로 성공했습니다.

```text
Git    : 2.55.0.windows.4
Python : 3.13.14
```

Python 설치 시 `Add Python to PATH`를 활성화하는 것을 권장합니다.

---

## 2. GitHub 저장소 Clone

CMD에서 원하는 위치로 이동합니다. 바탕화면에 설치하는 예시는 다음과 같습니다.

```cmd
cd %USERPROFILE%\Desktop
git clone https://github.com/MISEONG0216/molit-autonomous-driving.git
cd molit-autonomous-driving
```

파일 확인:

```cmd
dir
```

Windows 지원 파일이 `main`에 병합되기 전 작업 브랜치를 직접 테스트해야 하는 경우에만 해당 브랜치로 이동합니다.

```cmd
git checkout docs/team-readme-guide
git pull
```

Windows 지원이 `main`에 병합된 뒤에는 이 브랜치 전환 과정이 필요하지 않습니다.

---

## 3. Windows 환경 자동 설치

프로젝트 루트에서 실행합니다.

```cmd
setup_windows.bat
```

스크립트가 자동으로 다음 작업을 수행합니다.

1. Python 확인
2. Git 확인
3. `.venv` 가상환경 생성
4. pip 업데이트
5. `requirements.txt` 패키지 설치
6. MORAI Network Module `24.R2.0` 다운로드

실제 Python 3.13.14 환경에서도 필요한 패키지 설치가 정상 완료되었습니다.

설치가 끝나면 환경 점검을 실행합니다.

```cmd
check_windows.bat
```

모든 항목이 `[OK]`이면 기본 실행 환경 준비가 완료된 것입니다.

---

## 4. MORAI 실행 및 네트워크 설정

MORAI Simulator와 Python을 **같은 Windows PC에서 실행하는 기준**입니다.

MORAI를 실행하고 대회 시나리오를 불러온 뒤 센서를 설정합니다.

### GPS

```text
Destination IP   : 127.0.0.1
Destination Port : 9201
```

### IMU

```text
Destination IP   : 127.0.0.1
Destination Port : 1111
```

### CtrlCmd

```text
Host IP   : 127.0.0.1
Host Port : 9093
```

실제 차량을 코드로 제어할 때 Ego 차량은 다음 상태여야 합니다.

```text
AV / ExternalCtrl
```

---

## 5. GPS 통신 확인

먼저 GPS만 확인합니다.

```cmd
run_windows.bat gps
```

정상이라면 `0.000000`이 아닌 실제 위도/경도 값이 계속 출력됩니다.

검증 당시 약 다음 위치의 GPS가 정상 수신되었습니다.

```text
lat ≈ 37.239077
lon ≈ 126.773058
```

확인 후 종료합니다.

```text
Ctrl + C
```

GPS가 들어오지 않으면 MORAI GPS Sensor의 Destination IP/Port를 먼저 확인합니다.

---

## 6. IMU 통신 확인

GPS 테스트를 종료한 뒤 실행합니다.

```cmd
run_windows.bat imu
```

정상이라면 quaternion, angular velocity, acceleration 등의 IMU 데이터가 계속 출력됩니다.

확인 후:

```text
Ctrl + C
```

IMU가 들어오지 않으면 Destination Port가 `1111`인지 확인합니다.

---

## 7. Pure Pursuit Dry Run

GPS와 IMU가 모두 정상일 때 실제 차량을 움직이기 전에 Dry Run을 수행합니다.

```cmd
run_windows.bat pure-pursuit
```

Dry Run에서는 CtrlCmd를 MORAI로 송신하지 않습니다.

정상 실행 시 다음 항목이 계속 갱신됩니다.

```text
GPS
POSITION
YAW
NEAREST
PATH POS
PATH DIST
LOOKAHEAD
TARGET
LOCAL
THETA
STEERING
[DRY-RUN] CtrlCmd 송신 안 함
```

Windows 실제 검증에서 확인된 예시는 다음과 같습니다.

```text
GPS       : 37.239077, 126.773058
POSITION  : x=-130.852, y=-426.817
YAW       : 63.84 deg
NEAREST   : index=3
PATH POS  : x=-130.969, y=-427.015
PATH DIST : 0.230 m
LOOKAHEAD : index=9
TARGET    : x=-129.512, y=-424.393
LOCAL     : x=2.767, y=-0.134
THETA     : -2.76 deg
STEERING  : -0.03856 rad (-2.21 deg)
```

이 결과로 Windows에서도 다음 처리 흐름이 정상 동작함을 확인했습니다.

```text
MORAI GPS
   +
MORAI IMU
   ↓
GPS WGS84 → UTM Zone 52 → MORAI Local 좌표
   ↓
IMU Quaternion → Yaw
   ↓
전역경로 최근접 Waypoint 탐색
   ↓
Lookahead Point 탐색
   ↓
Pure Pursuit 조향 계산
```

Dry Run에서 특히 확인할 항목은 다음과 같습니다.

- `POSITION`이 전역경로 부근인지
- `YAW`가 차량 진행 방향과 맞는지
- `PATH DIST`가 비정상적으로 크지 않은지
- `LOOKAHEAD`가 정상적으로 잡히는지
- `LOCAL x`가 전방 목표점에서 양수인지

확인 후 `Ctrl + C`로 종료합니다.

---

## 8. 실제 Pure Pursuit 주행

Dry Run이 정상일 때만 실제 주행을 시작합니다.

MORAI에서 다시 확인합니다.

```text
Ego              : AV / ExternalCtrl
CtrlCmd Host IP  : 127.0.0.1
CtrlCmd Host Port: 9093
```

실행:

```cmd
run_windows.bat pure-pursuit-drive
```

직접 실행하려면:

```cmd
.venv\Scripts\python.exe src\pure_pursuit_udp.py --drive
```

현재 검증된 기준 설정은 다음과 같습니다.

```text
Target Speed       : 5.0 km/h
Lookahead Distance : 2.5 m
Local Path Size    : 50
Control Frequency  : 15 Hz
Global Path        : path/2026_molit_comp_global_path.txt
```

Windows MORAI 환경에서 이 설정으로 **실제 차량 제어 및 전역경로 1바퀴 완주에 성공**했습니다.

주행 중 문제가 발생하면:

```text
Ctrl + C
```

를 눌러 프로그램을 종료합니다. 프로그램 종료 시 정지 명령을 전송하도록 구현되어 있습니다.

---

## 9. 처음부터 실제 주행까지 전체 순서

```text
Windows 부팅
   ↓
Git / Python 확인
   ↓
GitHub 저장소 Clone
   ↓
setup_windows.bat
   ↓
check_windows.bat → 전체 [OK]
   ↓
MORAI Simulator 실행
   ↓
대회 시나리오 로드
   ↓
GPS Destination = 127.0.0.1:9201
   ↓
IMU Destination = 127.0.0.1:1111
   ↓
CtrlCmd Host = 127.0.0.1:9093
   ↓
Ego = AV / ExternalCtrl
   ↓
run_windows.bat gps
   ↓
GPS 수신 확인 → Ctrl+C
   ↓
run_windows.bat imu
   ↓
IMU 수신 확인 → Ctrl+C
   ↓
run_windows.bat pure-pursuit
   ↓
POSITION / YAW / PATH DIST / LOOKAHEAD 확인
   ↓
Ctrl+C
   ↓
run_windows.bat pure-pursuit-drive
   ↓
5 km/h / LFD 2.5 m 실제 주행
   ↓
전역경로 1바퀴 완주
```

---

## 10. 자주 사용하는 Windows 명령

### 최초 설치

```cmd
setup_windows.bat
check_windows.bat
```

### GPS

```cmd
run_windows.bat gps
```

### IMU

```cmd
run_windows.bat imu
```

### Pure Pursuit 계산만 확인

```cmd
run_windows.bat pure-pursuit
```

### 실제 차량 주행

```cmd
run_windows.bat pure-pursuit-drive
```

### CtrlCmd 단독 테스트

차량이 실제로 움직일 수 있으므로 필요한 경우에만 실행합니다.

```cmd
run_windows.bat ctrl
```

---

## 11. Ubuntu와 Windows의 관계

알고리즘을 OS별로 따로 관리하지 않습니다.

공통 파일:

```text
src/pure_pursuit_udp.py
src/morai_common.py
src/gps_test.py
src/imu_test.py
src/ctrl_test.py
path/2026_molit_comp_global_path.txt
requirements.txt
```

Ubuntu 실행 파일:

```text
setup.sh
check.sh
run.sh
```

Windows 실행 파일:

```text
setup_windows.bat
check_windows.bat
run_windows.bat
```

따라서 속도 프로파일, 곡률 분석, Lookahead 조정 등의 알고리즘 개발은 공통 `src/` 코드를 기준으로 진행합니다.

---

## 12. Windows 팀원의 Git 작업 방법

`main`에서 직접 개발하지 않습니다.

작업 시작 전에 최신 `main`을 받습니다.

```cmd
git checkout main
git pull origin main
```

개인 브랜치를 생성합니다.

```cmd
git checkout -b speed-tuning-이름
git push -u origin speed-tuning-이름
```

수정 및 MORAI 테스트가 끝나면:

```cmd
git status
git add .
git commit -m "Tune speed profile"
git push
```

그 후 GitHub에서 Pull Request를 생성하고 팀원 검토 후 `main`에 병합합니다.

---

## 13. 문제 해결

### `python` 명령을 찾을 수 없음

Python 설치와 PATH 설정을 확인합니다.

### Windows 환경을 처음부터 다시 만들고 싶음

`.venv` 폴더를 삭제한 뒤:

```cmd
setup_windows.bat
check_windows.bat
```

을 다시 실행합니다.

### GPS 값이 0 또는 수신되지 않음

```text
Destination IP   = 127.0.0.1
Destination Port = 9201
```

을 확인합니다.

### IMU가 수신되지 않음

```text
Destination IP   = 127.0.0.1
Destination Port = 1111
```

을 확인합니다.

### Dry Run은 되는데 차량이 움직이지 않음

```text
Ego = AV / ExternalCtrl
CtrlCmd Host IP = 127.0.0.1
CtrlCmd Host Port = 9093
```

을 확인합니다.

### `LOOKAHEAD : NOT FOUND`가 계속 출력됨

차량 위치, Yaw, 전역경로와 `PATH DIST`를 먼저 확인합니다. 정상 출발 위치에서는 Lookahead waypoint가 계속 탐색되어야 합니다.

### Windows 방화벽 경고

Python 또는 MORAI에 네트워크 접근 허용 창이 나타나면 대회용 로컬 UDP 통신에 사용하는 프로그램인지 확인한 뒤 필요한 네트워크 유형에서 허용합니다.

---

## 14. Windows 검증 결과

Windows 실제 테스트에서 다음 항목을 모두 통과했습니다.

```text
[PASS] Git / Python 실행
[PASS] setup_windows.bat
[PASS] check_windows.bat
[PASS] requirements 설치
[PASS] MORAI Network Module 24.R2.0
[PASS] GPS UDP 수신 : 127.0.0.1:9201
[PASS] IMU UDP 수신 : 127.0.0.1:1111
[PASS] GPS → UTM52 → MORAI Local 좌표 변환
[PASS] IMU → Yaw 계산
[PASS] 전역경로 최근접 waypoint 탐색
[PASS] Lookahead point 탐색
[PASS] Pure Pursuit Dry Run
[PASS] CtrlCmd UDP 송신 : 127.0.0.1:9093
[PASS] 실제 MORAI 차량 제어
[PASS] 5.0 km/h / LFD 2.5 m 전역경로 1바퀴 완주
```

따라서 현재 프로젝트의 Pure Pursuit 기준 버전은 **Ubuntu와 Windows 양쪽 MORAI 환경에서 실제 주행 검증 완료** 상태입니다.
