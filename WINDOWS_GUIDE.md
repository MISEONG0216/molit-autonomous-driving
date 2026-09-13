# Windows 실행 가이드

이 문서는 `molit-autonomous-driving` 프로젝트를 **Windows 10/11에서 실행하는 방법**을 정리한 문서입니다.

> 현재 기준 상태
>
> - Ubuntu 22.04: 실제 MORAI 환경에서 전역경로 1바퀴 완주 검증 완료
> - Windows 10/11: 실행 스크립트 및 환경 구성을 추가한 상태이며, 실제 MORAI 주행 검증은 아직 필요
>
> Windows에서 실제 주행이 성공하기 전까지는 Ubuntu 버전을 기준 안정 버전으로 봅니다.

---

## 1. 필요한 프로그램

Windows PC에 아래 프로그램을 먼저 설치합니다.

- Git for Windows
- Python 3.8 이상
- 대회용 MORAI Simulator

Python 설치 시 가능하면 `Add Python to PATH` 옵션을 체크합니다.

설치 확인은 CMD 또는 PowerShell에서 다음 명령으로 할 수 있습니다.

```powershell
git --version
python --version
```

---

## 2. 저장소 Clone

저장소 Collaborator 초대를 수락한 뒤 실행합니다.

```powershell
git clone https://github.com/MISEONG0216/molit-autonomous-driving.git
cd molit-autonomous-driving
```

---

## 3. Windows 환경 자동 설치

프로젝트 루트에서 다음을 실행합니다.

```powershell
setup_windows.bat
```

이 스크립트는 다음 작업을 수행합니다.

1. Python 확인
2. Git 확인
3. `.venv` 가상환경 생성
4. `requirements.txt` 설치
5. MORAI Network Module `24.R2.0` 다운로드

설치 후:

```powershell
check_windows.bat
```

실행 결과가 모두 `[OK]`이면 기본 환경이 준비된 것입니다.

---

## 4. MORAI 네트워크 설정

MORAI와 Python 코드를 **같은 Windows PC에서 실행하는 기준**입니다.

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

실제 차량 제어 전에는 Ego 차량을 다음 상태로 둡니다.

```text
AV / ExternalCtrl
```

---

## 5. 센서 통신 확인

### GPS

```powershell
run_windows.bat gps
```

정상이라면 `0.000000`이 아닌 실제 위도/경도 값이 출력됩니다.

종료:

```text
Ctrl + C
```

### IMU

```powershell
run_windows.bat imu
```

정상이라면 quaternion 및 가속도 값이 출력됩니다.

종료:

```text
Ctrl + C
```

---

## 6. Pure Pursuit Dry Run

실제 차량을 움직이기 전에 반드시 Dry Run을 먼저 수행합니다.

```powershell
run_windows.bat pure-pursuit
```

또는 직접 실행할 수 있습니다.

```powershell
.venv\Scripts\python.exe src\pure_pursuit_udp.py
```

정상이라면 다음 값들이 계속 출력됩니다.

```text
GPS
POSITION
YAW
NEAREST
PATH DIST
LOOKAHEAD
TARGET
LOCAL
THETA
STEERING
[DRY-RUN] CtrlCmd 송신 안 함
```

확인할 항목:

- GPS 값 정상
- POSITION 값이 전역경로 부근
- YAW 값이 차량 방향과 유사
- PATH DIST가 지나치게 크지 않음
- LOOKAHEAD가 정상적으로 탐색됨

---

## 7. 실제 Pure Pursuit 주행

MORAI에서 Ego 차량이 `AV / ExternalCtrl`인지 다시 확인한 뒤 실행합니다.

```powershell
run_windows.bat pure-pursuit-drive
```

또는:

```powershell
.venv\Scripts\python.exe src\pure_pursuit_udp.py --drive
```

현재 기준 설정:

```text
Target Speed       : 5.0 km/h
Lookahead Distance : 2.5 m
Local Path Size    : 50
Control Frequency  : 15 Hz
```

주행 중 `Ctrl + C`를 누르면 프로그램 종료와 함께 정지 명령을 전송합니다.

---

## 8. Windows 권장 실행 순서

```text
Windows 부팅
   ↓
MORAI Simulator 실행
   ↓
대회 시나리오 로드
   ↓
GPS Destination Port = 9201
   ↓
IMU Destination Port = 1111
   ↓
CtrlCmd Host Port = 9093
   ↓
Ego = AV / ExternalCtrl
   ↓
check_windows.bat
   ↓
run_windows.bat gps
   ↓
run_windows.bat imu
   ↓
run_windows.bat pure-pursuit
   ↓
run_windows.bat pure-pursuit-drive
```

---

## 9. Ubuntu와 Windows의 차이

핵심 Python 코드는 공통으로 사용합니다.

```text
src/pure_pursuit_udp.py
path/2026_molit_comp_global_path.txt
requirements.txt
```

운영체제별 차이는 실행 스크립트입니다.

### Ubuntu

```text
setup.sh
check.sh
run.sh
```

### Windows

```text
setup_windows.bat
check_windows.bat
run_windows.bat
```

따라서 구간별 속도 최적화나 Pure Pursuit 알고리즘 수정은 OS별 파일을 따로 만들지 말고 **공통 `src/` 코드에서 작업**합니다.

---

## 10. Git 협업 규칙

Windows 팀원도 Ubuntu 팀원과 동일한 Git 규칙을 사용합니다.

작업 시작 전:

```powershell
git checkout main
git pull origin main
git checkout -b speed-tuning-이름
```

작업 후:

```powershell
git status
git add .
git commit -m "Tune speed profile"
git push -u origin speed-tuning-이름
```

그 후 GitHub에서 Pull Request를 생성합니다.

`main`에서 직접 개발하지 않습니다.

---

## 11. Windows에서 문제가 생길 때

### `python` 명령을 찾을 수 없음

Python을 다시 설치하고 `Add Python to PATH`를 활성화합니다.

### `.venv` 오류

`.venv` 폴더를 삭제한 뒤 다시 실행합니다.

```powershell
setup_windows.bat
```

### GPS 값이 0

MORAI GPS Sensor의 Destination Port가 `9201`인지 확인합니다.

### IMU 값이 안 들어옴

MORAI IMU Sensor의 Destination Port가 `1111`인지 확인합니다.

### 코드가 실행되는데 차량이 움직이지 않음

다음을 확인합니다.

```text
Ego = AV / ExternalCtrl
CtrlCmd Host Port = 9093
```

### Windows 방화벽 경고

Python 또는 MORAI의 네트워크 접근 허용 창이 나타나면, 대회용 로컬 UDP 통신이 필요한 프로그램인지 확인한 뒤 사용하는 네트워크 유형에서 허용합니다.

---

## 12. Windows 검증 완료 기준

Windows 지원을 `검증 완료`로 표시하려면 최소한 다음 항목을 확인합니다.

- `setup_windows.bat` 성공
- `check_windows.bat` 전체 통과
- GPS 수신 성공
- IMU 수신 성공
- Pure Pursuit Dry Run 성공
- CtrlCmd 실제 송신 성공
- 5 km/h / Lookahead 2.5 m 기준 실제 주행 성공
- 가능하면 전역경로 1바퀴 완주

모든 검증이 끝난 뒤 README의 Windows 상태를 검증 완료로 변경합니다.
