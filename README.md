# MOLIT Autonomous Driving

국토교통부 무인차 대회용 **MORAI 기반 자율주행 프로젝트**입니다.

현재 Pure Pursuit 기준 버전은 실제 MORAI 환경에서 **Ubuntu와 Windows 모두 전역경로 1바퀴 완주가 확인**되었습니다.

> 팀 협업의 가장 중요한 원칙: **`main`에서 직접 개발하지 않습니다.**  
> 개인 브랜치에서 수정 → MORAI에서 테스트 → Pull Request → 검토 후 `main` 병합 순서로 진행합니다.

---

## 1. 현재 기준 주행 상태

현재 검증된 설정:

- Pure Pursuit Lookahead Distance: `2.5 m`
- 목표 속도: `5.0 km/h`
- Local Path Size: `50`
- Control Frequency: `15 Hz`
- 전역경로: `path/2026_molit_comp_global_path.txt`
- GPS 좌표 변환: WGS84 → UTM Zone 52 → MORAI Local 좌표
- MORAI UDP 직접 통신 방식

검증 상태:

```text
Ubuntu  : GPS / IMU / Dry Run / 실제 주행 / 전역경로 1바퀴 완주 ✅
Windows : GPS / IMU / Dry Run / 실제 주행 / 전역경로 1바퀴 완주 ✅
```

Windows에서 처음 설치하고 실행하는 전체 과정은 [`WINDOWS_GUIDE.md`](WINDOWS_GUIDE.md)를 참고합니다.

구간별 속도 최적화, 곡률 기반 감속, 동적 Lookahead 등의 실험은 반드시 개인 브랜치에서 진행합니다.

---

## 2. 실행 환경

### Ubuntu 검증 환경

- Python `3.8.x`
- MORAI SIM: Drive 대회용 시뮬레이터
- MORAI Network Module `24.R2.0`

Ubuntu 실행 스크립트:

```text
setup.sh
check.sh
run.sh
```

### Windows 검증 환경

Windows 실제 테스트에서 다음 환경으로 성공했습니다.

```text
Python : 3.13.14
Git    : 2.55.0.windows.4
```

Windows 실행 스크립트:

```text
setup_windows.bat
check_windows.bat
run_windows.bat
```

MORAI Network Module은 Ubuntu/Windows 모두 공식 `24.R2.0` 버전을 사용합니다.

---

# 3. 빠른 시작

## Ubuntu

```bash
git clone https://github.com/MISEONG0216/molit-autonomous-driving.git
cd molit-autonomous-driving
chmod +x setup.sh check.sh run.sh
./setup.sh
./check.sh
```

실행:

```bash
./run.sh gps
./run.sh imu
python3 src/pure_pursuit_udp.py
python3 src/pure_pursuit_udp.py --drive
```

## Windows

CMD에서:

```cmd
cd %USERPROFILE%\Desktop
git clone https://github.com/MISEONG0216/molit-autonomous-driving.git
cd molit-autonomous-driving
setup_windows.bat
check_windows.bat
```

실행:

```cmd
run_windows.bat gps
run_windows.bat imu
run_windows.bat pure-pursuit
run_windows.bat pure-pursuit-drive
```

Windows의 자세한 단계별 설명과 실제 검증 출력은 [`WINDOWS_GUIDE.md`](WINDOWS_GUIDE.md)를 참고합니다.

---

# 4. MORAI 네트워크 설정

MORAI Simulator와 Python을 같은 PC에서 실행하는 기준입니다.

```text
GPS
Destination IP   : 127.0.0.1
Destination Port : 9201

IMU
Destination IP   : 127.0.0.1
Destination Port : 1111

CtrlCmd
Host IP          : 127.0.0.1
Host Port        : 9093

Ego
AV / ExternalCtrl
```

---

# 5. Pure Pursuit 실행 전 확인

GPS:

```text
Ubuntu  : ./run.sh gps
Windows : run_windows.bat gps
```

IMU:

```text
Ubuntu  : ./run.sh imu
Windows : run_windows.bat imu
```

GPS와 IMU가 정상일 때 Dry Run을 실행합니다.

```text
Ubuntu  : python3 src/pure_pursuit_udp.py
Windows : run_windows.bat pure-pursuit
```

정상 Dry Run에서는 다음 항목을 확인합니다.

```text
POSITION
YAW
NEAREST
PATH DIST
LOOKAHEAD
TARGET
LOCAL
THETA
STEERING
```

실제 Windows 검증 예시:

```text
POSITION  : x=-130.852, y=-426.817
YAW       : 63.84 deg
NEAREST   : index=3
PATH DIST : 0.230 m
LOOKAHEAD : index=9
STEERING  : -0.03856 rad (-2.21 deg)
```

---

# 6. 실제 주행

현재 검증된 기준 설정:

```text
Target Speed       : 5.0 km/h
Lookahead Distance : 2.5 m
```

Ubuntu:

```bash
python3 src/pure_pursuit_udp.py --drive
```

Windows:

```cmd
run_windows.bat pure-pursuit-drive
```

주행 중 `Ctrl + C`를 누르면 프로그램을 종료하면서 정지 명령을 전송합니다.

---

# 7. 프로젝트 구조

```text
molit-autonomous-driving/
├── README.md
├── WINDOWS_GUIDE.md
├── TEAM_WORKFLOW.md
├── requirements.txt
│
├── setup.sh
├── check.sh
├── run.sh
│
├── setup_windows.bat
├── check_windows.bat
├── run_windows.bat
│
├── config/
│   └── network.json
│
├── path/
│   └── 2026_molit_comp_global_path.txt
│
├── src/
│   ├── morai_common.py
│   ├── gps_test.py
│   ├── imu_test.py
│   ├── ctrl_test.py
│   └── pure_pursuit_udp.py
│
└── third_party/
    └── MORAI-NetworkModule/
        # setup 스크립트 실행 시 자동 생성
        # Git 추적 제외
```

핵심 알고리즘은 운영체제별로 나누지 않고 공통 `src/` 파일을 사용합니다.

---

# 8. 팀 협업 규칙

`main`은 실제 주행이 검증된 기준 버전으로 유지합니다.

작업 시작:

```text
git checkout main
git pull origin main
git checkout -b 작업명-이름
```

예시:

```text
speed-straight-miseong
speed-corner-chulsu
curvature-analysis-younghee
dynamic-lfd-minsu
```

작업 후:

```text
git status
git add .
git commit -m "작업 내용"
git push
```

그 후 GitHub에서 Pull Request를 생성합니다.

```text
개인 브랜치
      ↓
MORAI 테스트
      ↓
Pull Request
      ↓
팀원 검토
      ↓
main 병합
```

MORAI 실제 주행 테스트가 완료되지 않은 코드는 가급적 `main`에 병합하지 않습니다.

자세한 협업 절차는 [`TEAM_WORKFLOW.md`](TEAM_WORKFLOW.md)를 참고합니다.

---

# 9. 구간별 최적화 권장 역할

현재 다음 목표는 규정 범위 안에서 안정성을 유지하면서 한 바퀴 주행 시간을 줄이는 것입니다.

### A. 전역경로 분석

- waypoint 분석
- 직선 / 완만한 커브 / 급커브 구간 분류
- 곡률 계산
- 속도 변경 기준 후보 생성

### B. 직선 구간 속도 최적화

- 직선 안정 속도 탐색
- 가속 시점 확인
- 커브 진입 전 감속 시작 위치 확인

### C. 커브 구간 최적화

- 급커브 안전 속도 탐색
- Lookahead Distance 조정
- 경로 이탈량 확인

### D. 전체 주행 통합

- 랩타임 비교
- 최대 `PATH DIST` 비교
- 경로 이탈 여부 확인
- 팀원별 최적 설정 통합

---

# 10. 테스트 결과 기록

각 실험에서는 최소한 다음 내용을 기록합니다.

```text
브랜치 이름:
테스트 날짜:
변경 내용:
구간:
설정 속도:
Lookahead:
완주 여부:
최대 PATH DIST:
경로 이탈 여부:
랩타임:
특이사항:
```

기준 버전:

```text
속도       : 5.0 km/h
Lookahead  : 2.5 m
Ubuntu     : 1바퀴 완주 성공
Windows    : 1바퀴 완주 성공
```

이 기준을 보존한 상태에서 개인 브랜치에서 최적화를 진행합니다.

---

# 11. 문제 해결

GPS가 들어오지 않을 때:

```text
Destination IP   = 127.0.0.1
Destination Port = 9201
```

IMU가 들어오지 않을 때:

```text
Destination IP   = 127.0.0.1
Destination Port = 1111
```

Dry Run은 정상인데 차량이 움직이지 않을 때:

```text
Ego = AV / ExternalCtrl
CtrlCmd Host IP = 127.0.0.1
CtrlCmd Host Port = 9093
```

Windows 설치/실행 문제는 [`WINDOWS_GUIDE.md`](WINDOWS_GUIDE.md)를 우선 확인합니다.

---

# 12. 기준 버전 보호

현재 `5.0 km/h + LFD 2.5 m` Pure Pursuit는 Ubuntu와 Windows에서 모두 전역경로 1바퀴 완주가 확인된 기준 버전입니다.

따라서 다음 작업을 할 때 기존 값을 이유 없이 삭제하거나 덮어쓰지 않습니다.

```text
구간별 속도 프로파일
곡률 기반 감속
동적 Lookahead
제동 로직
랩타임 최적화
```

새로운 설정은 개인 브랜치에서 MORAI 검증 후 Pull Request로 반영합니다.
