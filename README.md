# MOLIT Autonomous Driving

국토교통부 무인차 대회용 **MORAI 기반 자율주행 프로젝트**입니다.

현재 `main` 브랜치에는 실제 MORAI 환경에서 **전역경로 1바퀴 완주가 확인된 Pure Pursuit 기준 버전**이 저장되어 있습니다.

> 팀 협업의 가장 중요한 원칙: **`main`에서 직접 개발하지 않습니다.**  
> 개인 브랜치에서 수정 → MORAI에서 테스트 → Pull Request → 검토 후 `main` 병합 순서로 진행합니다.

---

## 1. 현재 기준 주행 상태

현재 기준 버전의 설정은 다음과 같습니다.

- Pure Pursuit Lookahead Distance: `2.5 m`
- 목표 속도: `5.0 km/h`
- Local Path Size: `50`
- Control Frequency: `15 Hz`
- 전역경로: `path/2026_molit_comp_global_path.txt`
- GPS 좌표 변환: WGS84 → UTM Zone 52 → MORAI Local 좌표
- MORAI UDP 직접 통신 방식

현재 기준 버전은 실제 MORAI 환경에서 **한 바퀴 완주가 확인된 안정 버전**입니다.

구간별 속도 최적화, 곡률 기반 감속, 동적 Lookahead 등의 실험은 반드시 개인 브랜치에서 진행합니다.

---

## 2. 기준 개발 환경

- Ubuntu `22.04`
- Python `3.8.x`
- MORAI SIM: Drive 대회용 시뮬레이터
- MORAI Network Module `24.R2.0`

MORAI Network Module 소스는 저장소에 직접 포함하지 않습니다.

`./setup.sh` 실행 시 MORAI 공식 GitHub 저장소의 `24.R2.0` 버전을 다음 위치에 자동으로 내려받습니다.

```text
third_party/MORAI-NetworkModule
```

---

# 3. 팀원이 처음 프로젝트를 받는 방법

## 3-1. GitHub 저장소 Clone

먼저 저장소 Collaborator 초대를 수락한 뒤 Ubuntu 터미널에서 실행합니다.

```bash
git clone https://github.com/MISEONG0216/molit-autonomous-driving.git
cd molit-autonomous-driving
```

## 3-2. 설치 스크립트 실행

```bash
chmod +x setup.sh check.sh run.sh
./setup.sh
./check.sh
```

`./check.sh` 실행 결과가 모두 `[OK]`로 나오면 기본 환경 준비가 완료된 것입니다.

Ubuntu에 `python3-venv`가 없다면 먼저 설치합니다.

```bash
sudo apt update
sudo apt install -y python3-venv
```

그 후 다시 실행합니다.

```bash
./setup.sh
./check.sh
```

---

# 4. MORAI 실행 및 네트워크 설정

Pure Pursuit 실행 전에 반드시 MORAI Simulator를 먼저 실행하고 **대회 시나리오**를 불러옵니다.

이 프로젝트는 MORAI와 Python 코드가 **같은 Ubuntu PC에서 실행되는 기준**으로 설정되어 있습니다.

따라서 IP는 다음과 같이 사용합니다.

```text
127.0.0.1
```

## 4-1. GPS 설정

MORAI GPS Sensor 설정에서 다음 값을 확인합니다.

```text
Destination IP   : 127.0.0.1
Destination Port : 9201
```

## 4-2. IMU 설정

MORAI IMU Sensor 설정에서 다음 값을 확인합니다.

```text
Destination IP   : 127.0.0.1
Destination Port : 1111
```

특히 IMU의 Destination Port가 다른 값으로 되어 있다면 반드시 `1111`로 변경합니다.

## 4-3. CtrlCmd 설정

MORAI Ego Network / Cmd Control 설정에서 다음 값을 확인합니다.

```text
Host IP   : 127.0.0.1
Host Port : 9093
```

Python 프로그램은 `127.0.0.1:9093`으로 차량 제어 명령을 전송합니다.

## 4-4. Ego 차량 제어 모드

실제 차량을 코드로 움직이려면 Ego Controller를 다음 상태로 전환합니다.

```text
AV / ExternalCtrl
```

이 설정이 되어 있지 않으면 Pure Pursuit 프로그램이 정상 실행되어도 차량이 움직이지 않을 수 있습니다.

---

# 5. Pure Pursuit 실행 전 통신 확인

MORAI를 실행한 뒤 새 터미널에서 프로젝트 폴더로 이동합니다.

```bash
cd ~/molit-autonomous-driving
source .venv/bin/activate
```

## 5-1. GPS 확인

```bash
./run.sh gps
```

정상이라면 `0.000000`이 아닌 실제 위도/경도 값이 계속 출력됩니다.

확인 후:

```text
Ctrl + C
```

## 5-2. IMU 확인

```bash
./run.sh imu
```

정상이라면 quaternion 값과 가속도 값 등이 계속 출력됩니다.

확인 후:

```text
Ctrl + C
```

GPS와 IMU가 모두 정상일 때 Pure Pursuit를 실행합니다.

---

# 6. Pure Pursuit 실행 방법

실제 주행 전에 반드시 **Dry Run**으로 좌표와 경로 계산이 정상인지 확인합니다.

## 6-1. Dry Run

```bash
cd ~/molit-autonomous-driving
source .venv/bin/activate
python3 src/pure_pursuit_udp.py
```

Dry Run에서는 차량 제어 명령을 MORAI로 전송하지 않습니다.

정상 실행 시 다음 정보가 계속 갱신됩니다.

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

### 확인할 항목

- GPS 값이 정상인지
- POSITION 값이 전역경로 부근인지
- YAW 값이 차량 방향과 비슷한지
- `PATH DIST`가 지나치게 크지 않은지
- `LOOKAHEAD : NOT FOUND`가 계속 발생하지 않는지

Dry Run이 정상이라면 종료합니다.

```text
Ctrl + C
```

---

# 7. 실제 Pure Pursuit 주행

MORAI의 Ego 차량이 `AV / ExternalCtrl` 상태인지 다시 확인합니다.

그 후 실행합니다.

```bash
cd ~/molit-autonomous-driving
source .venv/bin/activate
python3 src/pure_pursuit_udp.py --drive
```

정상이라면 터미널에 다음과 같이 표시되며 차량이 전역경로를 따라 움직입니다.

```text
[DRIVE] velocity=5.0 km/h
```

현재 기준 버전은 다음 설정입니다.

```text
Target Speed : 5.0 km/h
Lookahead    : 2.5 m
```

## 차량 정지

실행 중:

```text
Ctrl + C
```

를 누르면 프로그램이 종료되면서 정지 명령을 전송합니다.

---

# 8. 추천 실행 순서

팀원들은 매번 아래 순서대로 진행하면 됩니다.

```text
Ubuntu 부팅
   ↓
MORAI Simulator 실행
   ↓
대회 시나리오 로드
   ↓
GPS Destination Port = 9201 확인
   ↓
IMU Destination Port = 1111 확인
   ↓
CtrlCmd Host Port = 9093 확인
   ↓
Ego = AV / ExternalCtrl
   ↓
프로젝트 가상환경 활성화
   ↓
GPS 테스트
   ↓
IMU 테스트
   ↓
Pure Pursuit Dry Run
   ↓
Pure Pursuit --drive
```

---

# 9. 프로젝트 구조

```text
molit-autonomous-driving/
├── README.md
├── requirements.txt
├── setup.sh
├── check.sh
├── run.sh
├── .gitignore
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
        # setup.sh 실행 시 자동 생성
        # Git 추적 제외
```

---

# 10. 팀 협업 규칙

## 규칙 1. `main`에서 직접 개발하지 않기

`main`은 현재 실제 주행이 검증된 기준 버전입니다.

따라서 새로운 기능이나 속도 튜닝을 `main`에서 바로 수정하지 않습니다.

---

## 규칙 2. 작업 시작 전 최신 main 받기

```bash
git checkout main
git pull origin main
```

---

## 규칙 3. 개인 브랜치 만들기

브랜치 이름은 다음 형식을 권장합니다.

```text
speed-tuning-이름
```

예시:

```bash
git checkout -b speed-tuning-miseong
```

또는 작업 목적을 더 명확하게 표시할 수 있습니다.

```text
speed-straight-miseong
speed-corner-chulsu
curvature-analysis-younghee
dynamic-lfd-minsu
```

브랜치를 처음 만든 뒤 GitHub에 업로드합니다.

```bash
git push -u origin 브랜치이름
```

---

## 규칙 4. 작업 중 다른 사람 코드 가져오기

개인 브랜치에서 작업 중 `main`의 최신 변경 내용을 받고 싶다면:

```bash
git checkout main
git pull origin main

git checkout 내브랜치이름
git merge main
```

충돌이 발생하면 임의로 덮어쓰지 말고 팀원과 확인한 후 해결합니다.

---

## 규칙 5. 작업 저장

작업이 끝나면:

```bash
git status
git add .
git commit -m "작업 내용"
git push
```

Commit 메시지는 어떤 작업을 했는지 알아볼 수 있게 작성합니다.

좋은 예시:

```text
Tune speed for first straight section
Reduce speed before sharp left corner
Add curvature based speed profile
Test dynamic lookahead on corner section
```

피해야 할 예시:

```text
수정
테스트
최종
123
```

---

## 규칙 6. 작업 완료 후 Pull Request

개인 브랜치 테스트가 완료되면 GitHub에서 Pull Request를 생성합니다.

```text
개인 브랜치
      ↓
Pull Request
      ↓
팀원 검토
      ↓
MORAI 실제 테스트 결과 확인
      ↓
main 병합
```

**MORAI 실제 주행 테스트가 완료되지 않은 코드는 가급적 `main`에 병합하지 않습니다.**

---

# 11. 구간별 최적화 권장 역할

현재 프로젝트의 다음 목표는 단순히 차량을 완주시키는 것이 아니라 **규정 범위 안에서 한 바퀴 주행 시간을 줄이는 것**입니다.

팀원이 여러 명이라면 다음처럼 역할을 분담하는 것을 권장합니다.

### A. 전역경로 분석

- waypoint 분석
- 직선 / 완만한 커브 / 급커브 구간 분류
- 곡률 계산
- 속도 변경 기준 후보 생성

### B. 직선 구간 속도 최적화

- 직선에서 안정적으로 낼 수 있는 속도 탐색
- 직선 진입 시 가속 시점 확인
- 커브 진입 전 감속 시작 위치 확인

### C. 커브 구간 최적화

- 급커브 안전 속도 탐색
- Lookahead Distance 조정
- 경로 이탈량 확인

### D. 전체 주행 통합 및 비교

- 랩타임 비교
- 최대 `PATH DIST` 비교
- 경로 이탈 여부 확인
- 각 팀원의 최적 설정 통합

---

# 12. 테스트 결과를 남기는 방법

속도를 높였다고 해서 무조건 좋은 설정은 아닙니다.

각 테스트에서는 최소한 다음 항목을 기록하는 것을 권장합니다.

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

예시:

```text
브랜치 이름: speed-corner-miseong
변경 내용: 좌측 급커브 진입 전 감속
설정 속도: 직선 15 km/h / 커브 7 km/h
Lookahead: 2.5 m
완주 여부: 성공
최대 PATH DIST: 0.92 m
경로 이탈 여부: 없음
```

이렇게 기록하면 팀원별 결과를 객관적으로 비교하기 쉽습니다.

---

# 13. 현재 기준 코드는 반드시 보존

현재 `main`의 Pure Pursuit는 실제 한 바퀴 완주가 확인된 기준 버전입니다.

따라서 구간별 최적화 중 문제가 생겼을 때는 항상 `main` 기준으로 돌아와 비교합니다.

현재 Pure Pursuit 핵심 설정:

```python
VEHICLE_LENGTH = 1.0
LOOKAHEAD_DISTANCE = 2.5
LOCAL_PATH_SIZE = 50
TARGET_SPEED_KMH = 5.0
CONTROL_HZ = 15.0
```

현재 기준 파일:

```text
src/pure_pursuit_udp.py
```

현재 전역경로:

```text
path/2026_molit_comp_global_path.txt
```

---

# 14. 자주 발생하는 문제

## GPS가 모두 0으로 출력되는 경우

MORAI GPS Sensor에서 다음 항목을 확인합니다.

```text
Destination IP   = 127.0.0.1
Destination Port = 9201
```

---

## IMU 값이 나오지 않는 경우

MORAI IMU Sensor에서 다음 항목을 확인합니다.

```text
Destination IP   = 127.0.0.1
Destination Port = 1111
```

---

## Pure Pursuit는 실행되는데 차량이 움직이지 않는 경우

다음을 확인합니다.

```text
Ego Controller = AV / ExternalCtrl
CtrlCmd Host IP = 127.0.0.1
CtrlCmd Host Port = 9093
```

그리고 실제 주행 명령은 반드시 다음과 같이 `--drive` 옵션을 붙여야 합니다.

```bash
python3 src/pure_pursuit_udp.py --drive
```

---

## `ModuleNotFoundError`가 발생하는 경우

프로젝트 루트에서 다시 실행합니다.

```bash
cd ~/molit-autonomous-driving
./setup.sh
./check.sh
source .venv/bin/activate
```

---

## Git Pull 중 충돌이 발생한 경우

충돌 내용을 확인하기 전에 임의로 파일을 삭제하거나 덮어쓰지 않습니다.

```bash
git status
```

결과를 팀원과 공유한 뒤 해결합니다.

---

# 15. 중요 주의사항

- MORAI Simulator 실행 파일 자체는 이 저장소에 포함하지 않습니다.
- 각 팀원은 대회에서 지정된 MORAI Simulator를 별도로 준비해야 합니다.
- MORAI와 Python을 같은 PC에서 실행할 경우 현재 기준 IP는 `127.0.0.1`입니다.
- Pure Pursuit의 `--drive` 옵션은 실제 차량 제어 명령을 전송합니다.
- 실제 테스트 전 반드시 주변 차량 상태와 시뮬레이터 상태를 확인합니다.
- 새 Python 라이브러리를 추가한 경우 반드시 `requirements.txt`에도 반영합니다.
- 검증되지 않은 속도/조향 설정을 `main`에 바로 반영하지 않습니다.

---

# 16. 팀 개발 기본 흐름 요약

```text
main 최신화
   ↓
개인 브랜치 생성
   ↓
코드 수정
   ↓
MORAI Dry Run
   ↓
MORAI 실제 주행 테스트
   ↓
결과 기록
   ↓
git add / commit / push
   ↓
Pull Request
   ↓
팀원 검토
   ↓
main 병합
```

현재 `main`은 **안정적으로 완주할 수 있는 기준선**, 개인 브랜치는 **랩타임을 줄이기 위한 실험 공간**으로 운영합니다.
