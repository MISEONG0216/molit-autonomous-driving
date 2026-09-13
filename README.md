# MOLIT Autonomous Driving

국토교통부 무인차 대회용 MORAI 기반 자율주행 프로젝트입니다.

## 목표

- MORAI Simulator와 UDP 통신
- GPS/IMU 기반 차량 위치 및 자세 추정
- 전역경로 기반 Pure Pursuit 주행
- 팀원 PC에서도 동일한 환경을 쉽게 재현

## 기준 환경

- Ubuntu 22.04
- Python 3.8.x
- MORAI SIM: Drive 대회용 시뮬레이터
- MORAI Network Module `24.R2.0`

MORAI Network Module 소스는 본 저장소에 복사하지 않습니다. `./setup.sh` 실행 시 MORAI 공식 GitHub 저장소의 `24.R2.0` 버전을 `third_party/MORAI-NetworkModule`에 자동으로 내려받습니다.

## 현재 확인된 UDP 설정

- GPS: `127.0.0.1:9201`
- IMU: `127.0.0.1:1111`
- Ego CtrlCmd: `127.0.0.1:9093`

위 포트는 현재 개발 PC에서 실제 사용 중인 설정입니다. 다른 PC에서는 MORAI Network Settings와 `config/network.json`을 동일하게 맞춰야 합니다.

## 최초 설치

```bash
git clone https://github.com/MISEONG0216/molit-autonomous-driving.git
cd molit-autonomous-driving
chmod +x setup.sh check.sh run.sh
./setup.sh
./check.sh
```

Ubuntu에서 `python3-venv`가 없다면 다음 명령을 먼저 실행합니다.

```bash
sudo apt update
sudo apt install -y python3-venv
```

## 통신 테스트

MORAI Simulator를 실행하고 대회 시나리오 및 UDP 센서 설정을 완료한 뒤 테스트합니다.

```bash
./run.sh gps
./run.sh imu
```

차량 제어 테스트는 Ego Controller를 `AV - ExternalCtrl`로 전환한 뒤 실행합니다.

```bash
./run.sh ctrl
```

현재 제어 테스트는 `config/network.json`의 목표 속도(기본 5 km/h)로 3초간 직진 명령을 보내고 정지 명령을 전송합니다.

## 프로젝트 구조

```text
molit-autonomous-driving/
├── README.md
├── requirements.txt
├── setup.sh
├── check.sh
├── run.sh
├── .gitignore
├── config/
│   └── network.json
├── path/
├── src/
│   ├── morai_common.py
│   ├── gps_test.py
│   ├── imu_test.py
│   └── ctrl_test.py
└── third_party/
    └── MORAI-NetworkModule/   # setup.sh가 자동 생성, Git 추적 제외
```

## MORAI 실행 전 확인

1. MORAI Simulator 실행
2. 대회 시나리오 로드
3. GPS UDP 포트 `9201` 확인
4. IMU UDP 포트 `1111` 확인
5. CtrlCmd UDP 포트 `9093` 확인
6. Ego Controller를 `AV - ExternalCtrl` 상태로 전환
7. `./run.sh gps`, `./run.sh imu`로 센서 통신 확인
8. `./run.sh ctrl`로 저속 제어 확인
9. 이후 Pure Pursuit 통합 실행

## 개발 규칙

새 Python 라이브러리를 설치했다면 반드시 `requirements.txt`에도 추가합니다.

하루 개발 종료 시:

```bash
git add .
git commit -m "작업 내용"
git push
```

다른 팀원은 다음 명령으로 최신 버전을 받을 수 있습니다.

```bash
git pull
```

## 주의

MORAI Simulator 실행 파일 자체는 이 저장소에 포함하지 않습니다. 각 팀원은 대회에서 지정된 MORAI 시뮬레이터를 별도로 준비해야 합니다.
