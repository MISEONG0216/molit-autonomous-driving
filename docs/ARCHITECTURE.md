# Autonomous Driving Architecture

## 목표 구조

```text
GPS / IMU --------> Localization ----┐
Camera -----------> Lane ------------|
Camera -----------> Traffic Light ---|--> Mission Manager
LiDAR ------------> Obstacles -------|         |
Vehicle Status ----------------------┘         v
                                            Planning
                                               |
                                    Global / Local Path
                                               |
                                            Control
                              Pure Pursuit + Speed Control
                                               |
                                            CtrlCmd
                                               |
                                             MORAI
```

## 핵심 원칙

1. 대회 규정집 v1.1을 최우선으로 준수한다.
2. 센서 IP/PORT는 알고리즘 코드에 하드코딩하지 않고 `config/`에서 관리한다.
3. 미션 코드는 인식/판단 데이터만 전달하고 MORAI 차량을 직접 제어하지 않는다.
4. 최종 Steer/Accel/Brake 및 CtrlCmd 송신은 control 계층 하나에서 담당한다.
5. Sensor Driver / Perception / Planning / Control을 분리한다.
6. 각 모듈은 독립 DRY-RUN 테스트가 가능하도록 한다.
7. 기능 완성 시 브랜치 README에 환경, 라이브러리, 센서 설정, 실행법, 출력, 알려진 문제를 기록한다.
8. 검증된 모듈만 Mission Manager를 통해 통합한다.

## 권장 브랜치

- `feature/lane-detection`
- `feature/traffic-light`
- `feature/lidar-receiver`
- `feature/lidar-detection`
- `feature/lidar-tracking`
- `feature/static-avoidance`
- `feature/dynamic-avoidance`
- `feature/mission-manager`

## 개발 순서

현재 BASE 주행 보존 → network/sensor config 정리 → 차선 → 신호등 → LiDAR 공통 파이프라인 → 정적 회피 → 동적 대응 → Mission Manager → 전체 통합 → 주행 로그 기반 최적화
