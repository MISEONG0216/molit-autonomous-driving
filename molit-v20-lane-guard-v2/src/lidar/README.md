# LiDAR Module #1 - MORAI VLP16 UDP Receiver

MORAI VLP16 LiDAR의 UDP 데이터를 수신·파싱하고 기본 전처리를 수행한 뒤, 후속 LiDAR 모듈에 Point Cloud를 전달하는 모듈입니다.

## 담당 범위

처리 파이프라인:

```text
MORAI VLP16
→ UDP Packet Receive
→ Packet Validation
→ Azimuth 기반 Scan Segmentation
→ Scan Integrity Check
→ VLP16 Parsing
→ Invalid Point Removal
→ ROI Filtering
→ Point Cloud (N, 4)
```

Module #1에서는 Ground Removal, Clustering, Object Detection/Tracking, 정적·동적 장애물 판단, Local Path 생성, 차량 제어를 수행하지 않습니다.

## 파일 구성

```text
lidar/
├── README.md
├── lidar_parser.py
├── lidar_preprocess.py
├── lidar_receiver.py
└── tests/
    ├── test_lidar_freshness.py
    └── test_lidar_restart.py
```

- `lidar_parser.py`: MORAI VLP16 UDP payload → XYZ + Intensity 변환
- `lidar_preprocess.py`: NaN/Inf 및 센서 원점 근처 무효점 제거, ROI filtering
- `lidar_receiver.py`: UDP 수신, scan 분리/검사, background thread, freshness 및 restart 관리

## 출력 인터페이스

Module #1의 최종 출력은 다음 형식입니다.

```python
np.ndarray
shape = (N, 4)
dtype = float32
```

열 구성:

```text
[:, 0] = X [m]
[:, 1] = Y [m]
[:, 2] = Z [m]
[:, 3] = Intensity
```

좌표계:

```text
+X = Right
-X = Left
+Y = Forward
-Y = Backward
+Z = Up
-Z = Down
```

후속 모듈은 일반적인 ROS REP-103 `base_link` 좌표계와 같다고 가정하지 말고 위 인터페이스를 기준으로 사용해야 합니다.

## 기본 사용

```python
from lidar_receiver import LidarReceiver

receiver = LidarReceiver()

try:
    receiver.start()

    while True:
        points, stats = receiver.get_latest()

        if points is None or not stats["fresh"]:
            continue

        # points: np.ndarray (N, 4)
        # 후속 LiDAR 모듈에 전달

finally:
    receiver.stop()
```

Module #2 예시:

```python
points, stats = receiver.get_latest()

if points is not None and stats["fresh"]:
    obstacles = detector.detect(points)
```

Module #2에서는 UDP packet 구조를 다시 처리하지 않고 `points`를 입력으로 사용합니다.

## UDP 설정

현재 동일 PC에서 MORAI와 Python 코드를 실행하는 개발 환경:

```text
Receiver IP   = 127.0.0.1
Receiver Port = 1116

MORAI hostPort        = 1115
MORAI destinationIP   = 127.0.0.1
MORAI destinationPort = 1116
```

MORAI와 알고리즘을 서로 다른 PC에서 실행할 경우 실제 네트워크 구성에 맞게 IP를 변경해야 합니다.

## 현재 개발용 ROI

```text
X: -10.0 m ~ +10.0 m
Y:  -5.0 m ~ +30.0 m
Z:  -2.5 m ~ +2.0 m
```

이 값은 개발/테스트용이며 대회 공식 고정값이 아닙니다.

## Scan Segmentation

UDP packet의 첫 번째 azimuth를 사용하며 다음 조건으로 360° → 0° wrap을 판단합니다.

```python
previous_azimuth - current_azimuth > 180.0
```

현재 구현은 **packet-level scan segmentation**입니다. 따라서 개별 laser firing 수준에서 정확히 0°를 경계로 자른 scan이라는 의미는 아닙니다.

## Scan Integrity

현재 MORAI VLP16 환경에서 실제 측정한 정상 scan은 약 75 UDP packets였습니다.

```text
EXPECTED_PACKETS = 75
MIN_SCAN_PACKETS = 70
MAX_SCAN_PACKETS = 80

75 packets      → EXACT
70 ~ 80 packets → ACCEPTABLE
범위 밖         → INVALID
```

75 packets/scan은 현재 MORAI 환경에서 측정한 **경험적 값**이며 VLP16의 공식 packet-per-scan 사양이 아닙니다. Packet count 검사는 완전한 packet-loss detector가 아니라 scan sanity/integrity 검사입니다.

## Fresh / Stale

오래된 Point Cloud를 현재 장애물 정보로 사용하지 않도록 freshness를 제공합니다.

```text
STALE_TIMEOUT_SEC = 3.0
```

```python
points, stats = receiver.get_latest()

if points is not None and stats["fresh"]:
    # 최신 데이터 사용
    pass
```

3초는 현재 개발 환경의 소프트웨어 설정값이며 LiDAR 공식 사양이 아닙니다.

## Safety Cap

Azimuth wrap이 비정상적으로 오랫동안 검출되지 않을 때 scan buffer가 무한히 증가하지 않도록 보호합니다.

```text
MAX_ACCUMULATED_PACKETS = 200
```

200 packets까지 정상 wrap이 검출되지 않으면 partial scan을 폐기하고 재동기화합니다. 이 값도 LiDAR 공식 사양이 아닌 소프트웨어 보호 설정입니다.

## 검증 결과

현재 MORAI VLP16 환경의 정상 scan 예:

```text
packets      = 75
packet_delta = 0
raw points   = 28800
ROI points   = 4810
quality      = EXACT
```

검증한 항목:

- UDP receive 및 1206-byte packet validation
- VLP16 payload parsing, XYZ conversion, Intensity extraction
- Invalid point removal 및 ROI filtering
- Continuous scan receive 및 packet-level azimuth segmentation
- Scan integrity check
- Background receiver 및 latest-frame access
- Fresh / Stale detection
- Receiver stop / restart
- Restart 시 이전 frame 초기화
- 비정상 scan 누적 safety cap

## 테스트

문법 검사:

```bash
cd src/lidar
python3 -m py_compile lidar_parser.py lidar_preprocess.py lidar_receiver.py
```

Receiver:

```bash
python3 lidar_receiver.py
```

Freshness:

```bash
python3 tests/test_lidar_freshness.py
```

Restart:

```bash
python3 tests/test_lidar_restart.py
```

## 구현 기준

UDP packet 구조와 distance/intensity/azimuth 처리 방식은 MORAI SensorExample의 `LIDARprocess.py` 구현을 기준으로 작성했습니다.

VLP16의 16개 vertical laser angle은 VLP-16 제조사 User Manual의 firing order를 기준으로 사용했습니다.

다음 값들은 공식 LiDAR 사양값이 아닙니다.

- 현재 ROI
- 75 packets/scan
- 70~80 packet integrity 범위
- 3초 stale timeout
- 200 packet safety cap

## Module #2 전달 계약

```text
type  : numpy.ndarray
shape : (N, 4)
dtype : float32

column 0 : X [m]          (+Right / -Left)
column 1 : Y [m]          (+Forward / -Backward)
column 2 : Z [m]          (+Up / -Down)
column 3 : Intensity
```

Module #2의 권장 처리 경계:

```text
Module #1 Point Cloud
        ↓
Ground Removal
        ↓
Clustering
        ↓
Object Detection
        ↓
Obstacle[]
```

Module #2가 Module #1 내부의 UDP 수신 및 packet parsing 구현에 직접 의존하지 않는 것을 원칙으로 합니다.
