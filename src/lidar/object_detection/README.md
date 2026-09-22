# LiDAR Object Detection & Tracking

2026 대학생 AI/SW 모빌리티 경진대회를 위한 LiDAR 객체 검출 및 추적 모듈입니다.

현재 모듈은 LiDAR Point Cloud를 입력받아 장애물을 검출하고,
프레임 간 객체를 추적하여 속도 및 동적 객체 여부를 판단합니다.

---

## 전체 구조

```text
LiDAR Point Cloud
        ↓
Ground Removal
        ↓
DBSCAN Clustering
        ↓
Cluster Filtering
        ↓
Obstacle[]
        ↓
Object Tracking
        ↓
Ego Motion Compensation
        ↓
Velocity Estimation
        ↓
Moving Average Filter
        ↓
Dynamic Classification
        ↓
Tracked Obstacle[]

파일 구조
object_detection/
├── README.md
├── obstacle.py
├── object_detector.py
├── tracker.py
└── tests/
    ├── test_pointcloud.py
    ├── test_tracker.py
    ├── test_ego_transform.py
    ├── test_velocity_filter.py
    ├── test_dynamic_classification.py
    ├── test_track_lost.py
    └── test_full_pipeline.py

obstacle.py

객체 정보를 저장하는 Obstacle 데이터 구조입니다.

주요 정보:

cluster_id: 현재 프레임의 DBSCAN Cluster ID
track_id: 프레임 간 유지되는 Tracking ID
x, y: 객체 중심 위치
width, length: 객체 크기
distance: 차량 원점으로부터 거리
velocity_x, velocity_y: 추정 속도
is_dynamic: 동적 객체 여부
confidence: 신뢰도 (현재 미사용)

2. Object Detection

파일:

object_detector.py

처리 과정:

Point Cloud
    ↓
Ground Removal
    ↓
DBSCAN
    ↓
Cluster Filtering
    ↓
Bounding Box
    ↓
Obstacle[]

현재 DBSCAN 기본 설정:

eps = 0.5
min_samples = 5

Cluster Filtering에서는 Point 개수와 객체의 width / length를 이용하여
너무 작은 Cluster 또는 비정상적인 Cluster를 제거합니다.

입력

NumPy Point Cloud:

points.shape == (N, 3)

형식:

[x, y, z]

현재 기준 좌표계:

X : 차량 전방
Y : 차량 좌/우 방향
Z : 높이
단위 : meter
출력
obstacles, non_ground_points, labels = detector.detect(points)

obstacles에는 Filtering을 통과한 Obstacle 객체들이 저장됩니다.

3. Object Tracking

파일:

tracker.py

주요 기능:

Nearest Neighbor 기반 객체 Matching
Track ID 유지
객체 속도 추정
Moving Average 기반 속도 Filtering
Ego Translation 보상
Ego Yaw 보상
연속 Frame 기반 Dynamic Classification
Track Lost 처리
Tracker 입력
tracker.update(
    current_obstacles,
    dt,
    delta_x,
    delta_y,
    delta_yaw
)
current_obstacles: Object Detector 결과
dt: LiDAR Frame 간 시간
delta_x: 자차 X 이동량
delta_y: 자차 Y 이동량
delta_yaw: 자차 Yaw 변화량 (radian)
Tracker 출력

Tracking이 적용된 Obstacle[]을 반환합니다.

각 객체에는 다음 정보가 추가됩니다.

track_id
velocity_x
velocity_y
is_dynamic
Object Detection + Tracking

전체 연결 예시:

obstacles, non_ground_points, labels = detector.detect(points)

tracked_obstacles = tracker.update(
    obstacles,
    dt=dt,
    delta_x=delta_x,
    delta_y=delta_y,
    delta_yaw=delta_yaw
)
테스트

테스트 실행 시 object_detection 폴더에서 실행합니다.

전체 Pipeline 테스트:

PYTHONPATH=. python3 tests/test_full_pipeline.py

전체 테스트:

for test in tests/test_*.py; do
    echo "===== $test ====="
    PYTHONPATH=. python3 "$test"
    echo
done

현재 Synthetic Point Cloud 테스트에서 확인된 기능:

Ground Removal
DBSCAN Object Detection
Cluster Filtering
Track ID 유지
Velocity Estimation
Ego Motion Compensation
Moving Average Velocity Filter
Dynamic Classification
Track Lost / Reconnection
Object Detection + Tracking Full Pipeline
LiDAR Receiver와의 연결

현재 상위 src/lidar에는 LiDAR 수신 및 전처리 모듈이 별도로 존재합니다.

향후 전체 연결 구조:

MORAI LiDAR
     ↓
lidar_receiver.py
     ↓
lidar_parser.py
     ↓
lidar_preprocess.py
     ↓
Point Cloud [N, 3]
     ↓
object_detector.py
     ↓
Obstacle[]
     ↓
tracker.py
     ↓
Tracked Obstacle[]

현재 Object Detection / Tracking 모듈은 Synthetic Point Cloud 환경에서
기능 검증을 완료한 상태입니다.

실제 MORAI LiDAR 데이터와 통합 후에는 Point Cloud 좌표계,
LiDAR 주기, ROI, Ground 기준 및 Tracking 임계값에 대한 추가 검증과
튜닝이 필요합니다.


여기서 마지막 문장이 중요해. 아직 실제 MORAI LiDAR와 통합하지 않았으므로 **"완성됐다"고 과장하지 않고 Synthetic 데이터에서 검증됐다는 상태를 정확히 기록**한 거야.


