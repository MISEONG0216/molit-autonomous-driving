# LiDAR Perception

공통 LiDAR 파이프라인입니다.

1. MORAI LiDAR UDP 수신/파싱
2. 좌표계 확인 및 ROI
3. Ground 제거
4. Clustering
5. 객체 중심/크기 계산
6. Frame 간 Tracking
7. 정적/동적 판단

**Output:** `Obstacle[]`

정적/동적 장애물 미션이 같은 LiDAR perception 결과를 재사용하도록 구현합니다.
