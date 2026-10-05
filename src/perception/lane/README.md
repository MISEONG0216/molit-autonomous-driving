# Lane Detection

**Input:** Camera frame  
**Output:** `LaneResult`

1. Camera 수신
2. ROI / 전처리
3. 좌·우 차선 검출
4. 차선 중앙 및 offset 계산
5. Mission Manager/Controller에 결과 전달

초기 목표는 Camera 단독 조향이 아니라 GPS+IMU+Global Path 기반 주행의 횡방향 보조입니다.
