# Dynamic Obstacle Response

**Input:** tracked `Obstacle[]`  
**Output:** `MissionDecision` 또는 planning request

충돌 위험을 평가해 감속/정지/회피/차선변경 요청을 결정합니다. 초기 구현은 LiDAR 중심이며 필요한 경우 Camera 분류 결과를 추가합니다.
