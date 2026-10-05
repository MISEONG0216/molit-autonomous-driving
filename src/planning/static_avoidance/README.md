# Static Obstacle Avoidance

**Input:** `Obstacle[]`, lane/drivable boundary, current/global path  
**Output:** Local avoidance path

박스형 정적 장애물을 검출한 뒤 차선 경계를 침범하지 않는 회피 경로를 생성하고, 통과 후 Global Path 복귀를 판단합니다.
