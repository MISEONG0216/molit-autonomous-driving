# Perception

센서 데이터를 차량 제어에 사용할 수 있는 인식 결과로 변환합니다.

## Modules
- `lane/`: Camera 기반 차선 검출 및 Lane Offset
- `traffic_light/`: Camera 기반 RED/GREEN/LEFT 등 신호 상태
- `lidar/`: Point Cloud 전처리, clustering, obstacle tracking

## Rule
Perception 코드는 `Accel / Brake / Steer / CtrlCmd`를 직접 송신하지 않습니다.
결과는 `src/interfaces/models.py`의 공통 데이터 형식으로 상위 계층에 전달합니다.
