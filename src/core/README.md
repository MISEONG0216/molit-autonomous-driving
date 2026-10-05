# Core

통합 실행 계층입니다.

- `mission_manager.py`: 각 perception/planning 모듈의 결과를 종합해 행동을 결정
- 최종적으로 차량을 직접 제어하는 CtrlCmd 송신은 control 계층 한 곳에서만 수행
- 개별 미션 모듈은 MORAI로 Accel/Brake/Steer를 직접 송신하지 않음
