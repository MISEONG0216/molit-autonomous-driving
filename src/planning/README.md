# Planning

Perception 결과와 Mission Manager의 행동 결정을 실제 추종 가능한 경로/목표값으로 변환합니다.

- `static_avoidance/`: 정적 장애물 Local Path 생성 및 Global Path 복귀
- `dynamic_avoidance/`: 동적 장애물 충돌위험 기반 감속/정지/차선변경 요청

Planning 모듈도 MORAI CtrlCmd를 직접 송신하지 않습니다.
