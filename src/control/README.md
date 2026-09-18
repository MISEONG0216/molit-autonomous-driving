# Control

차량을 실제로 움직이는 최종 제어 계층입니다.

입력:
- Global/Local target path
- target speed
- lane correction 등

출력:
- Steer
- Accel
- Brake
- MORAI CtrlCmd

**원칙:** 프로젝트에서 실제 CtrlCmd 송신 책임은 이 계층으로 단일화합니다.
현재 Pure Pursuit 코드는 검증을 유지하면서 이후 단계적으로 이 구조로 이전합니다.
