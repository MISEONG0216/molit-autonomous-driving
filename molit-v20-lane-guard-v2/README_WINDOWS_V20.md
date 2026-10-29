# MORAI v20 Windows 실행 안내 (2026-10-29)

이 압축파일에는 원본 v20의 코드/모델/경로를 유지하면서 Windows 시작 스크립트 3개가 추가되었습니다.

**원본 코드 동작을 검증했다는 뜻은 아닙니다.** Python 단위 테스트는 통과했으나 Windows에서 실제 MORAI UDP 및 인식 모델 동작을 확인하지는 못했습니다.

## 0. 준비

- Windows 10/11, 대회용 MORAI, Git for Windows, Python 3.11.9 (64비트), PowerShell
- ZIP 전체를 풀고 폴더 내부에서 PowerShell/VS Code 터미널 열기
- 같은 컴퓨터에서 MORAI를 실행하는 경우 Control IP 기본값 127.0.0.1
- 서로 다른 컴퓨터에서 MORAI를 돌리면 `-ControlIP MORAI_PC_IP` 인자를 사용하세요.

## 1. 설치 (프로젝트 폴더의 PowerShell)

```powershell
py -3.11 --version
Set-ExecutionPolicy -Scope Process -ExecutionPolicy Bypass
.\setup_v20_windows.ps1 -ControlIP 127.0.0.1
```

스크립트는 `.venv` 생성, requirements 설치, MORAI NetworkModule 24.R2.0 다운로드, scikit-learn 설치, 통합 제어 IP 수정, 단위 테스트를 진행합니다.
가상환경이 다른 Python 버전으로 이미 만들어져 있다면 삭제하지 않고 실행을 멈춥니다. 이전 가상환경을 보관한 뒤 다시 실행하세요.

## 2. 센서 점검 (각 명령 실행 후 Ctrl+C)

```powershell
.\run_v20_windows.ps1 -Mode gps
.\run_v20_windows.ps1 -Mode imu
.\run_v20_windows.ps1 -Mode status
```

MORAI UDP 수신 Destination: GPS 9201, IMU 1111, Competition status 1909,
신호등 카메라 9291, LiDAR 1116, 측면 카메라 9293, 후방 카메라 1118.
EgoCtrlCmd 수신 Host 9093. MORAI Ego를 AV/ExternalCtrl로 지정해야 합니다.
포트별 IP는 'MORAI가 전송할 컴퓨터의 실제 IP'이며 같은 컴퓨터이면 127.0.0.1을 사용합니다.
`config/integration_network.json`의 `control.target_ip`를 꼭 확인하세요.

## 3. 통합 LiDAR + 신호등 실행 (서로 다른 PowerShell 터미널 3개)

1번 터미널, 신호등 모델/카메라:

```powershell
.\run_v20_windows.ps1 -Mode traffic
```

2번 터미널, LiDAR 정적 장애물 워커:

```powershell
.\run_v20_windows.ps1 -Mode lidar
```

3번 터미널, 제어 명령을 보내지 않는 DRY-RUN:

```powershell
.\run_v20_windows.ps1 -Mode dry -Traffic -Static -MaxSpeed 10
```

로그와 센서 확인 후 **낮은 속도의 안전한 빈 시나리오에서만** 실제 제어 테스트:

```powershell
.\run_v20_windows.ps1 -Mode drive -Traffic -Static -StaticControl -MaxSpeed 10 -ConfirmDrive
```

**경고:** `-MaxSpeed 10`은 정적 장애물 회피 25 km/h 상한을 10 km/h로 낮춰 검증하는 용도입니다. 전체 경로 속도 상한을 낮추는 것이며 실제 속도/안전성을 보장하지 않습니다.

## 4. 합류 감지까지 켜기

4번 터미널:

```powershell
.\run_v20_windows.ps1 -Mode merge
```

모니터링만 추가한 DRY-RUN:

```powershell
.\run_v20_windows.ps1 -Mode dry -Traffic -Static -Merge -MaxSpeed 10
```

합류 위험에 따라 제동·감속을 실제 적용하려면 검증 후 `-MergeControl` 추가. MiDaS는 첫 실행에 인터넷 다운로드/캐시가 필요할 수 있습니다.

## 5. 주의사항

- `run_windows.ps1 main`과 `src/main.py`는 v20 통합 제어가 아닙니다.
- `main_integrated_v1.py` 이외의 다른 EgoCtrlCmd 송신 코드를 함께 실행하지 마세요.
- WP590~760에서만 정적 장애물 회피 임무가 활성화됩니다. 이미 시작한 잠금 경로는 구간 밖에서도 유지될 수 있습니다.
- V20 문서상 57 km/h로 15m 앞 장애물을 만났을 때 안전하게 25 km/h로 감속하는 기능은 검증되지 않았습니다.
- LiDAR worker와 신호등 모델은 각각 정상 실행 로그/영상 수신이 확인되어야 합니다.
- 이 키트는 기존 저장소의 실행/설치 환경을 쉽게 구성하는 것이며 실시간 차량 검증이나 GUI 환경 설치를 대신할 수 없습니다.
