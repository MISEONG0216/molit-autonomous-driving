# MORAI v20 V2 — 다른 노트북 이전 및 GitHub 새 브랜치 업로드

본 패키지는 대화에서 제공된 v20 Windows Ready Kit + 차선 보정 V2 패치로 구성한 **독립 실행용 소스 스냅샷**입니다. 사용자의 노트북에만 남아 있는 수동 변경 사항이 있다면 포함되지 않았을 수 있습니다.

**중요**: 여기에서 GitHub 원격 업로드는 실행하지 않았습니다. `publish_to_github.ps1`을 기존 노트북의 PowerShell에서 직접 실행해야 합니다. GitHub 접근 권한 및 인증이 필요합니다. 모델 가중치(`src/trafficlight/*.ckpt`, `*.pt`)를 공개 저장소에 업로드할 권리가 있는지 확인하세요.

## A. 기존 노트북에서 GitHub 브랜치 발행

1. 이 ZIP을 `C:\MORAI\transfer_kit` 등에 압축 해제하고 `molit-v20-lane-guard-v2` 폴더로 이동.
2. Git for Windows 설치 후 GitHub 계정으로 인증할 수 있어야 함.
3. PowerShell:

```powershell
cd "C:\MORAI\transfer_kit\molit-v20-lane-guard-v2"
Set-ExecutionPolicy -Scope Process -ExecutionPolicy Bypass
.\publish_to_github.ps1
```

- 원격 저장소: `https://github.com/MISEONG0216/molit-autonomous-driving.git`
- 기준 브랜치: `lidar_object_detection` (있다는 전제)
- 새 브랜치: `v20-lane-guard-v2-20261030`
- 별도 임시 클론: `C:\MORAI\v20-github-publish` (이미 있으면 안전상 중단)
- **기존 작업 폴더와 원래 브랜치의 파일은 삭제하거나 바꾸지 않음.** 새 브랜치만 독립 배포용 소스 트리로 교체하여 커밋.
- 이미 같은 이름으로 브랜치가 존재하면 다른 이름을 사용:

```powershell
.\publish_to_github.ps1 -NewBranch v20-lane-guard-v2-20261030-b
```

- Git commit에서 `Author identity unknown` 발생하면 이름/이메일 설정 후 `C:\MORAI\v20-github-publish`에서 `git commit`과 `git push` 실행 (기존 clone이 남아 있으니 처음부터 재실행하지 않음).

```powershell
git config --global user.name "YOUR_GITHUB_NAME"
git config --global user.email "YOUR_GITHUB_EMAIL"
cd C:\MORAI\v20-github-publish
git commit -m "Add portable MORAI v20 V2"
git push -u origin v20-lane-guard-v2-20261030
```

GitHub에서 브랜치가 보이면 업로드 성공. **권한 없으면 저장소 관리자에게 협업자 권한을 요청해야 함.**

## B. 새로운 노트북 — 프로그램 준비

- MORAI Simulator (원래 노트북과 같은 시뮬레이터/맵/차량/센서 구성)
- Python **3.11.9 (64bit)**, 설치 시 Python Launcher (`py`) 포함
- Git for Windows, VS Code (선택)
- 가능한 여유 저장 공간과 RAM; CPU로 신호등 YOLO 돌리면 느릴 수 있음
- 인터넷 연결: pip와 MORAI NetworkModule 다운로드에 필요

PowerShell:

```powershell
py -3.11 --version
git --version
New-Item -ItemType Directory -Force C:\MORAI | Out-Null
cd C:\MORAI
git clone --single-branch --branch v20-lane-guard-v2-20261030 https://github.com/MISEONG0216/molit-autonomous-driving.git molit-v20-v2
cd .\molit-v20-v2
Set-ExecutionPolicy -Scope Process -ExecutionPolicy Bypass
$env:PYTHONUTF8 = "1"
.\setup_v20_windows.ps1 -ControlIP 127.0.0.1
.\check_v20_windows.ps1
```

**주의**: 설치 스크립트는 Windows PowerShell 5.1의 UTF-8 BOM 문제를 피하도록 수정되어 있습니다. `Ran 81 tests ... OK`와 `MORAI control destination: 127.0.0.1:9093`을 확인하세요. MORAI와 Python이 다른 노트북에서 실행되면 ControlIP만이 아니라 모든 센서 Destination IP도 Python 노트북의 IP로 수정해야 합니다.

## C. MORAI 센서 연결 설정

| 통신 | MORAI Destination / Python 수신 | 비고 |
| --- | --- | --- |
| GPS | 127.0.0.1:9201 | 기존 GPS HostPort=1111과 IMU 1111이 충돌했으므로 GPS HostPort 1120 추천 |
| IMU | 127.0.0.1:1111 | GPS HostPort과 중복 피하기 |
| Competition Vehicle Status | 127.0.0.1:1909 | Ego network 설정; 별도 상태 퍼블리셔 |
| 전방 신호등 카메라 | 127.0.0.1:9291 | **Camera 1**, Camera 7(1118)이 아님 |
| LiDAR | 127.0.0.1:1116 | 원본 1206 bytes 수신 가능해야 함 |
| 측면 카메라 | 127.0.0.1:9293 | 합류 모듈 테스트 시 |
| 후방 카메라 | 127.0.0.1:1118 | 합류 모듈 테스트 시 |
| EgoCtrlCmd | 127.0.0.1:9093 | Python이 MORAI로 전송, MORAI에서 수신 |

센서 프로필: `sensor_profiles/SURE_sensor4_original.json`과 `sensor_profiles/SURE_sensor4_gps_host1120.json` 포함. **MORAI 센서 편집 모드에서 Load** 후 각 카메라 `Connect`, 일시정지 해제. 모라이 카메라 프리뷰가 `Camera_7`이라면 전방 신호등 카메라와는 다를 수 있음. 먼저 Camera 1의 영상을 확인하세요.

GPS HostPort 1120은 실제 충돌하지 않는 빈 포트라는 가정하에 만든 프로필입니다. MORAI가 이미 개별 센서 설정을 갖고 있으면 새 파일로 덮어쓰기 전 백업하세요.

## D. 다른 노트북 실행 (각각 새 PowerShell 3개)

각 창에서 먼저:

```powershell
cd C:\MORAI\molit-v20-v2
Set-ExecutionPolicy -Scope Process -ExecutionPolicy Bypass
$env:PYTHONUTF8 = "1"
```

**터미널 1: 신호등 카메라**
```powershell
.\run_v20_windows.ps1 -Mode traffic
```

**터미널 2: LiDAR**
```powershell
.\run_v20_windows.ps1 -Mode lidar
```

**터미널 3: 메인 DRY-RUN (차 안 움직임)**
```powershell
.\run_v20_windows.ps1 -Mode dry -Traffic -Static -MaxSpeed 10
```

이때 `TRAFFIC RAW : ... age=숫자`, `STATIC ... age=숫자`, `COMP SPEED : ...` 출력 확인. `WAIT Competition Vehicle Status`면 MORAI `1909` 설정과 중복 포트 사용 여부 확인. 오류 없으면 메인만 Ctrl+C로 중지, 카메라와 LiDAR 계속 실행.

**저속 실제 주행 (차량 제어 전송)**
```powershell
.\run_v20_windows.ps1 -Mode drive -Traffic -Static -StaticControl -MaxSpeed 10 -ConfirmDrive
```

**57km/h 목표속도 상한 (저속 검증 후)**
```powershell
.\run_v20_windows.ps1 -Mode drive -Traffic -Static -StaticControl -MaxSpeed 57 -ConfirmDrive
```

`-MaxSpeed 57`은 실제 속도가 항상 57이라는 뜻이 아님. V2 차선 보정 WP220~320 최대 10km/h, 급커브 WP235~302 최대 8km/h, LiDAR 회피 중 최대 25km/h로 추가 제한될 수 있음. `config/lane_guard.json`의 GPS 경로 오른쪽 보정은 **실제 차선 검출이 아님**, 57km/h 상태에서 차선 유지나 장애물 회피가 검증된 것은 아님. 변경 직후 반드시 시뮬레이터 저속 검증 권장.

## E. 로그 CSV 위치

```powershell
explorer "C:\MORAI\molit-v20-v2\logs"
```

`integrated_v1_날짜_시간.csv` (통합 주행), `static_worker_날짜_시간.csv` (LiDAR 작업 로그). 추후 문제가 있으면 생성된 최신 CSV를 공유하세요. **logs 폴더는 Git에 업로드되지 않음.**

## F. 자주 만나는 오류

- `WinError 10013` (IMU): GPS HostPort 1111 ↔ IMU Destination 1111 충돌 가능. GPS HostPort를 1120으로 변경, IMU destination 1111 유지.
- `WinError 10048`: 같은 수신 포트에 프로그램 2개 띄우지 않기. `-Mode status`와 메인을 동시에 실행하지 않기.
- 카메라 `WAIT`: Camera 1 IP 127.0.0.1 Port 9291 / Sensor Connect / MORAI 일시정지 확인.
- `Torch ... +cpu`, `CUDA False`: GPU 가속은 없다는 뜻, CPU 실행 가능하지만 프레임 속도 확인.
- Git 인증 오류: Git Credential Manager 브라우저 로그인 또는 저장소 권한 확인. 비밀번호·토큰은 소스 파일에 저장하지 않기.
- GitHub 코드가 올라가도 MORAI 설치 자체는 별도로 필요하고, Windows의 가상환경 `.venv`는 머신마다 새로 만들어야 함.
