# 팀원용 GitHub 협업 가이드

이 문서는 `molit-autonomous-driving` 저장소에서 팀원들이 각자 구간별 속도 및 Pure Pursuit 설정을 최적화할 때 사용하는 작업 절차입니다.

> 기본 원칙: `main` 브랜치는 실제 MORAI에서 한 바퀴 완주가 확인된 기준 버전이므로 직접 수정하지 않습니다.

---

## 1. 처음 한 번만: 저장소 받기

GitHub Collaborator 초대를 수락한 뒤 Ubuntu 터미널에서 실행합니다.

```bash
git clone https://github.com/MISEONG0216/molit-autonomous-driving.git
cd molit-autonomous-driving
chmod +x setup.sh check.sh run.sh
./setup.sh
./check.sh
```

`./check.sh` 결과가 모두 `[OK]`이면 준비 완료입니다.

---

## 2. 작업 시작 전 항상 최신 main 받기

```bash
cd ~/molit-autonomous-driving
git checkout main
git pull origin main
```

이 단계는 다른 팀원이 병합한 최신 코드를 받기 위한 과정입니다.

---

## 3. 자기 브랜치 만들기

브랜치 이름은 작업 내용을 알 수 있게 만듭니다.

권장 형식:

```text
speed-tuning-이름
speed-straight-이름
speed-corner-이름
curvature-analysis-이름
dynamic-lfd-이름
```

예를 들어 팀원 철수가 커브 구간 최적화를 맡는다면:

```bash
git checkout -b speed-corner-chulsu
git push -u origin speed-corner-chulsu
```

이후부터는 해당 브랜치에서만 수정합니다.

현재 브랜치 확인:

```bash
git branch
```

예상 출력:

```text
  main
* speed-corner-chulsu
```

`*`가 붙은 브랜치가 현재 작업 중인 브랜치입니다.

---

# 4. 팀원별 작업 예시

## 예시 A — 직선 구간 속도 최적화 담당

브랜치 생성:

```bash
git checkout main
git pull origin main
git checkout -b speed-straight-miseong
git push -u origin speed-straight-miseong
```

주요 작업:

- 직선 구간에서 속도를 단계적으로 높여 보기
- 커브 진입 전에 감속이 필요한 시점 확인
- 최대 `PATH DIST` 확인
- 경로 이탈 여부 확인
- 랩타임 비교

테스트 예시:

```text
직선 속도: 10 km/h
커브 속도: 5 km/h
Lookahead: 2.5 m
최대 PATH DIST: 0.7 m
경로 이탈: 없음
```

---

## 예시 B — 급커브 최적화 담당

브랜치 생성:

```bash
git checkout main
git pull origin main
git checkout -b speed-corner-chulsu
git push -u origin speed-corner-chulsu
```

주요 작업:

- 급커브 진입 속도 조정
- 감속 시작 waypoint 확인
- Lookahead 변경 시험
- 좌/우 급커브에서 경로 이탈량 비교

테스트 예시:

```text
커브 진입 속도: 7 km/h
Lookahead: 2.5 m
최대 PATH DIST: 0.9 m
완주 여부: 성공
```

---

## 예시 C — 전역경로/곡률 분석 담당

브랜치 생성:

```bash
git checkout main
git pull origin main
git checkout -b curvature-analysis-younghee
git push -u origin curvature-analysis-younghee
```

주요 작업:

- `path/2026_molit_comp_global_path.txt` waypoint 분석
- 직선 / 완만한 커브 / 급커브 구간 구분
- 구간별 waypoint index 기록
- 곡률 또는 방향 변화량을 이용해 속도 후보 생성

이 담당자는 반드시 실제 속도를 바로 높이기보다 먼저 구간 분류 결과를 정리해 팀원과 공유합니다.

---

## 예시 D — Dynamic Lookahead 담당

브랜치 생성:

```bash
git checkout main
git pull origin main
git checkout -b dynamic-lfd-minsu
git push -u origin dynamic-lfd-minsu
```

주요 작업:

- 현재 검증된 `2.5 m`를 기준으로 시작
- 속도나 곡률에 따라 Lookahead가 바뀌는 방식 실험
- 기존 고정 2.5 m와 동일 조건에서 비교
- 급커브에서 `PATH DIST`가 증가하지 않는지 확인

> 검증된 기준값 `2.5 m`는 반드시 비교 기준으로 남겨 둡니다.

---

# 5. MORAI에서 코드 실행하는 순서

팀원은 코드 수정 후 아래 순서대로 테스트합니다.

```text
MORAI 실행
↓
대회 시나리오 로드
↓
GPS Destination Port 9201 확인
↓
IMU Destination Port 1111 확인
↓
CtrlCmd Host Port 9093 확인
↓
Ego 차량 AV / ExternalCtrl
↓
GPS 테스트
↓
IMU 테스트
↓
Pure Pursuit Dry Run
↓
실제 주행
```

터미널:

```bash
cd ~/molit-autonomous-driving
source .venv/bin/activate
```

GPS 확인:

```bash
./run.sh gps
```

IMU 확인:

```bash
./run.sh imu
```

Pure Pursuit Dry Run:

```bash
python3 src/pure_pursuit_udp.py
```

실제 주행:

```bash
python3 src/pure_pursuit_udp.py --drive
```

정지:

```text
Ctrl + C
```

---

# 6. 테스트 후 결과 기록

각 팀원은 최소한 아래 내용을 기록합니다.

```text
브랜치 이름:
담당자:
테스트 날짜:
수정한 파일:
테스트 구간:
직선 속도:
커브 속도:
Lookahead:
최대 PATH DIST:
완주 여부:
경로 이탈 여부:
랩타임:
특이사항:
```

예시:

```text
브랜치 이름: speed-corner-chulsu
담당자: 철수
테스트 날짜: 2026-09-13
수정한 파일: src/pure_pursuit_udp.py
테스트 구간: 첫 번째 좌측 급커브
직선 속도: 12 km/h
커브 속도: 7 km/h
Lookahead: 2.5 m
최대 PATH DIST: 0.84 m
완주 여부: 성공
경로 이탈 여부: 없음
랩타임: 전체 테스트 전이라 미측정
특이사항: 커브 진입 전 감속 필요
```

---

# 7. 작업 저장 및 GitHub 업로드

테스트가 끝나면 변경 내용을 확인합니다.

```bash
git status
```

변경된 파일을 저장합니다.

```bash
git add .
git commit -m "Tune speed for first corner section"
git push
```

처음 만든 브랜치인데 upstream 설정을 하지 않았다면:

```bash
git push -u origin 브랜치이름
```

---

# 8. Pull Request 만드는 방법

GitHub 저장소에 접속합니다.

```text
https://github.com/MISEONG0216/molit-autonomous-driving
```

본인 브랜치를 Push한 뒤 `Compare & pull request`를 선택합니다.

설정:

```text
base: main
compare: 본인 브랜치
```

PR 제목 예시:

```text
Tune speed for first straight section
Optimize speed before sharp left corner
Add curvature based speed profile
Test dynamic lookahead on corner sections
```

PR 본문에는 최소한 다음을 작성합니다.

```text
## 변경 내용
- 어떤 구간을 수정했는지
- 어떤 값을 변경했는지

## 테스트 조건
- 속도:
- Lookahead:
- 테스트 구간:

## 결과
- 최대 PATH DIST:
- 완주 여부:
- 경로 이탈 여부:
- 랩타임:

## 특이사항
- 문제점 또는 다음 테스트가 필요한 부분
```

---

# 9. Pull Request 검토 규칙

PR을 만들었다고 바로 `main`에 병합하지 않습니다.

권장 순서:

```text
개인 브랜치에서 수정
↓
Dry Run 성공
↓
MORAI 실제 주행 성공
↓
테스트 결과 기록
↓
Pull Request
↓
다른 팀원 검토
↓
필요하면 재테스트
↓
main 병합
```

다음 중 하나라도 해당하면 바로 병합하지 않습니다.

- MORAI 실제 테스트를 하지 않음
- 경로 이탈이 발생함
- 기존 baseline보다 명확히 불안정함
- 테스트 조건을 기록하지 않음
- 다른 구간에 미치는 영향을 확인하지 않음

---

# 10. 다른 팀원의 최신 코드가 main에 들어왔을 때

내 브랜치에서 계속 작업하고 있는데 `main`이 변경되었다면:

```bash
git checkout main
git pull origin main

git checkout 내브랜치이름
git merge main
```

예:

```bash
git checkout main
git pull origin main

git checkout speed-corner-chulsu
git merge main
```

충돌이 발생하면 임의로 파일 전체를 덮어쓰지 말고 어떤 변경을 남길지 확인한 뒤 해결합니다.

---

# 11. 절대 하지 말아야 할 것

- `main`에서 바로 속도 값을 수정하고 Push하지 않기
- 테스트하지 않은 코드를 `main`에 병합하지 않기
- 다른 팀원의 브랜치를 임의로 수정하지 않기
- `git push --force`를 함부로 사용하지 않기
- MORAI Simulator 실행 파일을 저장소에 올리지 않기
- baseline 값을 이유 없이 삭제하지 않기

현재 검증 baseline:

```text
Target Speed       : 5.0 km/h
Lookahead Distance : 2.5 m
완주               : 성공
```

---

# 12. 팀 작업 전체 흐름 한눈에 보기

```text
                ┌──────────────┐
                │     main     │
                │ 검증 baseline │
                └──────┬───────┘
                       │
             최신 main에서 분기
                       │
       ┌───────────────┼───────────────┐
       │               │               │
       ▼               ▼               ▼
 speed-straight   speed-corner   curvature-analysis
       │               │               │
       ▼               ▼               ▼
   MORAI 테스트     MORAI 테스트       분석/검증
       │               │               │
       └───────────────┼───────────────┘
                       │
                       ▼
                 Pull Request
                       │
                       ▼
                  팀원 검토
                       │
                       ▼
                 MORAI 재검증
                       │
                       ▼
                      main
```

가장 중요한 기준은 **빠른 코드보다 재현 가능하고 실제 MORAI에서 검증된 코드**입니다.
