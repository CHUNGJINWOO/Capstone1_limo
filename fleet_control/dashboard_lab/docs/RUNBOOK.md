# dashboard_lab 실행·복구 절차

기준선: `dashboard-lab-offline-20261113-v1`. 이번 정리는 문서·해시 목록만 추가했습니다. 아래 start/stop/복구 명령은 **향후 운영자가 필요할 때** 사용하는 절차이며 이번 작업에서는 실행하지 않았습니다.

## 현재 구성과 파일

|대상|위치/용도|
|---|---|
|개선 UI|http://127.0.0.1:18310/|
|기존 UI 비교 복사본|http://127.0.0.1:18311/|
|native 코덱 계측|loopback TCP18313, 웹 페이지 아님|
|코드|worktree의 `fleet_control/dashboard_lab/`|
|개인 상태|`${DASHBOARD_LAB_STATE:-$HOME/.local/state/capstone-dashboard-lab}`|
|인증|viewer.token: 읽기 / lab.token: 로컬 SIMULATION·REPLAY 조건만 변경|
|로그|optimized.log, baseline.log, browser-telemetry.jsonl|
|프로세스 식별|processes.json의 PID와 /proc 시작 tick 일치 여부|
|복구 유지 대상|tokens, sessions.json, inference.json, recorded_map_depth.json, 측정 results/|

토큰·sessions.json을 출력하거나 문서에 복사하지 않습니다. 상태 폴더0700, 인증/세션 파일0600을 유지합니다. 임시 venv는 `/tmp/capstone-dashboard-lab-venv`이며 재부팅·tmp 정리 후 사라질 수 있습니다. 코드/개인 상태/저장 입력은 별도로 보존해야 합니다. 이 경로는 이 노트북의 검증 환경 기록이고 다른 기기의 고정 경로 가정이 아닙니다.

## 조회와 기존 서비스 보호

worktree 루트에서 실행합니다.

```bash
export PYTHONPATH="$PWD/fleet_control"
export PYTHONNOUSERSITE=1
LAB_PY=/tmp/capstone-dashboard-lab-venv/bin/python
"$LAB_PY" -m dashboard_lab.run_local status
```

status는 로컬 registry PID/start tick만 확인합니다. ROS/SSH/robot에 접속하지 않습니다. HTTP `/health`는 프로세스 생존 확인용이며 센서·RMF 연결 성공을 뜻하지 않습니다. 운영8080/18190, 기존18210/18211은 이 lab 소유가 아닙니다. 포트 충돌 시 소유자 확인 없이 종료하지 않습니다.

## 처음 시작 또는 정상 종료 후 복구

18310/18311/18313이 비어 있고 기존 lab이 STOPPED인 경우에만:

```bash
"$LAB_PY" -m dashboard_lab.run_local start --stay --python "$LAB_PY" --mode SIMULATION
```

기본 JPEG 입력은 사용자 저장 Gazebo replay_frames 폴더입니다. 다른 위치는 `--assets /path/to/jpegs`, 기록 packet은 `--recorded /path/to/recorded.json`으로 지정합니다. JPEG가 없으면 실행 실패하며 가짜 카메라를 자동 생성하지 않습니다. 기본 recorded/inference 파일이 private state에 있으면 사용합니다. 브라우저 로그인 파일 선택에서 viewer.token 또는 lab.token을 선택합니다. 토큰 문자열을 주소·명령행에 넣지 않습니다.

브라우저가 끊겼다면 서버 생존→로그→자산 경로·인증 순으로 확인합니다. 서버가 살아 있으면 먼저 브라우저 재연결을 기다립니다. 연결 복구만으로 source가 LIVE가 되지 않습니다. SIMULATION/REPLAY 배너를 확인합니다.

## lab 서버 크래시 시

`--stay`는 **무조건 자동 재시작 supervisor가 아닙니다**. 자식 하나가 죽으면 finally에서 나머지 등록 자식에도 SIGTERM을 보냅니다. 먼저 두 자식의 status 및 기존 supervisor 종료 여부를 확인합니다. 자식이 남았다면 아래 stop은 PID/start tick이 일치하는 등록 자식에만 SIGTERM을 보냅니다.

```bash
"$LAB_PY" -m dashboard_lab.run_local stop
"$LAB_PY" -m dashboard_lab.run_local status
# 둘 다 STOPPED이고 포트가 비었는지 확인한 뒤 위 start 실행
```

stop은 비동기 SIGTERM이며 종료 완료를 기다려주는 명령이 아닙니다. STOPPED와 포트 해제를 확인한 뒤 재시작합니다. killall/pkill, 포트 기준 무차별 kill, 운영 서버 재시작은 사용하지 않습니다. 포트가 계속 점유되면 로그와 정확한 소유자만 조사하고 중단합니다.

sessions.json을 유지하면 같은 브라우저 세션은 재사용됩니다. 새 media boot에서 이전 프레임·검출은 거부됩니다. 자동 재연결 실패 시 서버·인증 상태를 확인한 뒤 새로고침/재로그인합니다. 임무나 주행 명령을 재전송하는 복구 기능은 없습니다.

## 의존성 소실·오류

venv가 사라졌다면 시스템에 패키지를 설치하지 말고 별도 사용자 venv를 만듭니다. 기존 NumPy/OpenCV/Pillow와 ABI 호환성을 먼저 확인하고 `requirements-lab.txt`의 transport 의존성을 해당 venv에만 설치합니다. 이전 검증은 system-site-packages NumPy1.21/OpenCV4.5를 사용했고 사용자 NumPy2를 PYTHONNOUSERSITE로 제외했습니다. requirements 파일만으로 새 OS에 전체 이미지 처리가 재현된다고 보장하지 않습니다. 이후 새 Python25개·JS6개 검사와 실행 smoke를 다시 수행해야 합니다.

## 종료·결과 보존

화면의 시험을 마친 뒤 해당 lab 터미널 Ctrl-C 또는 등록 자식 stop만 사용합니다. 위 status로 종료를 확인합니다. 토큰/세션은 삭제하지 않고 측정 JSON·로그를 날짜별 results/에 보관합니다. 기존 `docs/evidence/report.json`과 baseline 해시 목록을 새 측정으로 덮어쓰지 않습니다.

오늘 문서 정리에서는 이 절차에 따른 시작·종료·장애 주입을 하지 않았습니다. 이전 검증의 서버 복구15.71초는 수동 재실행 시간을 포함하며 운영 복구 SLA가 아닙니다.
