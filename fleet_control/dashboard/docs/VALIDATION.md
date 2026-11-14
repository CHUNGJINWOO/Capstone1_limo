# 1차 격리 검증 기록 — 2026-11-13

이 문서는 최초 격리 시험 시점의 기록입니다. 후속 실제 LIMO 시험·승인 배포·실측 결과는 [REAL_LIMO.md](REAL_LIMO.md)에 보존했습니다.

## 변경과 보존

- 신규 worktree `Capstone1_limo`, 브랜치 `codex/dashboard-visualization`, 기준 커밋419e798.
- 추가 범위는 `fleet_control/dashboard/`뿐입니다. 기존 fleet server/Agent/ROS 구동·안전 소스에는 수정이 없습니다. 기존 다른 worktree의 브랜치와 미커밋 변경을 보존했습니다. staging/commit/push/PR을 하지 않았습니다.
- 운영8080은 최초·최종 조사 모두 listener가 없었습니다. 새로 실행하지 않았습니다. 시험18190은 기존 PID7486이 계속 listen하는 것을 확인했고 중단·수정하지 않았습니다.
- 실제 LIMO는 기존 SSH 구성으로 **일회성 읽기 전용 프로세스·ROS graph·메시지 metadata 조사만** 수행했습니다. 기존 Agent wrapper가 관찰됐지만 구독으로 드러난 `/map`, `/odom`, `/scan`, `/tf`, `/tf_static`, `/limo_status`, `/auto_mapping/depth_status`는 조사 시 active publisher가 없었습니다. 토픽 이름 존재만으로 센서 동작을 판정하지 않았습니다.
- 실제 LIMO에 새 브리지를 배포하거나 기존 프로세스·장치·네트워크 설정을 변경/재시작하지 않았습니다. 모터 명령·임무 제출·취소·Gate reset을 보내지 않았습니다.

## 자동 검사

|검사|결과|범위|
|---|---|---|
|dashboard `test_visualization.py`|31/31 PASS|grid/clearing/경계/TF/rolling origin/stale/depth/프레임 key/권한/CSRF/제어 차단|
|dashboard `geometry.test.mjs` (직접 node 실행)|6/6 PASS|origin yaw, rolling TF, 화면 회전 역변환, PNG row, 원본 불변, depth 색상|
|기존 `test_dedupe.py`|14/14 PASS|모의 Agent·임시 DB 중복 방지|
|기존 `test_auth_reload.py`|2/2 PASS|임시 토큰 핫리로드/역할 변경|
|기존 `test_agent_recovery.py`|2/2 PASS|Agent 재시작 journal|
|기존 `test_recovery.py`|14/14 PASS|임시 HTTP/SQLite 장애·복구|
|기존 `test_cancel_results.py`|14/14 PASS|모의 goal handle 취소 결과; ROS 타입만 사용|
|기존 `test_ownership.py`|37/37 PASS|임시 데이터·자동 선택 로컬 포트 소유권/불명확 상태 복구|
|Python AST / JS `node --check`|PASS|새 소스 구문|

신규37개, 기존83개 검사가 통과했습니다. 기존 고정 포트/별도 ROS workspace를 사용하는 `run_checks.py` 전체46개는 이번에 다시 실행하지 않았습니다. 위 분리 회귀 결과를 그46개와 같은 검사라고 주장하지 않습니다. 기존 서버 코드를 바꾸지 않아 ROS 패키지 빌드 변경은 없습니다. PNG 표본을 읽는 시험 코드에서 기존 NumPy/Pillow 조합의 DeprecationWarning 1건이 있었으며 기능 실패는 없었습니다. 새 bridge는 기존 ROS 타입으로 실제 Python import/노드 실행을 검증했습니다.

최초 일반 sandbox HTTP 시험은 socket bind 권한으로 실패했습니다(기능 결함 아님). 허용된 로컬 socket 실행 환경에서 재실행하여31개 모두 통과했습니다. 최초 별도 YOLO는 기존 venv의 user-site `typing_extensions`가 차단돼 시작하지 못했습니다. 기존 검증 venv 정책을 그 **시험 YOLO 자식 프로세스에만** 재사용해 해결했으며 패키지를 설치하지 않았습니다. 시스템 ROS 프로세스는 user-site 차단을 유지합니다.

## 실제 브라우저

In-app Chromium에서 데스크톱1440×960과 모바일390×844를 확인했습니다. 모바일 document clientWidth375/scrollWidth375로 가로 넘침이 없고, 레이어 메뉴는 기본 접힘 및 내부 스크롤을 제공합니다. viewport 시험 설정은 끝난 뒤 reset했습니다.

확인: 로그인 파일 선택(토큰 노출 없음), 다크/라이트 전환, 지도 확대·맞춤·방향키 이동·보기 회전, Costmap 표시 토글/투명도, Depth 거리 범위, 패널 확대/복원, 패널 너비 방향키 조절, 작업/이벤트 전환·접기. 최종 콘솔 error 로그는 없었습니다.

- **실차 API 화면:** Mission Manager CONNECTED, PCU OFFLINE, 기존 map313×224/resolution0.05m/origin yaw0. 저장 지도 STALE 표시. RGB 송신 SENDER_DISCONNECTED이며 과거 영상·지연 수치를 현재 값으로 표시하지 않습니다. 실시간 map 갱신/실제 Depth/Costmap/경로 검증은 수행하지 못했습니다.
- **격리 시험 화면:** 합성 ROS map140×110/resolution0.05/origin yaw12.6°, global/local Costmap, 경로, 후보·명시적 선택 목표, TF 이동 궤적, 합성16UC1 Depth320×240 표시. 합성 데이터 배너를 지속 표시합니다.
- **RGB 시험:** 저장된 Gazebo JPEG를 별도 기존 YOLO 서버로 전송하여 CUDA 추론의 person/chair 결과 표시. 원본 촬영 시각을 복원한 것이 아니라 재생 시계 frame stamp를 명시합니다. 실차 RGB 재검증 결과가 아닙니다.

발견·수정: nanosecond timestamp가 JavaScript53-bit 정수 범위를 넘어서 프레임 정합 검사가 실패했습니다. `_ns` 필드를 응답 JSON에서 10진 문자열로 보존하고 JPEG 헤더와 정확히 비교하도록 수정했습니다. 정합되지 않는 frame은 보류합니다. 누적 `pair_mismatch`24건이 측정됐으며 stale 결과를 다른 영상에 얹어 성공으로 처리하지 않았습니다.

증거 화면은 `screenshots/`에 보관합니다. 시험 화면의 지도·Costmap·Depth는 합성 데이터입니다.

## 장애 주입

등록부의 PID와 `/proc` 시작 시각이 일치하는 **자체 시험 프로세스만** SIGSTOP/SIGCONT했습니다. finally로 복원했습니다. 실제 Wi-Fi/Tailscale 및 실차 프로세스는 건드리지 않았습니다.

|시험|결과|
|---|---|
|합성 ROS publisher 잠시 중단|Costmap/Depth STALE, RGB 계속 fresh; 브라우저 덮개 확인|
|합성 ROS publisher 재개|Costmap/Depth OK 복구|
|저장 RGB sender 잠시 중단|video fresh=false, Depth OK 유지(API 확인)|
|저장 RGB sender 재개|정합된 fresh RGB 자동 복구|
|신규 시험 gateway 일시 중단|4초 HTTP timeout 후 GATEWAY OFFLINE, 전체 과거값 배너/회색 표시·영상 STALE 덮개|
|신규 시험 gateway 재개|브라우저 및 bridge 자동 복구, 별도 사용자 재시작 불필요|

## 자원 측정 (시험 환경 표본)

10초 CPU/RSS 측정입니다. CPU100%는 논리 코어1개이며 시스템 전체 백분율이 아닙니다. RSS는 관측 시점 값으로 장시간 peak가 아닙니다. 브라우저 프로세스 CPU는 별도로 측정하지 않았습니다.

|자체 프로세스|CPU(코어1개 기준)|RSS MiB|
|---|---:|---:|
|live gateway|1.0%|36.1|
|test gateway|4.2%|38.8|
|읽기 전용 시험 bridge|4.8%|66.5|
|합성 ROS publisher|13.0%|64.2|
|저장 JPEG sender|0.4%|17.6|
|별도 실제 CUDA YOLO|10.0%|1738.0|

YOLO 프로세스 GPU 메모리5회 표본은 모두324MiB였습니다. GPU 사용률의 프로세스별 측정은 하지 않았습니다. 실차 CPU·카메라 인코딩 부하·실제 네트워크 대역폭은 미검증입니다.

- RGB 수신 약4.8~5.0fps, YOLO 평균 추론 약8~10ms(저장 Gazebo 프레임). 실제 로봇 RTT는 이 시험에 없으므로 공란입니다.
- 합성 Depth 표시 약1.0~1.2fps, 지도 canvas 약2.0~2.8fps, render p95 약0.7~1.1ms(데스크톱 관측). sensor/bridge는 기본1Hz이며 canvas fps는 ROS 센서 FPS가 아닙니다.
- 시험 gateway 앱 집계: upstream 약155kB/s, 브라우저 전송 총합 약214kB/s(열린 탭 및 probe 포함). NIC 전체 wire bandwidth가 아닙니다.
- live MM poll2Hz, video poll약4.9Hz. 브라우저 개수와 무관한 worker 구조를 확인했습니다. 다수 동시 브라우저 부하 시험은 수행하지 않았습니다. 실제 브라우저1개 안의 live/test 두 탭을 사용했습니다.
- bridge 수신667패킷 시점 rejected0/out_of_order0. 세션·sequence 거부 자체는 자동 검사로 별도 확인했습니다.

## 남은 실제 연동 사항

Claude의 Depth/Nav2/자동 맵핑 작업을 완료한 후 활성 publisher의 토픽·타입·offered QoS·frame·stamp·DaBai 인코딩/scale·CameraInfo·등록 여부를 확인해야 합니다. 실제 선택 Frontier 목표와 local trajectory 메시지가 기본 후보와 다르면 읽기 전용 어댑터를 추가/검증해야 합니다. 실제 TF 시각 정합, local rolling/clearing, SLAM map 실시간 갱신, CPU/대역폭을 모터 잠금 상태에서 확인해야 합니다. 현재 RMF 연결, 동적 객체 추적, 원격 정지, 지도 저장 확인 API는 미구현입니다.

## 변경 파일

- `core.py`: grid/TF/depth 검증·변환·상태 저장
- `server.py`: 인증된 loopback gateway·캐시·프레임 정합
- `bridge_node.py`: 독립 ROS subscriber bridge
- `run_local.py`, `__init__.py`: 독립 실행·자체 PID 관리
- `web/index.html`, `style.css`, `app.mjs`, `map.mjs`, `geometry.mjs`: UI·좌표·레이어·영상
- `tests/test_visualization.py`, `geometry.test.mjs`, `ros_fixture.py`, `replay_rgb.py`, `probe_current.py`: 격리 시험·측정
- `README.md`, `docs/INTERFACES.md`, `docs/VALIDATION.md`, `docs/screenshots/`, `docs/evidence/`: 실행·연동·검증 근거
