# 맵핑·기록 통합 조사 및 격리 bag 복사 (2026-11-17)

## 범위와 결과

**전체 요청 미완료.** 웹 맵핑·기록 API, 서버→Agent 실행, 로봇→노트북 네트워크 전송, 원격 조종 및 자동/수동 전환은 구현하거나 활성화하지 않았다. 실차·Gazebo를 구동하지 않았다. 이 문서는 다른 담당자의 Gate·Agent·서버 계약을 바꾸지 않는 독립 후속 제안이다.

### 기존 구현과 차단

- `experiments/real_auto_mapping_ready_20261113/`은 현장 대화형 실행, 준비 점검, 탐색 시작/중지, 지도·posegraph 저장, bag 기록을 지원한다. 직접 입력·소유 프로세스 검사를 요구하고 잠금이 기본이다. 기존 배포본을 수정하거나 해당 절차를 웹으로 우회하지 않았다.
- `capstone_ws/src/limo_capstone/scripts/run_recorder.py`는 별도 기록기이며 `exit.json`, `bag/`을 저장한다. 기록 경로 우선순위는 `--root` → `LIMO_RUNS_DIR` → 저장소 `local_data/raw_results/limo_runs` → `~/limo_runs`. 기본 bag은 파일 zstd 압축이다. 준비 배포본은 별도 `$HOME/.local/state/limo-real-auto-mapping/` 구조여서 동일한 종료 증거 계약을 갖는다고 가정하지 않는다.
- Frontier는 Nav2 목표를 직접 제출하고 stop이 cancel-all을 호출하며 별도 복귀 Goal도 존재한다. Mission Manager/Agent의 소유 Goal 경계와 WEB/RMF/EXPLORE 우선순위·불명 결과 재조정은 아직 해결되지 않았다. [현행 조사](../OPERATION_MANAGER_PHASE1_2_PROPOSAL.md). 탐색 stop ACK, 목표 terminal, 정지 증거는 서로 다르다.
- 실차 준비본의 명령 경로는 Nav2→`/cmd_vel_raw`→stall Gate→`/cmd_vel_locked` 또는 `/cmd_vel`. 요청된 Collision Monitor 포함 경로가 이 배포본에 실제로 연결되어 있다고 주장할 근거는 없다. 기존 Gate 후보, Cliff, Ground Memory, 서버·Agent의 안전 코드는 수정하지 않았다.
- Cliff 바퀴 지지 영역 FAIL과 Fleet Terminal/Gateway 접근 제어 FAIL은 해소하지 않았다. 과거 전원 돌발 차단·USB 재연결, 물리 정지/감시, 소유권·인증 증거 문제도 남아 있다. 이 작업은 실차 이동 허가가 아니다.

## 실제 추가한 코드

`fleet_control/scripts/verify_bag_copy.py`는 **이미 중지된 `run_recorder.py` 로컬 run**의 `exit.json`, `bag/metadata.yaml`, SQLite bag을 검사하고 로컬의 기존 보관 루트 아래 session ID로 복사한다. 파일 zstd 압축은 임시 디렉터리에 복원한 뒤 SQLite `integrity_check`를 한다. `ros2 bag info` 결과 코드, 파일별 SHA-256 및 크기를 검사한다. 동일 세션의 일치하는 부분 복사는 바이트 prefix 확인 후 재개하며, 충돌은 거부한다. 원본을 삭제하지 않는다. 새 디렉터리·복사 파일은 0700·0600으로 만든다. 원본 경로를 포함하는 로컬 전용 marker와 해시 receipt가 생긴다.

**한계**: 네트워크·SSH·인증·암호화·자동 업로드·웹 연동은 없다. 완료된 온보드→노트북 전송의 증거가 아니다. `exit.json`은 신뢰된 기록기가 작성해야 하며, 독립적인 프로세스 종료 증명이나 쓰기 중인 bag 동결을 제공하지 않는다. 출처를 신뢰하지 않는 폴더 또는 아직 사용 중인 bag에 사용하지 않는다. 완전한 crash-durability, 악의적 로컬 프로세스와 동시 수정 방어, 인증된 매니페스트, MCAP, 메시지 단위 압축, 지도/posegraph/로그, 자동 전송 이력은 미지원이다. bag 메타데이터와 bag 자체에는 개인정보가 있을 수 있다. 압축 검사에는 `zstd`, 복원된 DB 크기만큼 임시 디스크 공간이 필요하다. receipt는 센서 메시지 유무나 수집 품질을 증명하지 않는다.

```bash
source /opt/ros/humble/setup.bash
python3 fleet_control/scripts/verify_bag_copy.py \
  --run-dir /path/to/finished/run_recorder_run \
  --destination-root /path/to/existing/private/limo_runs \
  --session-id example_session
python3 -m unittest discover -s fleet_control/tests -p test_verify_bag_copy.py -v
```

이 명령은 실행 노트북에 이미 접근 가능한 파일만 복사한다. 실차에 SSH 접속하거나 저장 위치를 변경하지 않는다. 경로는 실제 환경에서 확인해야 한다.

## 오늘 검증

| 항목 | 결과 | 근거와 범위 |
| --- | --- | --- |
| 부분복사 재개·충돌·오류·손상·심볼릭 링크·중복 | PASS | 독립 Mock 8 tests; 프로세스 강제 종료나 실제 네트워크 중단 아님 |
| ROS 기록 및 정상 종료 | PASS | 격리 ROS, 합성 `std_msgs/String` 각각 1건, 비압축·zstd, SIGINT; 모터·Gazebo 없음. Fast DDS 네트워크 인터페이스 permission 오류가 있었으나 격리 프로세스 간 합성 메시지 1건 기록·확인됨 |
| 비압축 bag 로컬 검증·복사 | PASS | `/tmp/limo-bag-isolated-8jq841f7/`, DB SHA-256 `bb90e0e9e9edac24d5bfac4e843f34bab9f0bcb6336db8dc6892f8a75f0a73c0` |
| zstd bag 로컬 검증·복사 | PASS | `/tmp/limo-bag-zstd-n5cnkud2/`, 압축 DB SHA-256 `5d1623bebe015016a9745a2f2654e0fb0becf1a3023f1bde890fa41d60b6bca0` |
| Dashboard JS·Registry upload JS | PASS | `test_static_dashboard.mjs`, `test_registry_upload.mjs` |
| Safety Context core | PASS | `test_safety_context_v1.py` 5 tests |
| Safety Context 전체 | ERROR | 35개 중 HTTP 1개 서버 생성이 sandbox socket `PermissionError`; 나머지 34개 오류 없이 종료, 전체 PASS 아님 |
| 웹→서버→Agent 맵핑/기록·지도 저장/재로드 | SKIP | 기능 미연결 |
| 로봇→노트북 전송·중단/재전송 | SKIP | 실제 네트워크 전송 미구현 |
| Gazebo 조종·watchdog·전환 및 Gate fault | SKIP | 수동 조종·중재 경계 미구현 |
| 실차·모터 잠금 시험 | SKIP | 접속·실행하지 않음 |

## 후속 계약 및 구현 순서 제안

1. 소유권 담당자와 Fleet Server 단일 제어 평면, idempotency key, 세대/재시작, 외부 Goal 차단, Explore 복귀 목표, UUID별 terminal 증거, 정지·시간 신선도와 WEB/RMF/EXPLORE 우선순위를 합의한다. 임의의 프로세스 실행 API나 cancel-all을 공개하지 않는다.
2. 인증·RBAC·감사 및 Agent 고정 명령 allowlist로 세션 준비/시작/중지/지도 저장을 별도 상태 머신에 연결한다. 요청 수락과 탐색 종료·정지 확인을 분리하며 불명 상태는 차단한다. 배포본 대화형 물리 확인을 제거하지 않는다.
3. 세션 ID, 로봇/건물/층, bag·지도·posegraph·Gate 로그의 안정적인 완료 증거 및 원본 스냅샷 계약을 정한다. 기록 프로필은 실제 해당 환경 토픽·타입과 이미지의 민감성/용량을 확인한다. SQLite 외 포맷 및 중단 복구, 무결성 검사·빈 bag 경고를 추가한다.
4. 인증된 암호화 전송의 양 끝에서 고정된 저장 루트·전용 권한·재시도·해시 영수증 및 실패 이벤트를 검증한다. credentials/logs/개인정보를 기본 전송 목록에서 제외하고, 원본은 승인 없이 삭제하지 않는다. 네트워크 전송 수신기와 프로토콜의 안전성은 별도 시험한다.
5. Gazebo 전용 입력/소유권/로컬 watchdog을 Gate 담당과 통합하고 Collision Monitor→Gate 실제 연결을 검증한다. 후진 제한과 deadman·네트워크 단절·브라우저 중단·프로세스 종료·Fault 테스트 통과 전 수동 출력 금지.

공통 문서는 동시에 다른 작업이 진행 중이므로 수정하지 않고 본 독립 제안만 추가했다.
