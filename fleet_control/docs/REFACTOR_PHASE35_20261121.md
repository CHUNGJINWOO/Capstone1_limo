# Phase 3.5 구조 감사 및 점진적 리팩토링 (2026-11-21, Claude)

목표는 새 시스템이 아니라, 지금 동작하는 Fleet Server / Robot Agent / 웹 / 격리 시험 하네스를 더 이해하기 쉽고 안전하게 확장(Gazebo, 자동 기동, 방문자 웹, Open-RMF)할 수 있게 만드는 것이다.
모든 결과는 격리 시뮬레이션(ROS_LOCALHOST_ONLY=1, 사설 도메인)이다. 실차·운영 18190·LIMO SSH는 건드리지 않았다. Gate r11/r12 원본, 패치 06, 다른 담당자의 `capstone_ws` Gate 미커밋 변경은 수정하지 않았다.
`fleet_control/`은 이 브랜치에서 미추적이다. 따라서 변경 전 원본은 `local_data/raw_results/refactor35_claude_20261121/pre_change_originals/`에 보관했다(git diff 대신).

## 1. 감사 결과 (우선순위 · 영향)

| ID | 발견 | 영향 | 조치 |
|---|---|---|---|
| R1 | 토픽/노드 이름 문자열(`/cmd_vel_raw`, `/fleet/nav_cmd_vel`, `stall_detector_node` …)이 Arbiter·Watchdog·Bench·Reset Worker에 중복 하드코딩 | Gazebo/RMF 연결 시 한 곳만 바뀌면 잠금 그래프 판정이 조용히 어긋남 | **완료**: `topics.py` 상수로 통일(값 불변) |
| R2 | 엔드포인트 GID 16진 변환이 3곳에 서로 다른 의미로 구현(0 GID 유지 vs None) | 재사용 시 의미 혼동 → 증거 판정 오류 | **완료**: `ros_graph.py`에 모음. 각 호출부의 기존 의미는 `require_nonzero`로 유지 |
| R3 | "잠긴 속도 그래프" 판정이 3개(Arbiter, Bench legacy, Reset Worker fallback)이며 서로 다름. Reset Worker fallback(Arbiter 없음)은 그래프와 무관하게 **항상 True** | Arbiter 없는 구성에서 Gate reset 전제조건이 그래프 증거 없이 통과 | **특성화 시험 후 동작 변경**: fallback → **False(fail-closed)** |
| R4 | 4개 E2E 러너가 HTTP/결과/프로세스 헬퍼를 각자 복제(이미 `result=` 키 덮어쓰기 버그 1회) | 판정 은닉 버그 재발 위험 | **부분**: `tests/e2e_common.py`(예약 키 거부) 신설, 새 Gazebo 하네스만 사용. 기존 러너 이전은 러너별 전/후 회귀와 함께 후속 |
| R5 | Server `ingest_state`가 상태 수신·모드 전이·안전 판단을 한 함수에서 처리 | 방문자 웹/RMF 상태 추가 시 회귀 위험 최대 | **제안만**(고위험, 이번에 미변경) |
| R6 | Agent→Server 상태 JSON 스키마가 문서화·계약 시험 없이 암묵적 | 웹/RMF 어댑터가 필드 추측 | **제안만**: 스키마 계약 시험을 R5보다 먼저 |

## 2. 변경별 기록

| 변경 | 코드 | 동작 보존 | 검증 | 남은 문제 |
|---|---|---|---|---|
| R1 토픽 상수 | `agent/.../topics.py`(신규). `velocity_arbiter.py`, `motion_watchdog.py`, `manual_web_bench.py`, `gate_reset_worker.py`가 사용 | 예(문자열 값 동일) | 단위 PASS, E2E 아래 | 서버·웹 JS 쪽 문자열은 미통일 |
| R2 GID 헬퍼 | `agent/.../ros_graph.py`(신규). `action_status_evidence.gid_hex`(0 GID 유지), `gate_reset_worker._gid`·`motion_watchdog.own_output_gid`(0 → None) | 예. `test_ros_graph_helpers_characterization.py`로 기존 의미 고정 | PASS | — |
| R3 잠금 그래프 특성화 | `tests/test_locked_graph_contract_characterization.py`: 10개 그래프 × 3개 판정기의 현재 결과 고정 | (시험만) | PASS | 세 판정기 통합은 후속(안전 계약 변경이므로 별도 승인 대상) |
| R3 fallback fail-closed | `gate_reset_worker._graph_locked()` → `False` | **아니오(의도된 동작 변경)**. 이전 True였던 7개 그래프(canonical 포함)가 이제 False. Arbiter가 있는 표준 구성은 Arbiter 판정을 쓰므로 영향 없음 | Gate Reset E2E 13/13 PASS | Arbiter 없는 구성의 Gate reset은 이제 거부된다(의도) |
| R4 공용 헬퍼 | `tests/e2e_common.py`, `tests/test_e2e_common.py` | 해당 없음(신규) | PASS | 기존 러너 미이전 |
| s16 창 수정(시험) | `manual_e2e/run_manual_e2e.py` `zero_after(..., until=다음 입력 시작)` + 창 0.15 s 미만/표본 0이면 None(=FAIL) | 제품 동작 불변. 시험 판정만 수정 | 수정 후 Phase 1 **19/19 PASS**(1회, `phase1_after_s16fix`) | 1회 실행이라 간헐성 해소를 단정하지 않음 |
| Gazebo 백엔드 스위치 | `mode_e2e/stack_up.sh` `SIM_BACKEND=gazebo`, `web_mapping_isolated/stack.launch.py` `USE_SIM_TIME=1` | 기본값(미설정) 경로는 동일한 명령 생성(셸 전개로 확인) | Gazebo 시험은 `GATE_STALL_GAZEBO_20261121.md` | — |

## 3. 회귀 결과 (같은 스냅샷 기준)

- 기준선: `snapshot_before_refactor.sha256`. 단위 `unit_before/` 87 PASS / 21 FAIL. 21 FAIL은 변경 전부터 동일한 환경 의존 실패이며 목록이 같다.
- R1+R2 후 단위: 89 / 21(실패 목록 동일). R3+Gazebo 스위치 후 단위(`unit_after_r3_gz/`): **90 / 21**(실패 목록 동일. 새 시험 3개 PASS).
- E2E(`e2e_after_r1r2r3/`, Gate r12 + 입력 배타, 도메인 228):

| 묶음 | 결과 | 실패 원인 |
|---|---|---|
| Phase 1 (manual, 브라우저) | 18 / 1 | s16 pointer `zero_after_release` false. **하네스 창 겹침**: ptr_off+0.45~0.85 s 창(633.87–634.27)이 다음 gamepad 입력 시작(pad_on 634.194)과 겹쳤다. 리팩토링 이전 gate_input run14에서도 같은 간헐 실패가 있었다. 위 s16 창 수정 참조 |
| Phase 2 (mode, orig17) | 16 / 1 | f04_agent_restart_in_mapping: MAPPING 시작(SLAM 기동) 2 s 후 Gate가 `scan_stale` FAILSAFE로 래치 → FAULT→MANUAL 전이가 `GATE_NOT_NORMAL`로 실패. **Gate E(상태 발행 정지)와 같은 부류**이며 리팩토링 대상 코드(Agent 그래프 헬퍼)와 무관. Gate는 r12 원본 그대로. 이후 f04를 포함한 묶음을 추적 계측과 함께 5회 더 실행했고 f04는 5/5 PASS였다(E 미재현. 첫 실행의 항법 5건 FAIL은 지도 범위 `UNKNOWN_AREA`이며 Gate와 무관. [GATE_STALL_GAZEBO_20261121.md](GATE_STALL_GAZEBO_20261121.md) §1) |
| Phase 3 (nav, 고정 지도) | 18 / 0 | — |
| Gate Reset E2E | 13 / 0 | — |

- 스냅샷: 체인 도중 시험 파일(run_manual_e2e.py: Phase 1 종료 후 수정)과 새 Gazebo 파일이 추가되어 `SNAPSHOT_CHANGED`. 제품 코드(server/agent)는 체인 중 불변.

## 4. 판단과 대안

- R3를 "세 판정기 통합"으로 하지 않은 이유: Arbiter 판정은 실제 모션 허가를 결정하는 안전 계약이다. 통합하면 Bench legacy 경로의 판정도 바뀐다. 현재 결과를 시험으로 고정하고, 명백히 위험한 fallback(항상 True)만 fail-closed로 바꿨다.
- R5/R6은 고위험이라 손대지 않았다. 권장 순서: R6 스키마 계약 시험 → R5 분해(시험 보호 아래).
- 기존 E2E 러너를 `e2e_common`으로 옮기는 일은 러너 하나당 전/후 회귀 1회와 함께 진행한다.
