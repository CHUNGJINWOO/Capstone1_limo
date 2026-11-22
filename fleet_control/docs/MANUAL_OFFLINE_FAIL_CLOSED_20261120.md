# 수동 세션 오프라인 차단 후보 (2026-11-20)

## 상태와 기준

실제 staging 소스는 수정하지 않았다. 다른 에이전트 소유 공통 파일과 충돌 가능성이 있으므로 [독립 패치](manual_offline_fail_closed_candidate_20261120.patch)만 작성했다. `fleet_control/`은 staging Git에서 전부 미추적이므로 Git diff만으로 변경 여부를 판단할 수 없다. 조사한 HEAD는 `c804241a`, 감사 시점과 현재 `service.py`·`ownership.py`·`mapping.py`·`agent_node.py`·`manual_web_bench.py`·`motion_watchdog.py`의 SHA-256은 동일하다. 이전 패치는 현재 파일에 적용 검사로는 맞지만, 미완성 계약이므로 무조건 적용하지 않았다. 시스템 프로세스 조회는 이 실행 환경의 제한된 가시성만 제공하므로 다른 에이전트의 작업 종료를 증명하지 않는다.

## 수정 후보

기존 패치의 Offline/readiness 관측, 재연결 hello, housekeeping offline 및 유효하지 않은 lease/pull 시 session 폐기, Watchdog의 미수신 배열 구분을 계승한다. 새 후보는 여기에 `mapping_blocks_motion()` 검사와 **신뢰할 수 있는 외부 Goal 종료 증거 생산자가 없을 때 수동 요청 전체를 거부하고 기존 session을 폐기하는 차단**을 더한다. 정상적인 수동 조작도 불가능하므로 임시 차단 후보이지 완성된 기능이 아니다. Mapping 상태 조회 자체가 실패하면 오류로 진행하지 못하지만, 오류 발생 시 이전 세션을 폐기하는 모든 경계는 아직 완성되지 않았다.

후보는 Agent 독립 검사와 강제 종료, 서버/Agent boot·lease·세션 전 구간 결박, Agent-side 재생 방어, ticket 계약, 입력 sequence와 TTL의 종단 보장, Mapping 시작과 Manual 경합의 원자성, 외부 goal 차단, 인증된 상태 생산자, FollowPath 관측, Watchdog 상태 신선도를 구현하지 않는다. 기존 staging에는 ticket이 없다. Watchdog에서 수신된 `nav_active=0`을 허용하는 부분은 최초 미수신 오류만 줄일 뿐, 오래된 배열이나 숨겨진 Goal의 종료 증거가 아니다. 장애 발생시 Gate/MCU의 독립 정지 보장도 이 패치로 증명되지 않는다. 실제 적용·배포·실차 통합을 금지한다.

## 실패별 상태

| 기존 실패 | 실제 staging | 독립 후보에서 확인한 내용과 한계 |
|---|---|---|
| Offline 후 기존 session 재수락 | **FAIL** | 거부 관측시 폐기하고 같은 Agent boot·lease epoch에서 옛 세션 사라짐을 모의로 확인. 실제 네트워크 단절과 재접속·Agent 폐기는 미검증 |
| Mapping 실행/전환/UNKNOWN | **FAIL** | 열거 상태에서 서버 후보 거부·기존 session 폐기 확인. Agent 독립성·시작 경쟁은 미검증 |
| 활성 외부 Nav2 Goal | **FAIL** | 신뢰 가능한 종료 증거 생산자가 없어 서버 후보가 모든 수동 요청 거부. 실제 action 및 종결 결과 인증은 미구현 |
| 상태 배열 미수신 | **FAIL** | 서버 후보 전체 거부 및 Watchdog predicate가 `None` 거부. 실제 ROS 상태 전이 미검증 |
| 오래된 상태 | **FAIL** | 서버 후보 전체 거부만 확인. 상태 수신시각·publisher 교체에 대한 Watchdog 지속 감시 미구현 |
| 상태 publisher 중복 | **FAIL** | 서버 후보 전체 거부만 확인. 두 action 서버의 실제 graph 시험 미실행 |
| 잘못된 Goal 수 | **FAIL** | 서버 후보 전체 거부만 확인. 유효/유효하지 않은 증거 파서 자체는 미구현 |
| public 기준선 미수신·오래된 상태 | **기존 FAIL 2 유지** | 후보는 staging용이며 public 코드 변경·재시험 없음 |
| 독립 Watchdog 미수신 배열 | **기존 FAIL 유지** | 후보 predicate 단위 확인. 정상 ROS 노드·전체 체인 시험 미수행 |

일곱 FAIL을 각각 해결해 PASS로 바꾸었다는 의미가 아니다. blanket 거부는 구동을 막기 위한 임시 조치다. Mapping STOPPED/IDLE 확인, 인증된 NavigateToPose 및 FollowPath의 모든 전달/진행 중 UUID와 종결 결과, 외부 action 참여자를 통제할 수 있는 로컬 원자적 중재, server·Agent 양쪽 독립 검사 및 재무장 계약이 구현되어야 실사용 가능한 MANUAL 경로를 고려할 수 있다.

## 시험

- 기존 staging 실제 코드 + 모의 DB의 이전 회귀시험 재실행: **PASS 5 / FAIL 7 / SKIP 1**. unittest의 expected failure는 FAIL이다. [로그](../../local_data/raw_results/manual_offline_staging_baseline_20261120.log).
- 복사본에 후보 패치를 적용한 모의 서버·Watchdog predicate 시험: **PASS 7**. Offline 후 재사용, Mapping 여러 상태, 근거 없는 Nav2 차단, 타 사용자 요청, lease 상실, 두 thread 경합, 미수신 predicate를 확인했다. 두 thread는 신뢰 근거 부재로 모두 거부되므로 정상 소유권 경합의 성공 경로 시험은 아니다. [로그](../../local_data/raw_results/manual_offline_repro_20261120.log), [테스트](../tests/test_manual_offline_fail_closed_candidate_20261120.py), [재현 스크립트](../tests/run_manual_offline_fail_closed_candidate_20261120.sh).
- 현재 실행 환경에서 loopback TCP와 UDP 바인딩 모두 `Operation not permitted`: [확인 로그](../../local_data/raw_results/manual_offline_transport_probe_20261120.log). 따라서 실제 Fleet HTTP 및 격리 ROS 프로세스 기반 전체 체인은 **SKIP**, Gazebo도 **SKIP**. 단위시험에는 ROS 메시지 라이브러리 import만 사용했고 ROS 노드, 실제 로봇과 네트워크를 실행하지 않았다.
- Fleet HTTP → Agent → Arbiter → 독립 Watchdog → Gate 전 경로, 게이트 잠금 출력과 `/cmd_vel` 발행자 수의 이번 시험: **SKIP**. 이전 감사의 부분 체인 시험은 기존 보고에 보존돼 있으며 이번 전체 체인 통과 근거가 아니다.

## 통합 제안

Claude의 공유 작업 종료 후 실제 변경 내용과 위 patch의 문맥을 재확인한다. `service.py`와 `motion_watchdog.py`가 직접 충돌 대상이며, `ownership.py`, `mapping.py`, `agent_node.py`, Agent 수동 입력·Arbiter, UI와 mode 계약이 연관된다. 차단을 유지한 상태에서 우선 공통 session/lease/ticket/양 boot/sequence/입력 TTL 계약과 매 무효화 경로를 결정하고, 서버와 Agent가 각자 외부 Goal·Mapping·모드 증거를 검증하게 통합한다. 서버 단일 lock만으로 외부 ROS action 경쟁은 제어할 수 없다. 이후 격리 환경에서 실제 HTTP와 모든 노드를 조립하고, fault 주입 시 `/cmd_vel` publisher 0과 잠금 출력 0을 확인한다. 실차 연결·Mapping START·모터 구동은 별도 현장 검증 및 승인 전까지 금지다.

공통 문서는 병렬 변경 충돌 위험으로 수정하지 않았다. 통합 후 `CURRENT_STATUS.md`, `CODE_INDEX.md`, `ARCHITECTURE.md`, `VALIDATION.md`, `HANDOFF.md`에 실제 적용 파일과 PASS/FAIL/SKIP 및 실차 차단을 반영할 것을 제안한다.
