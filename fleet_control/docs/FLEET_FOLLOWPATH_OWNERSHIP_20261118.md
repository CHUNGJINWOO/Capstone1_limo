# Fleet → Agent → Nav2 → Gate → Gazebo 단일 임무 통합 (Phase 2, 2026-11-18)

**SIM_ONLY.** 격리 Gazebo(Office+LIMO) + 실제 Nav2 Humble + Gate r8 후보(수정 없음) + 실제 Fleet Server·Robot Agent. 실차 접속·모터 구동 없음. commit/push 없음. 모든 결과는 시뮬레이션이며 실차·SROS2·Gateway 증거가 아니다. 증거 폴더(`experiments/fleet_followpath_trust_20261118_N3p5/<run_id>/`)에는 시험 전용 토큰(`fleet/sec`, `auth.json`)이 있으므로 외부 공유 금지.

## 0. 공통 문서가 Git에 추적되지 않는 이유 (기록만, 파일 추가/삭제 없음)
현재 브랜치 `staging/fleet-control-on-progress-c804241`는 `6129d9f`(origin/main과의 merge-base)에서 갈라졌고 공통 문서 8종은 이후 `origin/main`의 `d9c7c42`(PR #9, #10) 등에서 커밋됐다. 이 브랜치의 `c804241` 트리에는 `VALIDATION/CODE_INDEX/CURRENT_STATUS/HANDOFF/DECISIONS.md`가 없고(`git ls-tree`), `.gitignore`로 무시되지도 않는다(`git check-ignore` 없음). 로컬 파일은 이 브랜치에서 **미추적 작업 사본**이며 `origin/main` 판과 내용이 다르다. 병합·추가는 하지 않았다.

## 1. 원인 (실제 코드와 같은 실행의 로그에 근거)
기준 FAIL 재현 `before01`(수정 전 코드, `office.py`): mission `m_969f097a54bd`.
| t(s) | 사건 |
|---|---|
| 49.400 | REST 임무 요청 → Server 수락, QUEUED→DISPATCHED |
| 49.637 | Agent가 journal에 예약한 UUID `ac7439f0…`로 NavigateToPose 제출, Nav2 status 2 |
| 49.697 | Server EXECUTING |
| **49.701** | **Server `EXECUTING → RECONCILIATION_REQUIRED (safety context missing, stale or changed)`** (EXECUTING 4 ms 뒤) |
| 49.719 | Agent 관측: `known=false`, `NAV2_STATUS_STALE_OR_UNKNOWN` (Agent 자신의 직접 status는 소유 UUID를 봄) |
| 49.728 | Gate 증거에 `follow_path` 자식 `3b465a50…` 첫 ACTIVE |
| 49.929 | Gate `supervisor:AUTHORITY_LOST` → `FAILSAFE_SENSOR_TIMEOUT`(LATCHED) |
| 49.947/49.978 | Agent `SAFETY_CANCEL … GATE_R7_STATE_INVALID` → Nav2 status 3 → 5(CANCELED), Server CANCELED |
원인은 서로 독립인 **두 결함**이었다(소스: `ros_adapter._nav2_goals`, `safety_protocol.sim_r7_nav_evidence`, `server/safety.py`).
1. **증거 지연(first, 이번 재현의 직접 원인)**: Agent의 직접 status 표본이 소유 UUID를 보이는데 Gate `nav_evidence`(약 10 Hz)는 아직 idle이면 "표본이 증거와 모순"으로 `None` → `known=false` → Server `safety_context` 무효 → 즉시 재조정 → heartbeat `execution_allowed=false` → Gate AUTHORITY_LOST. `follow_path` UUID와 무관.
2. **FollowPath 자식 = 외부 Goal 판정**: `foreign = [u for u in ids if u != owned_uuid]`이고 `ids`는 모든 Nav2 액션 서버(`follow_path` 포함)의 활성 UUID 전체다. 부모/자식 UUID는 정상적으로 다르므로 정상 FollowPath가 항상 foreign이 된다(기존 FAIL 기록 `C2x9/fleet02`의 `NAV2_FOREIGN_GOAL_ACTIVE`와 같은 결함). 부모-자식 연결 정보는 Nav2가 발행하지 않는다.
3. (수정 중 추가로 확인) **FollowPath 교체 겹침**: Nav2 컨트롤러는 약 1 Hz로 새 FollowPath goal을 받고, 이전 goal이 terminal 되기 전 한 Gate 표본(≤~0.1 s) 동안 두 goal이 동시에 활성이다. 첫 수정(자식 1개만 허용)은 이 순간 foreign으로 오판해 `fault01`에서 `RECONCILIATION_REQUIRED`가 재현됐다.
Terminal 경쟁(T5a1 수정: 소유 표현은 journal 영속 뒤에 제거)은 현재 코드에 이미 있고 별도 원인이다. 이번 모든 성공 실행에서 재조정 이벤트 0, 최종 Server/Journal/Nav2 일치.

## 2. 수정 (최소, Gate·Nav2·Cliff 무수정)
SIM_ONLY **제한 신뢰 모델**(DDS 인증이 아니며 외부 Goal 차단의 증명이 아니다). 조건 전부 만족할 때만 `follow_path` goal을 소유 Goal의 하위로 귀속하고 나머지는 모두 foreign 유지.
- `fleet_control/agent/limo_robot_agent/limo_robot_agent/safety_protocol.py`: `sim_classify_nav2_goals()` 신설, `sim_r7_nav_evidence()` 확장.
  - 소유 UUID가 `navigate_to_pose`의 유일한 활성 goal(추가 NavigateToPose goal = foreign), `follow_path` 활성 goal 1개 또는 교체 순간 정확히 2개(3개 이상 foreign), 소유 UUID가 `follow_path`에 없음, 다른 모든 액션(spin/backup/…)은 idle-proven, 모든 `follow_path` goal이 provenance 검사 통과, 2개면 같은 쌍이 ≤ 0.5 s.
  - 증거 지연: Gate 증거가 **완전 idle**이고 Agent 표본과의 차이가 정확히 소유 UUID 하나이며 제출 후 ≤ 1.0 s일 때만 `owned_pending_evidence`로 허용. 초과·다른 UUID·증거가 idle 아님이면 기존처럼 `None`(미확정, 이동 불허).
- `ros_adapter.py`: provenance = (a) 자식 UUID를 소유 Goal 제출 **이후** 처음 관측(모든 활성 UUID의 최초 관측 시각 기록), (b) ROS graph에서 `follow_path` 액션 클라이언트를 가진 노드가 전부 `bt_navigator*`(다른 노드는 foreign). 그래프를 못 읽으면 foreign. 같은 이름으로 위장한 노드는 구분하지 못한다(SIM 비인증).
- `server/fleet_server/safety.py`: `sim_subordinate_valid()`가 Agent의 하위 목록을 Gate 증거에서 **재유도**해 검증(주장만 믿지 않음). 활성 재개(pending continuation)에서 `[소유]` 또는 `[소유]+검증된 하위`만 허용.
- 유지: Fleet 명령 소유권, Agent Journal(UUID 사전 예약·영속), Cancel ACK≠Terminal, 하위 종료 근거 없이 상위 완료 선언 금지, Gate fault/E-Stop/watchdog 우회 없음. 모의 test-only 우회(`terminal_race_20261117_T5a1/sim_adapter.py`의 monkeypatch)는 사용하지 않았다.
- diff: `experiments/fleet_followpath_trust_20261118_N3p5/CHANGES_agent_server_followpath.diff`, 수정 전 원본 보존 `original_before/`.

## 3. 실제 Gazebo 결과
판정 근거: `result.json`, `timeline.txt/json`(`analyze.py`), `gate_status.jsonl`, `agent_snap.jsonl`, `agent_journal_final.json`, `fleet/db`, `gt.csv`, `bag/`.
| run | 내용 | 결과 |
|---|---|---|
| `before01` | 수정 전 코드, REST 임무 | **FAIL** (49.701 재조정 → AUTHORITY_LOST → CANCELED) |
| `web01` | 수정 1차(자식 1개) 웹(Chromium) | PASS (재조정 0) — 이후 `fault01`에서 교체 겹침으로 FAIL 재현 |
| `fault01` | 수정 1차 | **FAIL** (교체 겹침 → 재조정). 증거로 보존 |
| `web03` | **최종 코드**, Chromium UI 단일 목적지 | **PASS**: UI POST → 임무 `m_92639da78cfd` → Journal UUID `8fa56ba4…` → Nav2 SUCCEEDED(4) → Journal/Server `SUCCEEDED "goal reached"`, 목록에 SUCCEEDED 표시(`web_final.png`). Gate NORMAL 유지, 종료 후 Gate 출력 비영 0건, GT 정지 후 3.0 s 정지 유지, GT 이동 0.92 m(목표 1.0 m) |
| `fault05` | 최종 코드, 임무 A | PASS(SUCCEEDED, 재조정 0). 임무 B는 하네스 대기 누락 409(하네스 수정) |
| `fault06` | 최종 코드, A 완료 후 B 주행 중 **Agent SIGKILL** | **PASS**: Gate 출력 0까지 0.43 s(한계 0.75 s), 마지막 비영 0.409 s, GT 정지 0.57 s, 추가 이동 0.12 m. Agent 재시작은 재무장 안 함·이동 0, 운영자 취소·terminal(5) 확인, 자동 재개 없음. 단 Gate 재무장·Server `recover`는 **미해결**(Z9 not re-armed, Z11 `no_evidence` 409) — 기존 r8 결과와 같은 상태 |
| `comm01` | **통신 중단**(Server 리스너 종료) 주행 중 | PASS: Gate AUTHORITY_LOST, 출력 0 0.33 s, GT 정지 0.46 s, 추가 0.08 m, Agent가 자기 goal 취소(Nav2 5), 임무 `RECONCILIATION_REQUIRED`(불명확, 정상 종료로 처리 안 함) |
| `cancel01` | 운영자 **Cancel** | PASS: 임무 CANCELING(ACK) → Nav2 status 5 후에만 CANCELED, Gate NORMAL, 출력 0 0.47 s, GT 정지 0.54 s, 추가 0.07 m |
| `foreign_fp02` | **외부 FollowPath**(비 bt_navigator 노드) 주행 중 | PASS(감지·정지): foreign, basis `CHILD_PROVENANCE_UNCONFIRMED`, Gate AUTHORITY_LOST, 출력 0 0.52 s, GT 정지 0.59 s, 추가 0.08 m |
| `foreign_nav02` | **외부 NavigateToPose** goal | **FAIL(미차단)**: 외부 goal이 소유 goal을 선점(ABORTED 12 ms 뒤)하고 Agent는 foreign을 보고 이동 불허로 표시하지만 **Gate는 NORMAL 유지, 로봇은 외부 goal로 계속 주행**(5.3 s 창 끝까지 이동 중, 추가 0.74 m 이상). 하네스 내 "GT 정지 PASS"는 일시 감속 오탐이라 `fault_metrics.py`로 무효화 |
| 기타 | `fault02/03`, `web04`: Gate 기동 래치(`nav_action_idle_proven`/Fast DDS SHM·sim 센서 부재) 환경 실패. `fault04`, `web02`, `foreign_nav01`, `foreign_fp01`: Gate 자체 fault `collision_monitor:MEMORY_NOT_APPLIED_SHADOW`(기존 알려진 Gate 후보 문제, 소유권과 무관) | 무효 처리, 원본 보존 |
SKIP: **Goal 결과 지연·중복 주입**(실제 Nav2 결과를 안전하게 지연/복제할 지점 없음; 합성 시험만), 통신 중단 후 Server 재연결 복구, 실차.
전체 최종 코드 기준 유효 실행 수: 웹 PASS 1(web03), SIGKILL 1, 통신 1, 취소 1, 외부 FollowPath 1, 외부 NavigateToPose 1(FAIL). **반복성은 입증되지 않았다**(단일 표본씩; 수정 2차(최종) 코드 이후 시작한 14회 중 7회는 Gate shadow fault·기동 래치 등 환경성 실패로 무효).

## 4. 판정
1. **Fleet Goal 소유권 문제 해결: PASS (SIM_ONLY, 제한 신뢰 모델)** — 정상 FollowPath 자식·교체 겹침·증거 지연이 더 이상 foreign/재조정을 만들지 않음(web03, fault05/06, comm01, cancel01). 한계: provenance는 비인증 graph 증거.
2. **실제 Gazebo 단일 임무 완료: PASS** (web03, 웹 UI). 반복성 미입증.
3. **Nav2·Agent·Fleet Terminal 일치: PASS** (web03/fault06 등: UUID·SUCCEEDED·재조정 0). Server/Nav2 시각은 wall→mono 매핑 오차 ±0.3 s로 순서 판정은 하지 않음(순서 보증은 T5a1 `test_terminal_race`).
4. **Gate 출력 0·GT 정지: PASS** (web03: 비영 0건, 정지 3.0 s 유지).
5. **외부 Goal 차단 유지: 부분/FAIL** — 외부 FollowPath는 감지·정지(PASS), **외부 NavigateToPose는 차단 안 됨(FAIL, 기존 Gateway 접근 제어 FAIL과 동일 계열, 이번 수정으로 악화/개선 아님)**. Agent는 소유하지 않은 goal을 취소하지 않으며 Gate는 소유 goal의 권한 상실에만 반응한다.
6. **Agent SIGKILL·통신 중단: PASS**(정지) / 재시작 후 Gate 재무장·Server recover는 미해결.
7. **기존 회귀: PASS** — 신규 16 + Agent/Server 안전·r8·terminal·recovery·phase20 계열 합계 **276 passed**. `test_recovery.py` 11 failures는 **수정 전 원본 코드에서도 동일하게 실패**(safety context 요구로 인한 레거시 시험), 이번 변경과 무관(원본 복사본 대조).
8. **미해결 위험/후속**: (a) 외부 NavigateToPose 차단(Gateway/ACL, 별도 설계), (b) provenance 위장(같은 노드명) — SROS2 등 인증 필요, 실차는 이 신뢰 모델 사용 금지, (c) Gate `MEMORY_NOT_APPLIED_SHADOW`·기동 래치(Gate 담당), (d) 재시작 후 재무장/recover(`no_evidence`), (e) Goal 결과 지연·중복 실주입, (f) 반복성·장시간, (g) 하위 액션 spin/backup(복구 행동) 중 Fleet 임무는 foreign으로 취급되어 취소될 수 있음(보수적).
Gate r9/r10 Cliff 후보는 활성화하거나 안전 검증 완료로 간주하지 않았다(이번 시험은 r8 후보 사용).

## 5. 변경 파일과 재현
수정: `fleet_control/agent/limo_robot_agent/limo_robot_agent/{safety_protocol,ros_adapter}.py`, `fleet_control/server/fleet_server/safety.py`.
신규: `fleet_control/tests/test_nav2_subordinate_goals.py`(16), `experiments/fleet_followpath_trust_20261118_N3p5/`(`run_isolated.sh`, `office.py`, `web_office.py`, `faults_office.py`, `r8_agent_proc.py`, `isolation.py`, `web_probe.py`, `analyze.py`, `fault_metrics.py`, `queue.sh`, 각 run 폴더).
```bash
cd /home/cjinw/school/2026-2/Capstone1_limo
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=fleet_control/agent/limo_robot_agent:fleet_control/server python3 -m pytest -q fleet_control/tests/test_nav2_subordinate_goals.py
cd experiments/fleet_followpath_trust_20261118_N3p5
./run_isolated.sh <새 run_id> <미사용 domain 0-232> web_office.py                       # 웹 단일 임무
./run_isolated.sh <새 run_id> <domain> office.py                                         # 정상 + Agent SIGKILL
SIM_EXTRA="--scenario foreign_nav|foreign_followpath|comm_loss|cancel" ./run_isolated.sh <새 run_id> <domain> faults_office.py
python3 analyze.py <run>/office_r8_fleet_agent_kill ; python3 fault_metrics.py <run>/office_r8_fleet_agent_kill
```
각 실행은 사용자·네트워크·마운트 namespace, loopback only, `/dev` 마스크(모터 장치 없음)로 격리된다.
