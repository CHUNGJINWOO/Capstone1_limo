# G2: origin/main 통합 (2026-11-23, Claude; 로컬 작업트리만, commit 없음)

- 작업 위치: worktree `<WORKSPACE_PARENT>/Capstone1_limo_g2_20261123`, 브랜치 `integration/fleet-g2-main-v5b-20261123`(출발 `40b666ec`).
- 진행 상태: `git merge --no-commit --no-ff origin/main`(`ea2d9632`)을 시작했고 충돌 77개를 모두 해결했습니다. merge commit은 승인 전이라 만들지 않았습니다.
- 지키지 않은 것은 없음: staging(`c804241a`), G1(`0ca09dc4`), v5b(`40b666ec`) ref와 메인 작업트리의 미커밋 파일은 바꾸지 않았습니다. 실차·18190·모터는 쓰지 않았습니다.
- 기록: `local_data/raw_results/real_limo_integ_claude_20261122/g2_01/` (로그·해결 스크립트).

## 1. 해결 기준
- `fleet_control/`의 3-way 기준은 PR-B 커밋 `dab07d16`입니다(main의 `fleet_control` 시작점).
- add/add 파일은 공통 기준이 없어서 파일별로 판단했습니다.
- 원칙: v5b(R1·R2)의 안전 경로가 기준이고, main에서 새로 생긴 기능은 그 위에 얹었습니다. main 코드는 근거 없이 지우지 않았습니다.

| 영역 | 결정 | 근거 |
|---|---|---|
| `core.py`, `motion_watchdog.py`, `mock_robot_node.py` | v5b 그대로 | main의 `core.py` 1줄(`while ... not stop_evt.is_set()`)은 기존 `stop()`이 이미 `running=False`와 `stop_evt.set()`을 함께 하므로 기능 차이가 없음. 받으면 `test_recovery`의 대체 Event가 깨져 되돌림 |
| `ros_adapter.py` | v5b 기준 + Frontier 보강(§2) | main의 `goal_owner` 플래그 방식 대신 v5b의 `_do_frontier`가 이미 Frontier·Fleet goal 슬롯 공유, `FRONTIER_FENCED`, follow_path idle 증거를 구현함 |
| `agent_node.py` | v5b 기준 + main의 `camera_preview`, `isolated_ros_bag`(BagWorker) | main의 `execution_allowed = connected and not stop` 대체와 단순 종료 순서는 R1·R2보다 약해서 받지 않음 |
| `service.py` | v5b 기준 + main의 카메라 수신·조회, `manual_preview_status`, 새 Agent boot에서 bench 종료, bench 모드에서 AUTO 생성·dispatch 금지 | bench 모드 검사는 R1 안전·Lease 검사 **뒤**에 둠(R1 시험의 거부 사유 순서 유지) |
| `http_api.py` | v5b + `/agent/camera`, `GET .../camera/{rgb,depth}.jpg`, `GET/POST .../bag` | 운동 권한과 무관한 경로 |
| `recovery.py`, `ownership.py`, `safety.py`, `db.py` | v5b(main 변경은 이미 포함됨). `db.py`에는 main의 `operation_*` 테이블 정의가 들어왔으나 v5b `mapping.py` 스키마와 같은 `IF NOT EXISTS` 정의라 호환됨 | |
| `mapping.py` | v5b(R1 Mapping STOP·Lease) + main 호출 형태 `request_operation(robot, kind, actor, body)`·`operation_status(robot, 'bag')`를 받아 Bag으로 넘기는 분기 | |
| Bag | `server/fleet_server/bag_operation.py`(신규, main의 Bag 상태기계 그대로)와 main의 `bag_worker.py`·`bag_supervisor.py`·`owned_bag.py` | 같은 `operation_*` 테이블(kind `bag`). 추가 제한: Agent당 rosbag 소유자 1개(`recording_enabled`와 동시 사용 거부, Agent·서버 양쪽), START에는 Agent 계약 필요, STOP은 항상 가능 |
| `velocity_arbiter.py`, `manual_web_bench.py`, `mapping_worker.py`, `local_process.py` | v5b | main의 Arbiter는 Watchdog 없이 `/fleet/cmd_vel_selected → velocity_smoother → /cmd_vel_raw`로 출력함. v5b는 Arbiter → Watchdog → Gate |
| Web UI | v5b 콘솔에 main의 Operations·카메라·게임패드 패널을 `Field Ops` 탭으로 추가 | 패널이 기대하는 전역(`$`, `api`, `token`, `data`, `principalName`)은 `static/legacy_panels_shim.js`가 같은 세션 토큰으로 제공 |
| `capstone_ws`·`docs`·`experiments` 충돌 | main | main 쪽은 기능 추가(`manual_mapping_probe`는 기본 false이고 그때 출력도 `/cmd_vel_raw`로 같음, frozen map 저장, run 위치), 주석 정리, 개인 경로 `<repo>` 치환뿐임 |
| 공통 문서 8개 | v5b 판 + 끝에 "origin/main 판에만 있던 줄" 절 | START_HERE·DECISIONS는 main 판에 고유 줄이 없음 |

## 2. Frontier 목표의 R1 권한 (G2 보강, 안전 강화)
**결함(v5b에 이미 있던 것):** `_do_frontier`는 `execution_allowed()`(연결 + 상태 신선도)만 확인했습니다.
- 그래서 Lease 없음, 재조정 대기, 서버 계약 무효 상태에서도 Mapping 목표를 보낼 수 있었습니다.
- submission fence를 쓰지 않았습니다.
- 활성 Frontier goal은 Safety Context가 없어서 `check_active_goal_safety`가 None을 반환했고, 계약을 잃어도 취소되지 않았습니다.
- Codex가 R1 후속 과제로 남긴 "보조 실행 주체 세대 결합" 중 Mapping Worker 부분에 해당합니다.

**수정:**
- `safety_protocol.frontier_authority_fault`를 추가했습니다. 조건은 Fleet goal과 같습니다: 정확한 `reconciliation_required False`, 서버 계약, `connected`, 구조가 올바르고 만료되지 않은 Lease, 큐에 넣을 때 찍은 `contract_generation`.
- `start_frontier`가 세대를 찍습니다(`start_navigate`와 동일).
- `_do_frontier`는 Stage 1에서 검사하고, `_submission_fence()` 안의 Stage 3에서 다시 검사합니다.
- 활성 Frontier goal은 모니터가 같은 조건으로 취소 의무를 집니다.
- Cancel·STOP은 권한 없이도 처리됩니다.

**시험 `test_g2_frontier_authority_20261123`:**
- 수정 후 10/10 PASS.
- 수정 전(v5b) 트리에서는 하위 시험 포함 20건이 실패합니다. 연결 끊김만 수정 전에도 막혔습니다.

## 3. Bringup과 R2 Watchdog
**main 판의 문제:** `limo_fleet_bringup/navigation.py`가 `velocity_smoother`의 출력을 `/cmd_vel_raw`(Gate 입력)로 바로 보냈습니다. Watchdog이 없어서 Nav2 속도가 Watchdog을 거치지 않았습니다.

**수정(모터 잠금 구성 유지, `/cmd_vel_locked` 출력 그대로):**
- smoother 출력을 `/fleet/nav_cmd_vel`(Watchdog 입력)로 바꿨습니다. Arbiter를 켜면 `/fleet/cmd_vel_nav_smoothed`(Arbiter 입력)로 보냅니다.
- `safety.motion_watchdog()`(`fleet_motion_watchdog`, 출력 `/cmd_vel_raw`)를 launch에 추가했습니다.
- Agent 매개변수 이름을 v5에 맞췄습니다: `web_mapping_enabled`, `manual_locked_arbiter`, `require_motion_watchdog: true`. main의 이름도 함께 전달합니다.

**정적 시험 `test_g2_bringup_chain_20261123`(ROS Humble `launch` 필요):** 3/3 PASS. Gate 입력을 쓰는 것은 Watchdog뿐이고 `/cmd_vel`로 리매핑되는 노드는 없습니다. main 판 bringup에서는 같은 시험이 실패합니다.

**미검증·차단:**
- launch 전체 기동, DDS, Nav2 결합은 실행하지 않았습니다.
- 커밋된 Gate(`stall_detector_node`)에는 `require_motion_watchdog`와 입력원 검증 매개변수가 없습니다(제안 패치 미적용). Gate는 Watchdog이 없어도 거부하지 않습니다.
- v5 Agent는 mock bench를 Arbiter 없이 거부합니다(sys.exit). 그래서 main의 `manual:=true velocity:=false` 프로필은 기동이 거부됩니다(fail-closed).

## 4. 검증 결과 (모두 오프라인, worktree 기준)
| 시험 | 결과 |
|---|---|
| R1 계약·Mapping STOP·신규 G2·main mapping/bag/agent recovery (평문, 17개 모듈) | 210 실행: 201 PASS, 7 SKIP, **2 FAIL**(아래 A) |
| Server–Agent 루프백 E2E·Recovery·Watchdog admission·SIGKILL | 36/36 PASS |
| Dedupe / Ownership(포트 18083) | 14/14 / 45/45 PASS |
| R2 Watchdog (격리 ROS 2 Humble, `unshare -Urn`, domain 218) | 66/66 PASS |
| 격리 ROS 3 (mock action, late goal, shutdown) | 3/3 PASS |
| R3 배포 도구 (ROS source, 일반 사용자) | 11/11 PASS. `unshare -Urn` 안에서는 test_05가 FAIL(그 안에서는 root라 `chmod 0555` 쓰기 차단이 듣지 않음 — 환경 원인) |
| Gate reset worker / Arbiter 세대 / locked Arbiter (ROS) | 11 / 14 / 3 PASS |
| colcon build `limo_robot_agent`·`limo_fleet_bringup`·`limo_capstone` (임시 ws) | PASS, 진입점 `robot_agent`·`mock_robot`·`motion_watchdog`, launch 설치 |
| Bringup 체인 정적 시험 (ROS) | 3/3 PASS |
| 전체 discover (평문) | 1016 실행, 4 FAIL, 36 ERROR, 33 SKIP. 이전 R1.4.6 기준(974/2/33/22) 대비 새 이름 5개가 FAIL/ERROR: 아래 A 2개, B 2개, `test_map_registry` 1개(worktree에 미추적 `experiments/real_rmf_20261113`이 없음 — 환경) |

**A. main `test_mapping_session` 2건 (설계 차이, 미해결):**
- `owns_mapping_goal`: v5b는 UNKNOWN 세션에서도 예약된 자기 goal의 소유권을 유지합니다. 운동과 START는 막혀 있고 Worker가 goal을 멈추는 중이기 때문입니다. main은 거짓을 반환합니다.
- UNKNOWN 이후 늦게 온 결과: v5b는 첫 결과를 다시 쓰지 않고 현재 세션을 돌려줍니다. main은 409를 냅니다.
- 두 동작 모두 운동을 허용하지 않습니다. 이 시험은 fixture에 v5 Agent 알림 플래그(`mapping_worker`, `isolated_ros_bag`, `safety_contract_version`)만 넣었고 assertion은 바꾸지 않았습니다. 그 상태로 2건이 FAIL입니다.

**B. main `test_locked_velocity`·`test_web_gamepad_bench`·`test_cancel_results` (API 대체, 미해결):**
- main의 Arbiter와 bench 함수(`select_locked(mode, locked=...)`, `select_velocity`)와 `_do_nav` 기반 `goal_owner`를 시험합니다. v5b에서는 다른 구조(Watchdog 체인, `_do_frontier`)로 대체되었습니다.
- `test_cancel_results`는 ROS에서 17개 중 11개 PASS(취소·결과 의미는 유지)이고 6개 FAIL입니다(시험 Harness에 `_submission_fence` 등이 없음, Frontier를 `_do_nav`로 보내는 API).
- 같은 기능의 v5b 시험은 PASS입니다: Arbiter 세대 14, locked Arbiter 3, 신규 Frontier 10.
- main 시험을 v5b API로 옮길지, 대체되었다고 기록하고 은퇴시킬지는 결정이 필요합니다.

## 5. 남은 차단 조건
- merge commit, 로컬 이력 통합, push·PR: 승인이 필요합니다.
- 위 A·B: 결정이 필요합니다. 결정 전에는 PASS로 표시하지 않습니다.
- bringup의 실제 launch와 Nav2·DDS 결합은 미검증입니다. Gate의 Watchdog 필수화는 미적용입니다.
- Agent v5 candidate는 G2 트리로 다시 만들어야 합니다(v5b `c71b166f…`는 무효). `agent_node.py`, `ros_adapter.py`, `safety_protocol.py`, `package.xml`과 Bag·카메라 모듈이 바뀌었습니다.
- R4 차단 조건 전체(서버 선배포, 실차 배포, R3 이중 실패, 모터 잠금 스택 현장 검증, `/cmd_vel` 우회, Depth, 전원·USB/UART·STM32, E-Stop·제동거리)가 유지됩니다.

## 6. 후속 조정 (2026-11-23, 결정: main의 안전 의미를 제한적으로 채택, R1·R2 구조 유지)
### 6.1 UNKNOWN Mapping 세션의 Frontier goal: 승인과 추적 분리
- **수정 전(v5b):** `owns_mapping_goal`이 UNKNOWN·RECONCILING에서도 참이었습니다. 그래서 그런 세션의 활성 goal이 정당한 goal로 인정되어 서버가 로봇을 막지 않았습니다.
- **수정 후(`mapping.py`):**
  - `owns_mapping_goal`(승인용)은 MAPPING·PAUSING·STOPPING에서만 참입니다.
  - `tracked_mapping_goal`(추적 전용)은 UNKNOWN·RECONCILING 세션에서 boot, 세션, 예약 command id가 일치하면 식별 증거(session, boot, epoch, command_id, goal_uuid)를 돌려줍니다.
- **`recovery.py`(`_untrusted_mapping_goal`):** 그런 활성 goal이 보이면 승인하지 않고 **추적만** 합니다: 식별 증거와 경고 이벤트 `mapping_goal_untrusted`를 goal마다 1회 남깁니다. 일반 "unknown active mission ownership" 오류로 처리하지 않습니다.
  - 처음 구현에서는 `reconciliation_required`를 로봇 전체에 걸었으나, 격리 E2E f03에서 Lease 획득이 거부되어("cannot take control") 운영자가 RECONCILE을 못 하는 교착이 생겼습니다. 그래서 로봇 전체 차단은 걸지 않습니다. UNKNOWN 세션이 이미 새 운동을 모두 막고, Agent가 스스로 권한을 철회하고 goal을 취소하기 때문입니다. 회귀 시험 `test_operator_can_retake_the_lease_while_the_goal_is_tracked`.
- **추적이 끊기지 않는 근거:** Worker 쪽 `lose_session`이 `halt_goal`(cancel_frontier)로 goal을 멈추고 goal 기록을 저널에 남기는 동작은 그대로입니다.
- **새 운동 차단:** UNKNOWN에서는 `mapping_blocks_motion`이 참이고, START와 `reserve_goal`이 거부되며, Worker의 `frontier_allowed`는 MAPPING에서만 참입니다.
- **R1 경로:** Stage 1~3 fence와 Cancel/STOP 예외는 바뀌지 않았습니다.
### 6.2 UNKNOWN 이후 늦은 Worker 결과
- **수정 전:** 명령이 UNKNOWN이 된 뒤 도착한 result에 현재 세션을 그대로 돌려줬습니다(상태는 바뀌지 않음).
- **수정 후:** HTTP 409 `operation_not_active`로 거부하고, 세션과 이전 결과는 그대로 둡니다. 진단 이벤트 `mapping_late_result`(session, command, boot, outcome, evidence)를 남깁니다.
- **유지되는 것:** 이미 COMPLETED/FAILED로 확정된 결과의 재전송은 계속 멱등입니다. 세션·명령·boot 불일치는 기존대로 409입니다.
### 6.3 main `test_cancel_results` 17개: 분류와 조치
| 시험 | 분류 | 조치 |
|---|---|---|
| `test_live_or_ambiguous_foreign_goals_prevent_submission` | **A (실제 회귀)** | v5b는 navigate_to_pose 상태 publisher가 2개(서버 모호)여도 Fleet goal을 보냈습니다(수정 전 probe: sent 1). `safety_protocol`에 "publisher 정확히 1개" 규칙(`NAV_GOAL_OWNERSHIP_UNAVAILABLE`)을 추가했습니다. 시험은 실제 v5 선행조건을 쓰도록 이식했습니다(`test_g2_cancel_ports_20261123`, v5 fixture의 stub가 섞이지 않게 별도 프로세스). v5는 활성 외부 goal을 `NAV2_NOT_IDLE`로 부르므로 사례별 정확한 코드를 검사합니다 |
| `test_fleet_and_frontier_share_the_first_goal_slot` | B (+ 작은 A) | Frontier 진입점이 `_do_frontier`로 바뀌어 Harness에 v5 fixture(fence, 세션)를 추가했습니다. 불완전한 Nav2 증거에서 `_do_frontier`가 KeyError로 죽던 것을 거부로 바꿨습니다(`.get`). Fleet goal 기록에 `owner: 'fleet'`을 명시했습니다. 30회 반복 PASS |
| `test_follow_path_terminal_requires_active_observation_after_reset` | B | Harness에 v5 연속성 추적기(`follow_idle_tracker`)를 추가했습니다(의미는 동일) |
| `test_submission_exception_does_not_free_robot_for_next_goal` | B | Harness에 fence와 clock을 추가했습니다 |
| 나머지 13개(취소 완료 증거, UUID 일치, 수락 전 취소, 결과 소실, 늦은 피드백 차단 등) | 원래 PASS | 변경 없음 |
- 결과: ROS에서 17/17 PASS입니다. assertion은 바꾸지 않았고, 이식한 시험은 사례별로 더 정확한 코드를 검사합니다.
- 기존 v5 ROS 시험 2개(`test_ros2_isolated_action`, `test_ros2_late_goal_reconciliation`)의 fixture도 고쳤습니다. 초기 상태를 넣으려고 만든 두 번째 status publisher를 그 뒤에 제거합니다(새 규칙에 맞춤).
- 두 모듈을 한 프로세스에서 이어 돌리면 앞 시험의 action 서버가 DDS 그래프에 남아 publisher가 2개로 보이고, 새 규칙이 올바르게 거부합니다. 모듈별 프로세스·domain으로 돌리면 각각 PASS입니다(환경·격리 문제).
### 6.4 main Arbiter·Bench 시험 (`test_locked_velocity`, `test_web_gamepad_bench`) — 미해결, BLOCKED
- **서버 bench 프로토콜이 다릅니다:**
  - main: 샘플마다 바뀌는 ticket, 속도를 m/s로 받고 서버가 0~0.10·±0.30 범위를 검사, mock bench는 Arbiter 없이도 허용, 실차 bench 금지.
  - v5: 축 값 −1~1과 단조 증가 seq, 상한은 Agent(JoystickConfig)와 Arbiter에서 적용, MANUAL 모드 필수, mock bench는 잠금 Arbiter가 있을 때만, 실차 zero-output bench 허용.
- **main Arbiter 상한도 다릅니다:** AUTO ±0.10/±0.30(main) 대 Nav2 slow profile −0.10~0.15/±0.5(v5).
- **영향:** main `gamepad_panel.js`는 main 프로토콜(ticket, m/s)로 보내므로 v5 서버에서는 의미가 어긋납니다. 값이 축으로 해석되어 더 느려지지만 안전 방향입니다.
- **fixture만 v5에 맞춘 probe:** 8개 중 3개 PASS이고 5개는 프로토콜 차이로 FAIL(`g2_01/p5_gamepad_fixture_probe.log`). assertion을 바꾸지 않고는 통과할 수 없습니다.
- **조치:** 시험 파일은 origin/main 판 그대로 두었습니다. 프로토콜 통일(서버 ticket·m/s 범위 검사 채택 여부, 실차 zero-output bench 유지 여부)은 결정이 필요합니다.

### 6.5 v5 Bench 통일 결과

6.4에서 남겼던 프로토콜 결정은 v5로 확정했습니다. 웹 Gamepad와 키보드·마우스·터치 패널은 정규화 축과 증가하는 seq를 보내고, API는 normalized_axes_v1 표식이 없거나 ticket이 붙은 구형 샘플을 거부합니다. Operations는 Agent의 manual_web_bench 상태를 읽습니다. 기존 속도 설정은 최대 전진 0.10 m/s·회전 ±0.30 rad/s 안에서 축 배율로 적용합니다.

main의 test_locked_velocity와 test_web_gamepad_bench를 v5 함수 및 실제 선택 흐름에 이식했습니다. Bench는 Agent의 boot·Lease·계약 세대에 샘플을 결합하고, 이전 권한으로 가져온 응답은 폐기합니다. Mock은 잠금 Arbiter를 통해 출력하며 실차 Bench는 관측 전용입니다. AUTO v5 slow profile은 유지됩니다.

- 집중 36, Mapping·R1 83, Cancel 17, Fleet 루프백·Recovery 30, R2 66, R3 11, 격리 ROS 3건 통과. Mock Gamepad 요청 검증 및 격리 Chromium 수동 입력 8건 통과.
- Cancel과 Fleet 시험을 한 프로세스에 섞으면 실제 ROS 메시지와 스텁이 섞여 실패했습니다. 분리하여 각각 통과했습니다. R3 기본 샌드박스에서는 재빌드 타임아웃이 있었고, root 매핑에서는 권한 주입이 달라졌습니다. 일반 UID 환경에서 11건이 통과했습니다.
- Chromium은 모의 API로 시험했습니다. 기존 Web→Gate 전체 하네스의 고정 Gate 파일이 G2에 없어 해당 실행은 건너뛰었습니다. 전체 discover 및 실차 시험은 수행하지 않았습니다.
