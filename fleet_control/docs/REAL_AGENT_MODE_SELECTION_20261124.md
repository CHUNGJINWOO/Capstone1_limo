# 실차형 Agent(Arbiter 없음)의 운영 모드 진입 (G2, 2026-11-24, 격리 ROS·mock 검증, 미커밋)

**실차 검증 없음.** 모든 시험은 mock/simulated Agent, 격리 ROS, 합성 센서이며 vendor `limo_base`, STM32, 물리 모터는 포함하지 않는다. 실차의 Mapping·Fleet 미션 잠금(`mapping_real_blocked`, 미션 비활성, Safety Context 증명 부재)은 건드리지 않았다.

## 1. 모드 차단의 실제 원인 (소스로 확인)
1. 서버 `OperatingModeMixin._mode_confirm_blocker`는 `agent.locked_velocity_arbiter is True`와 `state.velocity_arbiter`(operating_mode·blocker)가 있어야 MANUAL/MAPPING/NAVIGATION을 확인한다. IDLE만 증거 없이 통과한다.
2. `VelocityArbiter.__init__`은 `robot.mode == 'mock' and simulated`가 아니면 `ValueError`이다("never instantiated by the real agent"). 따라서 실차 Agent는 증거를 낼 방법이 없었고 모든 모드가 `agent has no velocity selector to confirm …`로 막혔다.
3. 그 앞에 두 개의 능력 검사가 더 있었다: MANUAL은 `manual_web_bench`가 true여야 하고(실차 웹 bench는 없다), NAVIGATION은 `missions_enabled`가 true여야 한다(실차는 `allow_missions:=false`).
4. 실차 launch에서는 Nav2 smoother(와 온보드 조이스틱)가 Watchdog에 **직접** 쓴다. 모드에 따라 소스를 고르는 장치가 없었다. Arbiter가 하던 일(모드별 소스 선택, 소유한 goal 확인, cancel 요청 시 즉시 정지, deadman 재무장)이 실차 경로에는 없었다.
5. 부수 발견: 실차 launch가 Watchdog을 직접 띄우도록 바뀐 뒤에도 supervisor 예시가 별도 `watchdog` 서비스를 띄우고 있었다(기본 파라미터). Watchdog 두 개는 Gate가 거부하는 구성이므로 launch 인자 `start_motion_watchdog`로 한쪽만 띄우게 했다.

## 2. 선택한 구조 (서버의 selector 요구를 없애지 않고, 가짜 ACTIVE를 만들지 않음)
운영 모드 **권한 확인**과 **운동 경로 강제**를 분리하되 둘 다 사실에서 도출한다.

| 구성 | 역할 | 재사용한 코드 |
|---|---|---|
| `mode_policy.py` + Watchdog `mode_gate` | 모드별로 **하나의 입력 채널만** 전달: MANUAL=`/fleet/manual_cmd_vel`(온보드 조이스틱), MAPPING·NAVIGATION=`/fleet/nav_cmd_vel`(Nav2). 다른 채널의 비영 입력은 소스 충돌 → FAILSAFE. nav 채널은 Agent가 소유한 goal(MAPPING=frontier, NAVIGATION=fleet)이 있고 cancel 요청이 없을 때만 전달. 새 grant는 같은 메시지 두 번(≥0.15 s), 좁히는 변경(IDLE·goal 종료·cancel)은 즉시 | Watchdog의 단일 발행 지점·FAILSAFE·재무장, 속도 clamp와 같은 `/fleet/speed_limit` 동기화·게시자 검증 |
| `mode_selector.py` (`RealModeSelector`) | Agent 쪽 증거 생산자. 서버가 읽는 `state.velocity_arbiter`와 같은 형태(`kind: WATCHDOG_SELECTOR`)로 보고하되 속도를 **발행하지 않는다**(`motor_output` 항상 false). `velocity_arbiter`의 `common_blocker`·`manual_blocker`·`mode`·`update_generation`을 **상속해 그대로** 쓰고 그래프 검사만 실차 형태로 바꿈 | `VelocityArbiter`(서브클래스) |
| Watchdog 상태의 `mode_policy` | 선택기의 `ready`는 **Watchdog 자신이 같은 모드와 revision을 보고**해야 성립(독립 프로세스의 확인) | Watchdog 상태 JSON |
| 서버 `_mode_confirm_blocker` 확장 | `velocity_selector == WATCHDOG_SELECTOR`이면 서버가 증거를 **다시 검사**: Watchdog 모드·revision 일치, 서버 자신의 표로 정한 채널, 두 입력 채널 exclusive, 충돌 없음, Agent 그래프의 게시자가 허용 노드, 속도 제한 증거의 `server_verified`와 같은 Watchdog boot | 기존 속도 제한 증거 검증 |
| MANUAL 입력 경로 | 실차에는 웹 bench가 없으므로 selector 그래프 증거(허용된 조이스틱 게시자 하나, 채널 exclusive)가 있을 때만 `manual input path`로 인정 | — |
| MANUAL deadman 재무장 | MANUAL 허용 시점에 이미 눌려 있던 조이스틱은 한 번 놓기 전까지 전달하지 않음(0 명령 또는 0.5 s 침묵으로 재무장) | Arbiter `manual_generation_blocker` 규칙 |
| 연결 단절 | Agent가 실행 불가가 되면 그 모드 revision은 영구 철회 — 재연결해도 새 revision(운영자의 명시 요청) 전까지 Watchdog에 모드를 다시 알리지 않음 | R1 권한 회수 |

DDS 노드 이름과 GID는 그래프 식별자이며 **인증이 아니다**. Watchdog 상태 JSON도 인증되지 않은 토픽이다. 이 설계는 "제어권 인증"을 주장하지 않는다.

## 3. 수정·추가된 파일 (이번 작업)
- 신규(Agent): `mode_policy.py`, `mode_selector.py`; (시험) `test_mode_selector_server_20261124.py`, `test_real_mode_selector_20261124.py`, `test_watchdog_mode_gate_20261124.py`, `e2e_real_agent_modes_20261124.py`, `e2e_real_agent_navigation_20261124.py`.
- 수정(Agent): `motion_watchdog.py`(두 번째 입력 채널, 모드 게이트, 소스 충돌, 재무장, `last_output`), `watchdog_speed_link.py`(모드·goal 전달, 연결 단절 철회), `agent_node.py`(`watchdog_mode_selector` 파라미터와 보고), `topics.py`.
- 수정(서버): `operating_mode.py`(선택기 증거 재검사, MANUAL 입력 경로).
- 구성: 실차 launch(조이스틱 → `/fleet/manual_cmd_vel`, Watchdog `mode_gate`, `start_motion_watchdog`), `start_robot_agent_field.sh`(`require_motion_watchdog`, `watchdog_mode_selector`), supervisor 예시(Watchdog 한 곳, 파라미터), 격리 bringup(`mode_selector` 인자), `run_isolated_fleet.sh`(`FLEET_BAG`), `field_preflight_20261123.py`(G1~G4), launch·하네스 시험 갱신.
- 서버 변경은 `operating_mode.py` 한 파일, UI 파일은 건드리지 않았다.

## 4. 격리 ROS E2E (실제 Fleet Server·Robot Agent(Arbiter 없음)·Watchdog·Gate·Nav2)
스택: `FLEET_FLAGS='speed_clamp:=true mode_selector:=true manual:=false velocity:=false'`. 측정 지점은 Gate 출력(`/cmd_vel_locked`). 비영 명령원: **MAPPING은 실제 웹 Mapping worker의 Frontier goal**, MANUAL은 `manual_mapping_teleop` 이름의 **조이스틱 stand-in**, NAVIGATION은 stand-in(Fleet 미션 goal 없음).

`e2e_real_agent_modes_20261124.py` **45/45 PASS**(`p44_real_agent_modes_i.log`):
| 시나리오 | 결과 |
|---|---|
| IDLE → MAPPING | ACTIVE, 증거의 모드·revision이 서버와 일치, nav 채널, 두 입력 exclusive, 같은 Watchdog boot |
| MAPPING 양성 대조 | Frontier/Nav2 명령이 smoother·Watchdog·Gate를 통과, 최대 0.100 m/s |
| PAUSE → RESUME | PAUSE 후 Gate 출력 0(소유 goal 종료), RESUME 후 MAPPING 복귀(합성 지도에 frontier가 남지 않으면 출력은 없음) |
| PAUSE → SAVE → STOP | SAVE 접수, 그동안 출력 0, STOP 후 STOPPED·출력 0 |
| MAPPING → IDLE, NAVIGATION 요청(미션 비활성) | `409 mode_unsupported`(실차와 같은 설정), 모드 IDLE·출력 0, IDLE에서 컨트롤러 stand-in 0.30은 전달되지 않음 |
| IDLE → MANUAL → IDLE | ACTIVE, 조이스틱 stand-in이 manual 상한 0.10으로 잘려 전달, IDLE 후 출력 0(조이스틱이 계속 눌려 있어도) |
| MAPPING ↔ MANUAL | 전환 요청 시 mapping 세션이 먼저 STOPPED, MANUAL ACTIVE까지 출력 0, 이후 조이스틱 전달, MANUAL → MAPPING 복귀 |
| 소스 충돌(MANUAL 중 nav 채널에 비영) | Watchdog `MOTION_SOURCE_CONFLICT`로 FAILSAFE, 선택기 ready 해제 |
| 중복 게시자(manual 채널에 두 번째 게시자) | `VELOCITY_INPUT_NOT_EXCLUSIVE`, 전달 0; 게시자가 있는 동안 MANUAL은 진입 거부, 사라진 뒤 재진입 가능 |
| Lease 상실(MANUAL 중) | 모드 IDLE/FAULT, 출력 0; Lease 재획득이 모드를 **복원하지 않음**; 명시 요청으로만 재진입 |
| Watchdog 재시작(MANUAL 중) | kill 시 출력 중지; 새 Watchdog은 새 writer라 **Gate가 fail-closed**(서버 모드가 여전히 MANUAL이어도 출력 0); 죽은 endpoint가 사라질 때까지 기다린 뒤 운영자 Gate reset; 정책은 Agent에서 두 번 받아 다시 학습 |
| Agent 재시작(MANUAL 중) | heartbeat 상실로 출력 0; 새 boot로 재연결하면 서버 모드는 IDLE이며 조이스틱이 계속 눌려 있어도 출력 0 |
| 서버 정지(SIGSTOP 8 s, MANUAL 중) | 출력 0; 서버 복귀 후에도 **조이스틱은 저절로 되살아나지 않고**(모드 revision 철회, 선택기 `WATCHDOG_MODE_NOT_CONFIRMED`), 명시적 IDLE→MANUAL 요청이 필요 |
| 눌린 채 MANUAL 진입 | 진입은 ACTIVE지만 눌린 조이스틱은 전달 0, 한 번 놓았다 다시 누르면 전달 |

`e2e_real_agent_navigation_20261124.py` **12/12 PASS**(미션 활성 Agent, Arbiter 없음, `p45_real_agent_navigation_c.log`): MAPPING으로 Nav2를 기동한 뒤 IDLE → **NAVIGATION ACTIVE**(Watchdog이 NAVIGATION·revision·nav 채널 확인), 소유한 Fleet goal이 없으므로 컨트롤러 stand-in의 0.30은 **전달 0**, NAVIGATION → MANUAL → NAVIGATION → IDLE 전환, IDLE에서 컨트롤러가 계속 0.30을 제공해도 출력 0. 미션 요청은 `409 no_map_loaded`/`mode_not_navigation`로 거부(기록).

## 5. 이 구성으로 검증하지 못한 것
- **Fleet 미션 goal과 그 취소(NAVIGATION → CANCEL → IDLE의 CANCEL)**: 실차형 Agent에서는 Safety Context의 실차 Gate 증명이 없고(격리 증명은 Arbiter가 필요) 저장 지도 로드가 필요해 미션이 admission되지 않는다. 시험한 것은 모드 진입·퇴장과 "소유한 goal이 없으면 전달 0"이다. 미션 취소는 Arbiter 프로파일의 기존 시험이 다룬다.
- 계약 변경 주입(`safety_contract_version` 불일치): 단위 수준(Agent가 실행 불가이면 모드를 IDLE로 알림, 서버 `agent_mode_view`가 계약 무효 시 IDLE)만. 격리 스택에서 주입하지 않았다.
- 실차 조이스틱(`teleop_twist_joy`)의 실제 메시지 패턴(해제 시 0 명령 여부), 실제 launch의 `manual_mapping_probe` 구성, vendor `limo_base`, STM32, 물리 정지.
- 모드 재확인: 서버는 ACTIVE 모드를 계속 재검증하지 않는다. 연결 단절 뒤처럼 Watchdog이 IDLE인데 서버가 MANUAL로 남는 불일치가 생길 수 있다(선택기 `blocker`와 속도는 0으로 정직하게 보고하지만 UI는 이를 별도로 표시해야 한다).

## 6. 설계상 주의점
- MANUAL 진입은 서버 `mode_unsupported`로 막혀 있던 것을 "그래프에서 허용된 조이스틱 게시자 하나가 보일 때"로 풀었다. 조이스틱이 없는 실차 launch에서는 MANUAL이 계속 거부된다.
- NAVIGATION은 서버가 `missions_enabled`를 요구하는 것을 유지했다. 실차는 미션 비활성이라 계속 거부된다.
- 속도 상한은 올리지 않았다(manual 0.10, auto 0.15 등 기존 ceiling과 Watchdog clamp 그대로). 두 번째 입력 채널도 같은 clamp를 지나고 scope는 채널로 정해진다(manual 채널 → manual scope).

## 7. 후속 (2026-11-25): Gate 재시작 실패(56/57)의 원인, 수정, 서버의 ACTIVE 모드 지속 검증 (격리 ROS, 미커밋)
로그: `local_data/raw_results/real_limo_integ_claude_20261122/field_ready_01/` 의 `p51`~`p55`(원인 추적 trace `p51_gate_restart_trace_*`, 수정 후 `p52_*`, 반복 E2E `p53_*`, 회귀 `p54_*`·`p55_*`).

### 7.1 56/57 실패의 원인 (실제 시각으로 분류)
추적 도구: `tests/probe_gate_restart_20261125.py`(MANUAL에서 Gate를 SIGKILL 후 재시작, 조이스틱 stand-in은 계속 누른 상태; Gate 상태·Watchdog 상태/모드/revision/출력 나이·DDS publisher 생성/소멸·Gate 출력을 시각과 함께 기록). **주의: 이 추적에서 `/cmd_vel_raw`를 직접 구독하면 Agent의 그래프 잠금이 `VELOCITY_GRAPH_NOT_LOCKED`로 모드 진입을 거부한다(Gate가 유일한 구독자여야 함). 올바른 fail-closed 동작이므로, `/cmd_vel_raw`는 Watchdog `last_output` 나이와 Gate 출력으로만 관측했다.**
- 수정 전 6회(2회 실행) 중 3회에서 재시작한 Gate 출력으로 비영 명령이 나갔다(첫 실행 2/3, 둘째 실행 1/3; 첫 실행 로그는 덮어써져 남아 있지 않고 위 수치는 그 실행의 콘솔 출력이다). 남아 있는 trace `p51_gate_restart_trace_before_fix.jsonl`의 iter 0 시각(실행 시작 기준 초): Gate kill 105.911, Watchdog `FAILSAFE`(SAFETY_GATE_STATUS_LOST) 106.389 → `LOCKED` 106.587, Gate 재기동 107.914, **Watchdog `MOTION_AUTHORIZED` 복귀 109.388**, **Gate 출력 최초 비영 109.360**(새 Gate의 `/cmd_vel_locked` DDS endpoint 등록 109.402, 이전 Gate의 죽은 endpoint는 126.002에 소멸 = 약 20 s 잔존), 이후 629개 비영 샘플. Watchdog 출력 나이가 250 ms를 넘은 샘플은 0.
- 해석: Gate는 설계대로(입력이 250 ms 넘게 없으면 "명령 없음"이라 보고 무장) 동작했다. 문제는 **Watchdog**이다. 비인가(LOCKED) 구간에는 0을 발행하므로 Gate는 명령이 없다고 보고 무장했고, Gate 상태가 돌아와 Watchdog이 AUTHORIZED가 되자 **눌려 있던 조이스틱이 해제 없이 곧바로 전달**됐다(MANUAL 재무장 플래그 `man_rearmed`가 이전 AUTHORIZED 구간의 True로 남아 있었음). Gate 시작 시 Watchdog이 이미 AUTHORIZED이던 iter 1·2(수정 전 trace)는 Gate가 INITIALIZING에서 명령 때문에 무장하지 않았고 출력 0이었다. 즉 재현 여부는 타이밍에 달렸지만 **원인은 타이밍이 아니라 정책 공백**(비인가 구간을 지나 같은 입력이 자동 재개)이다.
- 수정 안 한 것: Gate 코드와 계약(공백=무명령, 무장 규칙)은 그대로다.

### 7.2 수정한 정책
1. **Watchdog "해제 후 재개"**(`motion_watchdog.py`): AUTHORIZED를 벗어날 때마다(그리고 부팅 때) MANUAL 입력은 **한 번 해제(0 명령, 또는 manual 채널 0.5 s 초과 침묵)가 관측되어야** 다시 전달된다. 해제 관측은 입력이 버려지는 동안에도 모든 수신 메시지에서 한다. 이전 episode의 해제는 다음 episode에 이월되지 않는다. nav 채널은 영향 없음(FAILSAFE 때 소유 goal이 취소되므로 새 goal 없이는 재개되지 않음). 상태에 `manual_need_release`, `release_rearms` 카운터를 노출.
2. **서버의 ACTIVE 모드 지속 검증**(`operating_mode.py`, WATCHDOG_SELECTOR Agent 한정; Arbiter Agent 경로는 변경 없음): ACTIVE(MANUAL/MAPPING/NAVIGATION)인 동안 최신 Agent 보고의 Watchdog 증거를 확인 시점과 같은 규칙으로 다시 검사한다(Watchdog 모드·revision·선택 채널·`MOTION_AUTHORIZED`·두 입력 채널 배타성·소스 충돌·허용 publisher·속도 제한 증거의 Watchdog boot 일치). 추가로 확인 시점의 **Watchdog boot**와 **trip 횟수**(`watchdog_trips`, Watchdog status의 `trips`)를 모드 기록에 고정해, Watchdog이 재시작했거나 FAILSAFE를 한 번이라도 거쳤으면 같은 모드·revision으로 다시 AUTHORIZED가 되어도 이전 확인을 인정하지 않는다.
   - **모드 변경과 운동 허가의 분리**: `GET .../robots/{id}` 응답에 `mode_motion: {verified, blocker, hard, mode, revision}`를 추가했다(`verified` None=해당 없음, False=불일치).
   - 불일치(`verified False`)이면 신규 운동 명령과 임무 승인은 `409 mode_unverified`로 거부된다(`require_mode`: manual 명령, mapping START/RESUME, 임무 생성 공통 경로).
   - 신선한 보고가 모순되는 상태가 3.0 s 지속되면 모드를 끝낸다(IDLE, 매핑 세션이 미해결이면 FAULT). **trip 횟수 증가는 유예 없이 즉시** 끝낸다. 보고가 없거나 오래된 경우(오프라인·stale)는 모드를 바꾸지 않고 `verified False`(운동 거부)로만 표시한다. 끝난 모드는 Watchdog이 정상으로 돌아와도 자동으로 되살아나지 않으며 새 모드 요청이 필요하다.

### 7.3 검증 (격리 ROS·mock만; 실차 아님)
- **재현 시험 수정 후**: 같은 추적에서 Gate 재시작 4/4회 모두 눌린 조이스틱의 출력 0, 해제 후 새로 누르면 정상 비영 출력(양성 대조 148~150 샘플). 침묵(해제) 후 재입력 시나리오 2/2는 새로 누른 시각 22~50 ms 뒤에 출력이 시작됐다(`p52_gate_restart_after_fix_*`).
- **단위**: Watchdog 모드 게이트 21(신규 6: Gate 중단 중 눌린 조이스틱, 침묵/0 해제, episode별 해제, 부팅 시 눌린 조이스틱·정지 상태의 조이스틱, nav 채널 무영향, 모드 전환 후 재누름), 서버 선택기·ACTIVE 검증 25(신규 10), 선택기 9, R2 `test_motion_watchdog` 22, stall backlog 23, 속도 clamp 15. 전체 unittest discover 1186개 실행: 실패 집합이 **원 기준 15bb5c59의 discover(1091개)와 동일**(1 FAIL, 28 ERROR는 모두 기존 환경성 오류), 차이는 스크립트형 `test_speed_limits_server_20261123`이 import 시 실행되고 `sys.exit(0)`하여 discover에서 ERROR로 표시되는 것뿐(통과, 종료 코드 0).
- **모드 E2E 64항목**(`e2e_real_agent_modes_20261124.py`, 신규 7 + 정책 변경 반영): 수정 후 **깨끗한 새 스택 3회 연속 64/64 PASS**(`p53_..._run3/4/5.log`). 포함: Gate 재시작 중 눌린 조이스틱(출력 0, 해제 후 새로 누르면 이동), 입력 중단, Watchdog 동결·종료/재시작, Agent 동결, 서버 중단·재연결, Lease 상실·재획득, deadman 해제 후 재무장, 이전 모드 자동 재개 금지(서버 모드가 IDLE로 끝났고 Watchdog이 복구되어도 되살아나지 않음), 정상 비영 명령 양성 대조.
  - 수정 전 과정에서 남긴 실패 로그(보존): night_c 56/57(Gate 재시작), run1 60/63(M10 Watchdog 재시작 기대값과 M14 두 항목: 이 결과로 trip 고정을 추가), run2는 시험 도구의 `ros2 service call`이 30 s 시간초과로 비정상 종료(도구에 재시도 추가).
  - **정책 변경에 따른 기대값 변경 1건**: M10 "Watchdog 재시작 뒤 같은 MANUAL이 곧바로 재개"는 더 이상 정책이 아니다(Watchdog boot가 달라지면 이전 확인을 인정하지 않으므로 서버 모드가 끝나고 새 MANUAL 요청 뒤에만 움직임). 이 변경은 의도된 정책이며 출력이 그 정책과 일치함을 M14에서 확인한다.
- **다른 회귀(현재 코드 기준)**: 속도 clamp 프로브 18/18; 웹 속도 제한 E2E(`e2e_web_watchdog_speed_limit`) 43 PASS 종료 0; NAVIGATION 모드 진입 12/12; 실제 launch 추출 하네스 4회 중 3회 42/42, **1회 41/42(C4: 두 번째 `/fleet/speed_limit` writer 등장 시 전달 중단이 늦음, `last_fault=WATCHDOG_SELF_STALL`; 적용된 한도는 0.06 그대로여서 상향은 수용되지 않았으나 지연 구간의 출력 크기는 기록하지 않았음. 재현되지 않았고 원인은 격리하지 못했으며 이번 변경이 속도 clamp 코드를 건드리지 않았다. 열린 항목)**; 필드 체인 프로브 3회: 28/29(첫 점검의 DDS 노드 이름 미발견), 23/28(합성 센서의 `lidar_stale`로 Gate가 FAILSAFE: 기존 간헐 문제), 29/29.
- **검증하지 못한 것**: 실제 조이스틱 메시지 패턴(정지 시 0을 보내는지 침묵하는지), Gate가 실차에서 재시작될 때 DDS 잔존 시간, vendor/STM32/E-Stop, 계약 변경 주입.

### 7.4 Fleet 임무 승인·취소(실차형 Agent)의 읽기 전용 분석과 제안 — BLOCKED(사용자 결정 필요)
코드로 확인한 차단 지점(모두 **의도된 실차 차단**이며 이번에 완화하지 않았다):
1. `agent_node.py:83` `allow_missions`는 real 모드에서 "승인된 물리 모터 격리 증거 없음"으로 거부. → 서버 `operating_mode.py:244`: NAVIGATION 진입은 `missions_enabled`를 요구(`mode_unsupported`).
2. `agent_node.py:96` `navigation_localization`(저장 지도 map_server/AMCL worker)은 SIM/mock + 격리 localhost 도메인에서만 허용 → 서버 `navigation.py:149`가 `localization_unavailable`로 LOAD/INITIAL_POSE/임무를 거부.
3. `safety.py:80-87` Safety Context의 Gate STALL_PENDING 연속 확인은 SIM r7 증명(`sim_r7_report_valid`: "real Agent 보고에는 절대 사용 불가") 또는 `locked_mock_proof`(mock + `locked_velocity_arbiter` + motor path LOCKED)만 인정. 실차형 Agent는 둘 다 만족하지 못한다. 일반 admission(Gate NORMAL)은 `state.safety_gate`, 실행 보고, Watchdog epoch, 지도·위치 확정이 맞으면 통과하는 구조라 (1)(2) 없이는 시험 자체가 불가능하다.
제안(계약을 약화하지 않고, 단계별): (a) 실차에서 먼저 **모터 경로 LOCKED 전용 "건식 임무"**: `allow_missions`와 별도로 `real_nav_dry_run`(기본 false, 모터 잠금과 WATCHDOG_SELECTOR가 둘 다 확인될 때만 허용)을 두고, 이 경우에만 저장 지도 localization worker를 real 모드에서 허용; (b) `locked_mock_proof`를 확장한 `locked_motor_proof`(mode real + `motor_path.status == LOCKED` + WATCHDOG_SELECTOR + Watchdog evidence `server_verified`)를 연속 확인 증명으로 인정 — 모터가 실제로 구동되지 않으므로 취소·승인 시퀀스를 로봇 하드웨어에서 검증할 수 있음; (c) 모터 잠금 해제(진짜 임무)는 별도 단계로 E-Stop·STM32 타임아웃·제동거리 측정값과 사용자 승인이 있을 때만; (d) 취소 경로는 기존 Nav2 취소 + Watchdog 소유 goal 취소 규칙을 그대로 사용하며 새 코드 없음. 이 제안의 (a)(b)는 안전 계약 변경이므로 사용자 승인 전에는 구현하지 않았다.

### 7.5 후속(2026-11-26)
조이스틱 재무장 정책(침묵은 해제가 아님), 건식 Fleet Mission(`real_nav_dry_run`), C4 조사, AUTO clamp 비교는 `REAL_NAV_DRY_RUN_20261126.md`에 있다. 이 문서 §7.2의 "0.5 s 침묵 = 해제" 서술은 그 문서의 정책으로 **대체**되었다(해제는 명시적 0 명령만).
