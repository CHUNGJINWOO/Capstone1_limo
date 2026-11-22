# 수동 세션 안전 계약 — 격리 분석 및 통합 제안

작성일 2026-11-20. **제안·시험 문서이며 운영 서버 또는 Agent의 수정·배포가 아니다.** 상위 근거 [수동 조작 통합 감사](MANUAL_CONTROL_INTEGRATION_AUDIT_20261120.md). 비교 시 staging HEAD `c804241ac9afb702616c7e9cd59478c8ea077a8f`, 공개 `origin/main` `ea2d9632408de51365fe9005a7e828f1c99a0b0b`. 현재 `fleet_control/`은 staging Git에 미추적이므로 Git diff만으로 이전 사용자 또는 Claude의 변경을 식별할 수 없다. 본 작업은 공통 웹·서버·Agent·실차 launch를 수정하지 않았다.

병렬 작업 확인: 조사 중 `agent_node.py` SHA-256이 `15e78713606500d9e98f6b9c36281d5ba1ba96ee56a511ec4fbd55e8fa01bfd3`에서 `6d6a4ef34ef1c2fd3d752f2233c65dd414d91810c4126492624c3c8da57a5a70`으로 바뀌었다. 이 파일은 미추적이어서 완전한 이전 스냅샷과 실제 diff를 확보하지 않았으며, 변경 원인·작성자·완료 상태를 단정하지 않는다. 변경 후 현재 코드를 다시 읽어 web bench의 real-only/모터 잠금 제한이 유지되는 것을 확인했고, 아래 HTTP/Agent fault 시험은 변경 시각 이후 실행됐다. Claude 완료가 확인되면 전체 관련 파일과 시험을 다시 대조해야 한다.

사용자는 확인 질문에 Claude의 공통 작업이 **아직 진행 중**이라고 답했다. 따라서 완료 후 최종 staging 재대조와 공통 파일 통합은 이번 결과에서 SKIP한다.

## 1. Offline 거부 후 세션 재수락의 원인

staging `service.py`의 `manual_bench`는 Fleet lock 안에서 Lease 확인 → 기존 `current` 조회 → `_manual_bench_robot()`의 online·reconciled 등 검사 → action/session/epoch/boot/TTL/seq 검증 순서다. `_manual_bench_robot()`은 offline이면 `ApiError(409)`를 발생시키지만 기존 `manual_benches[robot_id]`를 지우지 않는다. 재접속 뒤 같은 Lease epoch·Agent boot·session_id이고 monotonic `until`이 아직 남아 있으면 증가한 seq가 다시 수락된다. 이 오류 경로는 실제 수동 구동이 아닌 **영속도 벤치 세션**에서 확인했다.

`hello()`·`housekeeping()`의 `invalidate_movement()`는 Fleet navigation 명령·임무의 불확실성을 처리하지만, 현재 `safety.py`의 해당 함수는 in-memory `manual_benches`를 폐기하지 않는다. `hello`는 `state`, `state_ts`, `reconciled`를 초기화하므로 재수락에는 상태 복원·재조정도 필요하다. 기존 단위 재현은 이를 모의로 빠르게 복원한 것이며, 실제 프로세스의 250ms 이내 재조정 성공을 입증하지 않는다.

공개 `ea2d9632`는 `_manual_bench_robot()`의 offline/Mapping/Nav2 거부 시 `manual_benches.pop()`을 수행한다. 다만 이는 해당 실패 경로가 **실제로 호출된 경우**다. 공개 `hello`는 Agent boot가 바뀌었을 때만 기존 manual session을 제거하므로, 오프라인 중 서버가 어떤 요청도 받지 않았거나 동일 boot 재연결이 매우 빠른 경우의 완전한 fencing은 별도 증명이 필요하다.

## 2. 현재 검증 범위 — 구동 허용 근거 아님

| 값 | staging `manual-bench` | 공개 `manual-bench` | 통합에서 필요한 조건 |
|---|---|---|---|
| `session_id` | start마다 UUID; sample/stop은 현재 ID와 일치 필요; pull은 내부 session을 Agent에 제공 | 동일, 추가 ticket | ID를 robot·owner·lease epoch·양쪽 boot·모드 세대에 결박하고 이벤트 발생 시 즉시 폐기 |
| `lease_epoch` | operator start/sample/stop의 owner와 epoch를 매 요청 검사, pull도 현재 epoch 검사 | 동일 | 획득/갱신/만료/양도/force release와 session 폐기를 하나의 일관된 lock·transaction 경계에서 처리. 재획득에 이전 세션을 연결하지 않음 |
| `server_boot_id` | Fleet 인스턴스마다 생성, `manual_benches`는 메모리 전용이라 새 인스턴스는 비어 있음. sample body 또는 Agent pull 응답의 독립적인 세대 결박은 없음 | 동일한 in-memory 구조; sample에 명시적인 server boot 결박 없음 | 응답·세션·Agent 로컬 승인에 boot 세대 결박. 재시작/서버 연결 재수립 시 구 승인 무효, 새 handshake 전 0 |
| `agent_boot_id` | 세션 생성 때 저장; sample은 현재 Agent boot와 비교; pull은 요청 boot를 `check_session`으로 확인. 같은 boot의 offline/reconnect는 별도 세대 없음 | Agent boot 검증과 새 boot일 때 hello 중 세션 제거. 동일 boot의 재연결 세대는 별도 명시되지 않음 | 매 hello/reconnect마다 별도 재조정 세대 생성. 같은 boot 재연결도 기존 session 무효. Agent가 로컬에서도 boot/세대 일치를 확인 |
| 입력 `seq` | 세션별 정수, `current.seq < seq <= 2**53-1`; 한 번 승인한 seq의 중복·역순 거부 | 동일 + 현재 ticket 일치, 성공 시 ticket 변경 | session 세대별로 엄격 증가, 중복 pull은 유효 시간 연장 불가; 새 session 이외에서 seq reset 불가 |
| `ticket` | 없음. ticket 관련 시험은 SKIP | sample마다 일회용 ticket, 성공 응답에서 교체, 이전 ticket 재사용 거부 | public 계약을 재사용한다면 발행 시각/서버 단조 만료, session·epoch·boot·모드 세대에 결박. staging 정규화 [-1,1]와 공개 physical 단위를 명시적으로 구분, 실패 시 자동 fallback 금지 |
| Deadman/입력 만료 | 서버 TTL 250ms, Agent poll은 요청 시작 단조시각 + 서버 잔여 TTL. deadman=false여도 서버 sample의 입력값은 보존하지만 Agent 필터가 0으로 처리. Agent는 raw에 0만 발행 | 서버도 deadman=false일 때 sample 값을 0으로 저장; 벤치 snapshot은 TTL 만료 시 0 | release·timeout 순간 0/세션 폐기; Agent 및 독립 Watchdog이 서버나 브라우저에 의존하지 않고 만료. 새로운 수동 입력은 명시적 새 세션 + Deadman 재입력 |

공개 ticket/시퀀스와 staging 기존 API는 서로 호환되지 않는다. 기존 패널은 staging normalized 값을 ticket 없이 보낸다. 값을 변경하거나 public API로 통합하기 전에 호환 계약을 하나로 결정해야 한다.

## 3. Mapping·Nav2와 원자적 모드 조건

서버와 Agent의 **별도** 결정은 모두 다음을 만족해야 한다. 이 제안은 현재 구현됐다는 뜻이 아니다.

1. `AUTO`, `MAPPING`, `MANUAL`, `UNKNOWN`을 로봇별 단일 모드 세대로 관리한다. start/sample/pull과 Mapping/AUTO start/전환이 동일 권한 lock/transaction 하에서 상호 배타적이어야 한다. 단순한 웹 태그나 취소 요청은 전환 증거가 아니다. Mapping `STARTING/MAPPING/PAUSING/PAUSED/SAVING/STOPPING/UNKNOWN` 또는 외부 Mapping 동작 여부가 불명이면 manual admission은 거부하고 기존 세션도 폐기한다. 공개 `mapping_blocks_motion`은 Fleet 세션을 검사할 뿐 외부 실행의 부재를 증명하지 않는다.
2. `NavigateToPose`와 `FollowPath`를 최소 필수 범위로, 실제 활성화 가능한 `NavigateThroughPoses`, `Spin`, `BackUp`도 포함한다. 유효한 Action 상태 배열을 수신하지 못했거나, 실행·수락·취소 중 Goal이 있거나, terminal 결과/소유 UUID/서버 세대·발행자 ID가 모호하면 `UNKNOWN`으로 둔다. Agent 소유 handle이 없어도 외부 Goal이 없다고 단정하지 않는다. 후행 status와 명시적 완료 결과를 대조하기 전에는 MANUAL을 허용하지 않는다.
3. **상태 누락·지연·오류는 fail closed**: 유일한 기대 발행자와 식별자의 연속성, 실제 배열 수신, 오류 없는 수신 및 유효 시간/세대 근거가 필요하다. ROS Action 상태는 변화 시에만 발행될 수 있으므로 오래된 배열만 보고 무기한 idle이라고 추정하거나, 반대로 평상시가 항상 일정 주기로 갱신된다고 가정해서는 안 된다. 별도의 검증된 liveness/상태 증거 생산과 만료 정책을 합의해야 하며, 그 근거가 없으면 계속 차단한다. PID·DDS graph만으로 출처 인증을 주장하지 않는다.
4. 서버 lock은 ROS 외부 Goal이 lock 밖에서 발생하는 것을 막지 못한다. 따라서 진입 직전의 단일 검사만으로 원자성을 주장하지 않는다. Agent/독립 Watchdog이 ROS 입력 및 Goal의 변화를 지속 감시하고 경계가 깨지면 0, 명령 취소 및 수동 세션 무효화를 해야 한다. 완전한 소스 통제/접근 인증이 없는 경우 그 한계를 차단 조건으로 남긴다.

현재 staging은 수동 start에서 이 Mapping/외부 Goal 증거를 검사하지 않는다. 공개 서버는 Fleet Mapping 상태와 일부 Nav2 상태를 검사하지만 상태 배열 미수신을 놓치며, 공개 Arbiter도 Mapping을 독립적으로 검사하지 않는다. Agent·서버 중 한쪽의 검사만으로 통합을 승인할 수 없다.

## 4. 구현 시 서버·Agent 안전 불변조건

**서버:** operator 인증과 살아있는 lease owner/epoch, robot ID, server boot, agent boot + reconnect 세대, 유일한 session ID, 현재 모드 세대 및 증거를 매 start/sample/pull에서 확인한다. seq/ticket은 선형화하고, 서버가 발행한 입력 권한의 deadline을 검증해 지연·재전송·순서 역전이 수명을 연장하지 않게 한다. Mapping·Goal·Gate 증거가 악화되거나 단절/재시작/Lease 만료·양도/epoch 변경/Deadman release가 관찰되면 동 lock 안에서 sample·session·ticket을 폐기한다. 새로운 operator의 수동 제어는 새 session과 fresh proof를 요구한다. 권한 있는 STOP은 안전 방향이지만 물리 E-Stop의 대체가 아니다.

**Agent:** 서버 상태와 별도로 수신 boot/세대/session/seq/TTL을 비교하고 로컬 단조시간 deadline을 준수한다. 동일 seq를 중복 pull해도 초기 deadline을 뒤로 밀지 않는다. Agent 재시작, server boot 변화·연결 단절/재연결, lease 만료, Gate/odom/Watchdog stale, Mapping 미확인·Nav2 Goal 불명, 입력 에러·Deadman 해제 시 0 및 재무장 필요 상태로 고정한다. finite·범위와 속도 상한은 서버뿐 아니라 Agent/Arbiter에서도 검사한다. Agent가 멈춘 상황은 Agent 자신의 timer가 아니라 독립 Watchdog과 Gate가 처리한다.

**Watchdog/Gate:** 입력→검증된 Arbiter/속도 제한→독립 Watchdog→Safety Gate→잠금 출력 순서, fail closed, 명령 나이·liveness·중복 publisher를 주기적으로 검사한다. Gate와 물리 모터 컨트롤러가 마지막 속도를 유지하는 상황의 독립 정지와 실물 E-Stop, MCU timeout, 브레이크 거리는 아직 미확인이다. 실차 잠금 해제 금지.

**확인된 Watchdog 추가 불일치:** staging `motion_watchdog.py`의 `idle_proven()`은 `nav_active in (None, 0)`을 허용한다. publisher 1개와 합성 cancel-all의 성공·빈 응답이 주어질 때 **상태 배열을 받은 적이 없어도** `MOTION_AUTHORIZED`로 전환되는 것을 callback 단위 하네스에서 재현했다. `test_unreceived_status_is_not_idle_proof`의 기존 시나리오는 cancel 서비스가 준비되지 않은 별도 조건이어서 이 경우를 덮지 않는다. 이는 이번 사용자의 fail-closed 요구와 충돌하므로 통합 차단 조건이다. [예상 실패 로그](../../local_data/raw_results/manual_session_watchdog_missing_status_v2_20261120.log).

## 5. 이번 검증: PASS / FAIL / SKIP

| 시험과 범위 | 결과 | 증거/한계 |
|---|---|---|
| 현재 staging Fleet 메서드 + 모의 DB: stale ID/epoch/seq, 두 동시 브라우저, NaN/Inf, 순서 역전, Deadman false·TTL, 새 Fleet 메모리·Agent boot/Lease 세대 | PASS 5 / FAIL 7 / SKIP 1 | [로그](../../local_data/raw_results/manual_session_adversarial_staging_v3_20261120.log). FAIL 7은 예상 실패: 활성 Nav2, Mapping, 상태 배열 미수신·지연·오류, 중복 발행자, offline 후 session 재수락. Ticket은 staging 미구현으로 SKIP. 동시 브라우저는 같은 owner의 두 thread다. 새 Fleet 메모리는 객체 모델이며 OS 프로세스 재시작은 아님. |
| 공개 Fleet 메서드 + 모의 DB | PASS 11 / FAIL 2 | [로그](../../local_data/raw_results/manual_session_adversarial_public_v3_20261120.log). 이전 ticket 단회 사용·Mapping 상태·활성 Goal·offline 요청 후 폐기 등을 확인. FAIL은 상태 배열 미수신 및 시험이 추가한 오래된 `age_s`를 admission에서 무시하는 경우다. 공개 Agent는 `last_change_age_s`를 보고하므로 이 synthetic 필드 시험을 실제 지연 관측으로 보지 않는다. 모든 Nav2 Action 및 잠복 중 통신 단절은 미시험. |
| staging Agent의 늦은 응답 | PASS | [로그](../../local_data/raw_results/manual_session_agent_delay_20261120.log). 모의 HTTP 응답을 320ms 지연해 TTL 250ms 이후 도착시키고 실제 `ManualWebBench`가 expired/0을 계산함. 네트워크 실제 지연은 아님. |
| Fleet HTTP 중단·재생성 + Agent + 기존 Gate 코드 | PASS | [로그](../../local_data/raw_results/manual_session_server_restart_20261120/test.log). 격리 214·localhost·합성 센서. HTTP 서버 thread 종료 후 동일 DB로 **새 Fleet 객체와 HTTP 서버를 같은 테스트 프로세스에 재생성**, 이전 세션 409, raw/locked 0, `/cmd_vel` publisher 0. OS 서버 프로세스 전체 재시작은 별도 시험 필요. 원래부터 영속도 벤치이므로 비영속도 제어 중 정지 시험이 아니다. |
| 실제 Agent 프로세스 SIGKILL·재시작 + Gate | PASS | [로그](../../local_data/raw_results/manual_session_agent_restart_20261120/test.log). 동일 격리 설정, 새 Agent 프로세스 후 이전 세션 409, raw/locked 0, `/cmd_vel` publisher 0. 새 세션 자동 생성 여부의 실차 증명은 아님. |
| 공개 Arbiter 비영속도 입력 중 Agent SIGSTOP | PASS (범위 제한) | [로그](../../local_data/raw_results/manual_arbiter_freeze_20261120/test.log). 격리 213, fake HTTP/Nav2 + 합성 센서와 pass-through smoother. Gate 실제 코드에 0.08 m/s가 들어가는 상태에서 Agent 프로세스를 멈추자 Gate `cmd_vx_out=0`까지 0.293초, 이후 `/cmd_vel_locked` 22개 모두 0, `/cmd_vel` publisher/subscriber 0. 독립 Watchdog 경유는 아니며 1회 실험이다. SIGSTOP 프로세스는 재개하지 않고 시험 소유 프로세스만 종료했다. |
| Nav2 status 미수신 + Watchdog | **FAIL** | [로그](../../local_data/raw_results/manual_session_watchdog_missing_status_v2_20261120.log). 합성 callback·cancel 응답에서 `MOTION_AUTHORIZED` 발생. unittest의 `expected failure`는 안전 성공이 아니다. |
| 실제 18190 브라우저→HTTP→Agent→Arbiter→독립 Watchdog→실제 Gate 종단 | SKIP | staging에는 공통 파일 변경 전 Arbiter 없음, 안전 계약·상태 근거 미완성. |
| Gazebo, 실제 LIMO 잠금·구동 | SKIP | 실행하지 않았다. |

테스트 하네스: [메서드 경계](../tests/test_manual_session_adversarial_20261120.py), [지연 Agent](../tests/test_manual_delayed_agent_20261120.py), [격리 HTTP/Agent fault](../tests/manual_session_faults_isolated_20261120.py), [Arbiter 정지](../tests/test_manual_arbiter_freeze_20261120.py), [Watchdog 미수신](../tests/test_manual_watchdog_missing_status_20261120.py). 기존 실험 하네스를 **새 테스트 파일에 복사**해 확장했으며 원본과 로그는 덮지 않았다.

## 6. Claude 작업 완료 후 파일별 최소 통합안

Claude의 완료 여부가 확인되기 전에는 공통 파일을 변경하거나 통합 PASS라고 표시하지 않는다. 완료 확인 시 `git status --short --untracked-files=all`과 위 파일 해시를 다시 확인하고, 전체 미추적 `fleet_control/` 상태와 새 Mapping/Registry 소스를 직접 비교한다. 저장소 최상위 Git diff만으로 미추적 파일 내부의 병렬 수정은 식별되지 않으므로 이 문서의 해시와 담당자 작업 내용을 대조한다.

1. `static/index.html`: 기존 control 패널에 호스트 1개 및 독립 CSS/JS 로드. 기존 화면·Mapping markup 보존.
2. `static/app.js`: 기존 인증 `api()`에 독립 패널의 `mount()`를 연결; 현재 로봇·operator·lease·fresh Gate/모드/boot 증거만 읽는 안전 context 제공; 증거 미존재는 비활성. 기존 gamepad panel과 하나의 세션 소유권만 갖게 정리. Claude가 편집한 코드와 수동 병합.
3. `static/manual_control_panel.js`: staging normalized 입력과 public physical 입력/ticket 계약 중 하나를 명시적으로 선택하고, 한 요청씩 ticket/seq를 갱신한다. 숨김/blur/touch 종료/오프라인/Deadman 해제 시 stop 및 재무장, 늦은 응답으로 세션 부활 금지. 이전 후보는 public ticket을 보내지 않으므로 그대로 public에 연결 금지.
4. `service.py`·`ownership.py`·Mapping 담당 코드: 기존 Lease·Mapping transaction 보존. 위 세션 identity와 서버 단 admission/지속 검증·이벤트별 invalidate를 같은 lock 경계에 추가. Mapping/mission 요청과 manual 승인/종료를 교차 race 테스트한다. staging recovery의 `terminal_evidence_unavailable` 차단을 약화하지 않는다.
5. Agent `agent_node.py`, `manual_web_bench.py`, 공개 Arbiter 이식 후보: 현 staging의 real 벤치 영속도만 발행하는 제한과 공개 mock-only Arbiter 제한을 먼저 유지하고, 상호 배타 모드·모든 Goal/상태 증거·수동 세대/TTL을 로컬에서 검증한다. `motion_watchdog.py` 상태 미수신 차단을 수정·검증하는 작업은 소유자 통합 후 별도로 진행한다. 운영 실차 launch는 변경하지 않는다.
6. 이후 panel→실제 HTTP→Agent→Arbiter→독립 Watchdog→Gate(`/cmd_vel_locked`)를 격리 harness로만 조립하고 동시성·지연·나이·모드 전환·장애/재시작을 재시험한다. 성공해도 바로 실차 모터를 풀지 않는다.

**미해결:** 18190 공통 UI 결합, 전체 종단 모터 잠금 체인과 Watchdog 누락 배열 수정, 모드/ROS 외부 Goal의 신뢰 가능한 종결 증거, 동일 boot 재연결 race·상태 전달 지연, 물리 E-Stop/MCU/제동 검증. 이 문서 외 공통 문서 수정 없이 향후 `CODE_INDEX.md`, `CURRENT_STATUS.md`, `VALIDATION.md`, `HANDOFF.md`에는 Claude와 통합한 변경 및 실제 실행한 검증만 별도 반영한다.
