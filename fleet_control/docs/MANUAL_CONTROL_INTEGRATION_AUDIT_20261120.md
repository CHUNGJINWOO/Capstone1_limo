# 수동 조작 통합 감사와 잠금 Arbiter 시험 — 2026-11-20

## 비교 기준과 작업 보호

- staging `HEAD=c804241ac9afb702616c7e9cd59478c8ea077a8f`: `fleet_control/`은 현재 작업트리의 미추적·미커밋 실제 파일을 읽었다. 공통 `service.py` SHA-256 `10e8215737d13213ccc0676072766666501462661da8baf00beec9d4ac271962`, `ownership.py` `466338b4591f3e77e5842e7bbb0f39132591b1bdecf30be622e1711090e0f5db`, Agent `manual_web_bench.py` `67783e057f14d1ee30756cedf4a25134470c8b4ff2e3d8847788c82cf7007d15`. 동시 작업 중이므로 통합 직전 다시 확인해야 한다.
- 공개 기준선은 비교 시점 `origin/main=ea2d9632408de51365fe9005a7e828f1c99a0b0b`; `git show <commit>:<path>`로 읽었다. 공개 `manual_web_bench.py` SHA-256 `89f9cfc4a9ad30d809d8df67ef0fe8c4db7cc86b6ded809f8f148545cc802743`, `velocity_arbiter.py` `ddcad3a50051e61b32185dee69936692cab9217af1b06d16ed5e91561d377740`. ROS 시험은 이 두 해시를 검사한 임시 사본을 가져온다.
- 이 작업에서 공통 `index.html`, `app.js`, 서버·Agent·실차 launch, 기존 실험·로그를 수정하지 않았다. 공개 기준선과 staging 전체를 덮어쓰거나 한쪽 파일을 그대로 복사하는 통합은 금지한다.

## 파일별 구현 비교

| 대상 | 현재 staging | 공개 `ea2d9632` | 안전상 차이 |
|---|---|---|---|
| `server/fleet_server/service.py` | `_manual_bench_robot`: fresh/reconciled `real`, `simulated=false`, `missions_enabled=false`, 벤치 opt-in, 미완료 Fleet mission 거부. 250ms 세션, Lease owner/epoch, Agent boot, 단조 seq, [-1,1] 입력 확인. Agent로부터 독립적인 모터 잠금 근거·Mapping 상태·외부 Nav2 Goal 종료 증거는 여기서 확인하지 않음. | opt-in된 `mock`, simulated, missions disabled만 허용. 매 start/sample/pull에서 `mapping_blocks_motion` 검사, 명시된 active/unknown NavigateToPose 거부, locked arbiter일 때 FollowPath idle 요구. 매 sample마다 일회용 ticket 교체; 선속도 [0,0.10] m/s·각속도 [-0.30,0.30] rad/s. | staging 벤치 시작 시 Mapping/UNKNOWN과 외부 active Goal이 허용되는 것을 모의 Fleet 객체로 재현했다. 실제 Agent는 별도 모터 잠금·영속도 방어를 수행하므로 이 사실을 모터 구동 허용으로 해석하면 안 된다. 공개 서버도 `received_status_array`를 확인하지 않아, publisher만 있고 수신 상태 배열이 없는 `known=true, active_count=0`을 승인하는 조합을 재현했다. 공개 Mapping 검사는 Fleet의 세션 기록 범위이며 외부에서 별도 실행된 Mapping 부재의 증거는 아니다. |
| `server/fleet_server/ownership.py` | Lease owner/epoch와 만료를 확인하며 타입·유한성, DB 트랜잭션이 강화됨. `recovery_evidence`는 상태 수신·신선도와 Goal 수 등을 보지만 실모드에서는 인증된 terminal 근거가 없어 `terminal_evidence_unavailable`로 복구 거부. | Lease 기반 독점과 epoch, `recovery_evidence`의 online/reconciled·상태 시간·Nav2 known/active_count 검사. 빈 상태 수신 여부·Goal 기록의 명시적 종료·모든 보조 Action에 대한 충분한 근거는 증명하지 못함. | 두 버전의 Lease/복구 코드는 벤치 start의 완전한 권한 결정이 아니다. staging의 fail-closed 회복 정책을 약화하지 말고 별도 수동 세션 무효화와 원자적 상태 전이를 결합한다. |
| `agent/.../manual_web_bench.py` | opt-in real 벤치 전용: HTTP poll TTL≤250ms, Gate/odom 신선도, Agent 활성 Goal 유무, 단일 Gate·속도 발행 그래프 점검. 요청 선/각속도를 보고하지만 `/cmd_vel_raw`에는 **항상 0**만 발행, `/cmd_vel` 발행자 발생 시 차단. `robot.active`만으로 외부 Nav2 Goal 종료는 증명할 수 없고 Mapping 상태를 직접 검사하지 않음. | mock 전용 상태 계산기로 ROS 속도 발행자 없음. connected, TTL, NavigateToPose publisher 1개·active_count 0·`robot.active is None`을 검사. `received_status_array`, FollowPath 및 Mapping 상태를 이 벤치 클래스 자체가 확인하지 않음. | 공개 Arbiter가 추가 검사를 수행할 때와 벤치 단독 상태 계산을 구분한다. 격리 시험에서 상태 배열 미수신·FollowPath 불명일 때 벤치 snapshot은 양수였으나 Arbiter 출력은 0이었다. |
| `agent/.../velocity_arbiter.py` | 현재 staging 경로에는 없음. | mock·잠금 전용 구현: `/fleet/cmd_vel_nav`과 벤치 상태에서 AUTO/MANUAL 선택, `/fleet/cmd_vel_selected` 발행. `/cmd_vel` 발행·구독 0, Gate→`/cmd_vel_locked`, smoother·Gate graph 검사, Gate NORMAL, 활성 Goal 없음, Nav2 status 수신, FollowPath idle을 요구. | Mapping 상태 및 Lease·서버 세대의 독립 검사는 Arbiter 안에 없다. 공개 구현은 독립 watchdog을 이 클래스 그래프에 요구하지 않고 생산 안전 인증도 아니다. 선택 자체는 0.10 m/s·0.30 rad/s 범위다. |

공개 Agent의 일반 `_nav2_goals()`는 NavigateToPose 상태 위주이고, `follow_path_idle()`은 단일 controller 발행자와 한 번 active를 관측한 뒤 terminal 관측을 요구한다. 이것은 도움되는 제한이지만 NavigateThroughPoses·Spin·BackUp을 포함한 모든 외부 Goal의 생애주기와 재시작 뒤 종료 증명을 대체하지 않는다. staging `manual_mapping_input.py`의 r8 SIM-only Gate 증거는 여러 Action과 publisher GID를 검사하지만, 이를 실차에서 인증된 종결 증거라고 주장할 수 없다.

## 재현: Mapping·외부 Goal 차단과 한계

`tests/test_manual_bench_version_comparison.py`는 두 버전의 **실제 Fleet 클래스**를 따로 import하고 DB·Mapping 상태를 합성한 메서드 단위 시험이다. [staging 로그](../../local_data/raw_results/manual_contract_staging_20261120.log) / [공개 로그](../../local_data/raw_results/manual_contract_public_20261120.log). 테스트 출력의 `OBSERVED GAP`은 의도적으로 현 결함을 입증하는 것이며 안전 PASS가 아니다.

- staging: `MAPPING`, `UNKNOWN`, 외부 Goal active_count=1, FollowPath 증거 없음 각각에서 벤치 start 수락. 미완료 Fleet mission은 별도 기존 검사로 거부한다. 입력은 실제 모터가 아닌 영속도 벤치에만 도달한다.
- 공개: `MAPPING`/`UNKNOWN`, NavigateToPose active_count=1, locked arbiter의 FollowPath 미확인은 거부. 그러나 `known=true,publishers=1,active_count=0,received_status_array=false`는 서버에서 수락한다.
- 별도의 동일 프로세스 모의 상태 시험에서, staging은 `online=false`인 sample 요청을 거부한 다음 online이 즉시 복원되면 250ms가 지나기 전 같은 session의 다음 seq를 수락했다. 공개 서버는 해당 오프라인 요청을 처리할 때 세션을 지워 재수락하지 않았다. 이는 **오프라인 중 요청이 도착한 조건**의 비교이며 실제 Agent 종료·재연결 전체 E2E 결과는 아니다. 요청 없이 짧게 단절된 경우나 같은 boot 재연결의 재무장 정책은 별도 검증이 필요하다.
- 양쪽에서 다른 Lease 소유자와 NaN sample은 거부했다. 위는 synthetic service 단위 시험이지 실제 HTTP·로봇·모든 Nav2 Action 재현이 아니다.

같은 비교를 다시 실행할 때는 공개 패키지를 별도 임시 경로에 풀고 두 Python 프로세스에서 각각 import한다. 이 테스트는 현재 관찰한 결함을 재현하는 회귀 기준이므로, 담당자가 결함을 수정하면 예상값을 안전한 거부로 바꿔야 한다.

```bash
scratch=$(mktemp -d /tmp/manual-contract-XXXXXXXX)
git archive ea2d9632408de51365fe9005a7e828f1c99a0b0b fleet_control/server/fleet_server | tar -x -C "$scratch"
PYTHONPATH=fleet_control/server MANUAL_CONTRACT_VERSION=staging python3 fleet_control/tests/test_manual_bench_version_comparison.py
PYTHONPATH="$scratch/fleet_control/server" MANUAL_CONTRACT_VERSION=public python3 fleet_control/tests/test_manual_bench_version_comparison.py
```

## 이번 격리 ROS 시험

[시험 코드](../tests/test_public_manual_arbiter_locked_ros.py) · [결과 로그](../../local_data/raw_results/manual_control_public_arbiter_20261120.log) · [Gate 로그](../../local_data/raw_results/manual_control_public_gate_20261120.log).

- ROS domain **215**, `ROS_LOCALHOST_ONLY=1`, `ROS_DISCOVERY_SERVER` unset. 공개 기준선 그대로인 `ManualWebBench`와 `VelocityArbiter`의 SHA-256을 고정해 실행했다. 기존 실차 Gate **코드**를 별도 프로세스로 실행하되 `/cmd_vel:=/cmd_vel_locked`로 remap했고 드라이버는 실행하지 않았다. 테스트 내 `/cmd_vel` publisher와 subscriber 모두 0을 확인했다.
- 가짜 Agent HTTP가 0.08 m/s, 0.18 rad/s·Deadman의 fresh sample을 공급했다. 실제 공개 벤치의 snapshot → 실제 공개 Arbiter → `/fleet/cmd_vel_selected`를 통해 **합성** velocity_smoother가 값을 받았고, 실제 Gate의 상태 `cmd_vx_out>0` 및 `/cmd_vel_locked` 단일 발행을 확인했다. Graph 조건 때문에 그 토픽을 별도로 구독하지 않았다. 따라서 locked 토픽의 메시지 본문 직접 기록은 하지 않았다.
- 합성 Nav2 active, status 배열 미수신, FollowPath 미확인, Deadman 해제, sample 만료, Agent 연결 해제에서 Arbiter 선택 0 확인. 이 값은 테스트가 직접 주입한 Nav2 상태이며 실제 Nav2 Action 실행 시험은 아니다.
- 별도 관찰: 시험용 `mapping_active`가 참이어도 **Arbiter 자체는 그 필드를 확인하지 않아** 출력이 유지됐다. 공개 서버의 Mapping admission이 이 상황을 차단할 책임을 지며, 독립 방어 추가가 필요하다. 가짜 HTTP가 만료 후 동일 세션을 다시 내주면 Arbiter 단독으로 재개하므로 세션 fencing은 외부 구성 책임이다. 이 가짜 HTTP 결과만으로 실제 서버가 만료 세션을 재생한다고 주장하지 않는다.
- 새로운 패널→실제 Fleet HTTP→Agent→Arbiter 전체 체인, 실제 Nav2 action, Watchdog, Gazebo, 실제 LIMO는 **미시험**이다. Gate 코드는 실제 소스이나 나머지 ROS 그래프·센서는 합성이다.

재현 예 (임시 경로·ROS 도메인 사용 가능 여부를 먼저 확인):

```bash
scratch=$(mktemp -d /tmp/manual-arbiter-XXXXXXXX)
mkdir "$scratch/limo_robot_agent"
for f in __init__.py core.py manual_web_bench.py velocity_arbiter.py; do
  git show "ea2d9632408de51365fe9005a7e828f1c99a0b0b:fleet_control/agent/limo_robot_agent/limo_robot_agent/$f" > "$scratch/limo_robot_agent/$f"
done
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=215 ROS_LOCALHOST_ONLY=1 ROS_LOG_DIR="$scratch/roslog"
unset ROS_DISCOVERY_SERVER
export PYTHONPATH="$scratch:$PYTHONPATH"
export MANUAL_ARBITER_REPO="$PWD" MANUAL_ARBITER_GATE_LOG="$scratch/gate.log"
/usr/bin/python3 fleet_control/tests/test_public_manual_arbiter_locked_ros.py
```

## 서버와 Agent가 각각 강제할 불변조건

1. **서버 원자적 admission:** 인증된 operator, 현재 Lease owner·epoch·만료, 새로 확인된 server/Agent boot, 단일 사용자·브라우저 세션, 비중복 시퀀스와 유한한 범위, 단조 시간 TTL을 한 transaction/lock 경계에서 확인한다. 실제 이송 범위에는 연속 재생 방지 ticket 또는 동등한 일회성 메커니즘이 필요하며 계약을 명시한다.
2. **서버 모드/Goal fence:** AUTO·MAPPING·MANUAL을 서로 배타적으로 관리한다. Fleet Mapping STARTING/MAPPING/PAUSING/SAVING/STOPPING/UNKNOWN 등과 외부 Mapping의 불확실성은 차단한다. 모든 제어 Action의 Goal 생애주기·상태 발행자·신선도·Agent 자체 handle 및 재시작 후 종결 근거가 불명확하면 거부한다. 취소 요청은 종결 증거가 아니다. start뿐 아니라 sample/pull 중에도 재검사하고 경계가 깨지면 기존 세션을 폐기한다.
3. **세대 fencing:** Lease 만료·양도, server restart, Agent offline/reconnect/restart(동일 boot 재연결 포함), Gate 세대 변경, Mapping/AUTO 진입, Deadman 해제 및 UI 이탈 시 세션/입력/미확인 요청을 무효화한다. 일시 중단 후 같은 세션에 의한 자동 재개는 허용하지 않으며 명시적인 새 세션과 Deadman 재입력이 필요하다. 네트워크 단절 중 로봇이 서버에 의존하지 않고 만료해야 한다.
4. **Agent 독립 fence:** 서버가 수락하더라도 실측 상태·ROS graph·Gate/odom 신선도·모드·Goal 종결·Mapping 부재·Lease/세션 유효성을 매 출력 주기마다 확인한다. 미수신/불일치·중복 발행·시간 역행·NaN/Inf·만료·Deadman 해제는 0과 세션 무효화다. 다른 ROS 노드의 우회 입력과 Goal은 별도로 감시한다. Agent 재시작이나 process SIGSTOP도 독립 watchdog이 막아야 한다.
5. **중재·최종 경로:** Agent에서 선택한 단일 입력 → 검증된 속도 중재기/제한 → 독립 watchdog → Safety Gate → 모터, 잠금 검증에서는 Gate `/cmd_vel_locked`만 사용한다. Nav2·teleop·브라우저가 최종 명령을 우회하지 못하도록 graph 충돌 시 차단한다. Gate와 watchdog 실패, 실차 전원·통신 단절, MCU timeout, 제동 거리와 실제 물리 E-Stop은 별도의 현장 증거가 필요하다.

## Claude 공통 작업이 끝난 뒤의 최소 통합안 — 아직 미적용

아래 변경은 같은 기능을 중복 생성하지 않고 현재 각 파일과 재대조해 적용한다. `index.html`, `app.js`, `service.py`, `ownership.py`, Agent `agent_node.py`, `manual_web_bench.py` 및 Mapping/Registry 관련 파일은 **현재 충돌 가능성이 높은 파일**이다. 실차 launch는 별도 담당자 검토 전 변경하지 않는다.

1. **`static/index.html`**: 기존 control 탭 안에 `<div id="manualControlHost"></div>` 하나와 `manual_control_panel.css`/`manual_control_panel.js` 로드를 추가한다. 별도 페이지/서버는 만들지 않는다. 스크립트는 app 초기화보다 먼저 로드한다.
2. **`static/app.js`**: 기존 `api()`를 request로 전달하고 `ManualControlPanel.mount(host, {request: api, getContext: verifiedManualContext, isActive: ...})` 한 번만 호출한다. 선택 로봇·로그아웃·탭 변경 시 `stop()`, 종료 시 `destroy()`를 호출한다. 기존 Gamepad 벤치 패널과 동시 세션이 없도록 기존 패널을 통합하거나 제거하는 것은 담당자와 합의 후 하나만 남긴다. `verifiedManualContext`가 근거를 얻지 못할 때는 null/UNKNOWN을 반환하고, 클라이언트 경과시간을 state age에 더한다. 존재하지 않는 인증 근거를 임의 true로 채우지 않는다.
3. **`static/manual_control_panel.js`**: staging 경로는 normalized [-1,1]에 ticket이 없고, 공개 경로는 physical [0,.10]/[-.30,.30]과 매번 갱신되는 ticket을 요구하므로 현재 코드 그대로 공개 서버에 붙이면 안 된다. 승인된 계약 하나를 명시적으로 선택하고, 공개 계약이면 start의 ticket을 보관하고 직렬 sample마다 deadman=false를 0으로 바꿔 단위 변환 후 ticket을 보내고 새 ticket을 갱신한다. 양 계약을 실패 시 자동 fallback하지 않는다. 잘못된 응답·시간 초과·late start 응답은 즉시 세션을 중단한다.
4. **`server/fleet_server/service.py` + Mapping 담당 파일**: Claude의 새 코드와 병합한 뒤 `_manual_bench_robot` 및 수동 세션 경로에 위 admission/fencing 불변조건을 넣는다. Mapping 상태 확인과 Nav2 상태 배열 확인만 추가해 전체 terminal 증거라고 오인하지 않는다. mission 생성/Mapping 시작과 manual start의 원자적 배타성을 같은 lock/transaction에서 검증한다. server boot 또는 Agent 재연결 때 세션을 무효화한다.
5. **`server/fleet_server/ownership.py`**: 기존 staging의 fail-closed recovery를 유지하며 Lease 해제·만료/새 epoch를 수동 세션에 연결한다. 공개 기준의 약한 recovery 로직으로 staging을 대체하지 않는다.
6. **Agent `agent_node.py`, `manual_web_bench.py`, 독립 Arbiter**: 담당자가 단일 버전/계약을 정한 뒤 검증된 source 선택기를 격리 코드로 이식한다. 현 staging real 벤치의 영속도 보장과 공개 mock-only 제한을 동시에 보존한다. Gate·watchdog·모드·모든 Nav2 종결과 Mapping 증거를 로컬에서도 검사하고, 초기 명령은 항상 0, 명시적 session/Deadman 재무장만 허용한다. 테스트 통과 전 실제 launch 및 모터 경로는 바꾸지 않는다.
7. **테스트**: 패널→실제 HTTP→Mock Agent→Arbiter→독립 watchdog→실제 Gate(`/cmd_vel_locked`)의 격리 E2E를 추가한다. Mapping과 외부 NavigateToPose/NavigateThroughPoses/FollowPath/Spin/BackUp 주입, 불명·stale·cancel 중·발행자 교체, 세션 TTL/중복/응답 순서, Lease 변경, server/Agent 재시작·단절/복구에서 0 및 자동 재개 금지를 확인한다. 이후 Gazebo와 물리 잠금·E-Stop 시험은 각각 분리한다.

## 통합 순서 및 남은 차단

1. Claude의 Mapping/Registry 변경 종료 후 공통 파일을 다시 읽고 소유자와 인터페이스를 확정한다. 사용자별 독립 변경은 보존하며 충돌은 수동 병합 제안으로 검토한다.
2. 모드·증거·세션/단위/ticket 계약을 서버와 Agent에 함께 확정하고 단위·동시성·fault 시험을 먼저 통과한다. 여기까지 실차 enable 금지.
3. 격리 arbiter+watchdog+Gate E2E와 패널 실제 HTTP 브라우저 시험, PC/모바일 화면 검증을 수행하고 단절·재시작 재무장을 확인한다.
4. 별도 승인·현장 절차에 따라 Gazebo, 실차 모터 잠금, 물리 정지/통신 단절/제동과 배터리 검증을 진행한다. E-Stop은 웹 STOP 대체 대상이 아니다. 모든 차단 조건이 풀리기 전 실차 `allow_motor_drive:=true`를 적용하지 않는다.
