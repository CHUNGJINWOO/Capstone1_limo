# Open-RMF Fleet Adapter for LIMO (`pcu`) — 시험 환경 구현

상태: **모의 Nav2 및 짧은 Gazebo 프로파일에서 검증**. 실제 LIMO에는 어떤 이동 명령도 보내지 않았고, 운영 서버(8080)와 연결하지 않았다. 구조: `Open-RMF → Fleet Adapter → Mission Manager(fleet_server) → Robot Agent → Nav2`. Adapter는 Nav2·`/cmd_vel`에 접근하지 않고 Mission Manager HTTP API(인증·멱등성·소유권·복구 정책)만 사용한다.

| 구분 | 내용 |
|---|---|
| 구현 완료 | Fleet Adapter(`limo_fleet`/`pcu` 등록, 경로 → 임무 변환, 취소, 상태 보고), 서버 제어권 lease 연동, 5개 waypoint·4개 lane 그래프, RMF ↔ Nav2 좌표 변환 |
| 모의 환경 검증 | §9: RMF E2E(임무 수행·완료·취소), 웹/RMF 소유권 충돌(409), 서버 중단·재연결, Adapter·Agent 재시작, `RECONCILIATION_REQUIRED` 차단과 운영자 복구, 모의 Nav2 수신 목표 수 계측 |
| Gazebo 검증 | §11: 실제 Fortress plant/Nav2/AMCL에서 REST 이동·취소와 RMF 완료·취소, lease, 장애·재시작을 실행. 최종 수치는 별도 결과 문서 참고 |
| 실차 검증 | 없음 (Open-RMF 관련 실차 시험을 하지 않았다) |
| 미검증 | Gazebo 장거리·다양한 장애물 운용, 실차 이동, 다중 로봇 교통 협상, 장시간 운용, RMF 고아 작업 복구(§8b), 실제 RMF API 서버·대시보드 연동, 최신 서버·Agent의 운영 배포 |


## 1. 격리 설치 (Open-RMF 2.1.8, ROS 2 Humble)

- **설치 스크립트:** `bash scripts/install_rmf_isolated.sh list|install|env-only` (root 불필요, 저장소에는 바이너리가 없다). `RMF_ISOLATED_DIR`로 위치를 바꿀 수 있다(기본 `~/rmf_isolated`).
- 공식 ROS apt 저장소의 `ros-humble-rmf-fleet-adapter-python 2.1.8` 및 의존 패키지 25개(`.deb`, 7.4 MB)를 `apt-get download`로 받아 `dpkg -x`로 `~/rmf_isolated/prefix`에만 풀었다. 시스템 ROS 패키지는 설치·변경하지 않았다.
- 환경 파일 `~/rmf_isolated/env.sh`(ROS setup 다음에 source)가 `AMENT_PREFIX_PATH`, `LD_LIBRARY_PATH`, `PYTHONPATH`를 추가하고 `PYTHONNOUSERSITE=1`을 설정한다. **사용자 site-packages를 꺼야 하는 이유:** `~/.local`의 numpy 2.2.6은 Humble 바이너리(numpy 1.x용)와 충돌해 `rmf_adapter`가 segfault한다(시스템 numpy 1.21.5 사용).
- 이 버전의 Python API에는 `EasyFullControl`이 없어 `RobotCommandHandle` 방식(`follow_new_path`/`stop`)으로 구현했다.

## 2. 실행

```bash
source /opt/ros/humble/setup.bash && source ~/rmf_isolated/env.sh
cd fleet_control/rmf_adapter
python3 -u tests/test_runner_policy.py      # 정책 단위 시험 (ROS/RMF 불필요)
python3 -u tests/run_rmf_checks.py          # 종단 간 시험 (격리 서버 18090, 웹소켓 sink 18091, ROS 도메인 151/152)
```

| 변수 | 기본값 | 의미 |
|---|---|---|
| `RMF_TEST_PORT` | 18090 | 시험용 Mission Manager 포트 (운영 8080과 무관) |
| `RMF_TEST_WS_PORT` | 18091 | RMF 작업 상태를 받는 시험용 WebSocket sink |
| `RMF_E2E_TIMEOUT_S` | 1200 | 전체 시험 제한 시간(초과 시 정리 후 중단) |
| `RMF_ISOLATED_ENV` | `~/rmf_isolated/env.sh` | 격리 RMF 환경 파일 |

수동 실행: `SERVER_URL=<mission manager> TOKEN_FILE=<rmf-adapter.token> bash scripts/run_rmf_stack.sh` (schedule + dispatcher + adapter, 도메인 151, localhost 전용). **운영 8080에는 연결하지 말 것**(아래 §9).

## 3. 구조

| 파일 | 역할 |
|---|---|
| `limo_rmf_adapter/adapter_node.py` | RMF fleet `limo_fleet`·로봇 `pcu` 등록, 항법 그래프 생성, 위치/상태 보고, 입찰 응답, `RobotCommandHandle`(RMF 경로 → 목표 목록) |
| `limo_rmf_adapter/runner.py` | **정책 계층**(RMF 비의존): 단일 활성 임무, 소유권, 오래된 경로 폐기, 멱등 키, 취소 확인, 통신 장애, 고아 임무 정리 |
| `limo_rmf_adapter/mission_client.py` | Mission Manager API 클라이언트(stdlib, Bearer 토큰, 토큰 파일 권한 0600 요구) |
| `limo_rmf_adapter/coords.py` | RMF ↔ Nav2 `map` 좌표 변환 |
| `config/fleet.json` | 좌표 변환, waypoint 5개, lane 4개, 차량 특성 |
| `tests/` | 정책 단위 시험, E2E 시험, 시험 환경, WebSocket sink |

## 4. RMF ↔ Mission Manager 흐름

1. RMF 작업(`patrol`) → 입찰: Adapter가 **그 시점에** Mission Manager에서 소유권을 새로 확인하고(`check_available_now`), 가능하면 수락, 아니면 오류를 담아 거절 → 디스패처가 `FAILED_TO_ASSIGN`.
2. RMF가 `follow_new_path(waypoints)` 호출 → RMF 좌표를 Nav2 좌표로 변환, 현재 위치와 같은 선두 waypoint와 0.05 m 이내 중복 점을 제거 → 목표 목록.
3. 목표를 **하나씩** `POST /missions`(키 `rmf:<robot>:<boot>:<generation>:<index>`, `created_by`=`rmf-adapter`). 이전 임무가 terminal이 된 후에만 다음 목표를 보낸다.
4. 진행: 임무 `progress.distance_remaining_m`/속도로 RMF에 도착 예상 시간을 보고. 임무 `SUCCEEDED` → 다음 목표, 마지막 후 `path_finished_callback`.
5. RMF `stop()`/새 경로/취소 → `POST /missions/{id}/cancel`, **Mission Manager가 terminal 상태(CANCELED 등)를 보고할 때까지** 새 경로를 보내지 않는다.
6. RMF 작업 상태(`standby/underway/completed/canceled…`)는 RMF 라이브러리가 `--rmf-api-uri`로 방송하는 JSON(`task_state_update`)이다. 시험에서는 stdlib WebSocket sink(`tests/ws_sink.py`)가 받는다. (이 버전에서 레거시 `task_summaries` 토픽은 메시지를 발행하지 않는다.)

## 5. 좌표 연결

`p_rmf = R(θ)·p_nav2 + t`, `yaw_rmf = yaw_nav2 + θ`. 설정: θ = 0.3 rad, t = (20, 10) m (**항등 변환이 아님** — 실제 오프셋 변환이 검증되도록 의도적으로 선택한 시험값). RMF 지도 이름 `L1`. Nav2 지도는 Gazebo 자동 맵핑 결과 `experiments/auto_mapping_final_20261112/runs/run2/limo_sim_map`(origin −3.86, −7.06, 해상도 0.05 m).

## 6. Waypoint 5개와 lane 4개

| 이름 | Nav2 map (x, y) | RMF (x, y) |
|---|---|---|
| wp_a | (−2.40, −5.50) | (19.3326, 4.0364) |
| wp_b | (−3.40, −3.00) | (17.6384, 6.1292) |
| wp_c | (−3.07, −0.83) | (17.3124, 8.2998) |
| wp_d | (−2.73, 1.33) | (16.9989, 10.4638) |
| wp_e | (−2.40, 3.50) | (16.6729, 12.6344) |

lane(양방향): a–b, b–c, c–d, d–e. 시험이 모든 waypoint가 지도의 free 셀에 있고 모든 lane이 점유/미지 셀에서 0.2 m 이상 떨어져 있음을 검사한다(가구를 가로지르던 이전 배치는 이 검사에서 걸러 교체했다).

> **`wp_a`의 `charger: true`는 실제 충전 시설이 아니다.** RMF가 로봇 등록에 충전 waypoint를 하나 요구하기 때문에 넣은 **시험용 설정**이며, 배터리 잔량(SOC)은 보고하지 않는다(LIMO는 전압만 제공).

## 7. 소유권·충돌 정책 (서버 제어권 lease)

- **서버가 강제한다.** Adapter는 실행 중 `POST /robots/pcu/lock`(acquire/renew)으로 로봇의 제어권 lease(TTL 20 s, 5 s마다 갱신)를 잡는다. lease가 유효한 동안 소유자(토큰 이름 `rmf-adapter`)가 아닌 주체의 `POST /missions`는 **큐에 넣지 않고 HTTP 409 `ownership_conflict`**로 거절된다. 확인과 생성은 서버의 같은 락 안에서 수행되어 그 사이 틈이 없다(120회 경쟁 시험에서 동시 성공 0).
- lease를 잡을 때: 다른 주체의 미종료 임무가 있으면 409 `robot_busy`, `RECONCILIATION_REQUIRED`가 있으면 409 `reconciliation_required`. 갱신이 끊기면 lease가 만료되어 죽은 Adapter가 로봇을 영구히 막지 못한다(진행 중 임무는 영향 없음).
- Adapter는 **로컬 만료 시각**으로 자신을 가둔다: 서버에 닿지 않아 갱신하지 못하면 lease 만료 전에 새 임무 제출을 멈춘다. lease가 없거나 다른 소유자의 임무가 있으면 입찰을 거절(`FAILED_TO_ASSIGN`)하고 이미 받은 경로의 제출도 보류한다.
- 운영자 인수: admin이 `force_release`로 lease를 풀면 웹 운영자가 임무를 만들 수 있고, 그 동안 RMF 작업은 거절된다. 웹 임무가 끝나면 Adapter가 lease를 자동으로 다시 잡는다.
- 취소는 모든 제어 역할에 허용(정지는 안전한 방향). 소유자 식별은 토큰 이름이며 `rmf:` 키 접두어 위조는 인정하지 않는다.
- 로봇당 활성 임무 1개, 경로 generation으로 오래된 명령 폐기, 안정적인 멱등 키(불확실한 생성은 같은 키로 재시도)는 이전과 같다.
- `RECONCILIATION_REQUIRED`이면 새 임무를 만들지 않고 입찰도 거절한다. 복구는 §8.

## 8. 통신 장애·재동기화·복구

- Mission Manager 연결이 끊기면(연속 2회 실패) 링크 `SERVER_UNREACHABLE` → RMF에 로봇을 `offline`으로 보고. 단절 중에는 새 임무를 만들지도, 결과를 추측하지도 않는다.
- 재연결 후 활성 임무를 id로 다시 조회해 실제 결과를 한 번만 반영하고 `offline` 해제.
- Adapter 재시작: 상태 파일(`mission_id`만 저장, 비밀 없음)에 남은 임무는 **고아**로 보고 취소가 terminal이 될 때까지 새 경로를 막는다.
- Agent 재시작: Mission Manager가 `RECONCILIATION_REQUIRED`로 표시(목표 재전송 없음). Adapter는 해당 경로를 한 번만 실패로 보고하고 새 임무·입찰을 차단한다.
- **운영자 복구(admin 전용)**: `POST /missions/{id}/recover {note}`. Agent가 **측정한** 증거(최근 5 s 이내 state의 `execution.nav2_goals`: `navigate_to_pose/_action/status`상 활성 목표 0개, Agent의 목표 핸들 없음, 로봇 online·reconciled)가 있을 때만 받아들인다. 목표가 아직 활성이면 409 `goal_still_active`, 증거 부재/오래됨이면 409 `no_evidence`/`stale_state`. 증거가 없으면 새 이동 임무는 계속 차단된다. 복구된 임무는 SUCCEEDED/CANCELED가 아니라 `FAILED`+`RECOVERED_UNVERIFIED_OUTCOME`이고 서버는 로봇에 아무것도 보내지 않는다. Agent는 자체 Nav2 측정이 일치할 때만 저널을 닫는다(`recovered_commands`). 상세: `docs/API.md`.
- **Agent 쪽 정정**: 이미 끝난 목표(Agent가 직접 터미널 결과를 관측)에 대한 취소 명령은 `REJECTED`+`NOTHING_TO_CANCEL`(저널을 막지 않음)로 답한다. 이전에는 이 경합(RMF 취소와 자연 완료가 겹침)이 `RECONCILIATION_REQUIRED`로 저널을 영구 차단했고, 이는 3회 반복 시험 중 한 번의 실패에서 발견되었다. 알 수 없는 임무에 대한 취소는 여전히 `RECONCILIATION_REQUIRED`.

## 8b. Adapter 재시작 후 RMF가 이전 작업을 다시 할당하는 현상 (재현·분석)

`tests/repro_adapter_restart.py`, 3회 시도 모두 동일: 작업 실행 중 Adapter를 SIGKILL 후 즉시 재시작.
- 새 Adapter가 뜨면 디스패처가 이전 작업(`patrol.dispatch-0`)의 **award를 다시 보낸다**(`rmf_task/dispatch_request`는 transient-local). 새 Adapter는 로그에 `Received DispatchRequest award … before receiving BidNotice … ignored`를 남기고 **무시**한다. 따라서 **이전 작업은 다시 실행되지 않는다**: 재시작 후 생성된 임무 0개(새 작업 제외), 모의 Nav2가 받은 목표는 재시작 전 1개뿐이었다.
- 이전 작업은 RMF 쪽에서 `underway`로 **영구히 남는다**(디스패처는 계속 배정된 것으로 본다). `cancel_task_request`에도 응답이 없다(Fleet Adapter가 그 작업을 모른다). Adapter가 작업 상태를 영속하지 않는 RMF 2.1.8 구성의 동작이며 중복 실행은 아니지만 운영상 고아 작업이다 → 운영자가 무시/디스패처 재시작으로 정리해야 한다(미해결).
- 새 작업은 정상 수락·완료되고 목표는 정확히 1개 전송된다.
- 이전 Adapter가 남긴 Mission Manager 임무는 상태 파일로 인식해 새 경로 전에 취소한다(고아 임무 정책).

## 9. 시험 결과 (2026-11-12, 모의 로봇)

최종 코드로 `run_rmf_checks.py`를 **3회 연속 실행: 59/59, 59/59, 59/59 PASS**(각 약 2.5–3 분). 15개 필수 항목이 모두 PASS이며 항목 13(중복 실행)은 이제 모의 Nav2가 실제로 받은 목표 수를 센다(= Mission Manager가 확인한 navigate 명령 수, 같은 목표 0.3 s 내 이중 전송 없음). 항목 9는 서버 lease(409)를, 항목 14는 증거 기반 운영자 복구를 포함한다.

| 항목 | 결과 | 핵심 근거 |
|---|---|---|
| 1 정합성 | PASS | waypoint 5개 free 셀, lane 4개 0.2 m 여유, 변환 왕복 < 1e-9 |
| 2–7 등록·임무·완료 | PASS | RMF가 `completed` 보고, 모의 로봇이 wp_b 도착 |
| 8 취소 | PASS | 긴 구간(>1.2 m 남음) 실행 중 취소 → 해당 임무 CANCELED + cancel 명령 SUCCEEDED, 확인 전 후속 제출 없음, 취소된 목표 재전송 없음, RMF `canceled` |
| 9 소유권 | PASS | RMF lease 중 웹 생성 409(대기/실행 중 모두), 임무 미생성, admin 인수 후 RMF 입찰 거절 `FAILED_TO_ASSIGN`·임무 미생성, 웹 임무 후 lease 자동 재획득·새 RMF 작업 완료 |
| 10 서버 중단 | PASS | 약 1 s 내 offline 보고, 단절 중 제출 0, 재연결 후 작업 완료 |
| 11 Adapter 재시작 | PASS | 고아 임무 취소 후 재등록 |
| 12–14 Agent 재시작·복구 | PASS | RECONCILIATION_REQUIRED, 신규 임무 차단, 활성 목표 중 복구 거절 → 목표 종료 후 복구 수락(FAILED/RECOVERED_UNVERIFIED_OUTCOME) → 새 RMF 작업 완료 |
| 15 정리 | PASS | 시험 프로세스·포트 잔여 없음 |

보조 시험: 서버 소유권/복구 `tests/test_ownership.py` **37/37**(120회 경쟁 포함, 동시 성공 0), Adapter 정책 **33/33**, Agent 복구 2개, 기존 중앙 관제 회귀(`run_checks` 46/46, `test_dedupe` 14/14, `test_recovery` 14, `test_cancel_results` 14, `test_auth_reload` 2).

### 시험 중 발견·수정
- lane/waypoint 배치, 입찰 시 캐시된 소유권 확인의 경쟁 → 입찰마다 서버 조회.
- 같은 반복 시험 중 한 번 **RMF 취소와 자연 완료가 겹치면 Agent가 취소를 `RECONCILIATION_REQUIRED`로 응답해 저널이 영구 차단**되는 결함을 발견 → §8.
- 서버 복구 후에도 Agent 저널이 새 목표를 거부하던 문제 → `recovered_commands` + Agent 자체 측정.
- 한 번의 `run_checks`가 42/46으로 나왔고(어떤 항목인지 기록하지 못함), 이후 같은 코드로 5회 연속 46/46이었다. 원인 미규명(간헐).

## 10. 미검증·제한 사항

- Gazebo 전구간·다양한 장애물 회피 성능, 다중 로봇 교통 협상, 실제 RMF API 서버·대시보드, 실차는 시험하지 않았다. 운영 서버(8080)는 변경·재시작하지 않았으므로 **메모리 상의 코드는 이전 버전**이다(이 문서의 lease/복구 API는 시험 서버에서만 검증).
- 복구는 활성 Nav2 목표가 **스스로 끝나야** 가능하다. 에이전트가 핸들을 잃은 목표를 취소하는 운영자 명령은 구현하지 않았다.
- Adapter 재시작 후 이전 RMF 작업은 `underway`로 고아 상태로 남는다(§8b).
- lease는 서버 단일 프로세스 락 기반이며 다중 서버 구성은 고려하지 않았다. lease 소유자 구분은 토큰 이름이다(토큰 공유 시 구분 불가).
- 배터리 SOC는 보고하지 않는다. `wp_a`의 `charger: true`는 **실제 충전 시설이 아니라 RMF 등록 요건을 맞추는 시험용 설정**이다.


## 11. Gazebo Fortress 전용 프로파일

[구성·재현](../../gazebo/README.md) / [실행 결과·시간·실패 이력](../../gazebo/RESULTS.md).

기존 `config/fleet.json`은 모의용으로 보존합니다. `config/fleet_gazebo.json`은 저장된 같은 지도에서 짧은 a–b 구간에 5개 waypoint·4개 양방향 lane을 둡니다. 좌표 변환은 동일한 θ=0.3 rad, t=(20,10)입니다. 저장 지도 + AMCL로 map을 고정하고 wheel odometry로 Nav2 TF를 구성하며 F4는 기존 Safety Gate 필수 입력으로 유지합니다. world GT는 독립 이동 증거이며 Nav2 입력이 아닙니다.

RMF `compute_plan_starts`가 가까운 waypoint로 병합하면 `Start.location`이 생략되어 물리 위치 보고도 waypoint로 바뀌었습니다. Gazebo 세부 검사에서 약 16 cm 차이를 발견했습니다. 이제 waypoint/lane/time/orientation은 보존하면서 변환한 실제 Nav2 `(x,y)`를 `Start.location`에 전달합니다. 정합성 검사 허용치 0.1 m를 늘리지 않았습니다.

등록·취소·lease·장애·복구 HTTP 경로는 기존과 동일합니다. native Nav2 goal UUID와 Agent의 실제 goal acceptance 응답 및 서버 명령을 대조해 중복을 검사합니다. Adapter 재시작 시 Mission Manager 고아 임무 정리와 RMF 이전 작업의 고아 상태는 서로 다르며, §8b의 RMF 한계는 해결하지 않았습니다. 실차·운영 서버에는 배포하지 않았습니다.
