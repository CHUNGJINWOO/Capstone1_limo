# Capstone1 중앙 관제 (`fleet_control`)

노트북의 **중앙 관제 서버**와 LIMO의 **Robot Agent**, 그리고 **Open-RMF Fleet Adapter**로 이루어진 시스템입니다. 서버는 로봇 상태(위치·지도·센서·Nav2)를 모으고, 임무(Nav2 `NavigateToPose` 목표)를 만들고 취소하며, Agent가 그 목표를 로봇의 Nav2에 전달합니다. 서버와 Agent는 Tailscale 사설망 위에서 HTTP + Bearer 토큰으로 통신합니다.

> **읽기 전에:** 이 문서는 구현·시험·실차 검증을 구분해 적습니다. 시뮬레이션(모의 로봇)에서 통과한 것을 실차에서 검증된 것으로 읽지 마세요. 아래 [검증 범위](#검증-범위)를 먼저 확인하세요.

```text
웹 대시보드 ─┐
             ├─ 중앙 관제 서버(Mission Manager, REST API, SQLite) ── Tailscale ── Robot Agent ── Nav2 ── LIMO
Open-RMF ── Fleet Adapter ┘
```

## 검증 범위

| 구분 | 내용 |
|---|---|
| **구현 완료** (코드가 있음) | 웹 대시보드, 서버(REST API, 인증·역할, SQLite), Robot Agent(상태 수집·지도 업로드·명령 저널·Nav2 연동), 임무 생성·취소·상태, 중복 명령 방지, 통신 끊김·재연결, 제어권 lease, 운영자 복구, Open-RMF Fleet Adapter |
| **모의 환경 검증 완료** (노트북, 모의 Nav2, 독립 서버·ROS 도메인) | RMF → Mission Manager → 모의 로봇 임무 수행·완료·취소(RMF E2E), RMF와 웹 관제 사이 제어권 lease(409), 서버/Agent 통신 장애·재연결, Agent·Adapter 재시작, `RECONCILIATION_REQUIRED` 차단, 증거 기반 운영자 복구, 모의 Nav2의 실제 수신 목표 수 계측 |
| **실차 검증 완료** (실제 LIMO) | 2026-11-12 모터 잠금 상태에서 Agent를 짧게 실행해 Tailscale 경유 **상태 수신**만 확인: 위치(map 프레임)·지도·Nav2 상태·배터리 전압(백분율은 UNKNOWN)·LiDAR·Safety Gate 상태·모터 경로(LOCKED, `/cmd_vel` 발행자 없음). 이때 임무는 보내지 않았고 모터를 구동하지 않았습니다. 당시 배포된 Agent는 현재 소스보다 **이전 버전**입니다 |
| **미검증** | Gazebo와 Open-RMF 통합, Open-RMF를 이용한 **실차 이동**, 다중 로봇 운용, 실제 장시간 통신 운용, RMF 고아 작업 복구(아래 한계 참고), 최신 서버·Agent의 운영 배포(운영 중인 8080 서버는 이전 버전), 실차의 Depth 카메라 상태, 실차 임무 실행 |

### 알려진 한계 (숨기지 않는 항목)

- `tests/run_checks.py`는 한 번 42/46으로 실패한 적이 있습니다(어느 검사였는지 기록하지 못했고 **원인은 확인하지 못했습니다**). 이후 정리 중 다른 한 번의 45/46은 "지도 PNG가 상태보다 늦게 업로드되는" 시험 경합(시험이 한 번만 요청함)으로 확인해 시험을 고쳤고(최대 20 s 폴링), 수정 후 연속 6회 46/46이었습니다. 42/46이 같은 원인이었는지는 확인하지 못했으므로 **간헐적 문제가 남아 있을 수 있습니다.**
- Adapter를 재시작하면 RMF 디스패처가 이전 작업의 할당을 다시 보내지만 새 Adapter는 이를 무시합니다. 중복 실행은 없지만 그 이전 작업은 RMF에서 `underway`로 **고아 상태**로 남고 취소에도 응답하지 않습니다(RMF 2.1.8 구성의 동작). 해결하지 못했습니다.
- 운영자 복구는 활성 Nav2 목표가 스스로 끝나야 가능합니다. Agent가 핸들을 잃은 목표를 취소하는 운영자 명령은 없습니다.
- 통신 장애는 이미 보낸 Nav2 목표의 물리적 정지를 보장하지 않습니다. 이 시스템은 Safety Gate를 대체하지 않습니다.
- HTTP 자체는 TLS가 없습니다(아래 [Tailscale](#tailscale-사용-개념) 참고).

## 디렉터리

| 경로 | 내용 |
|---|---|
| `server/fleet_server/` | 서버(Python 표준 라이브러리만 사용): `service.py`(임무·명령 로직), `http_api.py`(REST, 인증), `auth.py`, `db.py`(SQLite WAL), `recovery.py`(재동기화), `ownership.py`(제어권 lease·운영자 복구), `static/`(웹 대시보드) |
| `agent/limo_robot_agent/` | ROS 2 패키지: `agent_node`(실차/모의 Agent), `core.py`(통신·재연결·명령 저널), `ros_adapter.py`(상태 수집, Nav2 연동), `mock_robot_node`(모의 Nav2 로봇) |
| `rmf_adapter/` | Open-RMF Fleet Adapter: `limo_rmf_adapter/`, `config/fleet.json`, `scripts/`, `tests/`, `docs/RMF.md` |
| `maps/` | 모의 로봇이 쓰는 저장된 Gazebo 자동 맵핑 결과(`limo_sim_map.pgm/.yaml`) |
| `scripts/` | `init_tokens.sh`, `run_server.sh`, `run_mock_stack.sh`, `limo_run.sh`(LIMO에서 실행) |
| `tests/` | 통합·중복 방지·복구·취소·인증·소유권 검사 |
| `docs/API.md` | HTTP 계약, 복구 프로토콜, lease·운영자 복구 |
| `docs/VALIDATION.md` | 검증 결과 요약과 재현 방법 |
| `rmf_adapter/docs/RMF.md` | Open-RMF 설계·좌표 변환·소유권 정책·시험 결과 |

## 시스템 요구사항

- Ubuntu 22.04, ROS 2 Humble (Agent, 모의 로봇, 테스트). 서버 자체는 Python 3.10 표준 라이브러리만 필요합니다.
- Agent/모의 로봇: `rclpy`, `nav2_msgs`, `action_msgs`, `nav_msgs`, `sensor_msgs`, `std_msgs`, `tf2_ros`, NumPy, Pillow(`package.xml` 참고). `limo_msgs`가 없으면 배터리는 UNKNOWN으로 남고 백분율은 계산하지 않습니다.
- RMF E2E 시험: Open-RMF 2.1.8(아래 격리 설치), 시험 코드는 NumPy·Pillow를 씁니다. `PYTHONNOUSERSITE=1` 환경에서 시스템 numpy 1.x를 사용합니다(사용자 site의 numpy 2.x는 `rmf_adapter`를 비정상 종료시킵니다).
- LIMO 연결: 서버와 LIMO가 같은 Tailscale 네트워크에 있어야 합니다.

## 빠른 시작 (모의 환경, 실차 불필요)

저장소 루트에서 시작합니다. 아래 예시는 운영 서버와 겹치지 않도록 임시 디렉터리와 8080이 아닌 포트를 씁니다.

```bash
cd fleet_control
export FLEET_DEMO=$(mktemp -d)                      # 임시 데이터/토큰 위치

# 1) 토큰 초기화 (값은 출력되지 않고 파일 위치만 출력됩니다)
PYTHONPATH=server python3 -m fleet_server init \
  --data-dir "$FLEET_DEMO/data" --secrets-dir "$FLEET_DEMO/secrets" --robots mock1

# 2) 서버 (loopback에만 바인딩)
PYTHONPATH=server python3 -m fleet_server serve \
  --data-dir "$FLEET_DEMO/data" --port 18082 --bind 127.0.0.1 &

# 3) 모의 로봇 + Agent (다른 ROS 도메인, localhost 전용)
source /opt/ros/humble/setup.bash
SERVER_URL=http://127.0.0.1:18082 ROS_DOMAIN_ID=142 \
  TOKEN_FILE="$FLEET_DEMO/secrets/agent-mock1.token" STATE_DIR="$FLEET_DEMO/agent_state" \
  bash scripts/run_mock_stack.sh &

# 4) 브라우저에서 http://127.0.0.1:18082/ 를 열고 $FLEET_DEMO/secrets/operator.token 의 내용을 붙여 넣어 로그인
```

대시보드에서 지도를 클릭해 목표를 입력하고 임무를 만들거나 취소할 수 있습니다. 모의 로봇은 합성 스캔·지도·배터리를 사용하며 **실제 센서·모터 검증이 아닙니다**.

## 서버 실행과 인증

```bash
# 최초 1회 (data/, secrets/는 .gitignore 대상이며 절대 커밋하지 않습니다)
bash scripts/init_tokens.sh
# 실행: loopback + Tailscale 주소에만 바인딩 (기본 포트 8080, PORT로 변경)
bash scripts/run_server.sh
```

`init`은 이미 `auth.json`이 있으면 거부합니다(`--force`는 모든 토큰을 교체하므로 운영 환경에서는 쓰지 마세요). 토큰은 `secrets/*.token`(평문, 권한 0600), 서버에는 SHA-256 해시만 `data/auth.json`에 저장합니다. 토큰은 URL·쿼리·본문에 넣지 않고 `Authorization: Bearer <token>` 헤더로만 보냅니다.

| 역할 | 권한 |
|---|---|
| viewer | 상태·지도·임무·이벤트 조회 |
| operator | 조회 + 임무 생성·취소 + 제어권 lease 획득/갱신/해제(자기 것) |
| admin | operator 권한 + lease 강제 해제 + `RECONCILIATION_REQUIRED` 운영자 복구 |
| agent | 자기 `robot_id`의 hello/state/reconcile/map/ACK만 |

- 인증 없이 접근할 수 있는 것은 정적 UI와 `GET /api/v1/health`뿐입니다. 같은 IP에서 인증 실패가 60초에 10회를 넘으면 429입니다.
- `auth.json`은 요청 시 핫리로드되며, Agent 토큰을 바꾸면 Agent를 재시작해야 합니다.
- RMF Adapter는 별도 operator 토큰(이름 `rmf-adapter`)을 쓰며, **토큰 이름**이 임무 소유자 식별에 쓰입니다. 토큰을 공유하면 소유자를 구분할 수 없습니다.
- 서버는 `0.0.0.0`과 공인 주소 바인딩을 거부합니다(loopback, 사설망, Tailscale 대역만 허용).

## Robot Agent

빌드(저장소 루트 기준, 팀 워크스페이스는 빌드·수정하지 않습니다):

```bash
source /opt/ros/humble/setup.bash
FLEET_WS="$HOME/.local/share/capstone-fleet-ws"
colcon --log-base "$FLEET_WS/log" build \
  --base-paths "$PWD/fleet_control/agent/limo_robot_agent" \
  --build-base "$FLEET_WS/build" --install-base "$FLEET_WS/install" \
  --packages-select limo_robot_agent
source "$FLEET_WS/install/setup.bash"
```

실행 예(실차에서 **상태 수신만**, 임무 거부 기본값):

```bash
export ROS_DOMAIN_ID=<ROS_DOMAIN_ID> ROS_LOCALHOST_ONLY=1
ros2 run limo_robot_agent robot_agent --ros-args \
  -p server_url:=http://<TAILSCALE_IP>:8080 \
  -p robot_id:=pcu -p mode:=real -p simulated:=false \
  -p token_file:="$HOME/.config/limo_fleet/agent-pcu.token" \
  -p allow_missions:=false \
  -p state_dir:="$HOME/.local/state/limo_robot_agent"
```

| 파라미터 | 의미 |
|---|---|
| `server_url` | 서버 주소(기본 `http://127.0.0.1:8080`) |
| `token_file` | 필수. 해당 로봇의 Agent 토큰 파일(권한 0600) |
| `robot_id` | 기본 `pcu` |
| `mode`, `simulated` | 화면 표시용 라벨이며 안전 권한을 주지 않습니다 |
| `allow_missions` | **기본 false**. false이면 모든 임무를 거부합니다 |
| `state_dir` | 명령 저널 위치. 재시작 사이에 **보존해야** 중복 실행 방지가 동작합니다 |

Agent는 속도를 발행하지 않고 Safety Gate를 우회하거나 reset하지 않으며, `/cmd_vel`에 발행자가 있거나 Nav2 액션 서버가 준비되지 않았으면 목표를 거부합니다.
`scripts/limo_run.sh`는 LIMO 위에서 Agent를 짧게 실행하는 보조 스크립트입니다(`SERVER` 필수, 기본 주소 없음; `TEAM_WS`/`OVERLAY_WS`/`LOG_DIR` 환경 변수로 경로 지정).

## Open-RMF 격리 설치와 Fleet Adapter

Open-RMF는 시스템에 설치하지 않고 사용자 홈 아래 별도 접두사에 풀어 씁니다(저장소에는 바이너리가 없습니다).

```bash
cd fleet_control/rmf_adapter
bash scripts/install_rmf_isolated.sh list      # 받을 패키지 확인(다운로드 없음)
bash scripts/install_rmf_isolated.sh install   # 공식 ROS apt 저장소의 .deb 다운로드 + 압축 해제(root 불필요)
source /opt/ros/humble/setup.bash && source ~/rmf_isolated/env.sh
python3 -c "import rmf_adapter; print('rmf_adapter OK')"
```

시험한 버전은 `ros-humble-rmf-fleet-adapter-python` **2.1.8**입니다(다른 버전은 미검증). 실행과 설계는 [RMF.md](rmf_adapter/docs/RMF.md)에 있습니다: `limo_fleet` 플릿의 로봇 `pcu`, 5개 waypoint·4개 lane, RMF ↔ Nav2 `map` 좌표 변환, 임무 소유권 정책.

> `wp_a`의 `charger: true`는 **실제 충전 시설이 아니라** RMF의 로봇 등록 요건을 맞추기 위한 시험용 설정입니다. 배터리 잔량(SOC)은 보고하지 않습니다(LIMO는 전압만 제공).

## Tailscale 사용 개념

- 서버(노트북)와 LIMO를 같은 Tailscale 네트워크(tailnet)에 올리고 Agent가 서버의 tailnet 주소로 접속합니다. 서버는 `--tailscale` 옵션으로 `tailscale ip -4`를 읽어 **loopback + tailnet 주소에만** 바인딩합니다(공개 인터넷에 열지 않습니다). 이 프로젝트는 Tailscale 설정을 바꾸지 않습니다.
- tailnet 구간은 WireGuard로 암호화되지만 서버의 HTTP 자체는 TLS가 아닙니다. 그래서 앱 수준 인증(Bearer 토큰)을 함께 씁니다.
- 실제 tailnet 주소·장치 이름·계정은 저장소에 적지 않습니다. 문서의 `<TAILSCALE_IP>`는 자기 환경의 주소로 바꿔 쓰세요.
- 팀원은 **자기 계정**으로 팀 tailnet에 참여합니다. 관리자 계정은 공유하지 않고, LIMO 접속 권한은 별도로 관리합니다.

## 임무·복구·제어권 (요약)

- 흐름: `QUEUED → DISPATCHED → ACCEPTED → EXECUTING → SUCCEEDED/FAILED/CANCELED/REJECTED/EXPIRED`. 취소는 `CANCELING`을 거치며 Nav2의 실제 canceled 결과를 확인해야 완료입니다.
- 식별자: `mission_id`(임무), `command_id`(Agent 전달 단위), `idempotency_key`(로봇별 생성 중복 방지). 같은 `command_id`는 다시 실행하지 않으며, 명령은 만료·발급 시각·순서로 검사됩니다.
- 불명확한 상황(핸들 소실, 전달 후 통신 단절, 소유권 불일치)은 `RECONCILIATION_REQUIRED`로 두고 새 이동 임무를 차단하며 목표를 다시 보내지 않습니다.
- **제어권 lease:** lease가 있으면 소유자가 아닌 주체의 임무 생성은 대기열에 넣지 않고 HTTP 409입니다(RMF가 가진 동안 웹 생성 거절).
- **운영자 복구(admin):** Agent가 측정한 증거(활성 Nav2 목표 0개 등)가 있을 때만 가능하며, 임무는 SUCCEEDED/CANCELED가 아니라 `FAILED`(`RECOVERED_UNVERIFIED_OUTCOME`)가 됩니다.

상세한 API와 프로토콜은 [docs/API.md](docs/API.md)를 보세요.

## 테스트 실행

모든 시험은 임시 디렉터리, 독립 포트, 독립 ROS 도메인에서 돌며 운영 서버(8080)나 실차를 건드리지 않습니다.

```bash
cd fleet_control
source /opt/ros/humble/setup.bash
python3 tests/run_checks.py            # 통합(모의 Nav2): 포트 18080, ROS 도메인 141
python3 tests/test_dedupe.py           # 중복 명령 방지
python3 tests/test_auth_reload.py      # 토큰 핫리로드
python3 tests/test_recovery.py         # 서버/Agent 재시작·장애 주입(실제 로컬 HTTP·SQLite)
python3 tests/test_cancel_results.py   # Nav2 취소 결과 해석
python3 tests/test_agent_recovery.py   # 운영자 복구 후 Agent 저널 규칙
python3 tests/test_ownership.py        # 제어권 lease·경쟁·운영자 복구 (포트 18083, OWN_TEST_PORT로 변경)

# Open-RMF (격리 설치 후)
source ~/rmf_isolated/env.sh
cd rmf_adapter
python3 tests/test_runner_policy.py    # Adapter 정책(ROS/RMF 불필요)
python3 -u tests/run_rmf_checks.py     # RMF E2E: 서버 18090, WebSocket sink 18091, ROS 도메인 151/152
```

| 환경 변수 | 용도 |
|---|---|
| `FLEET_MOCK_MAP_YAML` | 모의 로봇 지도(기본 `maps/limo_sim_map.yaml`) |
| `RMF_ISOLATED_ENV` | 격리 RMF `env.sh` 경로(기본 `~/rmf_isolated/env.sh`) |
| `RMF_TEST_PORT`, `RMF_TEST_WS_PORT`, `RMF_E2E_TIMEOUT_S` | RMF E2E 포트·전체 제한 시간(기본 1200 s) |
| `OWN_TEST_PORT` | 소유권 시험 포트 |

결과 수치와 재현 방법은 [docs/VALIDATION.md](docs/VALIDATION.md)에 있습니다.

## 안전 제한

- 실차의 `allow_missions`는 기본 false입니다. 이 시스템으로 실차를 움직이는 시험은 하지 않았습니다.
- 기존 Safety Gate, SLAM, Nav2, 자동 맵핑 코드와 팀 워크스페이스는 수정하지 않았습니다.
- 운영 중인 8080 서버와 실차 Agent는 최신 코드로 교체하지 않았습니다. lease와 복구 API를 쓰려면 서버와 Agent를 함께 업데이트해야 합니다(오래된 Agent는 새 프로토콜의 명령을 받지 못합니다). 운영 전환 전에 데이터와 저널을 백업하세요.

## 라이선스

현재 별도의 오픈소스 라이선스가 부여되지 않았습니다. 이 디렉터리의 코드와 문서는 포트폴리오로 공개되어 있을 뿐이며, 제삼자에게 사용·수정·재배포·상업적 이용을 허용한다는 뜻이 아닙니다. 에이전트 패키지의 `package.xml`/`setup.py`에 있는 `TODO: License declaration`은 라이선스를 아직 선언하지 않았다는 표시입니다.

외부 의존성(ROS 2, Nav2, Open-RMF 등)은 각자의 라이선스를 그대로 따르며 이 저장소에 포함하지 않습니다(Open-RMF 설치 스크립트는 공식 패키지를 사용자 환경으로 내려받을 뿐입니다).
