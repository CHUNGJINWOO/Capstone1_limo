# Agent v5 오프라인 candidate (2026-11-23, Claude; 실차·18190·commit 없음)
`candidate`는 기술 검증용 라벨이다(`deployment_approved=false`). 배포 승인이 아니다. 기록: `local_data/raw_results/real_limo_integ_claude_20261122/{deploy_candidate_agent_v5,v5_candidate_01}/`.

## 1. 산출물
| 항목 | 값 |
|---|---|
| tar | `deploy_candidate_agent_v5/limo_robot_agent_candidate_v5_20261123.tar.gz`, 32 파일 |
| tar SHA-256 | `33a9e55dbdb9e14438fa0aaa78c4e19b635741b1dd4759ed9e750f30ba6fe5c7` |
| FILES.sha256 SHA-256 | `be62c38a2da2658e34812cad327df390527c70303138f375f72d2aae38f587b7` |
| 소스 기준 | 작업트리 `fleet_control/agent/limo_robot_agent/` (HEAD `c804241a` 위의 **미커밋** 소스; manifest의 `git_head`는 HEAD일 뿐 소스와 같은 commit이 아니다) |
| 생성 | `limo_package_deploy_20261122.sh artifact --src-dir fleet_control/agent/limo_robot_agent --pkg limo_robot_agent --out-dir <OUT> --stamp v5_20261123 --status candidate` |
| 핵심 파일 SHA-256 | core `ab087e3c…`, safety_protocol `c21c93f1…`, ros_adapter `942084e2…`, motion_watchdog `c5d3a575…`, mock_robot_node `b553768e…` (전체: `v5_candidate_01/source_sha256.txt`) |

## 2. 검증(오프라인)
- 같은 소스로 두 번 생성한 tar와 FILES.sha256이 바이트 단위로 동일(결정성). `created_at_utc`·`source_dir`는 manifest에만 있고 tar에는 없다.
- `TAR.sha256`, 추출 후 `FILES.sha256` 32개 일치, 추출본 = 소스(`diff -rq`), py_compile 전체 OK. 비밀·`.db`·`.log`·pyc는 R3 도구의 차단 목록과 allowlist로 제외됨(dry-run 및 실생성 통과).
- v4(`97071d50…`) 대비 달라진 파일은 5개뿐: `core.py`, `safety_protocol.py`, `ros_adapter.py`(R1), `motion_watchdog.py`, `mock_robot_node.py`(R2). package.xml·setup.py·setup.cfg·resource·나머지 모듈은 v4와 동일.
- 진입점: `robot_agent`, `mock_robot`, `motion_watchdog` (setup.py console_scripts, 각 모듈에 `main`).
- 이번에 R3 시험·Agent 시험을 다시 돌리지 않았다(소스 무변경; Codex 검증 기록의 소스와 파일 해시 동일). colcon 빌드·설치 시험은 이번 후보에 대해 수행하지 않았다.

## 3. 포함·제외
- **포함(32):** 패키지 `limo_robot_agent` 전체(모듈 29개 `__init__` 포함, package.xml, setup.py, setup.cfg, resource 마커). Gate Reset·localization·mapping worker, velocity_arbiter, manual bench 등도 같은 패키지라 함께 들어가지만, 실행 여부는 시작 스크립트·launch가 정한다.
- **서버 전용(미포함):** `fleet_control/server/`(`service.py` 등).
- **시험·도구(미포함):** `fleet_control/tests/`, `scripts/`, `docs/`.
- **제외:** `__pycache__`, 토큰·`auth.json`·`.db`·로그·bag·실험 원본.
- 시작 스크립트(`start_robot_agent_field.sh` 등)와 launch는 로봇 쪽 파일이라 패키지에 없다. 로봇의 사본은 갱신되지 않았다.

## 4. Server–Agent 계약 호환성 (`v5_candidate_01/contract_probe.out`, 메모리 서버, 소켓 없음)
| 필드 | 실제 서버 응답 | v5 Agent 요구 |
|---|---|---|
| hello `server_boot_id`, `protocol_version` | 있음, 2 | 필수, protocol 정수 2 |
| 상태 응답 `server_boot_id` | 있음 | hello와 일치 |
| `safety_contract_version` | 1 | 정수 1 |
| `reconciliation_required` | 명시 `false` | 정확한 Boolean false |
| `lease` | Lease 없을 때 `null`(키 있음), 있을 때 owner/epoch/expires_at 포함 | 키 필수, 구조 정상·미만료 |
| 상태 응답 `protocol_version` | 없음 | 선택 항목(있으면 2) — 정상 |
- v4 Agent는 위 필드를 모두 요구하지 않아 구버전 서버와도 동작한다. **v5 Agent는 R1.4.5 이후 서버(명시 False·lease 키)가 먼저 떠 있어야** 운동 승인을 받는다. 구서버에서는 모든 운동이 거부된다(fail-closed). 따라서 순서는 서버 → 응답 계약 확인 → Agent v5.
- 운영 18190 서버는 조회·변경하지 않았다.

## 5. 패키징 문제 / 한계
1. 소스가 미커밋이라 후보와 commit이 연결되지 않는다. 배포 전 commit 범위 결정과 재생성이 필요하다.
2. `package.xml`에 `geometry_msgs`, `std_srvs`, `rcl_interfaces`, `python3-yaml`이 없다(코드는 import). v4에도 있던 기존 누락이며 로봇 ROS 설치에 포함되어 있을 가능성이 높으나 이번에 확인하지 않았다(미검증).
3. R3 패키지 교체는 비원자적이고 이중 실패 시 SRC가 없을 수 있다.
4. 로봇 쪽 시작 스크립트 사본은 구버전이다.

## 6. 현장 배포 차단 조건(전부 미완료)
Fleet 서버 선배포 승인, Agent v5 실차 배포 승인, R3 이중 실패 복구, 모터 잠금 스택에서 Mapping STOP·Arbiter·Watchdog·Gate·DDS 검증, `/cmd_vel` 우회 차단, Depth 연동, 전원·USB/UART·STM32 watchdog, 물리 E-Stop·제동거리, 보조 실행 주체 세대 결합, 응답 순서 검사.

## 7. 판정
**Agent v5 오프라인 candidate 준비 완료(기술 후보). 실차 배포는 BLOCKED**(위 6번).

## 8. 후속: 의존성 선언 보완과 격리 빌드 (v5b, 2026-11-23, Claude)
- **의존성 대조:** 코드가 직접 import하는데 `package.xml`에 없던 것: `geometry_msgs`(8곳, 예: `motion_watchdog.py`·`velocity_arbiter.py`), `std_srvs`(`gate_reset_worker.py`·`localization_worker.py`), `rcl_interfaces`(`localization_worker.py`), `yaml`(`mock_robot_node.py`·`slam_control.py`). 나머지 import(rclpy, nav_msgs, sensor_msgs, std_msgs, tf2_ros, nav2_msgs, action_msgs, unique_identifier_msgs, numpy, PIL)는 이미 선언되어 있다.
- **수정(`package.xml`만, 7줄 추가):** `<depend>` geometry_msgs, std_srvs, rcl_interfaces; `<exec_depend>python3-yaml`. 코드·R1·R2 로직·R3 도구는 무변경.
- **격리 빌드:** 후보 tar를 임시 colcon 워크스페이스(scratchpad, ROS Humble, `ROS_LOCALHOST_ONLY=1`)에 추출하고 `sha256sum -c FILES.sha256` 32개 OK 후 `colcon build --packages-select limo_robot_agent` **PASS**(설치 후 `package.xml`에 14개 의존성 반영). 설치된 실행 파일 `robot_agent`, `mock_robot`, `motion_watchdog`(`ros2 pkg executables`로 확인). 설치된 패키지의 모듈 전체 import 실패 0. stderr는 byte-compiling 비활성 경고뿐. 기존 v5 내용을 `--symlink-install`로 먼저 빌드한 결과도 PASS였다 — 즉 누락 선언이 빌드를 막은 것은 아니고(현 노트북엔 패키지가 설치됨), **rosdep/colcon 의존성 해석용 메타데이터 보완**이다.
- **파일 수:** tar 32, FILES.sha256 32줄, 소스 32 일치(manifest `file_count` 32).
- **재생성 `deploy_candidate_agent_v5b/`:** tar `c71b166febf6a9acc4608082c8f9e5581be6a4970c11a805928ee5352102aeaa`, FILES.sha256의 SHA-256 `5ed04f9220495da3c244e4ccb6ffd591de0b23eb38b26dbc73fa4e0c561851ab`; 두 번 생성 바이트 동일. 기존 v5(`33a9e55d…`)는 덮어쓰지 않고 보존. v5 대비 달라진 파일은 `package.xml` 하나(`7aa670d3…` → `5abdf9e8…`).
- **미검증:** `rosdep install`(이 노트북에서 rosdep 미초기화), 로봇의 ROS에 해당 패키지 설치 여부, 실제 Agent 실행. 소스는 여전히 미커밋이며 `candidate`(`deployment_approved=false`)다. 기록: `v5b_build_01/`.
- **commit 후보 목록(승인 시):** `fleet_control/agent/limo_robot_agent/**`(R1 3·R2 2 파일 + `package.xml`), `fleet_control/server/fleet_server/{service,mapping}.py`, R1~R3 시험(`tests/test_r1_4_*_20261123.py`, `test_r2_watchdog_shutdown_20261123.py`, `test_r3_deploy_tool_20261123.py`)과 수정된 기존 시험 7개, `scripts/limo_package_deploy_20261122.sh`, `tests/agent_shutdown_matrix_20261122.py`, `docs/R1_4_*`·`AGENT_V5_CANDIDATE_*`, `server/SAFETY_CONTRACT.md`. 공통 문서 8종·`AGENTS.md`·`local_data/`·`capstone_ws/`는 제외(별도 정책). 정확한 목록은 commit 승인 시 `git status`로 재확정.
