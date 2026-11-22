# 웹 관제 자동 맵핑 · C103 지도 등록 · 지도 클릭 목표 (2026-11-19)

로컬 `fleet_control/`(18190 관제에 실제 사용된 Spatial Monitor·Map Registry·Safety Context 기준)에 `origin/main`(`ea2d963`)의 Mapping 세션 계약을 **선별 이식**했다. `git merge`나 파일 통째 교체는 하지 않았다. 변경 전 소스 스냅샷은 `local_data/worktree_backups_20261119_c103_web/`에 있다.

## 1. 구현 범위와 경로

| 기능 | 경로 | 비고 |
|---|---|---|
| 영역(Area) 계층·지도 연결·계보(lineage) | `server/fleet_server/map_registry.py` | `map_areas`, `map_version_areas`, `map_version_lineage`. 추가 테이블만 두고 쓰기 1회만 허용(트리거). `map_versions` 행은 수정하지 않음 |
| C103 지도 등록 스크립트 | `scripts/register_c103_map.py` | 원본 해시 확인, UNKNOWN 보존 검사, 멱등 |
| Mapping 세션 서버 계약 | `server/fleet_server/mapping.py` (신규), `service.py`, `http_api.py`, `recovery.py` | START/PAUSE/RESUME/SAVE/STOP, Lease·세션·멱등성, Worker 증거, 이동 임무·SIM SLAM과 상호 배제 |
| Mapping Worker | `agent/.../mapping_worker.py`, `local_process.py` (신규), `agent_node.py`, `ros_adapter.py` | SLAM Toolbox + `fleet_frontier_candidates` 소유 실행, Frontier 목표는 Fleet와 같은 goal slot 사용, 모터 잠금에서만 실행 |
| 웹 UI | `static/mapping_panel.js`, `static/goal_check.js` (신규), `app.js`, `index.html` | 영역 선택, 지도 출처(센서) 표시, Mapping 카드, 목표 사전 검사 |
| 시험 | `tests/test_map_areas.py`, `tests/test_web_mapping_session.py`, `tests/test_goal_check.mjs`, `tests/web_mapping_isolated/` | Mock·격리 ROS 2 |

`origin/main`에서 가져오지 않은 것: Bag Worker/Supervisor(로컬은 `recording_session`이 담당), velocity_arbiter, gamepad 패널, `limo_fleet_bringup` 패키지(격리 시험은 `tests/web_mapping_isolated/stack.launch.py`로 대체).

## 2. 지도 등록 (P0)

- 계층: `paichai_univ`(배재대학교) / `paichai_c`(C관) / `paichai_c_1f`(1F) / 영역 `paichai_c_1f_c103_lounge`(C103 인근 라운지, `PARTIAL_FLOOR`).
- 버전: `c103_lounge_20261119_depthmem#959a701c0974`, `SLAM_SAVED_BUNDLE`, **UNVERIFIED**, lineage `IMPORTED`.
- 원본: `local_data/raw_results/limo_runs/20261119_auto_map_depth/map/`. SHA-256 pgm `7db92ad2…`, yaml `1c6d98ca…`, posegraph `e03c404c…`, data `f6c34238…`. 344×472, 0.05 m, origin (-8.69, -16.5, 0), frame `map`.
- 셀: free 70,229 / occupied 4,575 / unknown 87,564. 205 픽셀은 p=0.19608 > free_thresh 0.196이므로 UNKNOWN으로 유지된다(스크립트가 검사).
- 센서 근거:
  - 지도(raster)는 SLAM `scan_topic=/scan_slam`으로 생성. LiDAR=O, Depth=X.
  - 맵핑 중 회피용 Costmap은 Depth=O, 장애물 기억 노드=O. 근거: launch 인자, bag `/depth_obstacles/points` 1,528건, `/collision_monitor/memory_points` 10,047건.
  - 기억 노드 소스는 실행 후 /tmp 삭제로 보존되지 않음.
- 지도·posegraph 동시성: 로봇이 +453 s부터 정지한 상태에서 같은 종료 절차로 `map_saver` → `serialize_map`을 실행했다.
- 품질: Frontier가 남은 상태에서 탐색 정지(`EXPLORATION_STOPPED`). 아래 절반 약 5.7~6.6° 휨. 지상 실측 없음.
- `run_010`의 `map_001/002`(LiDAR-only)는 등록하지 않았다.
- **현장 Registry(`~/.local/share/robot-sim-projects/limo_field_20261113/data`)에는 아직 등록하지 않았다.** 격리 data-dir에서만 검증했다.

## 3. Mapping HTTP 계약

- `GET /api/v1/robots/{id}/mapping`: 세션 상태.
  - 상태: `IDLE|STARTING|MAPPING|PAUSING|PAUSED|SAVING|STOPPING|STOPPED|UNKNOWN`
  - 함께 반환: `command`, `result.evidence`, `last_save`, `registered`, `plan`
- `POST /api/v1/robots/{id}/mapping` (operator/admin, 202 = 접수만 의미). 요청 본문:
  - 필수: `{action, epoch, idempotency_key}`
  - START 옵션:
    - `mode`: `NEW_MAP` | `CONTINUE_MAP`
    - `parent_map_version_id`
    - `geofence`: [[x, y] × 3..64], map 프레임
    - `limits`: `{max_duration_s 30..3600 (기본 900), max_goals 1..200 (기본 40), min_battery_v 9..13}`
  - START 외 action: `session_id` 필수
  - SAVE 옵션: `floor_id`, `area_id`. 지정하면 등록까지 수행
- Agent 측 엔드포인트:
  - `POST /agent/operation`: `poll | accept | result | reserve_goal`
  - `POST /agent/mapping-map`: 수락된 SAVE의 bundle을 UNVERIFIED 새 버전으로 등록하고 area·lineage를 연결
- 결과 판정:
  - 필요한 증거가 빠지면 OK를 강등한다. goal을 멈추는 명령(PAUSE·STOP)은 UNKNOWN, 그 외는 FAILED.
  - SAVE가 FAILED이면 PAUSED로 유지하고, 저장된 산출물은 Worker 세션 폴더에 보존한다.
- 다음 경우 UNKNOWN이 되며, 해제 API는 없다:
  - Lease 만료·변경, Agent/Server boot 변경
  - 상태 보고 3초 초과, 명령 TTL 만료
  - Worker 보고 `UNKNOWN`
- 차단 규칙:
  - START·RESUME 전 Nav2 유휴 증거가 필요하다. 외부 goal, 진행 중 이동 임무, 수동 입력, SIM SLAM 세션이 있으면 거부한다.
  - 세션이 `IDLE/STOPPED`가 아니면(UNKNOWN 포함) `create_mission`과 `dispatch_next`를 막는다.
  - 실차(mode real 또는 simulated false) Agent는 `mapping_real_blocked`로 거부한다.
- `CONTINUE_MAP`:
  - 부모 버전이 posegraph와 data를 모두 가져야 한다. PGM/YAML만 있으면 거부.
  - Agent `mapping_continue` 능력이 있어야 하고, 부모 지도 위 localization이 `CONFIRMED`여야 한다.
  - 현재 Worker는 posegraph 로드를 구현하지 않았다. 따라서 서버는 `continue_unsupported`로 거부한다. **구현되지 않은 기능이다.**

## 4. 검증 결과

| 수준 | 내용 | 결과 |
|---|---|---|
| Mock | `test_web_mapping_session.py` 20개 (Lease·멱등·UNKNOWN 고정·증거 강등·Nav2/임무/SIM SLAM 상호 배제·실차 거부·goal 예약·CONTINUE 게이트·lineage·Geofence·종료 한계) | PASS (ROS 미소스 시 Worker 단위 2개 SKIP) |
| Mock | `test_map_areas.py` 5개, 기존 Registry 45개 | PASS |
| Node | `test_goal_check.mjs` 11개 (C103 실제 PGM 셀 수, origin yaw 왕복, 행 뒤집기, 0.32 m 여유 검사) | PASS |
| 회귀 | `fleet_control/tests/test_*.py` 전체 | 54 PASS / 17 FAIL. 실패 17개는 변경 전 백업에서도 같은 스크립트·같은 오류 수로 실패하므로 신규 회귀 없음 |
| 격리 ROS 2 (localhost, domain 221) | 실제 `async_slam_toolbox_node`, `fleet_frontier_candidates`(스크래치 오버레이 빌드), Nav2 6개 서버, ICP odom, 잠금 Gate. 정지 합성 센서이며 `/cmd_vel` 구독자 없음. `web_mapping_isolated/probe.py` | 22/22 PASS. 수행 흐름: START→MAPPING, Frontier goal ACCEPTED(owner=frontier), PAUSE에서 goal CANCELED와 FollowPath 종결 확인, SAVE→새 UNVERIFIED 버전(area·lineage), RESUME, STOP→owned 프로세스 종료, 중복 START 멱등, CONTINUE 거부 |
| 격리 ROS 2 장애 | `fault.py` | 6/6 PASS. Agent SIGKILL 2.5 s 후 UNKNOWN, PDEATHSIG로 SLAM·Frontier 종료, START·임무 차단. Agent 재시작 후 Worker UNKNOWN 유지, 자동 재개 없음 |
| 웹 UI (Claude 내장 브라우저, Chromium) | 격리 서버 28951 | 건물→층→영역→버전 선택, 출처 배너. Lease 30 s를 패널이 갱신하며 START→PAUSE→SAVE→STOP 진행. UNKNOWN 표시와 버튼 비활성. 지도 실제 클릭 → (-0.35, 0.06) 셀(52, 50) FREE → 확인 후 제출 → 서버 `safety_not_ready` 409 |
| Gazebo | — | **SKIP**. 웹 Mapping Worker는 `/cmd_vel` 구독자가 있으면 시작을 거부(모터 잠금 전용)하므로 Gazebo 구동 경로가 없음 |
| 실차 | — | **BLOCKED** (서버가 실차 Agent 거부, 실차 미연결) |

UI 시험에서 발견해 수정한 결함:
1. Registry `/meta` 응답에 evidence·area·lineage가 빠져 있었다.
2. 기본 Lease TTL 30 s가 맵핑 도중 만료되어 세션이 UNKNOWN이 되었고, 서버가 Frontier goal을 소유자 없는 goal로 판단해 reconciliation 차단을 걸었다. 수정 내용:
   - UNKNOWN 세션도 예약된 자기 goal의 소유권을 유지한다.
   - 패널이 세션 진행 중에 Lease를 갱신한다(탭을 닫으면 만료되어 UNKNOWN).

목표점 Nav2 제출은 서버 입장 단계에서 멈춘다. 격리 환경에 localization과 Safety Context 생산자가 없기 때문이다. 클릭 → Fleet Mission → **Nav2 도달은 미검증**이다.

## 5. 재현

```bash
# 1) explore_lite(fleet_frontier_candidates 포함)를 임시 오버레이로 빌드 (capstone_ws/install은 건드리지 않음)
source /opt/ros/humble/setup.bash; source capstone_ws/install/setup.bash
colcon --log-base /tmp/wm/log build --base-paths capstone_ws/src/m-explore-ros2 --packages-select explore_lite --build-base /tmp/wm/build --install-base /tmp/wm/install
# 2) 격리 스택 + 프로브
OVERLAY=/tmp/wm/install fleet_control/tests/web_mapping_isolated/run.sh /tmp/wm/run1 28951 221
python3 fleet_control/tests/web_mapping_isolated/probe.py /tmp/wm/run1 28951
fleet_control/tests/web_mapping_isolated/stop.sh /tmp/wm/run1
# Mock / Node
python3 fleet_control/tests/test_web_mapping_session.py; python3 fleet_control/tests/test_map_areas.py; node fleet_control/tests/test_goal_check.mjs
```

## 6. 남은 차단 조건 (실차)

- 전원 돌발 차단, USB–UART 중단, Depth 거리 스케일·지터, 물리 E-Stop 미검증.
- Depth 높이 기준 모순: 실차 launch는 `base_footprint`와 `base_link`를 같은 높이로 두는데, URDF상 `base_link`는 바닥 위 0.145 m이다(현장 실측 필요).
- 실차용 localization·Safety Context 생산자가 없어 이동 임무가 거부된다.
- 웹 Mapping은 SIM/격리 Agent에서만 허용된다.
- 현장 18190 서버와 Registry는 교체하거나 등록하지 않았다.

## 7. 제안 (미확정 설계, 미구현)

**CONTINUE_MAP 실제 구현**
1. Worker가 Registry bundle에서 posegraph·data를 받는다. Agent 읽기 권한이 필요하다.
2. SLAM Toolbox를 `mode: localization`으로 시작하거나 `map_file_name`과 `map_start_pose`를 지정해 `DeserializePoseGraph`로 부모 그래프를 적재한다.
3. 운영자가 초기 pose를 지정하고, 스캔 정합 점수와 `map→odom` 안정 구간을 확인해 `mapping_localization={status:CONFIRMED, map_version_id}`를 보고한다.
4. 그 뒤에만 mapping 모드로 전환한다.
5. 저장 결과는 `CONTINUE_MAP` lineage로 기록한다.
- 미확인 시 START는 차단된다.

**버전 복원**: 버전은 변경할 수 없으므로 복원 = 부모 버전을 다시 활성 지도로 선택하는 것이다. `set_floor_active_map`은 구현되어 있지만, 권한·Safety Context 정책이 정해지지 않아 HTTP로 노출하지 않았다.

**Geofence 저장**: 현재는 START마다 다각형을 입력한다. 영역별 허용 다각형을 Registry에 버전 귀속으로 저장하고, 문·강의실 출입구를 금지선으로 표기하는 안을 제안한다.

**EXPLORE → VERIFY → REVISIT → SAVE**
- VERIFY 지표 후보:
  - 벽 RANSAC 직선성·직교성(이번 분석 방식)
  - posegraph 노드 밀도와 루프 클로저 수
  - Frontier 재현 검사: 정적 지도만으로 BFS
  - 동일 구역 반복 스캔의 정합 잔차
- REVISIT 대상: 낮은 관측 수 셀 군집, 이중 벽 후보(평행 근접 선분), 잔차 큰 구간.
- 종료 조건: 시간·목표 수·배터리(구현됨) + 이동 거리 + Gate 상태.
- 반복 주행이 품질 향상을 보장한다고 가정하지 않는다. 지표와 재방문 기준을 먼저 검증한다.
- 장애물 기억 노드(일시 장애물)는 Costmap에만 쓰고 SLAM raster에는 넣지 않는다. 현재도 SLAM 입력은 LiDAR뿐이다.

## 8. 실차 영상

- Google Drive 원본 2개(`20261119_205820.mp4` 2.9 GB, `C103_lidar.mp4` 2.0 GB)는 식별했다. 커넥터는 base64로만 받을 수 있고 내장 브라우저는 Google 로그인을 요구해 **분석하지 못했다**.
- YouTube 웹 관제 영상은 열지 않았다. 같은 종류의 로컬 증거인 `fleet18190.mp4`의 프레임 2,778장으로 관제 화면을 확인했다.

## 9. 후속 조사·검증 (2026-11-19 야간)

### 9.1 Depth 높이 기준 (조사, 실측 필요)

| 근거 | 내용 |
|---|---|
| 실차 기록 TF (`20261119_auto_map_depth` bag) | `base_footprint→base_link` z=0, `base_link→camera_link` (0.084, 0, 0.03), `base_link→laser_link` z=−0.034, `base_link→*_wheel_link` z=**−0.100** |
| URDF (`limo_four_diff.xacro`) | wheel_radius 0.045 → 바닥은 `base_link` 기준 z≈−0.145 |
| 해석 | 실행 중 `base_footprint`(=`base_link`)는 바닥 위 약 0.145 m에 있다. 필터 `[0.025, 0.35]`와 Costmap Depth `[0.02, 0.40]`은 바닥 기준으로 약 **0.17–0.50 m**와 **0.165–0.545 m**에 해당한다 |
| 영상 독립 근거 | 관제 Depth 패널 4프레임에서 유효 바닥 띠가 화면 하단 28–31%를 차지했다. 수평 카메라라면 카메라 높이 약 0.15–0.2 m와 맞는다. 높이 0.03 m(`base_footprint`=바닥 가정)이면 하단 대부분이 최소 거리 미만이 되어 무효여야 한다. 컬러맵 원본은 유실되어 추정 근거로만 쓴다 |
| 결과 | 바닥 위 **약 0.17 m 미만의 낮은 장애물(의자 하부 연결봉, 바퀴 받침대, 턱)은 Depth 필터에서 제외될 가능성이 높다**. LiDAR 평면(바닥 위 약 0.11 m, URDF)에 걸리지 않는 높이는 두 센서 모두 보지 못할 수 있다 |
| 부수 결함 | 필터 경고가 존재하지 않는 키 `marked_points`를 읽어 저장애물 경고가 발생하지 않는다 (`depth_obstacle_filter_node.py`, 다른 에이전트의 미커밋 파일이라 수정하지 않음) |

**실측 필요**: 바닥→`base_link` 높이(바퀴 반경 포함), 카메라 광학중심 높이·pitch, LiDAR 빔 높이, 의자 연결봉·책상 하부 프레임 높이, DaBai Depth 수직 FOV·최소 거리.

**권고(미적용)**: 실측 후 기존 launch 인자 `base_footprint_z:=<측정값>`으로 `base_footprint`를 바닥에 둔다. 그다음 모터 잠금 상태에서 낮은 막대 정적 마킹을 시험한다. 필터 높이 값 자체는 근거 없이 바꾸지 않는다.

### 9.2 CONTINUE_MAP 경로 (격리 실험)

- 구현 경로: `async_slam_toolbox_node`의 `map_file_name:=<parent prefix>`와 `map_start_at_dock:=true`(또는 `map_start_pose`)로 부모 그래프를 적재하고 맵핑을 이어간다. 저장은 기존과 같은 `map_saver_cli` + `serialize_map`이며 새 디렉터리에 쓴다.
- 격리 결과 (domain 222/223, 부모 = 격리 NEW_MAP 저장본, 부모 파일 읽기 전용 444):

| 경우 | 결과 |
|---|---|
| 올바른 시작 자세 | 부모 known 셀 100% 동일 클래스(점유 100% 유지), 새 known 셀 +3,215, graph.data 15 KB→31 KB, 부모 파일 해시 불변 |
| 잘못된 시작 자세 (1.0 m 오프셋) | 점유 셀 49%가 정상 결과에서 최대 0.92 m 벗어남 → **벽 중복** |
| Registry | 자식을 `CONTINUE_MAP` lineage(부모 지정)로 등록. 기존 버전 bundle 해시 불변, 전체 무결성 OK |

- 결론: 기계적 재적재·확장·비훼손은 확인했다. 실차에서는 localization 확인 전 START 차단이 반드시 필요하다(현재 서버 게이트 유지). Worker 통합(부모 bundle 수신·초기 pose·확인 절차)은 **미구현**이다.

### 9.3 Gazebo 속도 경로 (검토 + 단위 시험)

- 기존 시뮬 경로: Nav2 → `/cmd_vel_raw` → `stall_detector_node` → `/cmd_vel` → gz bridge. Gate를 우회하지 않는다.
- Worker의 `locked()`를 기존 `motor_policy.motor_path_allowed`(`SIM_GATED`) 결과와 결합했다. Frontier goal은 다음 두 경우에만 허용된다:
  - `LOCKED`: `/cmd_vel` 발행·구독 0
  - `SIM_GATED`: mode sim·simulated·`allow_sim_safety_gate`·`use_sim_time`이고, Gate가 유일 발행자이며 소비자가 1개 이하
- 실차(`ARMED`)는 거부한다. 단위 시험 4개 PASS.
- Gazebo 실제 실행은 **SKIP**. 필요한 구성: 시뮬 launch에서 자체 SLAM·explore를 끄고(Worker가 소유), Agent에 `mode:=sim simulated:=true allow_sim_safety_gate:=true use_sim_time:=true web_mapping_enabled:=true`를 지정한다.

### 9.4 웹 제어 상실 시나리오 (격리 ROS 2, 실제 Nav2 goal·속도 토픽 관측)

| 시나리오 | Nav2 idle / `/cmd_vel_raw` 0 도달 | 판정 |
|---|---|---|
| 브라우저 종료 = Lease 미갱신 (TTL 10 s) | 만료 후 3.1 s / 3.2 s | PASS 10/10 |
| Agent↔서버 네트워크 단절 (서버 정지 20 s) | 4.3 s / 4.4 s, Agent가 로컬에서 정지 | PASS 10/10 |
| 서버 재시작 | 3.1 s / 3.2 s | PASS 9/9 |
| **Agent 강제 종료 (SIGKILL)** | 관측 창 끝까지 Nav2 goal 활성, `/cmd_vel_raw`·Gate 출력 약 14.6 s 지속 | **FAIL 6/9** |

공통 확인 사항:
- `/cmd_vel` 발행자 0 유지
- 세션 UNKNOWN 고정, 자동 재시작 없음, 새 START 거부

**P0 발견**: Agent가 죽으면 Agent가 제출한 NavigateToPose goal을 취소할 주체가 없다. 실차 Gate 출력은 `/cmd_vel`이므로 감독 없이 주행이 이어질 수 있다. Fleet 미션도 같은 구조다.

**제안(미구현)**: Gate가 Agent heartbeat를 요구하도록 한다. 상실 시 영속도와 래치를 적용한다. 기존 SIM `supervisor_heartbeat`·challenge 구조를 실차로 확장하는 방안이다. Agent와 독립된 소형 감시 노드가 heartbeat 상실 시 활성 NavigateToPose goal을 취소하는 방안도 함께 제안한다. Gate 소스는 다른 에이전트의 미커밋 변경이 있어 수정하지 않았다.

기타 수정:
- 시험 중 발견: `/cmd_vel`을 구독하는 모니터 하나만 있어도 Worker가 START를 거부했다. 의도된 fail-closed 동작이다.
- goal 저널이 세션 상실 시 갱신되지 않던 결함을 수정했다.

### 9.5 현장 Registry 백업·등록·재로드·복원 (사본 drill)

- 18190 미실행 확인.
- 원본 `~/.local/share/robot-sim-projects/limo_field_20261113/data`를 `local_data/field_registry_backups_20261119/data_backup_B0`(0700, `auth.json` 포함, 공개 금지)로 파일 단위 복사했다. 원본과 바이트 동일.
- 작업 사본에서 수행:
  1. C103 등록
  2. 서버(28961) 기동
  3. Registry reconcile 결과 손상·미등록 0
  4. 저장 bundle을 Nav2 `map_server`(domain 224)로 재로드 → 344×472, 0.05 m, origin (−8.69, −16.5), free 70,229 / occ 4,575 / unknown 87,564 (원본과 동일)
- 복원: 작업 사본을 B0로 교체 → B0 manifest와 바이트 동일, 버전 0, 서버 기동 정상.
- drill 전후 원본 해시 동일.
- 참고: 앞선 읽기 전용(`mode=ro`) 조회가 원본 `map_registry.db`의 `-shm/-wal` 수정 시각을 바꿨다(22:57). 내용 변경은 없다.
- 새 코드가 원본 DB를 열면 추가 테이블(영역·계보)이 생성된다. 구 서버는 이를 무시하지만 적용 전 백업은 필수다.

**현장 적용 절차(승인 후)**:
1. 18190 중지 확인
2. `cp -a data <백업>` + sha256 manifest
3. `register_c103_map.py --data-dir <현장 data>`
4. 서버 기동 후 `/health` 및 웹에서 건물→층→영역 확인
5. 문제 시 서버 중지 → 백업으로 디렉터리 교체 → manifest 비교

### 9.6 기존 실패 17개 분류 (최종 코드 기준)

| 분류 | 스크립트 | 의도한 환경에서 재실행 | 변경 전 백업 |
|---|---|---|---|
| ROS를 source하면 스텁이 대체되는 하네스 문제 | agent_safety_v1, gate_boot_replay_guard, phase20_gate_pending, phase20_gate_state_sequences, r8_reconcile_cancel, r8_supervisor_sim, server_agent_safety_e2e, terminal_agent_order | ROS 없는 환경에서 **8개 모두 PASS** | 동일 하네스 실패 |
| 네임스페이스 opt-in 필요 | ros2_isolated_action, ros2_late_goal_reconciliation, gate_gid_gap_ros, ros2_goal_boundary_isolated, ros2_goal_domain_boundary | `unshare --user --net --mount` + `CAPSTONE_PRIVATE_ROS_TEST=1`에서 **5개 모두 PASS** | 동일 |
| 내 실행기 버그 (ROS domain > 232) | manual_mapping_ros | domain 210에서 **PASS** | 동일 |
| 오래된 기대값 (코드는 정상) | phase20_agent_stub (1건: 예약 UUID 도입 전 값 `'11'*16` 단언), ownership (Safety Context v1 이전 "lease 없이 임무 생성"), recovery (11건이 `safety_not_ready`로 임무 생성 실패) | 여전히 FAIL | **동일하게 FAIL** |

- 신규 실패는 0이다.
- 실차 영향: ownership·recovery는 실차 복구 경로를 다룬다. 현재 Safety Context 계약에 맞는 픽스처로 갱신하기 전까지 해당 경로는 **현재 계약 기준으로 회귀 시험되지 않는다**(P1).

## 10. 실차 전환 판정 (2026-11-19 기준)

| 항목 | 판정 | 근거 |
|---|---|---|
| C103 지도 Registry 등록·재로드·복원 절차 | PASS (사본) | 9.5. 현장 DB 미변경 |
| 웹 Mapping 수명주기·소유권·멱등 | PASS (격리) | 22/22 재확인 |
| Lease 만료·네트워크 단절·서버 재시작 정지 | PASS (격리) | 9.4 |
| Agent 강제 종료 시 goal·속도 정지 | **FAIL** | 9.4 P0 |
| CONTINUE_MAP | SKIP (Worker 미구현, 기계적 경로만 PASS) | 9.2 |
| Gazebo 실주행 | SKIP | 9.3 |
| Depth 낮은 장애물 인식 | **FAIL (기준 프레임 불일치 추정)** / 실측 필요 | 9.1 |
| 지도 클릭 → Nav2 도달 | SKIP (localization·Safety Context 생산자 없음) | §4 |
| 물리 E-Stop, 전원·USB–UART 안정성, Depth 스케일 | SKIP (미검증) | 현장 |
| 기존 회귀 시험 | PASS (신규 실패 0) / ownership·recovery 갱신 필요 | 9.6 |

**판정: 실차 웹 Mapping·주행 전환 불가.** P0(Agent 종료 시 goal 정지)와 Depth 높이 기준 실측이 해결되어야 한다. 모터 잠금 상태의 지도 표시·연결 확인만 가능하다.
