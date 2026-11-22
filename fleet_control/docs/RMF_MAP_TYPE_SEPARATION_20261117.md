# `/map` 타입 충돌 해결 및 Gazebo rosbag 통합 검증 (2026-11-17)

**범위**: 격리 ROS(`ROS_LOCALHOST_ONLY=1`)의 실제 Gazebo Fortress(Office+LIMO+Open-RMF slotcar 플러그인), 실제 Fleet Server·Robot Agent·SLAM Toolbox·rosbag2·Chromium. **실차 연결·모터 구동 없음**(시뮬 로봇만). 시험 전용 토큰은 `local_data/raw_results/rmf_map_bag_gazebo_20261117/*/tokens`에만 있으며(Git 제외) 이 문서에 값을 싣지 않는다. commit/push 없음.
공통 문서(`CURRENT_STATUS`·`VALIDATION`·`HANDOFF`·`CODE_INDEX`)는 충돌 가능성 때문에 편집하지 않고 §9 제안으로 남긴다.

## 1. 원인 (조사 결과)
| 사실 | 근거 |
|---|---|
| `/map`에 OccupancyGrid(SLAM)와 BuildingMap(RMF) 공존 | 과거 `/tmp/limo-gazebo-web-map-20261117-a/run06/topic-graph.log`, 본 작업 `noshim_01/graph-0[123].log` |
| BuildingMap **구독자**는 Gazebo slotcar 플러그인 `plugin_tinyRobot1/2`(절대 이름 `/map` 하드코딩, `rclcpp::init(0,nullptr)`이라 launch remap 불가) | `strings libslotcar_common.so`에 `/map`, 위 graph 로그 |
| BuildingMap **발행자**는 RMF `building_map_server`(상대 이름 `map`)이며 프로젝트 `run_rmf_stack.sh`에는 없음. 과거 run06 발행자 수는 0 | `common.launch.xml`, run06 로그 |
| rosbag 실패: `Topic '/map' has more than one type associated…` | `noshim_01/recordings/*/recorder.log` 591회 |
| 추가 영향: BuildingMap 발행자가 `/map`에 있으면 Agent의 SLAM 사전 점검(`count_publishers('/map')`)이 웹 SLAM 시작을 거부(`SLAM/Nav2/map producer already detected`) | `noshim_01/slam/current.json`, 브라우저 `slamStatus RUNNING` 대기 timeout |
Nav2·SLAM Toolbox·map_server·Agent는 OccupancyGrid `/map`만 사용(변경 없음). Map Registry는 `/map` 토픽이 아니라 저장 파일을 쓴다.

## 2. 수정
- 최종 이름: **`/rmf/building_map`** (BuildingMap 전용). `/map`은 OccupancyGrid 전용 유지.
- `fleet_control/rmf_adapter/rmf_map_shim/` (신규): `rmf_map_remap.c`(`LD_PRELOAD`, `rcl_subscription_init`/`rcl_publisher_init`에서 **타입이 BuildingMap이고 이름이 `/map`·`map`인 경우만** 이름 변경), `build.sh`, `README.md`. RMF deb 바이너리는 수정하지 않음. 타입 판별 불가 시 통과.
- `fleet_control/agent/limo_robot_agent/limo_robot_agent/recording_session.py`: 고정 프로파일 `--extra`에 `/rmf/building_map` 추가(`/scan_slam` 유지, 운영자 입력 토픽 없음).
- 시험: `fleet_control/tests/test_rmf_map_shim.py`(신규), `test_recording_profile_topics.py`(신규). 기존 시험 파일은 수정하지 않음.
- 실제 Gazebo E2E 도구: `experiments/rmf_map_bag_gazebo_20261117_R4m2/`(`run.sh`, `run_e2e.py`, `verify_bag.py`, `verify.sh`, 과거 하네스에서 복사한 `sensor_only.launch.py`·`web_e2e.cjs`·`gate_r7.params.yaml`). Gate r9/r10, Cliff, Goal 소유권, 안전 정책은 수정하지 않았다(과거와 같은 r7 Gate 구성을 그대로 사용).
- 수정하지 않은 것: `run_recorder.py`(다른 에이전트의 미커밋 변경 존재), `slam_control.py`.

## 3. 실제 RMF·SLAM 동시 실행 (shim_01 / shim_02)
`graph-01..03.log`: `/map [nav_msgs/msg/OccupancyGrid]`, `/rmf/building_map [rmf_building_map_msgs/msg/BuildingMap]` 두 토픽이 **각각 단일 타입**. `/rmf/building_map` 발행자 `building_map_server` 1, 구독자 `plugin_tinyRobot1`·`plugin_tinyRobot2`(C++ rclcpp 경로 포함) 및 recorder. `/map` 구독자 `fleet_robot_agent`·recorder(OccupancyGrid). 플러그인 로그에 `[rmf_map_remap] BuildingMap /map -> /rmf/building_map`. **PASS**.
대조군 `noshim_01`(shim 없음): 위 §1과 동일한 충돌·SLAM 시작 거부·rosbag 오류 재현 → **FAIL(예상된 실패, 원인 확인용)**.

## 4. 실제 Gazebo rosbag 기록·재생 (`verify_bag.py`, 두 번 모두)
| 항목 | shim_01 | shim_02 |
|---|---|---|
| `/map` OccupancyGrid 메시지(타입 단일) | 2 | 3 |
| 마지막 `/map` 내용(265×202, 0.05 m/cell, known 53530, occupied 52) | PASS | PASS (동일 해시) |
| `/rmf/building_map` BuildingMap 메시지, level L1, nav_graph 1 | 1 | 1 |
| `/scan`·`/tf`·`/tf_static`·`/odom`·`/odom_nav2`·`/odometry/lidar_odom` | 333·5007·3·1663·333·333 | 356·5455·3·1780·356·356 |
| `/tf`에 `map→odom`(SLAM)·`odom→base_footprint` | PASS | PASS |
| metadata.yaml 토픽 17개와 SQLite 타입·개수 일치, 다중 타입 토픽 없음 | PASS | PASS |
| 다른 격리 domain에서 `ros2 bag play` → `/map` 복원(저장 맵과 동일 데이터 해시) | 2/2 | 3/3 |
| 재생 → BuildingMap 복원(level L1) | 1/1 | 1/1 |
| 저장된 `map.pgm` 크기 = bag 마지막 `/map`(265×202) | 일치 | 일치 |
**미충족(FAIL 유지)**:
- `/scan_slam`: 기록 메시지 0. 이 sim 그래프에는 `/scan_slam` 발행자가 없고 SLAM이 `/scan`을 읽는다(`/scan_slam`은 실차 scan normalizer의 출력). 기록 프로파일에는 있으나 **이 환경에서는 입력이 없어 검증 불가** → 실차 현장 graph에서 별도 검증 필요(`FIELD_MAPPING_PRECHECK_20261118.md`). 통과로 간주하지 않는다.
- CameraInfo·원본 영상: 기록 0. 이유: ① sim 그래프에 `CameraInfo`·RGB `Image` 발행자가 없음(있는 것은 `/camera/depth_image`, `/camera/depth/points`뿐; `graph-*.log`), ② 고정 기록 프로파일(`CORE_TOPICS`+`/scan_slam`+`/rmf/building_map`)에 영상·CameraInfo는 설계상 포함되지 않음(대용량, 별도 영상 recorder 경로). 필요 시 프로파일 변경 결정이 먼저 필요하며 이번에 추가하지 않았다.

## 5. 웹 SLAM·Map Registry 회귀 (실제 Gazebo, 두 번)
웹 녹화 시작 → SLAM 시작(RUNNING, 단일 slam_toolbox 발행자·신선한 scan/TF) → 중복 start 202/409/403 → 지도 저장(map.yaml/pgm/posegraph/data, Registry 등록 UNVERIFIED `…#926a001b33fc`) → SLAM 중지(STOPPED) → 저장 지도 `map_server` 재로드·lifecycle configure/activate → SLAM 중지가 bag에 영향 없음 → 웹 녹화 중지 → VERIFIED. 전 항목 **PASS**.
pytest: `test_recording_session_p0`·`test_slam_control` + 신규 2종 = 14, 확장 회귀(slam_sessions, slam_upload_auth, map_registry ×3, bag_copy_faults, verify_bag_copy, agent_safety_v1) 포함 총 **76 PASS**, 실패 0. 신규 `test_rmf_map_shim` 2 PASS(shim 없음=2타입, 있음=분리).

## 6. Posegraph 등록 문제 (진단만, 해결 아님)
실제 Office 지도: `map.posegraph` 4,972,217 B + `map.data` 16,066 B + `map.pgm` 53,545 B + yaml 121 B = 5,041,949 B. `slam/current.json`: `posegraph_saved:true`, **`posegraph_registered:false`**, Registry 행의 `posegraph_sha256`=NULL(지도 yaml/pgm만 등록).
원인(코드 근거): ① `agent_node.py` `upload_slam_map`은 합계 `< 3_800_000` B일 때만 posegraph를 페이로드에 넣는다. ② 서버 `/agent/slam-map` JSON 본문 한도 `MAX_MAP = 6 MiB`(`http_api.py`)이고 base64는 ×4/3이라 5.04 MB → 약 6.72 MB로 6 MiB(6.29 MB)를 넘는다. 즉 임계값만 올려도 413. ③ Registry 자체 한도는 posegraph 128 MiB, HTTP write 48 MiB로 Agent 경로 한도와 불일치. 해결안(분할/스트림 업로드 또는 별도 한도)은 구현하지 않았고 **미해결**이다. posegraph 크기는 지도 면적·시간에 따라 증가한다.

## 7. 재현 명령
```bash
cd /home/cjinw/school/2026-2/Capstone1_limo
PYTHONPATH=fleet_control/agent/limo_robot_agent:fleet_control/server python3 -m pytest -q fleet_control/tests/test_recording_session_p0.py fleet_control/tests/test_slam_control.py fleet_control/tests/test_recording_profile_topics.py fleet_control/tests/test_rmf_map_shim.py
export NODE_PATH=/home/cjinw/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules
experiments/rmf_map_bag_gazebo_20261117_R4m2/run.sh <새 run_id> <미사용 ROS domain 0-232>             # shim 사용
experiments/rmf_map_bag_gazebo_20261117_R4m2/run.sh <새 run_id> <domain> --no-shim                 # 충돌 재현 대조군
experiments/rmf_map_bag_gazebo_20261117_R4m2/verify.sh <run 출력 dir> <domain>                     # bag만 재검증
```
Gazebo는 약 1분 부팅 후 웹 흐름이 약 2분 걸린다. 같은 ID의 출력 디렉터리는 덮어쓰지 않는다. 시험 소유 프로세스만 종료할 것(`ROS_DOMAIN_ID`로 구분).

## 8. 판정 요약
| 항목 | 결과 |
|---|---|
| `/map` 충돌 해결(Gazebo+RMF 그래프에서 두 타입 공존) | **PASS** (shim, 2회) |
| SLAM 지도 데이터 rosbag 실제 기록·재생 | **PASS** (2회) |
| BuildingMap 기록·재생 | **PASS** |
| 웹 SLAM·저장·Registry·재로드·중지 | **PASS** |
| `/scan_slam` 기록 | **FAIL/미검증**(sim 미발행) |
| CameraInfo·원본 영상 기록 | **미기록**(sim 미발행 + 프로파일 설계 외) |
| Posegraph Registry 등록 | **미해결**(진단 완료) |
| 실차·장시간·shim 없는 RMF 소비자 호환 | **SKIP/미검증** |
시험 중 이슈: 과거와 달리 `&`로 기동한 하네스의 자식은 SIGINT를 무시해 종료 timeout이 났다 → 하네스에 SIGTERM 폴백 추가. 첫 `shim_01`은 verify 스크립트 경로 버그(`.zstd` 파일명)로 1차 verify 실패 후 스크립트 수정·재검증(동일 bag, 자료 보존). 시험 소유 잔존 프로세스(해당 domain)만 수동 종료했다.

## 9. Phase 2 주의사항 및 공통 문서 제안
- shim은 **Gazebo/RMF 쪽에만** 필요하다. 실차 Agent·SLAM·recorder에는 불필요. RMF 웹/시각화 등 새 BuildingMap 소비자는 `/rmf/building_map`을 쓰거나 shim 하에서 기동해야 한다. 근본 해결은 `rmf_robot_sim_common` 재빌드(토픽 파라미터화).
- 기록 프로파일 변경은 Agent 코드 수정이며 실차 배포 시 `/rmf/building_map`은 미발행이면 무해하나 `/scan_slam`은 실차에서 발행 확인 필요.
- Posegraph 한도는 지도 면적이 커지면 항상 걸린다.
- 제안(독립 patch, 미적용): `VALIDATION.md`에 §4·§5 PASS/FAIL 행, `CODE_INDEX.md`에 `fleet_control/rmf_adapter/rmf_map_shim/`·`experiments/rmf_map_bag_gazebo_20261117_R4m2/`, `CURRENT_STATUS.md`에 `/map`=OccupancyGrid·`/rmf/building_map`=BuildingMap 규칙, `HANDOFF.md`에 Posegraph 미해결·`/scan_slam` 현장 검증 항목.
