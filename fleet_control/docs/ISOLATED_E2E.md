# Fleet 격리 통합 시험

## 준비 및 실행

ROS 2 Humble 오버레이에 `limo_fleet_bringup`, `limo_robot_agent`, `limo_capstone`, `limo_depth_obstacles`, `explore_lite`를 빌드한다. 전용 호스트나 사용 중인 ROS 도메인과 겹치지 않는 로컬 환경에서 수행한다. 합성 스택은 첫 도메인과 다음 도메인을 사용한다.

```bash
source /opt/ros/humble/setup.bash
source <overlay>/install/local_setup.bash
run_dir="$(mktemp -d)/run"
fleet_control/scripts/run_isolated_fleet.sh "$run_dir" 28944 195
```

다른 터미널에서 같은 `run_dir`과 포트를 사용한다. 스택을 실행할 때마다 새 디렉터리를 사용한다. `Ctrl+C`로 하네스가 시작한 프로세스를 정리한다. 원본 런타임과 시험 데이터는 하네스 경로에 전달하지 않는다.

```bash
python3 fleet_control/tests/probe_isolated_fleet.py "$run_dir" 28944
python3 fleet_control/tests/probe_isolated_fleet.py --help
```

`--check-velocity`는 별도 새 실행에서 AUTO와 MANUAL 선택, 만료 후 영속도, 잠금 상태를 검사한다. 합성 입력은 정지 로봇의 /scan, /odom, RGB, Depth를 발행한다. SLAM Toolbox, Frontier 후보 검색, Nav2, Smoother, Gate, rosbag2는 실제 ROS 실행 파일이다. 운영 화면은 [로컬 Fleet](http://127.0.0.1:28944/)에서 시험용 `secrets/operator.token`으로 접속한다.

## 실행 구조 및 인터페이스

| 영역 | 기존 구현과 사용 경로 |
|---|---|
| Perception | 기존 Depth Filter는 `depth_filter:=true`로 호출한다. 센서 공급자가 `/scan`, RGB, Depth를 발행한다. 기본 영상 토픽은 `/camera/color/image_raw`, `/camera/depth/image_raw`다. |
| Localization | 기존 ICP와 단일 Odom TF Relay를 사용한다. `/odom`, `/odometry/lidar_odom`, `/odom_nav2`, `odom → base_footprint`를 사용한다. |
| Mapping | Agent가 `async_slam_toolbox_node`와 `fleet_frontier_candidates`를 소유한다. 발행은 `/map`, `/fleet/frontier_candidates`다. 저장은 `map_saver_cli`와 `/slam_toolbox/serialize_map`을 사용한다. |
| Navigation | 기존 Nav2 패키지와 Capstone 파라미터를 사용한다. Action은 `/navigate_to_pose`다. 중재 launch에서 controller와 behavior의 출력은 `/fleet/cmd_vel_nav`다. |
| Safety | Smoother 입력은 `/fleet/cmd_vel_selected`다. 출력은 `/cmd_vel_raw`를 거쳐 기존 Stall Safety Gate의 `/cmd_vel_locked`에 도달한다. 잠금 launch에 하드웨어 드라이버와 `/cmd_vel` 출력은 없다. |
| Teleoperation | 기존 Web Gamepad Lease·샘플 API 사용. Agent에서 Deadman과 TTL을 재확인한다. Arbiter는 NavigateToPose와 FollowPath의 종결 증거 및 단일 경로 그래프를 확인한다. FollowPath 상태는 관측한 controller endpoint가 현재도 단일하게 존재할 때 사용한다. |
| Fleet Bridge | 기존 Agent·Server·대시보드를 사용한다. 카메라는 boot-fenced `POST /agent/camera`, 읽기 권한이 있는 `GET /robots/{id}/camera/{rgb,depth}.jpg`로 전달한다. 프레임은 메모리에서 만료된다. |
| Bringup | 새 패키지 `limo_fleet_bringup`의 `isolated_locked.launch.py`가 기능별 모듈을 조합한다. 기존 패키지와 실차 launch 진입점은 원래 경로를 사용한다. |

모든 새 Worker와 Arbiter opt-in의 기본값은 false다. 전용 launch의 `fleet_goals:=true mapping:=true`는 중재 시험용으로 Fleet 목표를 허용한다. 이 프로파일에서 bag, manual, velocity는 false다. 운영용 격리 하네스는 Fleet 목적지 제출을 끄고 Mapping 소유 Frontier만 사용한다. 두 소스는 Agent의 같은 목표 슬롯을 사용한다. 기존 직접 제출형 `explore`와 통합 Worker를 같은 ROS 그래프에 실행하지 않는다.

독립 Bag Supervisor는 전용 프로세스와 익명 pipe로 세션을 소유한다. Agent가 종료되면 Supervisor가 EOF를 감지하고 해당 recorder만 SIGINT 종료한다. 통신이나 Lease가 끊기면 5초 heartbeat 만료를 사용한다. 녹화 중 300초, 합계 256 MiB, 디스크 여유 1 GiB를 감시하며 recorder 각 파일에 256 MiB 커널 제한도 적용한다. Linux parent-death signal이 Supervisor 자체가 죽을 때 recorder를 SIGKILL한다. Supervisor가 세션 상태를 남기지 못했으면 Agent가 종료 확인 뒤 해당 저널을 UNKNOWN으로 기록한다. 상태는 `bag_worker/sessions/<sid>/journal.json`, 로그는 같은 폴더, 가방은 `bag_worker/bags/<sid>`에 남는다. 정상 STOP은 실제 rosbag2 metadata와 `ros2 bag info`까지 확인한다.

비정상 결과를 가진 기존 세션·디렉터리는 보존한다. Agent 재시작 시 미결 상태를 인수하거나 재실행하지 않는다. 운영자는 Fleet 결과, 세션별 저널, PID/start ticks, ROS 그래프와 저장 산출물을 대조한다. 격리 재검증은 별도 새 실행으로 시작한다. 현재 UNKNOWN을 해제하는 Mapping/Bag 운영 API는 제공하지 않는다.

## 최종 검증 매트릭스

모든 실행에 개별 run ID, 명령 ID, Lease epoch, Agent/Fleet boot, Goal UUID, 최초 실패 시각을 기록한다. PASS 기준을 모두 관찰한 항목에만 PASS를 쓴다.

| 시나리오 | 실행 및 PASS 기준 |
|---|---|
| Web → Fleet → Agent → ROS 2 | 새 실행에서 운영 화면의 Lease 확보 후 START, PAUSE, SAVE, RESUME, STOP과 Bag START/STOP을 사용한다. 진행·결과의 command ID가 Agent 저널과 일치해야 한다. API 재현은 `probe_isolated_fleet.py`를 사용한다. |
| Bag 장애 주입 | 세션별 저널에 기록된 소유 PID와 start ticks를 대조한다. 새 실행마다 Agent SIGKILL, Supervisor SIGKILL, recorder SIGKILL, Fleet 중단, Lease 만료, 디스크·시간·용량 한계를 각각 주입한다. 대상 recorder 종료, 다른 recorder 생존, 지연·부분 metadata의 UNKNOWN/FAILED, 동일 세션 재실행 차단을 확인한다. 디스크 한계는 시험 전용 파일시스템으로 격리한다. |
| Mapping 수명주기 | 프로세스 기동과 실제 map/후보 발행 준비 시점을 기록한다. 활성 목표 중 PAUSE·STOP을 요청하고 목표의 UUID terminal과 다음 Goal 억제를 확인한다. PAUSE는 Frontier 제출을 멈추고 저장을 위해 SLAM과 지도 처리를 유지한다. 수락된 목표의 PAUSE·STOP은 해당 목표 제출 이후 FollowPath 활성 상태를 관측하고 종결도 확인한다. 상태 관측이 누락되면 UNKNOWN으로 남을 수 있다. 상태 증거가 없으면 UNKNOWN을 유지한다. SAVE의 raster와 posegraph에 각각 장애를 주입한다. |
| 동시 최초 Goal | 일반 ROS 어댑터 경쟁은 `test_cancel_results.py`의 양쪽 제출 동기화 테스트를 사용한다. 실동시 제출은 `fleet_goals:=true mapping:=true`에서 동일 운영자 Lease로 Fleet 목적지와 Mapping START를 경쟁시킨다. Goal 예약·Mission 상태를 수집하고 둘 이상의 Action Goal이 수락되면 실패로 판정한다. |
| AUTO·MANUAL 전환 | `--check-velocity`로 조작한다. 실제 Gamepad에서는 Deadman 해제·입력 타임아웃·브라우저 연결 유실과 장치 분리를 각각 확인한다. `/cmd_vel` 여분 발행자를 주입한 격리 시험에서 Arbiter 출력이 영속도로 내려가야 한다. |
| 재시작과 네트워크 | 같은 임시 DB·Agent 저널을 재사용하는 별도 실행에서 Server/Agent 재시작 조합을 수행한다. SQLite 결과, Controller terminal, Lease epoch를 대조하고 새 Goal/recording 자동 재실행을 실패로 판정한다. |
| 패키지와 Gazebo | 새 오버레이에서 기존 진입점과 `limo_fleet_bringup` launch 인자를 검사한다. 기존 `limo_integrated_auto_mapping_sim_diff.launch.py`에서 Gazebo 장애물 회피 회귀를 수행하고 `/cmd_vel` 경로를 기록한다. 이후 전용 오버레이에 Gazebo 센서·시뮬레이션 구동 경로를 별도로 연결한 검증은 잠금 프로파일 결과와 구분한다. |
| 웹 다중 로봇·영상 | 첫 로봇에서 RGB·Depth·지도 수신, 두 번째 로봇의 영상 없음, 전환 시 이전 프레임 제거를 확인한다. Gamepad 보정·버튼 프로필 및 오류·UNKNOWN 표시를 검사한다. |
| 맵 재로드 | 새 ROS 도메인에서 Nav2 Map Server에 생성된 `map.yaml`을 주고 map의 크기·해상도·원점과 미관측 셀 분류를 대조한다. Worker는 trinary PGM에서 미관측 픽셀 205를 보존하도록 free threshold 0.196을 사용한다. SLAM Toolbox `DeserializePoseGraph`에 저장된 `graph` prefix를 주고 반환값과 재생성된 지도를 확인한다. |

실차는 별도 승인 후 기존 현장 Runbook을 사용한다.
