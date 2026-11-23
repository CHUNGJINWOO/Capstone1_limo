# 자율주행 설계 (자동 맵핑 · 센서 융합 · 위치추정 · 주행 · 기록)

작성일: 2026-11-10 / 문서 범위: 자율주행, 동적 장애물 회피
상태 표기: **[확인]** 파일·교재에서 직접 확인 / **[제안]** 설계 제안 / **[미확인]** 실행·실측 전이라 단정 불가

> 실차 검증 범위는 `docs/REAL_ROBOT_STATUS.md`, 설정 분리는 `docs/CONFIG_SIM_VS_REAL.md`를 본다. 설계에 적힌 내용은 별도 표기가 없으면 구현·검증된 것이 아니다.
>
> 이 문서는 설계 합의 기록이다. 구현·검증된 내용은 각 단계의 합격 기준을 통과한 뒤 `DEVELOPMENT_LOG.md`에 별도로 적는다.

## 1. 합의한 결정

| 항목 | 결정 | 상태 |
|---|---|---|
| 목표 | LiDAR + 깊이 카메라를 함께 써서 LiDAR가 놓치는 장애물을 보완하며 자동 맵핑·주행 | [제안] |
| 주행 모드 | 제자리 회전이 필요하므로 **차동(Differential) 모드**. 실로봇에서 그 모드로 운용했을 것으로 보이나 코드로는 확인 불가 | [미확인] |
| 바닥 전선 | 넘을 수는 있으나 바퀴에 걸리면 이동에 문제 → **가급적 피하는 소프트 회피**. 인식은 보조, 걸림 감지·복구가 안전장치 | [제안] |
| 기록 | ROS logger + 구조화 이벤트(JSONL) + rosbag2, 실행(run)마다 폴더로 묶음 | [제안] |
| 변경 방식 | 한 번에 하나씩 바꾸고 같은 시나리오로 비교. 팀 공용 리포는 사전 공지 후에만 변경 | 원칙 |

## 2. 구조

```
센서   LiDAR(/scan) · IMU(/imu) · 바퀴(/odom) · Depth 카메라
 │
① 상태추정 (odom→base)   EKF 계열(F1b / F5 …), TF는 odom_tf_relay 한 곳만 발행
② 지도·위치 (map→odom)   지도 없음: SLAM Toolbox(mapping)
 │                       지도 있음: AMCL (또는 SLAM Toolbox localization)
③ Costmap                static + obstacle(LiDAR + Depth) + inflation (+ keepout/비용 필터)
④ Planner → Controller → (Collision Monitor) → stall gate → /cmd_vel
⑤ 임무    explore_lite(맵핑) · patrol_node(순찰) · guide action(안내)
⑥ 관측    ROS logger · JSONL 이벤트 · rosbag2 · run 디렉터리
```

## 3. 센서 융합

| 대상 | 2D LiDAR | Depth 카메라 | 방침 |
|---|---|---|---|
| 의자·책상 다리 | 스캔 높이에 걸릴 때만 | 높이 윈도우 안이면 가능성 높음 | 카메라로 보완 [제안] |
| 낮은 프레임·상판 | 높이에 따라 놓침 | 윈도우 상·하한에 의존 | 로봇 높이에 맞춰 윈도우 점검 |
| 유리 | 투과·반사로 불안정 | 투명체에서 값이 비거나 뒤쪽 벽으로 나올 수 있음 | **실물 실측 전 단정 불가** [미확인] |
| 바닥 전선 | 불가 | 현재 하한 때문에 걸러짐 | §6 |

- [확인] `integrated_nav2_params.yaml`의 depth 소스: `min_obstacle_height 0.02`, `max_obstacle_height 0.40`, marking/clearing 모두 True. 높이 2 cm 아래는 설계상 걸러진다.
- [확인] `depth_obstacle_filter_node`가 `/depth_obstacles/clearing_points`를 발행하지만 costmap 소스로는 쓰이지 않는다. 지나간 장애물이 남을 수 있어 마킹+클리어링 구성을 점검한다 [제안].
- [제안] SLAM 지도는 LiDAR로만 만들고, depth는 costmap(local/global)에만 넣는다. `explore_lite`는 costmap으로 frontier를 찾으므로 depth 장애물 근처 frontier는 막힌다 (코드 구조상 추론, 실행 확인 전).
- [확인] 시뮬레이션에 `glass_partition` 모델이 있다. 반투명 visual 때문에 gpu_lidar 광선이 유리를 통과하도록 만든 모델이다. LiDAR가 유리를 놓치는 상황은 재현되지만 **깊이 카메라가 유리를 어떻게 보는지는 시뮬레이션과 실물이 다를 수 있다.**

## 4. 위치추정

| 층 | 역할 | 지도 없음 | 지도 있음 |
|---|---|---|---|
| odom→base | 연속적이나 드리프트하는 자세 | EKF (바퀴+IMU 등) | **동일하게 필요** |
| map→odom | 지도에 맞춘 보정 | SLAM Toolbox (scan matching + loop closure) | AMCL 또는 SLAM Toolbox localization |

- [확인] 교재는 `robot_localization`을 위치 추정용으로 소개하고, AMCL 파라미터 예시(p.360~364)에 `robot_model_type: "nav2_amcl::DifferentialMotionModel"`이 있다.
- [제안] 기본 추정기는 **F1b(바퀴 vx,vy + IMU yaw)**. F5(ICP+IMU)는 같은 시나리오의 bag으로 오프라인 비교 후 승격. 근거: 특징 없는 복도에서 ICP가 퇴화하고 SLAM Toolbox의 scan matching도 같은 한계를 겪을 수 있어 바퀴+IMU가 안정화 역할을 한다 (합성 데이터 시험·코드 분석 기반, 실로봇 비교 전).
- [제안] 지도 있을 때: 교재 기준 AMCL을 기본, SLAM Toolbox posegraph localization을 비교 대상으로 둔다.
- 차동 모드 제자리 회전: 4륜이면 바퀴 슬립으로 바퀴 yaw 오차가 커질 수 있고 회전 중 스캔이 왜곡된다. F1b는 yaw를 IMU로 받아 슬립에 덜 민감하다 [미확인].

## 5. 주행 알고리즘

[확인] 교재 8.7.1: Limo가 Differential Mode일 때 DWB 사용. 교재 파라미터는 `wego_2d_nav/params/diff_navigation_params.yaml`(리포에 포함, 컨트롤러 `dwb_core::DWBLocalPlanner`)이며 `navigation_diff_launch.py`와 함께 차동 베이스라인으로 쓸 수 있다.
[확인] 현재 `integrated_nav2_params.yaml`은 RPP, `use_rotate_to_heading: false`, `allow_reversing: false`, `use_collision_detection: false`. 시뮬레이션 모델은 Ackermann 전용(`limo_ackermann_fortress.urdf`, `limo_eval/model.sdf`의 AckermannSteering 플러그인). **교재(차동+DWB)와 프로젝트 설정(Ackermann 가정+RPP)이 다르다.**

| 후보 | 장점 | 단점 |
|---|---|---|
| A. 차동 + DWB (교재) | 제자리 회전·Spin/BackUp 복구 정상, 교재 파라미터로 시작, 로컬 장애물 반응 | 튜닝 항목 많고 진동 가능 |
| B. 차동 + RPP(+rotate-to-heading) | 경로 추종이 부드럽고 속도 조절 | 로컬 회피 의존도 낮음 |
| C. Ackermann + RPP (현재) | 기존 설정 유지 | 제자리 회전 불가, 복구 행동 부적합 |

[제안] A를 베이스라인으로, B와 같은 시나리오에서 비교. 비교 지표는 `nav2_action_monitor`가 이미 기록하는 값(성공/실패, 소요 시간, `number_of_recoveries`, abort 수, 최소 장애물 거리, stall 이벤트)으로 한다.

확인 필요 설정 [미확인]:
- `min_y_velocity_threshold: 0.5` (교재 0.001). 오타 가능성.
- progress checker `required_movement_radius 0.12`, `movement_time_allowance 5.0` (교재 0.5 m, 10 s). stall gate가 로봇을 멈추면 abort → `explore_lite`가 해당 frontier를 blacklist (Humble의 `NavigateToPose` Result는 `Empty`라 abort 원인 구분 불가).
- `stall_detector`가 `abs(cmd_wz) > 0.05`도 "움직이려는 명령"으로 본다. 제자리 회전 중 오탐 여부는 시뮬레이션에서 확인.

## 6. 바닥 전선 (소프트 회피)

- 방어 3겹: 예방(인식해 비켜 감) / 완화(전선 근처 감속, 제자리 회전 회피) / 사후(걸림 감지 후 후진·회피 복구, `stall_detector` 기반).
- Nav2 `ObstacleLayer` 표시는 치명(lethal) 취급이라 "넘을 수 있으나 피하고 싶음"에 그대로 쓰기 어렵다. 비치명 비용 방식은 공식 문서로 확인 후 결정한다 [미확인].
- **선행 실측:** (a) LIMO가 전선을 넘을 때의 거동(걸림·슬립·odometry 튐), (b) 전선이 놓인 바닥의 depth+RGB를 0.3~2 m에서 bag으로 기록해 오프라인으로 분석(바닥 평면 잔차 등). 결과에 따라 정적 keepout / 바닥 평면 잔차 / RGB 검출 중 현실적인 것을 고른다.

## 7. 기록 설계

| 층 | 내용 | 용도 |
|---|---|---|
| ROS logger | 노드별 텍스트 로그 (기본 `~/.ros/log`). 교재 2.7: DEBUG/INFO/WARN/ERROR/FATAL | 어떤 노드가 어떤 에러로 죽었는지 |
| 구조화 이벤트 | `nav2_action_monitor`의 goal/feedback/status JSON, stall 상태, `/auto_mapping/depth_status` | 주행 타임라인·지표 집계 |
| rosbag2 | 센서·TF 원본 | 재현, 필터 오프라인 비교, 튜닝 |

### run_recorder.py (단계 0)

`capstone_ws/src/limo_capstone/scripts/run_recorder.py` — 기존 파일을 수정하지 않는 신규 스크립트. 주행 동작에 관여하지 않는다.

```bash
source /opt/ros/humble/setup.bash
python3 run_recorder.py --dry-run                      # 계획만 출력
python3 run_recorder.py --label corridor_f1b --snapshot-params \
    --launch-args "estimator:=f1b use_depth_camera:=true"
python3 run_recorder.py --label cable_test --extra /camera/color/image_raw   # 영상은 필요할 때만 지정
```

- 저장 위치: `<root>/<시각>_<label>/` (`meta.json`, `params/`, `bag/`, `exit.json`). `<root>`는 다음 순서로 정한다: ① `--root` 인자 → ② `$LIMO_RUNS_DIR` → ③ 프로젝트 저장소(`capstone_ws/src/limo_capstone/`)가 확인되면(`local_data/` 미생성 신규 clone 포함) `<저장소>/local_data/raw_results/limo_runs/` → ④ 그 외(실차 온보드 단독 워크스페이스 등 저장소를 찾지 못한 경우)는 기존 `~/limo_runs/`.
- 종료: Ctrl+C(SIGINT) 또는 SIGTERM → rosbag2에 SIGINT를 전달해 파일을 정상 마감.
- 한계: `exit.json`은 **기록기 프로세스**의 종료 사유다. launch/개별 노드의 종료 원인은 ROS logger 로그와 이후 단계(launch `OnProcessExit` 핸들러)에서 다룬다.
- 토픽 목록은 프로젝트 코드·launch에서 확인한 이름이다. Nav2 costmap 토픽은 표준 이름이므로 `ros2 topic list`로 확인한다.
- rosbag2 옵션(`--max-bag-size`, `--compression-*`)은 Humble에서 `ros2 bag record --help`로 확인한다.
- 시뮬레이션 재생: `ros2 bag play <bag> --clock` + `use_sim_time:=true`.

## 8. 단계표

| 단계 | 내용 | 합격 기준 |
|---|---|---|
| 0 | 관측 기반: run 디렉터리·bag·메타데이터 (동작 변경 없음) | 실제 환경에서 bag이 생성·재생되고 `exit.json`이 남음 |
| 1 | 안전: 모터 구동 시 게이트 강제, `ABORTED` 처리 보강 | 게이트 정지 후에도 frontier가 사라지지 않음 |
| 2 | 센서 융합: depth 마킹+클리어링, 윈도우 점검 | 의자 다리·책상 프레임 모형 회피 |
| 3 | 추정기 비교: F1b 기준 vs F5 (bag 오프라인) | 복도 드리프트·퇴화 지표 비교표 |
| 4 | 차동 모델 준비 + 컨트롤러 A vs B 비교 | 같은 시나리오 지표표 |
| 5 | 지도 저장 후 AMCL 주행, 순찰(`NavigateThroughPoses`/`FollowWaypoints` 클라이언트), 안내 액션 | 지도 기반 왕복 순찰 |
| 전선 | 선행 실측(§6) 후 방식 결정 | 걸림 없이 전선 구역 통과 또는 회피 |

## 9. 열린 항목

- 실로봇 주행 모드 확인: `ros2 interface show limo_msgs/msg/LIMOStatus`, `ros2 topic echo /limo_status --once` (교재 p.183).
- **차동 시뮬레이션 모델 없음 [확인]:** 시뮬레이션은 Ackermann 모델뿐이다. 차동 모델은 `model.sdf`/URDF의 조향 힌지를 고정하고 AckermannSteering 플러그인을 Gazebo의 diff-drive 시스템으로 바꾸는 작업이 필요하다 (플러그인 파일명은 설치 환경에서 확인). 별도 단계에서 진행한다.
- 자동 맵핑 구현이 둘이다: `explore_lite` 경로(이 저장소)와 `AutoMapping` 액션(`limo_workspace`). 동시 구동 시 `/navigate_to_pose` goal이 서로 선점할 수 있어 통합·역할 분담은 팀과 협의한다.
- 팀 공용 리포 변경은 올리기 전에 먼저 공유한다.

## 10. 근거 자료

- [WeGo] Limo_ROS2_교재.pdf: 2.7 ROS2 Logging(p.68~69), 5.4.1 Limo 구동 Differential/Ackermann(p.182~186), 8.7.1 DWA(p.398), 8.7.2 Limo Controller Server(p.400~403), 8.8.1 Costmap(p.404~405), AMCL(p.360~364, 절 번호 미확인), 10장 ex10_1/ex10_2(p.420~435)
- Navigation2 공식 (humble 브랜치) `nav2_msgs/action/*.action`
- 프로젝트 파일: `config/integrated_nav2_params.yaml`, `fusion_f5_improved.yaml`, `ekf_tuned.yaml`, `launch/*sim*.launch.py`, `models/glass_partition/model.sdf`, `wego_2d_nav/params/diff_navigation_params.yaml`
