# 후속 제안 (제안만, 구현·적용하지 않음)

> **정리본 상태 (2026-11-11).** **검증 수준:** 후속 제안 목록. 구현·검증 전의 **제안**이다. 이 기록은 당시 설정과 환경에서 얻은 값이며 이후 정리된 파일 이름·값으로 다시 측정한 것이 아니다. 전체 목록: `experiments/README.md`.

## A. Frontier BFS 완화를 제3자 소스 수정 없이 시험하는 방법

### 배경
`explore_lite`의 frontier 탐색은 로봇에서 가장 가까운 비용 0 셀에서 시작해 **비용 0 셀로만** 확장한다(`frontier_search.cpp`의 `map_[nbr] <= map_[idx]`). 로봇이 inflation(비용 1–98) 안에 멈추면 시작 셀이 작은 섬이 되어 frontier가 0개로 보이고, explore는 `No frontiers found, stopping`으로 영구 종료한다. 갈라진 costmap 스냅샷 2개에서 BFS가 비용 99 미만 셀을 지나가게 하면 도달 셀이 5.3k → 26.2k, 유효 frontier가 7 → 9개(북동쪽 2개 포함)가 됐다(오프라인).

### 방법: 별도 변형 패키지
1. `src/m-explore-ros2/explore`는 그대로 두고, 새 작업 공간 `experiments/safety_gate_verification_20261110/variants/explore_variant_ws/src/explore_relaxed/`에 복사한다. 패키지명, 노드 실행 파일명(`explore_relaxed`)만 바꾸고 원본과 같은 인터페이스를 유지한다.
2. 변경은 한 곳이다. `frontier_search.cpp` BFS 확장 조건을 파라미터 `bfs_max_traverse_cost`(기본 0 = 원본 동작)로 일반화한다.
   - 기본 0: `map_[nbr] <= map_[idx]` (원본과 동일)
   - 98: `map_[nbr] < INSCRIBED(99)`이면 비용에 관계없이 이웃으로 확장(inflation 통과 허용)
   frontier 정의(미지 셀이 비용 0 이웃을 가짐)와 `min_frontier_size`는 그대로 둔다.
3. 별도 colcon 작업 공간으로 빌드해 `source install/setup.bash`를 기존 작업 공간 위에 겹쳐서(overlay) 사용한다. 기존 `capstone_ws/install`은 건드리지 않는다.
4. 변형 launch(`variants/`)는 sim launch를 복사해서 explore 노드만 `explore_relaxed`로 바꾼다.
5. 시험은 비교 가능하게 한 번에 한 변수만 바꾼다.
   - A/B: 같은 launch에서 `bfs_max_traverse_cost` 0 대 98
   - 기록: 이미 만든 `scripts/frontier_tracer.py`(원본 규칙 재현)를 그대로 쓰고, 종료 시점의 `reachable0/free0`, frontier 수, 북동쪽 SLAM 미지 셀을 비교
   - 규모: 종료 유형이 섞이므로 조건당 최소 6회(Differential 기준 조기 종료는 6회 중 2회였다)
6. 판정 기준
   - 조기 종료(`No frontiers found`인데 방 내부 미지 > 5%) 비율이 줄어드는지
   - 새로 생긴 frontier 목표에서 플래너 실패·blacklist가 늘지 않는지(frontier `middle`이 inflation 안에 있을 수 있어서 위험 요소)
   - safety gate 개입(STALL/FAILSAFE) 횟수가 늘지 않는지
   - 이동 거리, 총 탐색 시간이 크게 나빠지지 않는지
7. 위험·보류 사항
   - frontier 목표가 inflation 안에 놓이면 Nav2가 목표를 거부하거나 `Failed to make progress`가 늘 수 있다. 필요하면 후속으로 목표 셀을 비용 0 쪽으로 밀어주는 단계를 따로 검토한다.
   - 원본 `explore`와 변형을 한 launch에서 동시에 띄우지 않는다(목표 경쟁).

## B. Ackermann 교착: 당시 Costmap 스냅샷 기록 방법 (플래너 교체는 보류)

교착(ack_run1)은 4회 중 1회만 났고 재현이 안 되어서, **발생 시점의 상태를 반드시 남기는** 기록을 먼저 갖춘다.

1. 항상 켜 두는 기록(rosbag2, 순환 저장)
   ```bash
   ros2 bag record -o <새 폴더>/ack_stuck_bag --max-bag-duration 120 \
     /global_costmap/costmap /local_costmap/costmap /map /tf /tf_static \
     /plan /local_plan /cmd_vel_raw /cmd_vel /scan /depth_obstacles/points \
     /stall_detector/status_json /ground_truth/world_odom /navigate_to_pose/_action/status
   ```
   (`/global_costmap/costmap`은 `always_send_full_costmap: True`라 용량이 크므로 120 s 단위로 잘라 오래된 조각은 지운다.)
2. 트리거 기반 스냅샷(읽기 전용 노드, `frontier_tracer.py`의 스냅샷 로직을 일반화)
   - 트리거: GT 속도 0이 60 s 이상 + 활성 Nav2 목표 존재, 또는 플래너 실패 로그가 10 s에 5회 이상, 또는 gate가 STALL_LATCHED/FAILSAFE
   - 저장: 트리거 시점의 global·local costmap, SLAM `/map`, 로봇 자세(맵·월드), 최근 목표와 `/plan`, gate 상태, 직전 60 s의 `cmd_vel_raw` 요약
   - 별도로 NavFn이 실패한 시점에 **로봇 셀 주변 비용 격자**(±0.6 m)를 텍스트로 남긴다(이번 분석에서 쓴 ASCII 출력 방식)
3. 확정해야 할 질문(스냅샷으로 답할 것)
   - 플래너가 모든 목표에서 실패하는 시점의 로봇 셀과 이웃 셀 비용은 무엇인가(inscribed 고리에 갇혔는가)
   - 그 비용이 정적 SLAM 지도에서 왔는가, obstacle layer(LiDAR/depth/virtual bumper)의 marking에서 왔는가
   - `Failed to create a plan from potential when a legal potential was found`(NavFn)가 첫 실패에서 일관되게 나오는가
4. 안전·운영
   - 기록은 읽기 전용이다. 속도를 발행하지 않고 `reset_latch`를 호출하지 않는다.
   - 기존 실험 폴더에 쓰지 않고 새 폴더(`experiments/.../stuck_recordings_<날짜>/`)에 저장한다.
