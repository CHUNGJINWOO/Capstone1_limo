# explore_relaxed (EXPERIMENTAL)

`explore_lite`(`src/m-explore-ros2/explore`, **수정하지 않음**)를 복사해서 만든 실험용 변형 패키지다. 기본 파라미터에서는 원본과 같은 동작을 한다. 기존 자동 맵핑 launch의 탐색 노드를 대체하지 않는다.

패키지 이름 `explore_relaxed`, 실행 파일 `explore_relaxed`, 노드 이름 `explore_relaxed_node`, 토픽은 `explore_relaxed/{frontiers,status,resume,candidates}`, C++ 네임스페이스는 `explore_relaxed`, `frontier_exploration_relaxed`라서 원본과 동시에 실행해도 충돌하지 않는다.

## 원본과의 차이

| 항목 | 원본 explore_lite | explore_relaxed |
|---|---|---|
| frontier BFS 확장 | 비용이 같거나 낮은 이웃으로만(= 비용 0 셀로 이어진 영역) | 그대로. 단 `bfs_max_traverse_cost > 0`이면 비용 ≤ 그 값(OccupancyGrid 단위 1–98)인 셀도 지난다. 비용 99(inscribed) 이상, lethal, unknown은 절대 지나지 않는다(값은 98로 제한, 내부 비용 252 이하) |
| frontier 정의 | 미지 셀 + 비용 0인 4-이웃 | 동일 |
| 후보 선택 | 비용 순으로 첫 번째 비블랙리스트 후보 | 동일. `check_reachability=true`이면 후보마다 주행 가능성 검사를 통과한 첫 후보 |
| 주행 가능성 | 없음 | `reachability_mode`: `costmap`(알려진 비-inscribed 셀로 이어지는지), `nav2`(Nav2 플래너 `ComputePathToPose`, 읽기 전용), `both` |
| 실패 기록 | abort된 활성 목표만 blacklist | 위 + 검사 실패를 0.25 m 격자별 연속 횟수로 기록(`unreachable_blacklist_after`, 기본 3)해서 반복 선택을 막음. 이번 주기에 도달 가능한 후보가 없으면 `no_reachable_patience`(5) 주기까지 종료하지 않고 대기 |
| 진단 | 없음 | `explore_relaxed/candidates`(JSON): 발견·유지·검사·도달 가능 수, BFS 통계(최대 방문 비용), 후보 12개의 도달 가능 플래그 |
| `dry_run` | 없음 | true이면 발견·선택·발행만 하고 목표를 보내지 않으며 종료하지도 않는다 |
| 실행기 | 단일 스레드 | 2 스레드(기본 콜백 그룹은 직렬 유지, Nav2 계획 요청용 클라이언트만 reentrant 그룹) |

기본값(`bfs_max_traverse_cost: 0`, `check_reachability: false`, `dry_run: false`)은 원본과 같다. 검증: `test/equivalence_test.cpp`가 **원본 소스(수정하지 않은 `frontier_search.cpp`)와 변형을 같은 바이너리에 넣어서** 무작위 지도 60개와 실제 costmap 스냅샷에서 frontier 목록이 동일한지 비교한다.

## 빌드 (기존 작업 공간과 분리)
```bash
source /opt/ros/humble/setup.bash
source <repo>/capstone_ws/install/setup.bash          # 기존 작업 공간 (읽기만)
cd <이 변형 작업 공간>                                  # variants/explore_variant_ws
colcon build --packages-select explore_relaxed \
  --cmake-args -DORIGINAL_EXPLORE_DIR=<repo>/capstone_ws/src/m-explore-ros2/explore   # 동등성 시험 포함
source install/setup.bash                             # overlay
ros2 run explore_relaxed equivalence_test <costmap dump...>
```
