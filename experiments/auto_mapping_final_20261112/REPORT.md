# Gazebo 자동 맵핑 완주 시험 (2026-11-12)

목표: 기존 차동 구동 시뮬레이션 launch(`limo_integrated_auto_mapping_sim_diff.launch.py`)로 LIMO가 수동 조작 없이 실내를 탐색하고 SLAM 지도를 만든 뒤 자동 저장하는 것. 기존 launch·월드·SLAM·Nav2·explore_lite·Safety Gate는 수정하지 않았다(기준선 보존). 실차 LIMO는 사용하지 않았다. Git commit/push 없음.

## 결과 요약 (6회 실행)

| 실행 | launch | 종료 사유 | sim s | 실제 이동(Gazebo GT) | 방 내부 커버리지 (시작 자세 정렬 / 벽 맞춤 정렬) | 벽 잔차 p95 | 남은 미탐색 구역(≥0.3 m²) | 게이트 FAILSAFE |
|---|---|---|---|---|---|---|---|---|
| run1 | 기준선 | explore 종료 | 211 | 16.1 m | 94.1% / **99.0%** | 9.7 cm | 벽 두께 띠 2개(0.49, 0.39 m²) | 0 |
| **run2** | 기준선 | explore 종료 | 295 | **26.6 m** | 94.4% / **99.3%** | 4.8 cm | 벽 두께 띠 1개(0.37 m²) | 0 |
| run3 | 기준선 | **게이트 FAILSAFE 유지(실패)** | 31 | 0.05 m | 58.4% / 59.0% | 5.0 cm | 큰 구역 3곳 | 1 |
| v1 | 자동 복구 변형 | explore 종료 | 200 | 8.3 m | 79.5% / 82.1% | 5.0 cm | 8.58, 7.08 m² | 0 |
| v2 | 자동 복구 변형 | explore 종료 | 298 | 17.9 m | 92.8% / **98.0%** | 18 cm | 벽 두께 띠 2개 | 0 |
| v3 | 자동 복구 변형 | explore 종료 | 211 | 9.1 m | 81.2% / 85.1% | 7.0 cm | 6.67, 5.31 m² | 0 |

- 완주(커버리지 98% 이상) 3회: run1, run2, v2. 조기 종료 2회(v1, v3). 실패 1회(run3). 표본이 작아서 성공률이 아니라 사례 관찰이다.
- 주 결과물은 **run2**(기준선 launch 그대로, 가장 길게 이동, 99.3%).
- 모든 실행에서 `/cmd_vel` 발행자는 `stall_detector_node` 하나였고 게이트를 우회하거나 `reset_latch`를 호출한 적이 없다(`operator_unlatch_count` 0).
- Nav2 goal 상태의 "aborted"는 대부분 explore_lite가 약 2 s마다 목표를 갱신해서 생기는 교체(preemption)이며, "succeeded"가 0으로 보이는 것도 같은 이유다(`safety_gate_verification_20261110/README.md` §5 참고).

## 결과물 (실행별 `runs/<tag>/`)

| 파일 | 내용 |
|---|---|
| `limo_sim_map.pgm`, `limo_sim_map.yaml` | **자동 저장된 지도**(explore_lite가 `EXPLORATION_COMPLETE`를 발행한 뒤 3 s 후 `auto_map_saver.py`가 map_saver_cli로 저장). 해상도 0.05 m |
| `map.png` | 저장된 지도 4배 확대 이미지 |
| `path_overlay.png` | 지도 위에 이동 경로 표시: 빨강 = SLAM 좌표계 경로(`/tf`), 파랑 = Gazebo 실제 경로(v1~v3, run3만 GT 기록), 초록 = 시작, 파랑 점/주황 점 = 끝 |
| `path_map_frame.csv`, `gt_world_path.csv` | 경로 원자료(SLAM 좌표계 / Gazebo 월드 GT, GT는 run3·v1~v3 bag에서 추출) |
| `coverage.json` | 커버리지 계산 결과와 정의, 미탐색 구역 목록, 벽 정렬 보정값 |
| `summary.json`, `timeseries.csv` | 모니터 기록(이동, 지도 증가, 게이트 상태, 목표 처리, RTF, CPU) |
| `saved.json`, `launch.log` | 자동 저장 기록(저장 이유 `exploration_complete`), 전체 launch 로그 |
| `bag/` | 짧은 rosbag(37~65 MB, 약 230~330 sim s): `/clock /tf /tf_static /scan /odom_nav2 /map /cmd_vel_raw /cmd_vel /explore/status /stall_detector/status_json`(v1~v3, run3는 `/ground_truth/world_odom` 추가). 깊이 영상은 용량 때문에 제외 |

### 저장소에 포함한 범위와 제외한 것 (`docs/PUBLICATION_POLICY.md` 기준)

- 포함: 스크립트, 변형 launch, 이 보고서, 실행별 지도(`.pgm/.yaml`)와 이미지, `coverage.json`, `summary.json`, `timeseries.csv`, 자동 저장 기록(`saved.json`).
- 제외(로컬에만 있음): 실행별 `bag/`(37~65 MB), `launch.log`, 지도 중간 스냅샷, 경로 원자료 `path_map_frame.csv`·`gt_world_path.csv`(20 Hz 시계열). 경로는 `path_overlay.png`로 확인하고, 원자료는 재현 명령으로 bag에서 다시 만든다(bag이 필요하므로 같은 실행을 다시 하면 된다).
- 기록을 올릴 때 `saved.json`과 `run3/saver.log`의 개인 기기 절대 경로를 `<repo>`로 바꿨다(그 외 내용과 측정값은 그대로).

## 커버리지 정의

- 기준: `lounge_evaluation_world.sdf`의 바깥 벽 4면 안쪽 면(x ∈ [-5.9, 5.9], y ∈ [-4.4, 4.4], 103.84 m²). 가구는 빼지 않는다(가구 밑 셀은 미탐색으로 센다).
- 커버리지 = 방 내부에서 SLAM 셀이 free(<25) 또는 occupied(≥65)인 면적 ÷ 103.84 m².
- **정렬 두 가지를 모두 적는다.** SLAM 지도 좌표를 월드 좌표에 맞추는 방법에 따라 값이 달라진다.
  - 시작 자세 정렬(기존 `analyze_map_quality.py` 방식): 시작 순간의 GT와 SLAM 자세 한 쌍으로 맞춘다. 세 번의 기준선·변형 완주 실행 모두 회전이 6.6~6.9° 같은 방향으로 어긋났고, 그래서 벽 잔차(평균 약 20 cm)가 크게 나오고 커버리지가 낮게(94%) 계산된다.
  - 벽 맞춤 정렬: 바깥 벽 4면에 지도의 점유 셀이 가장 잘 맞도록 회전·이동을 추가로 찾는다. 잔차가 줄어든다(run1 평균 2.9 cm·p95 9.7 cm, run2 평균 2.2 cm·p95 4.8 cm, v2 평균 7.8 cm·p95 18 cm). run1·run2의 지도 기하는 벽 기준으로 정확하고, v2는 상대적으로 덜 정확하다.
  - 같은 지도를 두 방법으로 계산한 것이라서, 낮은 값은 보수적인 하한, 높은 값이 지도 기하와 맞는 추정치다. 모니터의 `slam_err_xy_m`(run1·run2에서 약 0.6~0.7 m)도 같은 시작 자세 정렬을 쓰므로 정렬 오차가 포함된 값이다. 벽 맞춤 정렬로 보정한 뒤의 위치 오차는 따로 계산하지 않았다.
- 미탐색 구역은 벽 맞춤 정렬 기준으로 센 연결 성분(0.3 m² 이상)이다. 완주 실행의 남은 것은 모두 벽 두께 띠(예: 유리벽 y ≈ -1.5의 0.1 m 폭)라서 실제 탐색 누락이 아니다.

## 실패와 원인

- **run3 (게이트 FAILSAFE로 중단):** 기동 15 s 후 `[DEPTH SENSOR STALE] PointCloud age=0.400s exceeds 0.38s`가 한 번 나서 게이트가 `FAILSAFE_SENSOR_TIMEOUT`이 됐다(깊이는 바로 다음 상태 메시지에서 정상). 시뮬레이션 launch는 게이트에 `require_operator_unlatch: True`를 넘겨서 운영자 `reset_latch` 없이는 풀리지 않는다. 자동 맵핑은 운영자가 없으므로 모니터가 15 s 뒤 중단했다. 이전 `diff_run2`(11/10)와 같은 유형이다.
- **최소 변경:** `launch/sim_diff_autorecover.launch.py`(기준선 launch의 사본, 차이는 `require_operator_unlatch` True → False 한 줄). 실차 launch가 이미 쓰는 정책이다. 게이트는 그대로 동작해 모든 입력이 신선하고 명령이 거의 0일 때만 출력을 푼다. 기존 게이트를 끄거나 명령 경로를 우회하지 않았다.
- **검증 한계:** 변형으로 3회 연속 완주했지만 그 3회에는 FAILSAFE가 한 번도 발생하지 않아서(`failsafe_events` 0), 시뮬레이션 안에서 FAILSAFE가 자동 복구되는 장면은 확인하지 못했다. 복구 논리는 가짜 센서 시험(`gate_harness.py`)으로만 확인했다.
- **조기 종료(v1, v3):** explore_lite가 `No frontiers found, stopping.`으로 끝났는데 남동쪽(약 7~9 m²)·북동쪽(약 7 m²)이 미탐색이었다. 이전 분석(`safety_gate_verification_20261110/explore_relaxed_eval/REPORT.md`)은 통로가 좁아 inflation이 frontier와 맞닿은 비용 0 셀을 없애는 경우로 봤고 `explore_relaxed`로 완전히 해결되지 않았다. 이번 실행에서 원인을 다시 증명하지는 않았다. 같은 launch인데도 완주한 실행(run1, run2, v2)에서는 이 구역이 탐색됐고, 조기 종료한 실행에서는 남았다(탐색 순서가 실행마다 달라지는 것으로 보이나 확인하지 않음).

## 재현 명령

전제: ROS 2 Humble, Ignition Fortress, 저장소 `capstone_ws`가 빌드돼 있을 것(`capstone_ws/install`). 아래는 저장소 루트에서 실행한다.

```bash
cd experiments/auto_mapping_final_20261112
# 기준선 launch 그대로(권장 재현). 태그는 runs/<tag>에 저장되며 덮어쓰지 않는다.
bash scripts/run_final.sh myrun 1200
# 게이트 자동 복구 정책 변형
LAUNCH_FILE=$PWD/launch/sim_diff_autorecover.launch.py FAILSAFE_HOLD=120 bash scripts/run_final.sh myrun_v 1200
# 후처리(ROS 환경 필요): bag에서 경로 추출, 지도 이미지·커버리지 계산
source /opt/ros/humble/setup.bash
python3 scripts/path_from_bag.py runs/myrun
python3 scripts/post.py runs/myrun
```

- `ROS_DOMAIN_ID`는 기본 121, `ROS_LOCALHOST_ONLY=1`이다. 스크립트는 자신이 시작한 프로세스 그룹만 종료한다.
- 실행 시간: sim 200~300 s(RTF 약 0.85, 벽시계 4~6분).

## 실차 적용 전에 남은 차이

- 시뮬레이션 LiDAR는 빔 수가 고정이지만 실차 LiDAR는 회전마다 242~245개로 달라서 정규화 노드가 필요하다(실차 launch의 `scan_normalizer` 옵션, 모터 잠금에서 효과 확인).
- 시뮬레이션의 회전 슬립과 휠 오도메트리 오차는 실차와 다르다. 실차 회전 오차는 미측정이다.
- 센서 지연: 시뮬레이션은 깊이 age가 한계(0.38 s)에 가깝게 튀는 일이 드물게 있다. 실차의 깊이 경로 여유는 약 0.13 s로 측정된 적이 있다.
- 게이트 정책: 실차 launch는 `require_operator_unlatch=False`이지만 실차에서 센서 FAILSAFE 자동 복구의 안전성은 검증되지 않았다.
- 실차에는 Gazebo GT가 없어서 커버리지는 지도와 사람이 아는 평면도로만 평가할 수 있다.
- 이 결과는 월드 하나(lounge)의 6회 실행이다. 다른 환경, 장애물 위치, 긴 주행, 모터 구동 중 안전은 확인하지 않았다.
