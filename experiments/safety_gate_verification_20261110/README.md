# 2026-11-10 안전 게이트 검증 및 자동 맵핑 시뮬레이션 평가

> **정리본 상태 (2026-11-11).** **검증 수준:** 대부분 시뮬레이션 기록. 실차 관련 항목은 `docs/REAL_ROBOT_STATUS.md`를 기준으로 한다. 이 폴더의 일부 데이터(로그, 시계열, 지도 이미지)는 보존 브랜치에만 있다. 이 기록은 당시 설정과 환경에서 얻은 값이며 이후 정리된 파일 이름·값으로 다시 측정한 것이 아니다. 전체 목록: `experiments/README.md`.

Git commit/push 없음. 실차 모터 구동 없음. 이 폴더의 파일은 모두 새로 만든 것이며 기존 결과물을 덮어쓰지 않았다.

## 1. 구성

| 경로 | 내용 |
|---|---|
| `baseline/originals/` | 이번 작업 전 상태의 파일 복사본(`*.pre_session`)과 현재 신규 파일 사본 |
| `baseline/patches/mine_*.patch` | **이번 작업이 바꾼 부분만** 분리한 diff |
| `baseline/patches/ALL_tracked_changes_vs_HEAD_20261110.patch` | 추적 파일의 전체 미커밋 변경(기존 + 이번) 스냅샷 |
| `baseline/git_status_after_session_work.txt`, `sha256_after_session_work.txt` | 작업 후 git 상태와 핵심 파일 해시 |
| `scripts/` | gate 단독 시험, 전체 sim 시험, 자동 맵핑 모니터·분석 스크립트 |
| `logs/` | gate 시험 로그, sim 시험 로그 |
| `auto_mapping/<run>/` | 실행별 `summary.json`, `timeseries.csv`, `launch.log`, `final_map.*`, `map_quality.json` |
| `auto_mapping/comparison.md|json`, `final_maps_montage.png` | 7회 실행 비교표와 최종 지도 모음 |

## 2. 기준선: 기존 미커밋 변경과 이번 변경의 구분

작업 시작 시 이미 있던 미커밋 변경(내가 만든 것이 아님):

- 수정(M): `config/integrated_explore_params.yaml`, `config/integrated_nav2_params.yaml`(Ackermann 튜닝 10줄), `launch/limo_integrated_auto_mapping_real.launch.py`, `limo_capstone/lidar_scan_matcher_odom_node.py`, `limo_capstone/stall_detector_node.py`(기존 45줄 변경), `setup.py`
- 신규(??): `config/ekf_tuned.yaml`, `integrated_nav2_diff_params.yaml`, `integrated_nav2_lidar_only_params.yaml`, `ukf_tuned.yaml`, `launch/limo_integrated_auto_mapping_sim_diff.launch.py`, `limo_capstone/complementary_filter_node.py`, `models/limo_eval_diff/`, `rviz/`, `scripts/*`, `urdf/limo_diff_fortress.urdf`, `docs/`, `experiments/`, `notebook/`, `src/` 등

이번 세션이 바꾼 것(`baseline/patches/mine_*.patch`):

| 파일 | 변경 |
|---|---|
| `limo_capstone/stall_detector_node.py` | INITIALIZING, 주행 중 odom 감시, 복구 조건 강화 |
| `launch/limo_integrated_auto_mapping_sim.launch.py` | Nav2 명령 경로(smoother·behavior_server → `/cmd_vel_raw`), Ackermann 복구 BT 연결 |
| `launch/limo_integrated_auto_mapping_sim_diff.launch.py` | Nav2 명령 경로 |
| `launch/limo_integrated_auto_mapping_real.launch.py` | gate 1개, 명령 경로, Ackermann 복구 BT |
| `models/limo_eval_diff/model.sdf` | 바퀴 마찰 `mu=0.3, mu2=1.0, fdir1` |
| `config/integrated_nav2_params.yaml`, `integrated_nav2_lidar_only_params.yaml` | `default_nav_to_pose_bt_xml` 자리 키 3줄 |
| `config/navigate_to_pose_ackermann_no_reverse.xml` | 새 파일 |

이번 자동 맵핑 평가 단계에서는 `capstone_ws/src` 아래 파일을 **수정하지 않았다**(평가 전용 스크립트는 이 폴더에만 있다).

## 3. 평가 조건

- 실제 리포 launch를 `start_explore:=true`로 실행(Differential: `..._sim_diff.launch.py`, Ackermann: `..._sim.launch.py`). 월드, 센서, 구동 모델은 변경하지 않았다.
- SLAM Toolbox, Nav2, explore_lite, safety gate, F4 추정기, depth 필터를 함께 실행. depth_mode=required, sensor_timeout_sec=0.38 기본값 유지.
- 모니터(`scripts/auto_mapping_monitor.py`)는 읽기 전용이다. 속도를 발행하지 않고 `reset_latch`를 호출하지 않았다. 모든 실행에서 `operator_unlatch_count = 0`.
- 정지 조건: 시간 상한, explore 종료, 게이트가 FAILSAFE/STALL_LATCHED로 15 s 이상 유지(자동 해제하지 않고 기록 후 중단), 목표 없음, 지도 증가 없음, 로봇 정지.
- 지도 품질 기준: 월드 SDF의 외벽 4면 안쪽 면과 유리벽 선(가구는 기준에 넣지 않음). SLAM 지도 좌표계와 월드 좌표계는 시작 시점 GT와 TF로 정렬.
- 실행 횟수: Differential 3회, Ackermann 4회(+75 s 연기 시험 1회 `_smoke_diff`, 낡은 모니터, 비교에서 제외). 표본이 작아서 통계가 아니라 사례 관찰이다.

## 4. 결과 (7회 실행)

`auto_mapping/comparison.md` 참고. 지도 기준: 방 내부 103.84 m².

| 실행 | 종료 사유 | sim s | 이동 m | 방 내부 지도 | 벽 커버리지 | 게이트 |
|---|---|---|---|---|---|---|
| diff_run1 | explore 종료 | 224 | 11.5 | 91% | 88% | FAILSAFE 0 |
| diff_run2 | FAILSAFE 유지 | 136 | 12.1 | 91% | 88% | FAILSAFE 1 (DEPTH_STALE) |
| diff_run3 | explore 종료 | 225 | 13.3 | 92% | 91% | FAILSAFE 0 |
| ack_run1 | 로봇 정지 | 254 | 2.4 | 58% | 49% | 0 (NORMAL인 채 정지) |
| ack_run2 | FAILSAFE 유지 | 60 | 10.6 | 65% | 39% | FAILSAFE 1 (DEPTH_STALE) |
| ack_run3 | STALL 유지 | 26 | 1.5 | 47% | 49% | STALL_LATCHED 1 |
| ack_run4 | explore 종료 | 217 | 22.9 | 98% | 100% | 0 |

공통(전 실행): RTF 0.82–0.87(5 s 창 최저 0.79), 시스템 CPU 평균 39–44%(24코어, load 9–10), Gazebo 약 1.5코어, depth 필터·변환 약 1.1코어. `/cmd_vel` 발행자는 모든 실행에서 `stall_detector_node` 하나. 프로세스 사망 0. SLAM 위치 오차(정렬 후, GT 대비) 평균 0.02–0.11 m, 최대 0.13 m. 외벽 위치 오차 평균 0.02–0.06 m(표준편차 0.05–0.11 m).

## 5. 해석 주의

- 완주 4건(diff 1·3, ack 4)은 모두 explore가 "No frontiers found, stopping."으로 끝낸 것이다. Differential 3회는 모두 북동쪽 약 6 m² 구역(좁은 통로 쪽)을 알지 못한 채 끝났고, Ackermann 4회차만 이 구역을 지도에 넣었다.
- explore_lite가 약 2 s마다 목표를 갱신해서 Nav2가 이전 목표를 aborted로 보고한다. 액션 상태의 "aborted" 수는 대부분 이 교체(preemption)이며, Nav2 "SUCCEEDED"는 0회였다. 실제 실패는 로그의 `Failed to make progress`, `failed to generate a valid path` 등으로 센다.
- 모니터 v1(diff_run1, ack_run1)은 CPU 표본 추출이 콜백을 막아 스트림 간격이 부풀었다(diff_run1의 0.5 s 간격 1건은 모니터 쪽 지연으로 판단). v2(이후 실행)는 별도 스레드로 분리했다. 두 번째부터의 간격 최대는 0.36 s 이하였다.
- 게이트 종료 사유 라벨 `gate_failsafe_held`에는 STALL_LATCHED도 포함된다(ack_run3).
- `scripts/auto_mapping_monitor_v1_used_for_run1.py`는 diff_run1과 ack_run1에 쓴 원본이다.

## 6. 추가 조사 (읽기 전용 probe)

- `auto_mapping/_stuck_pose_probe/`: ack_run1에서 정지한 위치(월드 (-0.26, 1.36), 테이블·의자 사이 좁은 곳)에 로봇을 생성해서(launch 인자 `spawn_x/y/yaw`, 모델 변경 없음) 비용지도와 플래너를 조회했다. 그 위치의 global costmap 비용은 99(inscribed), local costmap은 80이었다. 로봇 반경 0.18 m, inflation 0.28 m. 이 상태에서 가까운 목표(앞·뒤·좌·우 1 m)에는 경로가 나왔다. 따라서 ack_run1의 지속적인 plan 실패(모든 목표에 대해 약 14회씩)는 "그 자리가 막혔다"는 단일 원인으로는 설명되지 않고, 로봇이 비용지도의 inscribed 영역에 들어간 상태에서 후진·제자리 회전이 불가능해 빠져나오지 못한 것과 겹쳤을 가능성이 높다. 이 연결은 직접 재현하지 못했으므로 추정이다.
- 이 probe sim에서도 유휴 상태 sim 73 s에 gate가 `lidar_stale=True, est_odom_stale=True`로 FAILSAFE에 들어갔다(같은 시점 SLAM 큐 가득 참 로그). 이 경우 FAILSAFE는 probe와 별개의 일시적 처리 지연으로 보이며 원인은 확정하지 못했다.

## 7. 2차 분석 (11-10 후반)

원인 분석과 제안은 [ANALYSIS_phase2_failure_causes.md](ANALYSIS_phase2_failure_causes.md)에 있다. 추가된 폴더:

| 경로 | 내용 |
|---|---|
| `depth_analysis/` | depth 단계별 지연 측정(기존 설정, `processing_hz=20` 변형) |
| `low_obstacle_analysis/` | 레일 bench 3종(기존, 필터만, 필터+costmap) |
| `stuck_analysis/` | ack_run1 정지 자세 재현, NavFn 대 Smac Dubins 플래너 비교 |
| `frontier_analysis/` | explore_lite 알고리즘 tracer 결과와 costmap 스냅샷 |
| `variants/` | 변형 launch·yaml(검증용, 리포 원본에 적용하지 않음) |
| `driver_outputs/` | 각 시험 실행 스크립트의 표준 출력 |

주의:
- 앞 절 5의 "Differential 3회 모두 북동쪽 미탐색" 서술은 표본 편향이었다(ANALYSIS 4절 참고).
- 이 단계에서도 `capstone_ws/src` 파일은 수정하지 않았다.
- tracer 개발 중 버그로 비어 있던 시도 폴더 3개(`diff_trace3_attempt*`의 첫 번째 시도)는 삭제하고 재실행했다. 그 외 기존 결과는 덮어쓰지 않았다.
- 시험 스크립트 안에서 `pkill -f` 패턴이 같은 셸 명령 문자열을 죽이는 일이 있어서 정리는 `scripts/cleanup_sim.sh`로 분리했다.

## 8. explore_relaxed 실험 (11-11)

보고: [explore_relaxed_eval/REPORT.md](explore_relaxed_eval/REPORT.md). 변형 패키지: `variants/explore_variant_ws/src/explore_relaxed/`(원본 `m-explore-ros2`는 수정하지 않음). 실험 launch 생성: `variants/make_explore_variant_launches.py`. 무효 처리한 실행은 `explore_relaxed_eval/_invalid_*`에 보존했다. 정리·격리 스크립트: `scripts/kill_stray_launches.sh`, `scripts/cleanup_sim.sh`.
