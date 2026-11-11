# 2026-11-11 sim 설정 반영 검증 (processing_hz 20, 낮은 레일 임계값)

변경 파일(sim launch만): `launch/limo_integrated_auto_mapping_sim.launch.py`(Ackermann), `launch/limo_integrated_auto_mapping_sim_diff.launch.py`.
변경 전 값은 같은 폴더의 `S0_*`(기준), 1단계 `S1_*`, 2단계 `S2_*` 결과와 `../depth_analysis/`의 지연 측정에 있다. 패치: `patches/`.

## 1단계: depth 필터 `processing_hz` 10 → 20
| 300 s 탐색, status 나이(시작 5 s 제외) | 변경 전 | 변경 후 |
|---|---|---|
| Differential 프레임 버림 / p99 / max / 여유(0.38 s까지) | 33.2% / 0.208 / 0.252 / 0.128 | 0% / 0.151 / 0.153 / 0.227 |
| Ackermann | 26.7% / 0.206 / 0.211 / 0.169 | 0% / 0.150 / 0.153 / 0.227 |
| 필터 CPU, 시스템 CPU, RTF (Diff) | 58%, 39.6%, 0.856 | 59%, 43.0%, 0.891 |
| 필터 CPU, 시스템 CPU, RTF (Ack) | 59%, 43.6%, 0.848 | 60%, 43.5%, 0.874 |

Nav2 목표(0.5 m): Ackermann, Differential 모두 SUCCEEDED. `/cmd_vel` 발행자는 `stall_detector_node` 하나.
`sensor_timeout_sec`은 0.38 그대로, gate 코드는 수정 없음.

## 2단계: sim 전용 낮은 레일 임계값
필터 `min_height -0.02`, costmap depth `min_obstacle_height -0.03`(공용 YAML은 수정하지 않고 launch의 `RewrittenYaml` override).
override 결과를 원본 YAML과 비교하면 차이는 그 두 값뿐(Ackermann은 BT 경로 포함)이었다.

| bench (Ackermann = Differential 동일) | 변경 전 | 변경 후 |
|---|---|---|
| 필터 출력 최소 z / 총 점 수 | 0.025 / 2567 | -0.02 / 3638 |
| 레일 근처 필터 점 | 1238 | 1735 |
| local, global costmap의 레일 선 | `[100,99,99,99,99,99,100]` | `[100,100,100,100,100,100,100]` |
| 바닥 정면 통로(레일 bench) 필터 점 | 0 | 0 |
| 열린 바닥 bench: 통로 필터 점 / 통로 ≥99 셀 / 전체 ≥99 셀(local, global) | 0 / 23·391 / 824, 1529 | 0 / 23·391 / 824, 1529 (동일) |

## 안전 게이트
- `FAILSAFE` 시험(주행 중 depth 필터 종료): Ackermann, Differential 모두 FAILSAFE로 들어가고 이후 이동 0.0000 m, 해제하지 않음.
- 2단계 첫 Ackermann 목표 시험은 목표 전에 `DEPTH_STALE`(age 0.400 s)로 gate가 FAILSAFE에 들어가 움직이지 않았다(`nav/S2_ack_goal`). 필터가 처리한 stamp가 53.0 → 53.4로 0.4 s 건너뛴 정지였고, `processing_hz 20`으로도 막지 못한 사례다. 해제하지 않고 다시 실행해서(`S2_ack_goal_retry2`) SUCCEEDED를 확인했다.
- 시험 스크립트의 시작 직후 TF 경쟁(첫 시도 2건)은 스크립트 문제여서 `nav_test.py`를 고쳤다.
