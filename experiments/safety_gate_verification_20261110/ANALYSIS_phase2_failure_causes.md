# 2차 분석: 자동 맵핑 실패 원인 (depth stale, 낮은 레일, Ackermann 교착, frontier 종료)

> **정리본 상태 (2026-11-11).** **검증 수준:** 시뮬레이션 실패 원인 분석. 이 기록은 당시 설정과 환경에서 얻은 값이며 이후 정리된 파일 이름·값으로 다시 측정한 것이 아니다. 전체 목록: `experiments/README.md`.

2026-11-10. `capstone_ws/src` 아래 파일은 이번 분석에서 **수정하지 않았다**(해시가 `baseline/sha256_after_session_work.txt`와 동일). 시험은 읽기 전용 probe와 `variants/`의 변형 파일로만 했다. 안전 게이트 우회, 자동 reset, FAILSAFE 비활성화 없음. 실차 구동 없음. commit/push 없음.

표기: **[확인]** 이번에 직접 측정·재현한 사실, **[추정]** 근거는 있으나 직접 증명하지 못한 것, **[미검증]** 시험하지 않은 것.

---

## 1. Depth stale (DEPTH_STALE)

### 확인
- 원인 조건: `depth_obstacle_filter_node.publish_status`(20 Hz)가 **마지막으로 처리한 cloud의 stamp 나이**(`now - last_stamp`)가 `data_timeout_sec`(sim 0.38 s)를 넘으면 `DEPTH_STALE`을 낸다. gate는 이 status를 보고 FAILSAFE로 들어간다. 이벤트 2건(diff_run2, ack_run2)의 로그가 모두 `age=0.400s`였다(status 주기 0.05 s의 배수).
- 단계별 지연(sim 시계 기준 나이, Differential 탐색 중 359 sim-s 측정, `depth_analysis/base_diff_explore/`):

| 단계 | p50 | p99 | max |
|---|---|---|---|
| 카메라 렌더 → 브리지 수신 (`/camera/depth_image`) | 0.006 | 0.028 | 0.067 |
| → depth→points 변환 후 (`/camera/depth/points`) | 0.032 | 0.062 | 0.077 |
| → 필터 출력 (`/depth_obstacles/points`) | 0.040 | 0.067 | 0.081 |
| status가 보는 나이(정상 상태) | 0.100 | 0.208 | 0.252 |

  즉 카메라→필터 출력 지연은 0.08 s 이하이고, 스트림 수신 간격(sim)도 최대 0.22 s였다(0.38 s 미만).
- **status 나이가 커지는 구조적 이유**: 필터는 cloud를 `time.monotonic()`(벽시계) 기준 `processing_hz=10`(0.1 s)로 걸러낸다. 카메라는 sim 10 Hz인데 RTF가 0.86이라 벽시계로는 약 8.6 Hz로 오고, 도착 간격이 0.1 s보다 짧은 프레임이 버려진다. 측정에서 **카메라 프레임 3580개 중 1188개(33.2%)가 필터에서 버려졌다**. 그래서 처리된 프레임 간격이 sim 0.14 s(중앙값)~0.22 s가 되고, status 나이 분포가 0.10 s대와 0.20–0.25 s대 두 봉우리로 갈린다. 한도 0.38 s까지 정상 상태의 여유는 **0.128 s**였다.
- 이벤트 시점 로그에는 모두 같은 시각대에 `slam_toolbox: Message Filter dropping ... queue is full`이 있고(diff_run2는 `Planner loop missed its desired rate`, `Behavior Tree tick rate exceeded`도), depth 쪽 스트림 간격 자체는 0.38 s를 넘지 않았다. 즉 depth만의 문제가 아니라 그래프 전체의 일시 지연이 겹쳤다.
- CPU와의 관계: 시스템 CPU 평균 40%(24코어, load 약 9), Gazebo 약 1.6코어. CPU 포화로 보이지 않는다. RTF 10 s 창 최저 0.76, 평균 0.86.
- 모니터 쪽 부풀림: 첫 두 실행(모니터 v1)의 0.5 s 간격은 모니터의 CPU 표본 추출이 콜백을 막은 것으로, v2에서는 분리했다.

### 단일 변수 비교 (`variants/sim_diff_depth_hz20.launch.py`: 필터 `processing_hz` 10 → 20, 그 외 동일)
| 항목 | 기존 | processing_hz=20 |
|---|---|---|
| 프레임 버림 | 33.2% | 0.2% |
| status 나이 p95 / p99 / max (시작 5 s 제외) | 0.202 / 0.208 / 0.252 | **0.104 / 0.108 / 0.152** |
| 한도 0.38 s까지 여유(max 기준) | 0.128 s | **0.228 s** |
| 필터 CPU / 시스템 CPU / RTF | 58% / 39.6% / 0.856 | 59% / 42.7% / 0.871 |

타임아웃, gate, watchdog은 바꾸지 않았다.

### 추정 / 미검증
- [추정] 이벤트는 "처리 프레임 간격(최대 0.2 s) + 일시 지연 0.15 s 이상"의 합이다. 변형은 이 지연 허용치를 0.13 s에서 0.23 s로 늘리지만, 이벤트가 사라진다는 것은 **증명하지 못했다**(이벤트 빈도는 매핑 7회, sim 약 19분에 2회였고, 이번 측정 두 번(sim 약 12분)에는 한 번도 발생하지 않았다).
- [미검증] 일시 지연의 직접 원인(sim 시계 점프, 프로세스 스케줄링 등)은 확정하지 못했다.
- [미검증] 변형으로 자동 맵핑 전체를 다시 돌려 이벤트 빈도를 비교하지 않았다.

---

## 2. 의자 하부 레일(낮은 장애물)

### 확인 (`low_obstacle_analysis/`, 로봇을 ack_run3 충돌 위치의 레일 정면 0.73 m에 생성, 이동 없음)
- sim의 `base_footprint`는 TF상 바닥에 있다고 가정되지만 실제 모델은 바닥에서 약 0.045 m 떠 있다. 필터 좌표계(`base_footprint`)에서 **바닥 높이는 z = -0.044**로 측정됐다(카메라 높이 TF 0.13 m).
- 따라서 `min_height 0.025`(필터)와 `min_obstacle_height 0.02`(costmap, depth 소스)는 실제로는 **바닥에서 약 0.07 m 이상**만 장애물로 본다. 레일(바닥 위 0.023–0.047 m)은 z = -0.022 ~ +0.002에 해당한다.
- 깊이 카메라는 레일을 **본다**: 필터 입력 cloud(`clearing_points`)의 z 히스토그램에 바닥 봉우리(-0.05 구간 2048점) 외에 z -0.02 ~ +0.01 구간에 466점이 있다(레일).
- 그 점들이 필터에서 제거된다: 기존 설정의 `/depth_obstacles/points` 최소 z는 0.025(레일 점 없음).
- 단계별 변형(`variants/`):

| 변형 | 변경 | 필터가 낸 레일 근처 점 | costmap의 레일 선(local/global) |
|---|---|---|---|
| V0 기존 | — | 최소 z 0.025, 레일 없음 | `[100, 99, 99, 99, 99, 99, 100]` (끝 100은 의자 다리, 가운데 99는 다리의 inflation) |
| V1 | 필터 `min_height` 0.025 → -0.02 | 최소 z -0.02, 레일 포함(+497점) | 변화 없음(가운데 99) |
| V2 | V1 + costmap depth `min_obstacle_height` 0.02 → -0.03 | 동일 | **`[100,100,100,100,100,100,100]`** (local·global 모두 레일이 lethal) |

- 2D LiDAR는 레일 위쪽 평면이라 다리만 보인다(스캔 반환 0.61–0.66 m, 레일 0.73 m 지점의 별도 반환 없음).
- 결론: [확인] 원인은 "장애물 감지 실패"이고 구체적으로는 sim의 base_footprint 높이 불일치 때문에 **필터 임계값과 costmap 임계값 두 곳이 모두** 레일을 걸러낸다. footprint/inflation 문제가 아니다(inflation은 다리에서 99를 만들 뿐이다). 필터만 고쳐서는 costmap에 반영되지 않는다.
- 이 불일치는 sim 전용이다. real launch는 `base_footprint_z` 기본값 0.0(바닥에 위치)이라서 같은 보정을 적용하면 안 된다. costmap yaml(`integrated_nav2_params.yaml`)은 real launch도 쓰므로 sim launch에서만 덮어써야 한다.

### 미검증
- V2로 자동 맵핑을 다시 돌려 ack_run3 같은 충돌이 사라지는지 확인하지 않았다(bench에서 costmap 반영까지만 확인).
- `RewrittenYaml`로 launch에서 costmap 값만 덮어쓰는 방법은 시험하지 않았다(시험한 것은 값을 바꾼 yaml 사본).
- 바닥 노이즈가 있는 실제 카메라에서 -0.02 임계값이 바닥을 장애물로 오검출하는지는 sim에서 검증할 수 없다.

---

## 3. Ackermann 좁은 공간 교착 (ack_run1)

### 확인
- 정지 위치: 월드 (-0.26, 1.36), 테이블 2 서쪽 가장자리(SLAM 지도 장애물까지 0.19 m). sim 15 s부터 254 s까지 움직이지 않았고 이후 목표에서 플래너 실패 166회(12개 목표 각각 약 14회), 복구 대기 35회로 풀리지 않았다.
- 이 위치를 재현(같은 자리에 생성, `stuck_analysis/`)하면 global costmap 비용이 **99(inscribed)**, local이 80이다(`robot_radius 0.18`, `inflation_radius 0.28`). 즉 비용지도상 로봇이 장애물 영역 안에 있다. 실제 차체 반폭은 약 0.1 m라서 costmap이 보수적이다.
- 정적 SLAM 지도만으로는 갇히지 않는다: 최종 지도에서 로봇 반경 0.18 m 여유를 둬도 로봇 셀에서 이어지는 자유 영역이 48.5 / 49.6 m²였다.
- 같은 위치에서 새로 시작한 플래너는 ack_run1에서 실패했던 목표를 포함해 **8개 목표 모두 성공**(경로 1.06–6.69 m). 따라서 ack_run1의 전면 실패는 시작 자세와 정적 지도만으로 설명되지 않는다.
- 운동학 반영: 전역 플래너는 `NavfnPlanner`(격자 최단경로, 방향과 회전 반경을 고려하지 않음), 컨트롤러는 RPP(후진·제자리 회전 없음). 모델의 조향 한계(`wheel_base 0.20`, `steering_limit 0.5236`)에서 최소 회전 반경은 약 0.35 m이다. 이 제약은 플래너에 반영되지 않는다.
- 변형 시험(`variants/sim_ack_smac_dubin.launch.py`, 플래너만 `SmacPlannerHybrid` DUBIN(전진 전용, 반경 0.35 m)로 교체): 같은 자세에서 **8개 목표 모두 `Starting point in lethal space`로 실패**했다. NavFn은 시작 셀을 비우고 계획하지만 Smac은 시작이 inscribed 영역이면 거부한다. 이 상태에서는 Smac으로 바꾸는 것만으로 해결되지 않는다.

### 추정 / 미검증
- [추정] Ackermann은 NavFn 경로를 RPP로 따라가다 곡률 제한 때문에 모서리를 inflation/inscribed 영역으로 깎아 들어가고, 후진(gate가 금지)과 제자리 회전(불가)이 없어서 빠져나오지 못한다. 직접 관측한 것은 "정지 자세가 inscribed 비용 영역"까지다.
- [미검증] 그 시점의 실제 비용지도 상태(왜 모든 목표에서 플래너가 실패했는지). 4회 중 1회만 발생하고 재현하지 못했다. 다음 단계는 같은 실행에서 정지 시점의 costmap을 기록하는 것이다.

---

## 4. Frontier 탐색 종료와 목표 교체

### 확인 (explore_lite 알고리즘을 실제 `/global_costmap/costmap`에 그대로 구현한 읽기 전용 tracer, `frontier_analysis/`)
- 정의: BFS가 로봇 위치에서 가장 가까운 **비용 0 셀**에서 시작해 **비용이 같거나 낮은 이웃으로만** 확장한다. 즉 **비용 0 셀로 이어진 영역만** 본다. frontier는 그 영역의 비용 0 셀과 맞닿은 미지(255) 셀이다. `min_frontier_size 0.35 m` 미만은 버린다.
- 이 tracer가 explore의 `No frontiers found, stopping`과 정확히 같은 시점에 frontier 0개를 냈다. Differential 6회를 추적한 결과 종료 유형이 세 가지였다.

| 유형 | 실행 | 종료 시점 상태 |
|---|---|---|
| A. 정상 완주 | diff_trace1, attempt1, attempt2 | costmap 미지 비율 0.0–0.9%, 북동쪽 SLAM 미지 51–66 / 2800셀(북동쪽 탐색됨) |
| B. 로봇의 비용 0 영역이 섬으로 갈라짐(조기 종료) | diff_trace2_b, attempt3 | costmap 미지 13–16%, 북동쪽 SLAM 미지 2057 / 2129 / 2800셀. 로봇 셀 비용 60–99(inflation 안)이고 로봇의 비용 0 연결 영역이 자유 공간 21.8k셀 중 5.0k, 또는 2셀로 줄었다. frontier 0개 |
| C. costmap 일시 초기화 | ack_run1, 시험용 probe 실행 | 종료가 전역 costmap clear 요청 0.21 s, 0.69 s 뒤. probe 실행에서는 종료 후 frontier 2개(크기 16.5 m와 2.7 m, 북동쪽)가 다시 계산됐다 |

- B형을 오프라인으로 평가: 갈라진 시점의 costmap 스냅샷에서 BFS가 **비용 < 99인 셀까지 지나가도록** 바꾸면 도달 가능 셀이 5.3k → 26.2k(전체)로 늘고 유효 frontier가 7 → 9개(북동쪽 2개 포함)가 된다. 비용 < 50으로만 완화하면 효과가 없다.
- 기존 보고의 정정: 앞 보고서의 "Differential 3회는 모두 북동쪽을 탐색하지 못했다"는 표본 편향이었다. 이후 추적한 Differential 6회 중 3회는 북동쪽을 탐색했다. 조기 종료(B형)는 추적한 6회 중 2회에서 나왔다. 초기 3회 중 diff_run1과 diff_run3도 북동쪽을 탐색하지 못한 채 끝났지만 tracer가 없어서 유형은 분류하지 못했다.
- `min_frontier_size`는 원인이 아니다: B형에서 frontier 총수 자체가 0(또는 0.1–0.2 m짜리 조각만 남음)이다. 북동쪽 통로 입구는 폭이 좁아 비용 0 띠가 얇은 것이 영향을 줄 수 있으나 직접 확인하지는 못했다.
- 목표 교체: explore 타이머 2 s(`planner_frequency 0.5`)마다 목표를 다시 고른다. 목표는 frontier에서 **로봇에 가장 가까운 셀**(`middle`)이고, 동일 판정(`same_point`)은 1 cm 이내라서 로봇이 움직이거나 지도가 갱신되면 거의 매번 새 목표가 된다. 로그에서 목표 간격 중앙값 2.04 s, 연속 목표 사이 거리 중앙값 Differential 1.65–5.0 m, Ackermann 0.6–1.2 m(0.5 m 미만으로 바뀐 비율은 6–50%)였다. 이 때문에 Nav2 `SUCCEEDED`가 0회였고, "aborted" 수는 교체가 대부분이다. 이 교체와 실패의 직접 인과는 찾지 못했다.
- Humble에는 `NavigateToPose` 결과에 `error_code`가 없어서 explore는 활성 목표가 abort되면 무조건 blacklist한다(교체로 abort된 이전 목표는 id 검사로 걸러진다). 이번 9회의 종료에서 "All frontiers traversed/tried out"은 한 번도 없었다.

### 추정 / 미검증
- [추정] B형의 직접 원인은 "로봇이 inflation 안에서 멈추고, 그 근처 비용 0 셀이 작은 섬이다"이다. 로봇이 문틀 부근에서 멈췄음은 좌표로 확인했지만(월드 약 (-2.1,-1.8)) 비용 0 띠가 끊긴 원인(문틀 inflation, 일시 marking 등)은 가리지 못했다.
- [미검증] BFS 완화(또는 시작 셀 선택 변경)를 실제 explore에 적용한 실행. explore는 `src/m-explore-ros2`(제3자 코드)라서 수정하지 않았다.
- [미검증] 목표 교체를 줄였을 때(`planner_frequency` 등)의 효과.

---

## 5. 제안 (적용하지 않음). 한 번에 하나씩 적용·검증할 것을 권장

| 순위 | 내용 | 근거 수준 | 비고 |
|---|---|---|---|
| 1 | sim launch 두 개의 depth 필터에 `processing_hz: 20.0` | 단일 변수 비교 완료(여유 0.128 → 0.228 s) | sim 전용. real launch는 이미 `processing_hz 4`, `data_timeout 0.75` |
| 2 | sim launch에서 depth 필터 `min_height -0.02`와 costmap depth `min_obstacle_height -0.03`을 **함께**(둘 중 하나만으로는 costmap에 안 들어감) | bench에서 레일이 local·global costmap의 lethal이 되는 것까지 확인 | sim 전용. 자동 맵핑 재시험 전 |
| 3 | explore의 BFS 완화(비용 < 99 통과) 또는 로봇이 inflation 안일 때의 시작 셀 선택 변경 | 갈라진 스냅샷 2개에서 오프라인 확인 | 제3자 코드 수정이므로 팀 결정 필요 |
| 4 | Ackermann 교착: 정지 시점의 costmap 기록 → 원인 확정 후 inflation 확대 등 | 미확정 | Smac 교체만으로는 안 됨(시작이 inscribed면 거부) |
| 5 | 목표 교체: `planner_frequency` 하향 등 | 인과 미확인 | 우선순위 낮음 |

시험 스크립트, 변형 파일, 원시 결과는 이 폴더의 `scripts/`, `variants/`, `depth_analysis/`, `low_obstacle_analysis/`, `stuck_analysis/`, `frontier_analysis/`에 있다.
