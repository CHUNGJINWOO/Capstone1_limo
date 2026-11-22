# Gate 일시 중단(E) 분석 · 정지 중 출력의 안전 영향 · Gazebo 웹 제어 통합 (2026-11-21, Claude)

범위:
- 모든 결과는 **격리 시뮬레이션**이다(ROS_LOCALHOST_ONLY=1, 사설 도메인, Ignition `IGN_PARTITION`, IGN_IP=127.0.0.1).
- 실차·LIMO SSH·운영 18190은 사용하지 않았다. 실차 모터 잠금과 실차 Mapping START 차단은 그대로다.
- Gate는 **candidate r12 원본**을 그대로 실행했다. r11/r12/패치 06 원본과 다른 담당자의 `capstone_ws` Gate 미커밋 변경은 수정하지 않았다.
- 원자료: `local_data/raw_results/gate_stall_gazebo_claude_20261121/`(이하 `RAW/`). 변경 전 원본 사본: `local_data/raw_results/refactor35_claude_20261121/pre_change_originals/`.
- Phase 3.5 구조 감사와 리팩토링은 [REFACTOR_PHASE35_20261121.md](REFACTOR_PHASE35_20261121.md)에 기록했다.

## 1. E: 첫 지연 지점과 원인

**결론: 원인을 특정하지 못했다(미해결).** 추적 계측을 붙인 실행에서는 정지가 한 번도 재현되지 않았다. 계측 없는 실행에서는 오늘도 1회 재현되었다.

| 실행 | 계측 | 트리거 | Gate 제어 루프 간격 ≥100 ms | 상태 발행 간격 ≥250 ms | 센서 FAILSAFE |
|---|---|---|---|---|---|
| 리팩토링 회귀 Phase 2(`refactor35…/e2e_after_r1r2r3/phase2`) | 없음 | f04 MAPPING 시작(SLAM 기동) 2 s 후 | (미계측) | (미계측) | **1회**: `scan_stale=True, lidar_stale=True`, 13:53:37 |
| `RAW/repro_e1` | 추적+프로브+외부 증인 | Agent 재시작 6, SLAM 기동 6(LOAD 0: 하네스가 미검증 지도로 409) | 0 / 10,579 tick | 0 | 0 |
| `RAW/repro_e2` | 〃 | Agent 재시작 10, SLAM 기동 10, LOAD 1 | 0 / 19,816 | 0 | 0 |
| `RAW/phase2_traced_1` | 추적+프로브+증인 | Phase 2 전체 16개(항법 준비 5개는 지도 범위 `UNKNOWN_AREA`로 조기 실패. Gate와 무관) | 0 / 5,558 | 0 | 0 |
| `RAW/phase2_traced_2`, `_3` | 〃 | MAPPING 시작·일시정지·저장·MAPPING 중 Agent 재시작 등 7개, 각 8/8 PASS | 0 / 4,407, 0 / 4,456 | 0 | 0 |
| `RAW/phase2_traced_noprobe_4`, `_5` | 추적+증인, **프로브 끔** | 〃, 각 8/8 PASS | 0 / 4,406, 0 / 4,457 | 0 | 0 |

추적 실행에서 측정한 Gate 내부 값:
- 제어 루프 간격: p50 20.0 ms, p99 20.5 ms, 최대 27.7 ms.
- 가장 긴 콜백: `control_loop` 최대 3.3 ms, `/cmd_vel_raw` 1.7 ms, `/scan` 0.7 ms.
- `/scan` 수신 지연(헤더 stamp→Gate 콜백): p50 1.3 ms, 최대 9 ms.
- 외부 증인 프로세스: scan 최대 간격 106 ms.

판단 근거와 남은 가설:
- 이전 기록(PHASE3 문서 §3 E)에서 외부 기록기는 `/scan`을 계속 받았고, 같은 순간 Gate 자신의 상태 발행만 0.35~0.4 s 멈췄다. 따라서 **발행자나 DDS 전달 경로가 아니라 Gate 프로세스 안쪽**(executor가 돌지 못함, GIL 점유, OS가 프로세스를 실행하지 않음)이다. 이것이 지금까지의 최선의 증거이며, 이번 실행으로 더 좁히지는 못했다.
- 이번에 만든 계측은 세 경우를 구분하도록 설계했다(`gate_trace.py` 문서 주석):
  - 모든 스레드가 늦고 `run_delay`가 증가 → OS 스케줄링·프로세스 정지
  - 프로브는 늦지만 `run_delay`는 평탄 → GIL을 잡은 긴 콜백 또는 C 호출
  - 콜백은 제시간인데 scan이 늦게 몰려 옴 → DDS 전달 지연
  - ROS 시각(메시지 stamp)과 단조 시계(`CLOCK_MONOTONIC`)는 분리 기록한다. 증인 프로세스와 같은 시계를 쓴다.
- **계측이 현상을 가리는 가능성**: 프로브 스레드가 5 ms마다 깨어나 프로세스를 계속 스케줄되게 만들 수 있다. 그래서 프로브를 끈 실행(위 표 마지막 행)을 따로 했다. 프로브를 꺼도 2회 모두 재현되지 않았다. 따라서 "프로브가 가린다"는 가설은 지지되지 않았다. 다만 콜백 래퍼와 flusher 스레드는 남아 있었으므로 계측 영향을 완전히 배제하지는 못한다.
- 평균 CPU가 낮다는 이유로 E를 배제하지 않는다. 재현 빈도는 실행당 r11 2/8, r12 1/8이다(이전 기록). 트리거당 확률이 낮아 수십 회 반복으로도 놓칠 수 있다.
- **다음 조치(제안)**: 계측 부담이 거의 없는 방식으로 장시간 반복한다.
  - Gate 외부에서 `/proc/<pid>/task/*/schedstat`·`status`(상태 R/S/D)를 20 ms마다 읽는 별도 프로세스로 기록한다. Gate 프로세스 안에는 스레드를 넣지 않는다.
  - 증인은 그대로 둔다.
  - 정지가 포착되면 그 구간의 스레드 상태를 본다. `D`(I/O 대기)라면 로그 쓰기 등 블로킹 호출이고, `R`인데 실행되지 않았다면 CPU 경쟁이며, `S`라면 executor 대기다.

## 2. 재현 여부

- **계측 없는 E2E에서는 재현됨**: 오늘 1회(Phase 2 f04, MAPPING 시작 2 s 후). 그 결과 FAULT→MANUAL 전이가 `GATE_NOT_NORMAL`로 실패했다.
- **계측 실행에서는 재현 안 됨**: 위 표의 계측 실행 7회 모두 0회. 트리거:
  - Agent 재시작 21회: repro 16 + E2E f04 5.
  - 단독 SLAM 기동 16회(repro). 여기에 E2E MAPPING 시작(실행당 4~5회 × 5회)이 더해진다.
  - repro LOAD 1회. phase2_traced_1의 항법 LOAD는 초기 위치 단계에서 실패했다.

## 3. Gate 정지 중 출력의 안전 영향

### 3.1 주입 시험(`RAW/stall_injection_1`, 24 case)

조건과 방법:
- r12 원본을 쓰고, 입력 배타 = `fleet_motion_watchdog`이다.
- 가짜 로봇(`gate_r11_input/fake_sensors.py`)은 하부 드라이버를 두 모델로 모사한다.
  - hold: 마지막 명령을 계속 유지한다(Ignition DiffDrive와 같은 모델).
  - timeout: 마지막 Gate 출력 후 0.3 s에 정지한다.
- 0.08 m/s로 주행 중 D초 정지를 넣는다. 정지 방법은 둘이다.
  - freeze: SIGSTOP. Gate만, 또는 Gate와 Watchdog 모사 writer를 함께 멈춘다.
  - delay: 살아 있는 Gate의 다음 제어 timer 콜백을 D초 지연시킨다.

| D | 정지 직전 출력 | 정지 중 출력 수 | 재개 직후 | 정지 중 이동(hold) | 정지 중 이동(timeout 0.3 s) |
|---|---|---|---|---|---|
| 0.2 s | 8/8 case에서 0.08(0 아님) | 0(1 case는 정지 직전 발행분이 늦게 도착한 1건) | Gate는 NORMAL 유지, **0.08 명령을 곧바로 다시 출력** | 0.016 m(= 0.08×0.2) | 0.016 m |
| 0.35 s | 0.08 | 0 | 7/8 case NORMAL 유지, 0 출력 1~2개 후 0.08 재출력. freeze_gate_timeout 1건은 FAILSAFE | 0.024~0.028 m | 0.024 m |
| 0.5 s | 0.08 | 0 | 8/8 case `FAILSAFE_SENSOR_TIMEOUT` 래치, 이후 0. freeze_both_timeout 1건은 재개 직후 0.08 한 개가 관측됨(정지 전 발행분 도착인지 재개 후 발행분인지는 이 기록으로 구분 불가) | 0.04 m | 0.024 m |

해석:
1. Gate가 멈춘 동안에는 **이동 명령도 정지 명령도 나가지 않는다.** 정지 직전 마지막 출력은 0이 아니었다. 소프트웨어 Gate는 자기가 멈춘 동안 정지를 보장할 수 없다.
2. 그 동안의 실제 움직임은 **하부가 결정한다.**
   - hold 드라이버: 정지 시간 내내 마지막 속도로 움직인다.
   - timeout 드라이버: timeout 이후 멈춘다.
   - Watchdog를 함께 멈춰도 결과는 같았다(Watchdog도 Gate 위쪽이라 Gate 아래로는 아무것도 보내지 못한다).
3. **센서 timeout(0.38 s)보다 짧은 정지는 아무 흔적도 남기지 않는다.** Gate는 NORMAL인 채로 하던 명령을 이어간다. 0.38 s를 넘으면 센서 FAILSAFE로 래치된다. 하지만 정지 중 이동은 이미 일어난 뒤다.
4. 상한: 정지 중 최대 이동은 `v × min(D, 하부 timeout)`이다. 하부가 hold라면 `v × D`다. Gate 입력 상한 0.20 m/s와 0.5 s 정지를 넣으면 hold에서 0.10 m다.

### 3.2 Gazebo 실측(`RAW/gz_target6` f04)

- 웹 MANUAL로 회전 중 Gate 프로세스를 0.35 s SIGSTOP했다.
- 정지 직전 출력은 (0.0, 0.3)이고, 정지 중 출력은 0개였다.
- **Gazebo 로봇은 그동안 계속 회전했다(0.042 rad)**. Ignition DiffDrive는 명령 timeout이 없어 마지막 명령을 유지한다.
- 재개 후 Gate 출력은 0이었다.

### 3.3 한계(실차 판정 아님)

- 이 결과는 Gate/드라이버 경계의 **모델 시험**이다. **실제 LIMO STM32의 명령 timeout은 측정하지 않았다.**
- 실차에서 정지 중 이동 거리는 STM32 timeout(미측정)과 제동에 달려 있다.
- 실차 전 필요 조치(제안):
  - 하부 명령 timeout 측정
  - 하드웨어 E-Stop
  - Gate 자기 정지 감지(재개 시 경과 시간이 임계 이상이면 0 유지 + 별도 사유 기록). Gate 정책 완화가 아니라 추가다.

## 4. 실제 코드 변경

| 파일 | 변경 | 성격 |
|---|---|---|
| `fleet_control/tests/mode_e2e/stack_up.sh` | `SIM_BACKEND=gazebo`이면 sim을 Gazebo launch로 바꾸고 `USE_SIM_TIME=1`, `IGN_PARTITION`(필수), `IGN_IP=127.0.0.1`, `GZ_SPAWN`, `RMF_DEMOS_HOME`, `ODOM_ACTIVE_TOPIC`을 export한다. Agent에는 `use_sim_time:=true`를 준다 | 시험 전용. 기본값(미설정) 경로는 같은 명령을 생성(셸 전개로 확인) |
| `fleet_control/tests/web_mapping_isolated/stack.launch.py` | `use_sim_time` ← `USE_SIM_TIME`. odom relay 입력 ← `ODOM_ACTIVE_TOPIC`(기본 `/odom`) | 시험 전용. 기본값 동일 |
| `fleet_control/tests/gazebo_web/gazebo_world.launch.py`(신규) | Office 월드 + `limo_eval_diff`. bridge는 ROS `/cmd_vel_locked` → Gazebo `/cmd_vel`. `/scan`은 별도 bridge. normalizer, ground truth, truth adapter | 시뮬 전용 |
| `fleet_control/tests/gazebo_web/gz_truth_adapter.py`(신규) | Gazebo world odom → `/e2e/ground_truth`(spawn 좌표계) | 시뮬 전용 |
| `fleet_control/tests/gazebo_web/run_gazebo_web.py`(신규) | 정상 9개 + 장애 8개 시나리오 하네스. `e2e_common` 사용 | 시험 |
| `fleet_control/tests/manual_e2e/run_manual_e2e.py` | s16 `zero_after` 창을 다음 입력 시작 전까지로 제한. 너무 짧거나 표본이 없으면 FAIL | 시험 판정 수정 |
| `experiments/…/gate_r12_reset/{gate_trace.py, scan_witness.py, repro_gate_stall.py, analyse_gate_stall.py, stall_injection.py}` | 진단 도구(새 파일). `gate_r11_input/fake_sensors.py`에 `SIM_DRIVER_TIMEOUT`(기본 0.3 = 기존 동작)과 ground truth 발행 추가(원본 사본 보존) | 진단 |
| Agent(`topics.py`, `ros_graph.py`, R3 fail-closed) | [REFACTOR_PHASE35_20261121.md](REFACTOR_PHASE35_20261121.md) | 구조/동작 |

Gate 정책(파라미터·판정)은 바꾸지 않았다. Gazebo 실행의 Gate 파라미터는 kinematic E2E와 같다: r12, `input_exclusive_publishers=[fleet_motion_watchdog]`, 0.20 m/s / 0.60 rad/s, `require_operator_unlatch`.

## 5. Gazebo에 연결된 ROS 노드·토픽(g00 실측, 매 실행 동일)

```text
Web(dashboard, headless Chrome) / HTTP → Fleet Server(127.0.0.1:28961, --require-operating-mode)
  → Robot Agent /fleet_robot_agent (VelocityArbiter) → /fleet/nav_cmd_vel (writer 1개)
  → /fleet_motion_watchdog → /cmd_vel_raw (writer 1개, reader = Gate 1개)
  → /stall_detector_node (r12) → /cmd_vel_locked (writer = Gate 1개, reader = /gz_bridge)
  → ros_gz_bridge → Ignition /cmd_vel → DiffDrive(limo_eval_diff)
Nav2: controller/behavior → /fleet/cmd_vel_nav → velocity_smoother → /fleet/cmd_vel_nav_smoothed → Arbiter(우회 없음)
ROS /cmd_vel: publisher 0, subscriber 0
/clock: /gz_bridge 1개 / /scan: /gz_scan_bridge 1개(10.0 Hz, 최대 간격 0.11 s) / /odom: /sensor_normalizer_node 1개
/odom_nav2 + TF odom→base_footprint: /odom_tf_relay_node 1개 (입력 = 스택 자체 ICP /odometry/lidar_odom, 아래 6.3)
시작 시 AMCL·map_server·SLAM 0개. MAPPING 중 SLAM 1개, NAVIGATION 중 map_server 1개 + AMCL 1개(g04 실측)
실시간 배율 1.0, 기동 19~20 s
```

- Office 월드의 rmf 데모 로봇(`plugin_tinyRobot1/2`)도 `/tf`를 발행한다. 다만 자기 트리(`world→tinyRobotN/*`)만 쓰며 LIMO 프레임(map/odom/base_footprint/base_link/laser_link)과 충돌하지 않는다(g00 TF 검사).
- 하네스 관찰자는 `/cmd_vel_raw`·`/fleet/nav_cmd_vel`를 구독하지 않는다. 처음에 구독했을 때 Arbiter 잠금 그래프 계약(reader 정확히 1개)이 `VELOCITY_GRAPH_NOT_LOCKED`로 MANUAL을 거부했다. 계약이 의도대로 작동한 것이며 하네스를 고쳤다. 상류 명령은 Gate 상태의 `cmd_vx_in`으로 기록한다.

## 6. 웹 MANUAL / MAPPING / NAVIGATION 결과

실행별 하네스 수정이 있었다(아래 6.4). 각 실행의 요약은 `RAW/gazebo_runs_matrix.txt`에 있다.

| 시나리오 | 결과(실행 1~5, target6) | 요점 |
|---|---|---|
| g00 기동·그래프·시계 | 6/6 PASS | 위 5절 |
| g01 웹 MANUAL 주행 + Deadman 해제 | 6/6 PASS | 0.24 m 이동, Gate 최대 0.10 m/s. 해제 → Gate 0까지 0.095~0.171 s, Gazebo 정지 0.127~0.198 s, 관성 이동 0.014~0.020 m |
| g02 실제 대시보드 keyboard / pointer / gamepad | 3 PASS / 2 FAIL | keyboard·gamepad는 매번 이동했고 해제 0.6 s 후 Gazebo 속도 0. **pointer 입력이 간헐적으로 명령을 만들지 못함**(1회는 Gate 출력 0, 1회는 0.011 m). 미해결 |
| g03 웹 MAPPING START → Frontier 탐색 → PAUSE/SAVE/STOP | 2 PASS / 4 FAIL | PASS(target6): 13.85 m 탐색, 후보 41개. FAIL: 아래 6.1 Frontier 교착 |
| g03b 대체 지도(웹 MANUAL + 하네스 SLAM) | 3 PASS(웹 지도가 성공한 2회는 SKIP) | **웹 MAPPING 경로가 아니다.** 항법 시험용 지도 확보 목적 |
| g04 저장 지도 LOAD + 초기 위치 → CONFIRMED | 4 PASS(ICP odom 이후 전부) | 웹 지도: match 0.907, AMCL–truth 차 0.047 m. map_server 1 + AMCL 1, SLAM 0 |
| g05 웹 목표 → Nav2 경로 추종 | 1 PASS / 5 FAIL | PASS(실행 3): SUCCEEDED, 목표 오차 0.09 m, 경로 86점, Arbiter source NAV2. FAIL: 아래 6.2 물리 정지 |
| g06 NAVIGATION → MANUAL(목표 진행 중) | 1 PASS | 임무 CANCELED, Gate 0까지 0.053 s, Gazebo 정지 0.15 s. 나머지 FAIL은 g05 실패의 후속(`safety_not_ready`) |

- 웹 목표는 `POST /missions`로 보냈다. 웹 목표 Drawer가 보내는 요청과 같다. 지도 캔버스 클릭 자체는 구동하지 않았다(**미검증**).

### 6.1 Frontier 교착(제품 동작 발견)

- 넓은 홀에서 첫 scan 하나로 만든 지도에는 Frontier 후보가 로봇 근처 1개뿐이었다(`min_frontier_size` 0.35 m).
- 그 목표(약 0.25 m)에 도달해도 SLAM은 새 scan을 넣지 않았다(`minimum_travel_distance` 0.2 m / heading 0.2 rad). 지도가 그대로라 후보도 그대로였다.
- Worker는 이미 시도한 셀을 다시 제안하지 않는다(`attempted`). 그래서 **탐색이 멈춘다.**
- 경로가 0.0~0.1 m에 그친 실행에서 저장 지도는 free 733셀, occupied 29셀이었다. 경로가 1.8 m·13.9 m로 시작된 실행은 정상 탐색했다.
- MAPPING 모드에는 수동 주행 경로가 없다(설계: Arbiter는 MAPPING에서 Agent 소유 Frontier 목표의 Nav2 출력만 통과).
- 수정은 제안만 한다: 시도한 셀 재시도(지도 변경 후), 또는 시작 시 제자리 회전 목표. 다른 담당자 파일(`fleet_frontier_candidates.cpp`)과 제품 코드는 바꾸지 않았다.

### 6.2 물리 정지와 Gate STALL 래치(참 양성)

- `STALL LATCHED`가 4번 발생했다: 바퀴 0.08~0.15 m/s, LiDAR 속도 0, 1.5 s.
- 그때마다 Gazebo truth 속도는 직전 2 s 평균 0.000~0.011 m/s였다. 로봇이 **실제로 막혀 있었다.**
- 막힌 원인은 LiDAR 평면에 보이지 않는 물체로 추정한다(스폰 근처 rmf 데모 로봇·충전기 등). **확인하지 않았다.**
- 같은 위치에서 다음 목표도 1.5 s 안에 다시 래치되고 Agent가 `SAFETY_CANCEL`했다(g05 FAIL의 원인).

### 6.3 Gazebo 휠 odom 오차(시뮬 설정 결정)

- Gazebo 단독 측정(`RAW/odom_probe`): 0.22 rad/s로 20 s 회전한 뒤 0.8 m 전진했다.

| 출처 | 회전 | 거리 |
|---|---|---|
| truth | 2.68 rad | 0.61 m |
| wheel `/odom` | 4.40 rad(+64 %) | 0.80 m(+31 %) |
| ICP `/odometry/lidar_odom` | 2.43 rad(−9 %) | 0.58 m(−6 %) |

- DiffDrive odom은 미끄러지는 모델의 명령 바퀴 속도를 그대로 적분한다.
- 휠 odom을 쓰면 지도가 회전 방향으로 어긋났다. AMCL match는 0.14였고 Localization Worker가 CONFIRMED를 **올바르게 거부**했다.
- 그래서 Gazebo 백엔드는 relay 입력을 스택의 ICP odom으로 했다. 2026-11-13 Office 검증 실행과 같은 선택이다. Gate 입력·임계값은 그대로다.

### 6.4 실행별 하네스 수정(제품 변경 아님)

| 실행 | 발견 | 수정 |
|---|---|---|
| smoke1–3 | truth adapter 파라미터 형(INTEGER), 관찰자의 잠금 토픽 구독, TF 검사 과잉 | 수정 |
| 1 | 웹 MAPPING 교착, Watchdog 재시작 직후 Arbiter 준비 전 시작 | 대체 지도 g03b, 준비 대기 |
| 2 | 휠 odom 오차로 지도 불량, 전진 반복으로 가구 충돌 | ICP odom, 후진 시도 |
| 3 | 웹 bench는 전진 전용(`max(0, v)`) | 장애 시나리오는 제자리 회전 |
| 4 | Watchdog 고스트로 Gate reset 40 s 거부, LOAD `graph occupied` | guard 75 s, LOAD 재시도 |
| 5, target6 | 탐색 경로 밖 목표 → 물리 정지 | 지나간 셀 우선 목표 |

## 7. 정상 주행·장애 정지 기록(Gazebo)

시간은 사건 시각부터다. "Gate 0" = `/cmd_vel_locked` 마지막 0 아닌 출력. "정지" = Gazebo truth |v|<0.01 m/s, |w|<0.02 rad/s가 된 마지막 시각.

| 사건 | Gate 0 | Gazebo 정지 | 사건 후 이동 | 실행 수 |
|---|---|---|---|---|
| Deadman 해제(g01) | 0.095–0.171 s | 0.127–0.198 s | 0.014–0.020 m | 6 |
| 웹 끊김(샘플 중단, f01) | 0.237–0.279 s | 0.269–0.325 s | 0.005–0.031 m | 5 |
| Agent kill(f02) | 0.749–0.868 s | 0.795–0.940 s | 0.007–0.091 m | 4 |
| Watchdog kill(f03) | 0.776–0.846 s | ≤0.864 s | 0.001–0.040 m | 4 |
| LiDAR 정지(scan bridge SIGSTOP, f05) | 0.334–0.336 s(FAILSAFE 0.356 s) | ≤0.389 s | ≤0.007 m | 2 |
| AMCL kill(목표 진행 중, f06) | 0.501 s | 0.604 s | 0.079 m(0.17 m/s 주행 중) | 2 |
| NAVIGATION→MANUAL(g06) | 0.053 s | 0.150 s | 0.010 m | 1 |
| Gate 0.35 s 정지(f04) | — | — | 0.042 rad 회전(정지 중) | 1(측정) |
| bt_navigator 정지 + 취소(f07) | 0.987 s | 판정 불가 | 0 | 1(**결론 불가**: 사건 0.5 s 전 이미 STALL로 정지) |
| 외부 publisher 주입(f08): `/cmd_vel_raw`, `/fleet/nav_cmd_vel`, `/cmd_vel` 각 1.5 s | Gate 0 아닌 출력 0 | 이동 ≤0.002 m | — | 5 |
| 재시작 후 이전 세션 명령 재사용(f02) | 409 `mode_not_manual`, Gate 출력 0 | — | — | 4 |

- 매 실행의 명령/Gate/truth 표본은 `RAW/<run>/samples.json`에, 시나리오별 값은 `result.json`에 있다.
- `/cmd_vel_locked`(Gazebo 모터 토픽 자체) 주입은 하지 않았다. Gate 아래에는 소프트웨어 방어가 없다. 이것은 3절과 같은 한계이며 실차의 `/cmd_vel`에도 해당한다.

## 8. 회귀 결과

| 묶음 | 결과 | 비고 |
|---|---|---|
| 단위(`refactor35…/unit_after_r3_gz`) | 90 / 21 | 21 FAIL은 변경 전과 같은 환경 의존 목록 |
| Phase 1 manual(브라우저) | 18 / 1 → 창 수정 후 재실행 **19 / 0**(1회, `refactor35…/phase1_after_s16fix`) | s16 pointer: 하네스 창 겹침 |
| Phase 2 mode | 16 / 1 | f04: E(본 문서 1절) |
| Phase 3 nav(고정 지도, s10 포함 묶음) | 18 / 0 | — |
| Gate Reset E2E | 13 / 0 | — |
| s10 FollowPath 회귀 | Phase 1 묶음 안에서 PASS | — |

## 9. Codex 검증 명령

```bash
cd fleet_control/tests && for t in test_locked_graph_contract_characterization.py test_ros_graph_helpers_characterization.py test_e2e_common.py; do python3 $t || echo FAIL $t; done
```

```bash
local_data/raw_results/refactor35_claude_20261121/unit_run.sh local_data/raw_results/refactor35_claude_20261121/unit_codex_1
```

```bash
local_data/raw_results/refactor35_claude_20261121/regress.sh e2e_codex_1
```

```bash
cd fleet_control/tests/gazebo_web && OVERLAY=<explore_lite overlay> MANUAL_E2E_NODE_PATH=<playwright node_modules> GZWEB_OUT=<new dir> python3 run_gazebo_web.py 28961 224 --browser
```

```bash
cd experiments/safety_chain_fix_20261113/harness/gate_r12_reset && python3 stall_injection.py --out <new dir> && OVERLAY=<overlay> python3 repro_gate_stall.py <new dir> 28962 227 --rounds 10 && python3 analyse_gate_stall.py <that dir>
```

- 단위·E2E는 ROS Humble, `capstone_ws/install`, explore_lite overlay가 필요하다.
- Gazebo는 Ignition Fortress 6.18과 `~/rmf_demos_humble`(Office 모델·도어 플러그인; `RMF_DEMOS_HOME`으로 변경 가능)이 필요하다.
- 도메인 231은 다른 사용자가 쓰므로 피한다.

## 10. 실차 전 남은 차단 조건

1. **E(Gate 정지)의 원인 미확인.** 정지 중 Gate는 정지 명령을 낼 수 없다(3절).
2. **STM32 명령 timeout 미측정, 하드웨어 E-Stop·제동거리 미검증.** 3절의 상한이 실차에서 얼마인지 모른다.
3. Frontier 교착(6.1). 실차 Mapping START는 계속 차단한다.
4. 웹 pointer 입력 간헐 무반응(g02).
5. Watchdog 재시작 후 DDS 고스트로 Gate `NOT_EXCLUSIVE` 래치와 reset 거부(약 40 s). 안전 방향이지만 복구가 느리다.
6. 지도 캔버스 클릭 목표, 방문자 웹, RMF는 미검증이다.
7. Gazebo 결과는 합성이다. 바퀴 odom 오차·LiDAR 비가시 장애물 같은 시뮬 특성이 결과에 섞여 있다(6.2, 6.3).
