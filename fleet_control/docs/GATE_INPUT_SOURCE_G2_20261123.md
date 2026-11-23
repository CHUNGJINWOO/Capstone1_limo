# Safety Gate: `/cmd_vel_raw` 입력 출처 검증 (G2, 2026-11-23)

범위: G2 worktree의 Gate(`capstone_ws/src/limo_capstone/limo_capstone/stall_detector_node.py`)가 승인된 Watchdog 입력만 운동 명령으로 인정하게 만들고, 우회 경로가 막혔는지 격리 ROS에서 확인한다. 실제 LIMO 접속·배포, 운영 서버 18190, 모터 잠금 해제, merge commit, push·PR, 새 worktree 없음. staging의 미추적 소스와 기존 실험 파일은 읽기만 했다.
기록: `local_data/raw_results/real_limo_integ_claude_20261122/field_ready_01/p13*`~`p17*`.

## 1. 무엇이 문제였나
- Gate는 `/cmd_vel_raw`의 모든 메시지를 보낸 사람과 무관하게 사용했다. Watchdog과 Arbiter는 자기 입력만 검사하고, 모터 직전 마지막 단계의 입력은 아무도 검사하지 않았다. NaN·Inf·범위 밖 값도 그대로 통과했다(`abs(NaN) > 임계`가 False라서 `want_motion`이 False가 되고 NaN이 그대로 출력).
- 재현(수정 전, 저장소의 Gate): 격리 E2E g01 외부 값 22개·g02 24개·f07b 60개가 `/cmd_vel_locked`까지 도달, Gate 상태는 NORMAL 유지. v5b에서도 같음(G2 회귀 아님).
- Humble rclpy에는 메시지별 발신자 GID가 없다(`take_message` MessageInfo에 타임스탬프만). 그래서 출처 검증은 **그래프 수준**일 수밖에 없다.

## 2. 변경 (Gate와 구성 3곳)
| 파일 | 내용 |
|---|---|
| `capstone_ws/src/limo_capstone/limo_capstone/stall_detector_node.py` | 제안 패치 05(r11)를 현재 Gate에 맞게 적용: 9개 hunk 중 2개는 다른 에이전트의 미커밋 `max_abs_linear_x` 문맥 때문에 수동으로 옮김. 추가로 r13의 **writer 교차 검증**과 **출력 토픽(`/cmd_vel`) 외부 발행자 감지**를 이식. 새 파라미터 모두 기본 꺼짐(`input_exclusive_publishers: ['']`, `input_writer_status_topic: ''`, `exclusive_cmd_vel_topic: ''`) = 기존 동작 그대로 |
| `capstone_ws/src/limo_fleet_bringup/limo_fleet_bringup/safety.py` | 격리 bringup의 Gate에 검증 켬: 허용 발행자 `fleet_motion_watchdog`, writer 상태 `/fleet/motion_watchdog/status`, 한계 0.20 m/s·0.60 rad/s(Nav2 최대 0.15/0.5 위), 출력 토픽 `/cmd_vel_locked` |
| `fleet_control/tests/test_gate_input_source_20261123.py` | Gate 판정 단위 시험 24개(격리 ROS) |
| `fleet_control/tests/probe_field_chain_20261123.py` | G1~G9 확인 추가(Watchdog·Arbiter·Gate를 실제로 띄운 스택) |
| `fleet_control/tests/test_g2_bringup_chain_20261123.py` | bringup Gate 파라미터 정적 확인 |

## 3. 출처 검증 방식과 한계
**규칙**: Gate는 `/cmd_vel_raw` 메시지를 다음이 모두 참일 때만 쓴다 — 발행자 endpoint가 **정확히 1개**, 노드 이름이 허용 목록, 그 endpoint의 GID가 **Watchdog 자신이 상태 메시지(`output_gid`, `boot_id`)로 알린 GID와 같고**(1 s 이내 최신), 바인딩 이후 변하지 않았고, 값이 유한하며 한계 안.
| 상황 | 동작 |
|---|---|
| endpoint 2개 이상(외부 발행자, 같은 이름 흉내, SIGKILL된 Watchdog의 잔여 endpoint ~16 s) | `NOT_EXCLUSIVE` — **sticky**, 0 출력, FAILSAFE. 외부가 떠나도 운영자 Gate 리셋 전까지 유지 |
| 허용 이름이 아닌 단독 발행자 | `UNEXPECTED_PUBLISHER`, sticky, 바인딩 안 함 |
| Watchdog 재시작(GID 변경) / 상태의 `boot_id` 변경 / 상태가 다른 GID를 알림 | `PUBLISHER_CHANGED` / `WRITER_RESTARTED`, sticky |
| endpoint GID 누락(전부 0) / GID 속성 없음 / 그래프 조회 예외 / 노드 이름 미해석 | `GID_UNKNOWN` / `LOOKUP_ERROR` / `NODE_UNRESOLVED` — 0 출력, 복구되면 settle 0.5 s 후 재사용(sticky 아님) |
| Watchdog 상태가 없거나 1 s보다 낡음 | 바인딩하지 않음(`SETTLING`) → 0 출력 |
| NaN/Inf/범위 밖 | 0, `INVALID_TWIST`, 0.5 s 유지; 거부된 운동 명령은 arming도 막는다 |
| 출력 토픽에 Gate 외 발행자 / 같은 이름의 두 번째 endpoint / 조회 오류 | `cmd_vel:FOREIGN_PUBLISHER`, `DUPLICATE_GATE_ENDPOINT`, `LOOKUP_ERROR` — Gate 결함으로 FAILSAFE(sticky 아님, 사라지면 복구) |
모든 원인은 Gate 상태 JSON의 `cmd_input`(state, sticky, publishers, bound_gid, writer_confirms_gid, counts, last_drop, output_topic_faults)와 `fault_reasons`, 로그(`[INPUT]`, `[OUTPUT]`)에 남는다.

**한계(명시)**:
1. 노드 이름과 발행자 수만으로는 신뢰할 수 없다(다른 프로세스가 같은 이름을 쓸 수 있다). 그래서 GID 바인딩과 writer 상태 교차 검증을 더했다. 그래도 **인증은 아니다**(`validation: GRAPH_ONLY_UNAUTHENTICATED`): `/cmd_vel_raw` 발행자가 **단독으로, 허용된 이름으로, 진짜 Watchdog이 뜨기 전에** 나타나고 **상태 메시지까지 위조**하면 받아들여진다. DDS 보안(SROS2)이 필요하다.
2. 두 번째 발행자가 보이기까지 DDS endpoint 발견 지연 + Gate 타이머(0.1 s)가 있어 첫 순간에는 진짜 Watchdog의 값이 약 0.2~0.3 s 더 나갈 수 있다(외부 값은 나가지 않는다). 측정은 §5.
3. 드라이버(`limo_base`)는 `/cmd_vel`의 모든 발행자를 구독한다. Gate가 0을 내보내도 **다른 노드가 `/cmd_vel`에 직접 쓰는 것을 막을 수 없다** — 감지와 보고만 한다(`exclusive_cmd_vel_topic`). 실제 차단은 구동 전원 경로(물리)나 DDS 접근 제어가 맡는다.
4. 실차 launch(`limo_integrated_auto_mapping_real.launch.py`)는 Watchdog이 없어서(제안 02b 미적용) 이 검증을 **켜지 않았다**. 켜면 Watchdog 없이 Gate 입력이 영구히 0이 된다(fail-closed). 실차 launch에 Watchdog을 넣을 때 같은 파라미터를 함께 켜야 한다.

## 4. 단위·정적 시험
- `test_gate_input_source_20261123` **24/24 PASS**(격리 ROS): 정상 Watchdog만 / Watchdog+외부 / 외부만 / 허용 이름 흉내 / Watchdog 종료·재시작 / GID 변경 / boot 변경 / 잔여 endpoint / GID 누락 / GID 속성 없음 / 조회 실패와 복구 / 이름 미해석 / 상태 메시지 누락·지연·다른 GID / 비유한·범위 밖 / 리셋 후 재바인딩, 외부가 남은 채 리셋하면 다시 sticky / 출력 토픽 감지 / 기본 꺼짐은 기존 동작.
- 정적(소스 검색): 저장소의 launch·노드 중 `/cmd_vel`에 쓰는 것은 Gate뿐이다(`stall_detector_node.py:199`). **예외**: `limo_ros2_application`의 `limo_control.py`, `move_to_pose.py`, `move_limo.py`(제조사 데모, 기본 launch에는 없음)와 `wego_2d_nav`의 `navigation_only_launch.py`(`cmd_vel_smoothed → cmd_vel` 직결)는 Gate를 거치지 않는다. 이들이 LIMO에 설치되어 있고 실행되면 우회다.
- bringup 정적 시험(`test_g2_bringup_chain`) 3/3: Gate 입력은 Watchdog만, Gate 파라미터 정확.

## 5. 격리 ROS 시험 (수정 전 → 후)
**(a) 격리 Gate 하네스 `run_gate_input.py`**(staging 소스, 읽기 전용; Gate 출력 비영(nonzero)/전체 샘플)
| 사례 | 수정 전(저장소 Gate) | 수정 후 |
|---|---|---|
| 정상 Watchdog만 | 150/150 | 150/150 |
| Watchdog + 외부 0.18 | 150/150 (**외부 값 출력**) | 13/150, 최대 0.08 = Watchdog 값(외부 0.18은 0), 이후 NOT_EXCLUSIVE sticky |
| 외부 발행자만 | 185/201 | **0/200** |
| 자리 교체(swap) | 191/200 | **0/200**, 리셋 후 정상 |
| 원래 종료 + 사칭자 잔류 | 201/201 | **0/200** |
| 비유한·범위 밖(nan, inf, 5.0, 9.0) | 모두 통과 | **전 구간 0**, sticky, 리셋 후 정상 |
| Watchdog SIGKILL + 재시작 | 178/184 | 10/185(잔여 endpoint 감지 전 Watchdog 값), ghost 16 s 후 리셋하면 정상 |
| 단독 사칭(허용 이름으로 혼자) | 150/150 | 150/150 — **알려진 한계**(하네스의 가짜 writer는 상태를 알리지 않는다; 상태 교차 검증을 켜면 단위 시험대로 바인딩되지 않음) |
| 입력 중단·정지 | 마지막 비영 0.215 s | 0.219 s (Gate 입력 만료 0.25 s) |
| Gate 재시작 | 운동 명령 중 arming 안 함 | 같음 |

**(b) 모드·저장 지도 E2E(g01·g02·f07b, 같은 하네스·같은 Gate 파일의 전/후)**
| 시험 | 수정 전 | 수정 후 |
|---|---|---|
| g01 MANUAL 중 `/cmd_vel_raw` 외부 발행 | **FAIL** 외부 값 22개 도달, Gate NORMAL | **PASS** 0개 |
| g02 MAPPING 중 | **FAIL** 24개 | **PASS** 0개 |
| f07b 저장 지도 E2E의 Gate 입력 부분 | **FAIL** 60개 | **PASS** 0개 |
| g03 NAVIGATION 중 | 실행 불가(`safety_not_ready`) | 실행 불가(`safety_not_ready`) — 저장소의 Gate는 Safety Context(r7+ 필드)를 만들지 못해 Fleet 임무 자체가 막힌다. **미검증**. 후보 Gate r13으로는 다른 사유(`localization_not_confirmed`)로 실패 |

**(c) 실제 Watchdog·Arbiter·Gate를 띄운 bringup 스택(`probe_field_chain --faults`)**: 27/27 → 29/29 PASS(G1~G9 포함)
- G1 정상 MANUAL 입력이 Arbiter→Watchdog→Gate를 거쳐 출력(최대 0.050 m/s).
- G2 외부 `/cmd_vel_raw` 값 0.15는 출력에 한 번도 나오지 않음(누출 0).
- **G3 정지 시간**: 두 번째 endpoint가 관측자에게 보인 뒤 **0.045 s**에 출력이 0(프로세스 시작 후 1.15 s에 보임 = `ros2 topic pub`의 Python 시작 시간).
- G4 원인 기록(`NOT_EXCLUSIVE`, sticky, `cmd_input:NOT_EXCLUSIVE`), G5 외부가 떠나도 0 유지, G6 운영자 리셋, G7 리셋 뒤 모드(FAULT→MANUAL)를 다시 선택하면 재바인딩·정상 주행.
- G8·G9 출력 토픽 외부 발행자 감지와 자동 해제(결과는 §7).
- 기존 항목: `/cmd_vel_raw`는 Watchdog만 발행, `/cmd_vel`에 끝점 없음, 외부 `/cmd_vel` 발행 시 Arbiter 정지·Mapping 철회, Watchdog 정지 시 정지, Lease 반납.

**정지 시간 요약**
| 경로 | 측정 |
|---|---|
| 외부 발행자 출현 → Gate 출력 0 | 0.045 s(하네스 `normal_plus_rogue`는 최대 13샘플 ≈ 0.26 s 동안 Watchdog 값만) |
| Watchdog 정지/SIGSTOP → 출력 0 | 0.22 s |
| 비유한 값 → 출력 0 | 즉시(해당 메시지부터), 이후 sticky |

## 6. 이번에 확인하지 못한 것
실차 launch에서의 동작, 실제 DDS 지연·유실, 구동 드라이버와 STM32 쪽 입력 처리, 드라이버가 두 발행자를 섞는 동작(Gate 0과 외부 값 교대), SROS2, g03(NAVIGATION 중)과 r7+ Gate 계약.

## 7. 실제 체인 프로브, 최종 회귀, n03/n05/f08 (Gate와 별도 분석)
**실제 Watchdog·Arbiter·Gate bringup 체인 프로브**(`probe_field_chain_20261123.py --faults`, 격리 ROS): p14 27/27, p14b 27/27, p19b 29/29(G1~G9 PASS). p19는 MAPPING 중 합성 센서 타임아웃(`lidar_stale`)으로 Gate가 fail-safe해 23/28이었고 재실행에서 29/29 — 프로브 5회 중 1회 플레이크로 그대로 기록한다. G3 정지 시간은 외부 발행자가 그래프에 보인 순간부터 0.045 s(다른 실행 0.000 s).

**최종 회귀(스택 미기동 상태, 전부 OK)**: 계약 271(skip 7), loopback 36, Dedupe 14, Ownership 45, Gate 단위 24, bench/cancel 28, R2 66, ROS 3개 모듈, Arbiter 등 28, R3 11, bringup 정적 점검, 그리고 이름에 gate가 들어간 저장소 시험 12개 파일 전부 OK. 스택이 떠 있으면 R2/R3/agent-shutdown의 프로세스 점검이 스택 프로세스를 보아 실패하므로 스택을 내린 뒤 실행해야 한다.

**g01/g02/g03/f07b (같은 Gate 파일, 수정 전 → 후)**: g01 FAIL(22 누출)→PASS, g02 FAIL(24)→PASS, f07b FAIL(60)→PASS. g03은 저장소 Gate에 r7+ Safety Context가 없어 Fleet 미션이 `safety_not_ready`로 막혀 전/후 모두 실행 불가. r13 후보 Gate에서는 g03이 `localization_not_confirmed`로 실패한다. 즉 g03은 이번에 검증되지 않았다.

**n03/n05/f08 — Gate 수정과 무관한 별도 문제(미해결, 후속 과제)**
- 재현: r10 후보 Gate(미션이 동작하는 유일한 Gate)로 5회 중 2회(r8, rep1) 세 시험 모두 FAIL, 3회 PASS. 간헐적.
- 공통 원인(관측): 실패 실행에서 같은 "유효" 목표(지도 중앙 근처 (-0.2, ±0.2))가 서버 goal-check(여유 0.20 m)는 통과하지만 Nav2 planner가 `failed to create plan with tolerance 0.30`으로 계획하지 못한다. n05의 두 번째 목표와 f08의 `new_goal_after_recovery`도 같은 목표라 함께 실패한다. r8의 f08에는 `idle_not_proven` 20회도 보였다.
- 의존: 시뮬레이션 지도 탐색의 무작위성, costmap(`robot_radius` 0.18, `inflation_radius` 0.28)과 goal-check `GOAL_CLEARANCE_M=0.20`의 불일치 가능성(추정, 미확정).
- Gate 입력 검증의 영향 없음(r10 Gate에는 검증이 없다). v5b 기준선 비교는 그 기준선 시험이 다른 이유(`operation_state_conflict`)로 p01에서 실패해 못 했다.
- 최소 수정은 적용하지 않았다. 후속: goal-check와 Nav2 planner 일관성 수정, 또는 하네스가 planner 사전 계획으로 목표를 고르게 하기.
