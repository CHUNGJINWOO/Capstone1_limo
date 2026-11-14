# Gazebo × Open-RMF 실행 결과 (2026-11-12)

**최종 Gazebo 69/69 PASS, 필수 항목 15/15 PASS**, 약 349.6초. RMF patrol → Mission Manager 임무 → 실제 Gazebo plant 이동(native world GT 변위 >0.5m) → RMF `completed`를 확인했습니다. 합성 모의 Nav2를 Gazebo 검증으로 사용하지 않았습니다. 한 노트북의 짧은 프로파일 1회 최종 통과이며 통계적 안정성·장거리 운용을 보증하지 않습니다.

[재현 명령·구성](README.md), [선별 결과 JSON](results.json). 원시 로그/DB/토큰/저널은 시험별 로컬 임시 폴더에 보존했고 공개 파일에 복사하지 않았습니다. 브라우저 조작은 이번에 수행하지 않았으며 웹 사용자의 생성·취소는 동일 HTTP API로 검사했습니다.

## 필수 15개 항목

| 번호 | 검사 | 결과·근거 |
|---|---|---|
| 1 | 지도·좌표 연결 | PASS: waypoint 5개 free, lane 4개 ≥0.2m 여유, 변환 왕복 <1e-9m, RMF 보고와 Nav2 변환 좌표 <0.1m |
| 2 | RMF 등록 | PASS: limo_fleet/pcu, 지도 L1 |
| 3 | patrol 생성 | PASS: 표준 RMF task API 수락 |
| 4 | 입찰·배정 | PASS: fleet/robot 배정 확인 |
| 5 | 경로 → 임무 | PASS: rmf-adapter 소유 임무 생성, 순차 실행 |
| 6 | Gazebo 실제 이동 | PASS: native GT 변위 >0.5m 및 Nav2 도착 위치, 실제 sim 시간 진행 |
| 7 | RMF 완료 | PASS: task_state_update의 completed |
| 8 | 취소 | PASS: >1.2m 남은 구간에서 취소, 임무 CANCELED + cancel 명령 SUCCEEDED, 확인 전 후속 제출 없음, RMF canceled, 정지·목표 미재전송 |
| 9 | 웹/RMF 소유권 | PASS: RMF lease 중 웹 생성409 및 임무 미생성, admin 인수 후 웹 이동·RMF 입찰 거절, 종료 후 lease 재획득; 임무 중첩 없음 |
| 10 | 서버 중단·재연결 | PASS: offline 보고, 단절 중 새 제출 없음, 재연결 후 같은 작업 완료 |
| 11 | Adapter 재시작 | PASS: Mission Manager 고아 임무 terminal 후 재등록 (이전 RMF 작업의 고아 상태는 별도 미해결) |
| 12 | Agent 재시작 | PASS: 핸들 소실을 RECONCILIATION_REQUIRED 처리, 재전송 없음 |
| 13 | 중복 실행 | PASS: native Nav2 UUID 26개 = 확인된 navigate 명령 26개, 수락 audit 전부 일치, command 중복·같은 목표 이중 전송 없음 |
| 14 | 불명확 상태 차단·복구 | PASS: 신규 이동 차단, operator 복구403, 목표 비활성 증거 확인 후 admin 복구 FAILED/RECOVERED_UNVERIFIED_OUTCOME, 새 RMF 작업 완료 |
| 15 | 종료·정리 | PASS: 소유 프로세스 및 자체 HTTP/WS 포트 잔여 없음 |

추가 REST 확인: robot 상태200/online, map 프레임 TF pose OK, Nav2 READY, 지도 메타데이터200 및 실제 PNG200(923바이트, 179×238, 0.05m/셀). REST 단독 이동은 SUCCEEDED 및 native GT 변위 >0.4m, 단독 취소도 terminal CANCELED였습니다.

## Gazebo 구간별 경과 시간

아래는 최종 실행의 wall time이며 장애 대기·센서 준비·복구 대기를 포함합니다. 속도 벤치마크가 아닙니다.

| 구간 | 시간 | 결과 |
|---|---:|---|
| 0. static checks of the RMF map <-> Nav2 map connection | 0.1 s | PASS |
| 1. RMF alone, then the isolated Mission Manager + simulated robot + adapter | 69.1 s | PASS |
| 2. RMF task -> Mission Manager mission -> Gazebo Nav2 | 30.7 s | PASS |
| 3. cancel from RMF | 17.5 s | PASS |
| 4. ownership: server-side control lease (RMF holds it) vs web UI | 48.9 s | PASS |
| 5. fault injection: Mission Manager (server) outage during an RMF task | 36.2 s | PASS |
| 6. adapter restart in the middle of a mission | 21.1 s | PASS |
| 7. agent restart in the middle of a mission -> RECONCILIATION_REQUIRED | 24.3 s | PASS |
| 7b. operator recovery of RECONCILIATION_REQUIRED (evidence-gated) | 95.5 s | PASS |
| 7c. goal accounting: every goal the Gazebo Nav2 received corresponds to exactly one acknowledged Mission Manager command | 2.0 s | PASS |
| 15. cleanup | 4.1 s | PASS |

## 회귀 검사

| 검사 | 이번 결과 | 경과 시간 |
|---|---:|---:|
| Agent 패키지 colcon build | 1개 패키지 성공 | 1.47 s |
| tests/run_checks.py | 46/46 | 약30.7 s |
| tests/test_dedupe.py | 14/14 | 약1.9 s |
| tests/test_recovery.py | 14개 통과 | 약12.6 s |
| tests/test_cancel_results.py | 14개 통과 | 약1.1 s |
| tests/test_auth_reload.py | 2개 통과 | 약0.2 s |
| tests/test_agent_recovery.py | 2개 통과 | 약0.3 s |
| tests/test_ownership.py | 37/37 | 약17.3 s |
| tests/test_sim_motor_policy.py | 12개 통과 | 약0.4 s |
| rmf_adapter/tests/test_runner_policy.py | 33/33 | 약9.1 s |
| rmf_adapter/tests/run_rmf_checks.py (최종 좌표 수정 후) | 59/59, 필수15/15 | 약190.4 s |

단위/관제 검사의 근사 시간은 각 로컬 로그 생성~최종 수정 시각(약 ±1초)이며 내부 계측시간과 다를 수 있습니다. Gazebo/최종 RMF 시간은 스크립트 구간 계측입니다. 관제 통합은18130/164, 소유권18133, 모의 RMF는18140·18141/166·167, Gazebo는18120·18121/162·163을 사용했습니다. 기존 시험의 기본 포트/도메인은 변경하지 않고 환경변수로 분리했습니다. 빌드·설치·로그는 임시 디렉터리만 사용했습니다.

## 발견한 실패와 수정

- 최초 launch 변형에서 Humble의 launch source location/Node executable 자료형을 잘못 다뤄 시작 실패 → 설치된 API에 맞춰 변형 launch만 수정했습니다.
- Nav2 lifecycle 활성화 전에 목표를 보내 NAV2_UNAVAILABLE → 센서/위치·Nav2 활성·Gate NORMAL 대기를 추가했습니다.
- 초기 장거리 F4-TF 구성에서 map/GT 불일치·경로 실패 → wheel-TF + 저장 지도 AMCL, 짧은 별도 그래프를 선택했습니다. F4 단독 원인으로 확정하지 않았고 장거리 문제는 미해결입니다.
- F4를 완전히 제외했을 때 Gate의 필수 lidar odometry 입력이 없어 INITIALIZING/FAILSAFE → F4 안전 입력을 복원했습니다. Gate 비활성화/reset/임계값 변경은 하지 않았습니다.
- 초기 최종 후보는67/69, 다음 후보는67/68이었습니다. 거절된 새 작업과 이전 작업의 위치 보정 임무를 총개수로 비교한 시험을 분리했고, 좌표 검사는 실제 코드 결함(waypoint 병합 시 위치 snap)으로 밝혀 `Start.location`에 실제 변환 위치를 보존했습니다. 허용치0.1m는 유지했습니다.
- 좌표 수정 첫 구현은 RMF 2.1.8의 Start.time getter(timedelta)/constructor(datetime) 차이로 Adapter가 종료됐습니다(Gazebo15/17, 모의8/10). 기존 Start 객체의 location setter만 쓰도록 바꾸고 두 전체 E2E를 재실행했습니다.
- 관제 회귀 최초44/46은 LiDAR/Gate 최초 NO_DATA 수신 경합이었습니다. 준비 대기만 추가했으며 실제 검사 기대값 OK/NORMAL은 유지하고 재실행46/46이었습니다. 과거의 원인 미확정42/46 기록은 상위 문서에 남겨뒀습니다.
- 자체 WS sink가 정리 검사 시 아직 listen하던 시험 결함은 명시적 shutdown/close로 수정했습니다.

## 검증하지 않은 범위와 보존 사항

- 실제 LIMO 접속·임무·모터 구동, 실차 배포, 브라우저 UI 재시험은 하지 않았습니다. sim-only opt-in 12개 단위 검사에서 real/default 거부를 확인했습니다.
- 원본 Gazebo/SLAM/Nav2/자동 맵핑/Gate 코드 및 팀 워크스페이스는 수정하지 않았습니다. Gate 시작만20초 지연했고 입력·알고리즘·임계값·latch는 유지합니다. RMF/Adapter의 직접 속도·Nav2 명령 경로는 추가하지 않았습니다.
- 운영8080 서버와 운영 모의 Agent를 중단·교체하지 않았습니다. 실제 네트워크/Tailscale 설정과 시스템 ROS 패키지도 변경하지 않았습니다.
- 종료 SIGINT 처리 중 원본 Python ROS 노드 일부에서 `rcl_shutdown already called`/KeyboardInterrupt 예외가 기록됐습니다. 기능 시험 중 노드 종료와는 구분하며, 기존 종료 처리 소스는 수정하지 않았습니다. 시험 소유 프로세스·포트 잔여 없음은 별도로 검사해 통과했습니다.
- 장거리 위치추정 정밀도, 다양한 장애물 회피 성능, 다중 로봇/장시간 운용은 미검증입니다. RMF Adapter 재시작 후 이전 RMF 작업이 underway로 남는 한계는 유지됩니다.
- 다음 단계는 map/world 정합성·장거리 localization 진단과 RMF 고아 작업 복구입니다. 실차 모터 잠금 통신 시험은 별도 승인 후 진행합니다.
- commit/push/PR 생성·라이선스 추가는 하지 않았습니다.

## 수정·추가 파일

- `fleet_control/README.md`
- `fleet_control/agent/limo_robot_agent/limo_robot_agent/agent_node.py`
- `fleet_control/agent/limo_robot_agent/limo_robot_agent/motor_policy.py`
- `fleet_control/agent/limo_robot_agent/limo_robot_agent/ros_adapter.py`
- `fleet_control/docs/API.md`
- `fleet_control/docs/VALIDATION.md`
- `fleet_control/gazebo/README.md`
- `fleet_control/gazebo/RESULTS.md`
- `fleet_control/gazebo/localized_sim.launch.py`
- `fleet_control/gazebo/observe.py`
- `fleet_control/gazebo/results.json`
- `fleet_control/rmf_adapter/config/fleet_gazebo.json`
- `fleet_control/rmf_adapter/docs/RMF.md`
- `fleet_control/rmf_adapter/limo_rmf_adapter/adapter_node.py`
- `fleet_control/rmf_adapter/tests/gazebo_env.py`
- `fleet_control/rmf_adapter/tests/rmf_env.py`
- `fleet_control/rmf_adapter/tests/run_gazebo_checks.py`
- `fleet_control/rmf_adapter/tests/run_rmf_checks.py`
- `fleet_control/tests/run_checks.py`
- `fleet_control/tests/test_sim_motor_policy.py`
