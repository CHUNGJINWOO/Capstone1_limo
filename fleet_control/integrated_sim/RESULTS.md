# 통합 시뮬레이션 결과 (2026-11-13)

환경: 노트북 1대(24 논리 코어, RTX 5060 Laptop GPU, ROS 2 Humble, Ignition Fortress, Open-RMF 2.1.8 격리 설치), **시뮬레이션만**. 실제 LIMO·모터·Tailscale·운영 8080은 사용하지 않았습니다. 아래 수치는 `run_integrated.sh all` 한 번(실행 ID `final4`)의 기록이며, 이 문서에 없는 것은 **미검증**입니다. 반복 횟수가 적어(장애물 시나리오는 개발 중 4회 실행) 통계적 신뢰도를 주장하지 않습니다. 시험 중에도 이 노트북에는 이전 날짜의 사용자 Nav2 프로세스 여러 벌이 계속 떠 있었고(건드리지 않음) 약간의 배경 부하를 줬습니다.

## 1. 통합 실행에서 확인한 것 (한 번의 실행, 13개 검사 중 12 PASS)

| 항목 | 결과 |
|---|---|
| 구성 요소가 모두 한 실행에서 가동 | Gazebo(물리, 저장 지도 AMCL, LiDAR/Depth/RGB), Nav2, 기존 Safety Gate(NORMAL), Robot Agent(`mode=sim`, `SIM_GATED`), Mission Manager, Fleet Adapter + RMF, 영상 송신(proxy 경유), AI 호스트(CUDA YOLO11n), 로봇 쪽 consumer, 대시보드 — PASS |
| 명령 경로 | RMF patrol 작업 → Adapter → Mission Manager 임무(`created_by=rmf-adapter`) → Agent → Nav2 → Safety Gate → Gazebo LIMO 이동 → RMF `completed` — 모든 시나리오에서 확인 |
| 영상/AI 경로 | 움직이는 로봇의 Gazebo RGB → sender → AI 호스트(CUDA) → 결과 → 로봇 쪽 consumer: 확인(§3) |
| 대시보드 | 별도 실행(`up`)에서 브라우저로 확인: 로봇 카드(ONLINE, SIMULATED, 지도 STATIC, Nav2 READY, Gate NORMAL, 모터 경로 SIM_GATED), 임무 목록(RMF 임무 SUCCEEDED 표시), AI 패널 RECEIVING·검출 표시. **지도 캔버스 위 로봇 화살표를 이번 실행에서 스크린샷으로 다시 확인하지는 않았습니다**(이전 단계의 UI 시험에서 확인) |
| 종료 | 등록 프로세스·포트 잔여 없음(`leftovers: none`) |

## 2. 장애물 회피 시나리오 (lounge 월드, RMF 임무 중)

`wp_a → wp_c` 임무가 EXECUTING이고 로봇이 장애물 중심 1.5 m 이내가 되었을 때 0.5×0.5×0.6 m 박스를 경로(wp_b→wp_c 구간 중앙) 위에 **생성**했습니다(장애물 없는 실행은 회피 성공으로 세지 않음).

| 지표 (`final4`) | 값 |
|---|---|
| 장애물 생성 시점 | 임무 EXECUTING 중, 로봇–박스 중심 거리 1.41 m |
| costmap 증거 | 박스 주변 최대 비용 local 100, global 100(lethal) |
| LiDAR / Depth | LiDAR 최소 거리 0.18 m, `/depth_obstacles/points` 점 최대 13,548(장애물 없는 baseline 6,772–14,527 — 점 수만으로 장애물 기여를 구분하지 못함). Nav2 설정은 두 소스(`/scan`, `/depth_obstacles/points`)를 모두 장애물 층으로 사용하지만, **각 소스가 costmap에 따로 기여한 정도는 분리해 확인하지 않았습니다**(미검증) |
| 경로 변화 | 생성 뒤 계획 갱신 15회, 직선(wp_b→wp_c)에서 최대 횡 이탈 0.875 m, 계획 길이 1.96 → 최대 2.25 m |
| 이동 거리 / 시간 | 3.82 m(Ground Truth 적분), 26.8 s wall(25.7 s sim) |
| 최소 간격 | 로봇 중심–박스 표면 0.151 m |
| 충돌 | **1회 FAIL 판정**: 보수적 기준(차체 외접 반경 0.155 m)보다 0.004 m 작아 충돌 가능으로 기록. 접촉 센서가 없어 **실제 접촉 여부는 확인하지 못했습니다**. 개발 중 같은 시나리오의 다른 3회는 최소 간격 0.215, 0.258, 0.241 m(미충돌 기준 통과) |
| RMF | `completed` |
| 판정 | 회피·도착·RMF 완료는 4회 모두 확인. 간격은 실행마다 0.15–0.26 m로 편차가 크고 **여유가 작습니다**(박스를 인플레이션 경계 가까이에서 지나감). 충돌 없음을 안정적으로 보장한다고 말할 수 없습니다 |

움직이는 사람 시나리오(`person`): 사람이 경로를 가로지르는 동안 임무는 `completed`, 충돌 판정 없음, 사람과의 최소 거리 0.99 m. **로봇이 사람 때문에 경로를 바꾸거나 감속했다는 증거는 얻지 못했습니다**(사람이 경로에 닿기 전에 지나감) — 이 시나리오는 YOLO 검출 시험이며 사람 회피 성공으로 보고하지 않습니다.

## 3. YOLO (RTX 5060 CUDA, 움직이는 로봇)

| 항목 | 값 |
|---|---|
| 모델/장치 | YOLO11n COCO, `cuda:0`, 클래스 person/chair/couch/dining table만 |
| 영상 | 카메라 약 14.5 fps(sim 15 Hz, RTF 0.96), 송신 10 fps 상한, JPEG(`image_transport` compressed 재사용) 약 1.1 Mbit/s |
| 검출 FPS | 약 10 fps (9.6–10.0) |
| 추론 | 호스트 로그 평균 9.5–10.8 ms, p95 16.8–19.1 ms. 호스트 총 처리 평균 12.8–14.3 ms(p95 21–23 ms) |
| 로봇 쪽 왕복(전송→결과 수신, 한 시계) | consumer 기준 평균 47–50 ms, p95 약 81–83 ms. 송신 노드 측정 RTT 평균 14.5–16.7 ms |
| 오래된 결과/프레임 폐기 | 정상 구간 0 / 0, 연결 단절 구간은 §5 |
| 검출 예(시나리오 `person`, 4 Hz로 표본한 서로 다른 결과 기준) | person 9건(신뢰도 평균 0.48, 최대 0.70), chair 1건(0.26). 이전 실행(`final3`)에서는 person 8건(평균 0.70, 최대 0.89), chair 16건(평균 0.58, 최대 0.84) — 실행마다 장면 타이밍에 따라 크게 달라 일반화 성능이 아닙니다 |
| 기록 | 각 결과에 클래스, 신뢰도, bbox, **원본 카메라 stamp**(AI 호스트가 되돌림), 호스트·로봇 쪽 시각이 `ai_status.jsonl`/`ai_host_frames.jsonl`/`consumer.jsonl`에 남음 |
| "검출 없음"과 "통신 장애" 구분 | consumer 상태: 정상 수신 중 검출 0개 = `FRESH`·n=0, 장애 = `STALE`/`NO_DATA`와 링크 `DISCONNECTED`/`NO_RESULT`. 대시보드도 두 문구를 따로 표시 |
| 항법과의 관계 | YOLO 결과는 Nav2·Safety Gate의 입력이 아닙니다(consumer는 읽기 전용). 회피는 LiDAR/Depth costmap만 사용 |

## 4. 성능 (정상 구간, 영상 ON)

| 지표 | 값 |
|---|---|
| Gazebo RTF(sim 시간/wall 시간) | 0.92–0.97 |
| CPU(전체 24 코어 대비 %) | 평균 46–48, 최대 56 |
| 프로세스별 CPU(코어 1개 = 100) 평균 | Gazebo 약 727, Agent 82, 로봇 쪽 metrics 65, observer 53, AI 호스트 14, republish 6, sender/consumer 약 2 |
| GPU | 이용률 평균 약 5 %(최대 7 %), VRAM 508–683 MB(렌더링 + YOLO) |
| RAM(시스템 전체 사용) | 최대 약 13.7 GB / 23.7 GB |

### 영상 ON/OFF 비교 (각 2회, 같은 wp_a→wp_c)

| | 영상 ON | 영상 OFF |
|---|---|---|
| RTF | 0.961, 0.972 | 0.963, 0.953 |
| CPU 평균 % | 46.9, 48.2 | 48.0, 47.7 |
| 임무 시간 wall | 11.1, 21.8 s | 18.8, 16.1 s |
| 이동 거리 | 2.30, 2.83 m | 2.74, 2.89 m |
| Nav2 | 성공 | 성공(OFF 1회는 canceled 1건 동반) |
| Gate | NORMAL(일부 STALL_PENDING 2샘플) | NORMAL(일부 STALL_PENDING 2샘플) |

표본이 작고 실행마다 경로가 달라 시간 차이(11–22 s)가 영상 때문인지 구분되지 않습니다. **영상 스트리밍이 항법 성능이나 Nav2/Safety Gate 루프에 눈에 띄는 영향을 줬다는 증거는 이 시험에서 얻지 못했습니다**(영향이 없음을 증명한 것은 아님).

## 5. 장애 주입(긴 RMF patrol 5개 작업 진행 중, 로컬 프록시·프로세스 제어만 사용)

| 주입 | 관찰 |
|---|---|
| 영상 송신 노드 정지(약 7 s) → 재시작 | AI 호스트 `SENDER_DISCONNECTED` → `RECEIVING` 복구 |
| AI 호스트 종료(약 6 s) → 재시작 | 송신 측 재연결 후 `RECEIVING` 복구 |
| 영상 지연 1.5 s, 프레임 손실 70 %, 링크 정지(blackhole) 각 약 7–8 s | AI 호스트 `STALLED`/`RECEIVING`, consumer `STALE` 표시 후 복구 |
| Mission Manager 종료(약 8 s) → 재시작 | 서버 HTTP 0(불통) 확인, 재시작 후 Agent 재연결, 진행 중 RMF 작업 완료 |
| 결과 | RMF 작업 5/5 `completed`, Nav2 목표 7건(성공 6, canceled 1 — Adapter의 `cancel_mission` 동반, 원인 상세 미분석, aborted 0). **영상 장애 구간의 Safety Gate는 모두 NORMAL**, 서버 중단 구간도 NORMAL(전체 실행 NORMAL 184샘플). 영상 장애는 로컬 Nav2/Safety Gate 루프를 멈추지 않았고 로봇은 계속 이동했습니다 |
| 상태 분포(AI 호스트 폴링, 약 0.25 s 간격) | RECEIVING 451, SENDER_DISCONNECTED 33, STALLED 25, NO_SENDER 7. consumer: FRESH 108, STALE 33 |
| 한계 | AI 호스트 재시작으로 누적 카운터가 초기화되어 이 구간의 프레임/폐기 증감 수치는 무효입니다. 임무 통신 단절의 안전 정책은 기존 구현(서버 단절 시 Adapter 제출 중단, Agent 재연결·상태 동기화)을 그대로 사용했으며 새 정책을 만들지 않았습니다. Wi-Fi/Tailscale 설정은 건드리지 않았습니다 |

## 6. 개발 중 발견한 문제와 원인

- **통제 실패(해결)**: 브라우저 탭이 이전 실행의 오래된 토큰으로 새 서버를 계속 조회해 서버의 인증 실패 잠금(HTTP 429)이 같은 주소의 Agent·Adapter까지 막았고, 한 번의 전체 실행이 오염됐습니다(결과에서 제외, 탭을 닫고 재실행). 서버 잠금은 의도된 동작입니다.
- 실행기 결함(해결): 포트 TIME_WAIT를 충돌로 판단, ROS 2 데몬 노드를 도메인 사용 중으로 판단, 상태 응답 경쟁(`robot()` None), 내가 연 WebSocket sink 포트를 잔여로 오판, Gate의 정상 사전 경고 `STALL_PENDING`을 실패로 취급한 검사(기준을 영상 장애 창 `NORMAL` + 전체 실행 failsafe 없음으로 분리).
- 스모크 중 Safety Gate가 `FAILSAFE_SENSOR_TIMEOUT`(LiDAR odometry stale 0.38 s)으로 래치된 적이 있었습니다. 시스템 부하 급증(내 ROS CLI 기동) 시점이었고 정상 실행에서는 재현되지 않았습니다. Gate를 수정·reset하지 않고 스택을 재시작했습니다.

## 7. 검증하지 않은 것

실제 LIMO(영상·Nav2·모터), 실제 네트워크(Tailscale/Wi-Fi) 지연·손실, 장시간 운용, 사람 회피 성공, 박스 접촉의 물리적 확인, LiDAR/Depth 개별 기여 분리, 장애물 위치·크기·속도의 다양한 조합, 다중 로봇, RMF Web 설치·실행([RMF_WEB_REVIEW.md](RMF_WEB_REVIEW.md)는 문서 조사만), 장애 주입 중 대시보드 화면의 실시간 관찰.

## 8. 회귀 검사 (같은 워크트리, 통합 변경 포함)

이 세션에서 통합 워크트리(Codex 미커밋 변경 + 영상 패널 변경을 겹친 것)로 실행했습니다.

| 검사 | 결과 | 시간 |
|---|---|---|
| `test_sim_motor_policy` | 통과 | 0 s |
| `test_dedupe` | 14/14 | 1 s |
| `test_auth_reload` | 통과 | 0 s |
| `test_recovery` | 통과 | 12 s |
| `test_cancel_results` | 통과 | 0 s |
| `test_agent_recovery` | 통과 | 0 s |
| `test_ownership` | 37/37 | 18 s |
| `tests/run_checks.py` | 46/46 | 30 s |
| `rmf_adapter/tests/test_runner_policy.py` | 33/33 | 9 s |
| `rmf_adapter/tests/run_rmf_checks.py` | **47/51 FAIL**(2회 동일) | 214 s |
| `rmf_adapter/tests/run_gazebo_checks.py` | **66/68, 65/69 FAIL**(2회, 실패 항목이 다름) | 284 s, 626 s |

- **이전 세션 보고(RMF E2E 59/59, Gazebo 69/69)는 이 세션에서 재현되지 않았습니다.** 같은 두 검사를 **수정하지 않은 Codex 워크트리**(`codex/rmf-gazebo-integration`, 통합 변경 없음)에서도 실행했을 때 RMF E2E는 같은 47/51로 실패했고 Gazebo 검사는 "Gazebo/AMCL not ready"(90 s 대기 초과)로 7/8에서 중단됐습니다. 따라서 이번 통합 변경이 원인이라는 증거는 없지만, **원인을 찾지 못했습니다**.
- RMF E2E 실패는 7번 구역(Agent 재시작 중 임무): 시험이 기대하는 `RECONCILIATION_REQUIRED` 시점에 임무가 이미 완료되어(RMF `completed`) 해당 항목들과 이어지는 복구 구역이 실패했습니다(타이밍 경쟁으로 추정, 미확인). Gazebo 검사 실패 항목은 Adapter 재시작 뒤 고아 임무, Agent 재시작/복구 구역(`CANCELING`), Nav2 목표 수 불일치(mock=17, acked=16) 등으로 실행마다 달랐습니다.
- 이 노트북에는 이전 날짜의 Nav2 프로세스 여러 벌이 계속 떠 있어 시간 민감한 시험에 부하를 줬을 수 있습니다(영향 크기 미측정).
- 통합 실행기(`run_integrated.py`)의 시나리오는 위 E2E 검사 코드를 재사용하지 않고 별도로 구현했으므로, 통합 결과(§1–§5)는 이 E2E 실패와 독립입니다. 그러나 같은 Agent 재시작/복구 경로를 통합 실행에서 재검증하지 않았습니다.
