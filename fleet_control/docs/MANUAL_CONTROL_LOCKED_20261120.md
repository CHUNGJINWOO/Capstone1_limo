# 18190 수동 입력 패널 — 모터 잠금 후보 (2026-11-20)

## 범위와 현재 배포 상태

이 후보는 독립적인 `static/manual_control_panel.js`·`manual_control_panel.css`다. Claude가 같은 시각 웹 Mapping/Registry/공통 화면을 변경하므로 `index.html`, `app.js`, `operations_panel.js`, `service.py`, `ownership.py`, Agent 또는 실제 launch를 수정하지 않았다. **현재 18190 화면에는 이 후보가 연결되어 있지 않다.** 기존 `app.js`의 Gamepad 벤치 패널도 유지된다. 기존의 `manual-bench` 경로와 경쟁하지 않도록 두 패널을 함께 활성화하지 않는다.

로컬 staging의 Gamepad 코드는 `app.js` 내부 closure여서 직접 재사용 가능한 별도 `gamepad_panel.js`가 없었다. origin/main에는 Gamepad 패널과 잠금 전용 `velocity_arbiter.py`가 있으나 현재 staging에는 없고, origin/main 벤치 프로토콜은 ticket 등 계약도 다르다. 이번 후보는 staging 서버의 `POST /robots/{id}/manual-bench` start/sample/stop 및 기존 Agent의 요청값 계산·영속도 벤치를 재사용한다. Gamepad의 표준 왼쪽 스틱과 버튼 4 읽기만 독립 구현했다. **이 코드는 velocity arbiter로 연결된 실차 이동 경로가 아니며, 서버/Agent 기능이 추가되기 전에는 사용성 시험 후보로만 취급한다.**

## 입력 및 종료 정책

- 키보드 W/위·S/아래, A/왼쪽·D/오른쪽 + 별도 Space Deadman. INPUT/TEXTAREA/SELECT/contenteditable 포커스 시 키 명령을 받지 않고 진행 중이면 세션을 중지한다. 시작 전에 누른 키의 반복 이벤트는 새 누름으로 취급하지 않는다.
- 마우스/터치 가상 조이스틱은 포인터 누름이 Deadman이다. pointer capture 실패, 해제, 취소, 캡처 손실 시 세션을 중지한다. 게임패드는 장치 하나, 왼쪽 두 축, L1(버튼 4)을 사용하며 해제·분리·입력 갱신 정체 시 중지한다. 장치 변경, 패널 비활성, 숨김, blur, pagehide, offline, API 오류, 서버/Agent boot ID·Lease epoch 변경에서도 중지하고 자동 재개하지 않는다.
- 표시는 정규화 입력을 기존 벤치의 최대 0.10 m/s 및 0.30 rad/s 요청으로 환산한 값이며, 실제 속도나 모터 출력이 아니다. 후진 요청은 벤치에서 0으로 제한된다. 모든 좌표/축은 유한한 [-1,1]인지 검사한다.
- 80ms 간격으로 기존 세션에 증가하는 시퀀스를 보내며 요청 타임아웃은 180ms다. 서버 기존 sample TTL 250ms와 Agent의 독립 만료 처리는 브라우저 종료 시 마지막 방어선이다. 정지 요청은 가능한 한 기존 stop API로 전달하며 실패 시 TTL 만료에 의존한다. 소프트웨어 정지는 물리 E-Stop이 아니다.

## 공통 화면 통합 제안 — 아직 적용하지 않음

1. 담당자가 편집을 마친 뒤 `index.html`에 `/manual_control_panel.css`·`/manual_control_panel.js`와 별도의 패널 호스트를 추가한다. `app.js`의 기존 `api(path, options)`를 `request`로 전달한다. 기존 Gamepad 벤치 패널의 세션은 새 패널 시작 전에 종료하고, 양쪽 동시 사용을 차단하거나 기존 패널을 하나로 교체한다. 새 서버나 별도 ROS 프로토콜을 추가하지 않는다.
2. `ManualControlPanel.mount(host, {request: api, getContext, isActive})`를 호출한다. `isActive`는 선택된 수동 조작 패널이 실제 표시 중인 경우만 true다. 로봇 변경·로그아웃·탭 전환에서는 `panel.stop()`을 호출하고, 패널 폐기 시 `panel.destroy()`를 호출한다.
3. `getContext()`는 아래 모든 값을 **현재의 검증된 서버/Agent 증거**에서 계산한다. 하나라도 불명확하면 false/UNKNOWN/null이어야 한다. 브라우저가 임의로 true나 IDLE을 채워서는 안 된다. 폴링 응답의 `state_age_s`에는 브라우저가 응답을 받은 후 경과한 단조 시각을 더한다. Agent 재연결/서버 재시작의 식별자는 응답이 새로 확인되기 전에는 불명으로 취급한다.

```text
{robotId, canControl, online, agentReady, motorLocked,
 leaseOwned, leaseEpoch, mode: IDLE|AUTO|MAPPING|UNKNOWN,
 navIdleProven, gateNormal, evidenceAgeMs, agentBootId, serverBootId}
```

`navIdleProven`은 현재 Goal 취소 요청이나 빈 UI 목록이 아니라, NavigateToPose·NavigateThroughPoses·FollowPath·Spin·BackUp의 신선한 상태와 단일 신뢰 가능한 발행자, 모든 활성 Goal의 종결/정리 증거가 갖춰졌을 때만 true여야 한다. Mapping이 실행 중이거나 세션 종료 증거가 불확실하면 mode는 MAPPING/UNKNOWN이다. `motorLocked`는 실제 모터 격리 증거와 Gate 경로가 확인된 경우에만 true다. 현재의 일반 대시보드 값만으로 이를 모두 만족한다고 가정할 수 없으므로 **검증된 증거 생산·전달이 추가되기 전에는 입력을 비활성으로 유지한다.**
4. 서버/Agent 담당자와 별도 통합: 서버와 Agent가 Lease·유효 Agent·Mapping 미실행·Nav2 모든 Goal 종결 증거·중복 입력·안전 Gate와 모터 잠금을 독립적으로 확인하고, 재시작 세션 fencing 및 입력 TTL을 강제해야 한다. 웹이 검증됐다고 해도 서버 측 차단을 대신하지 않는다. 실제 움직임용 경로는 검증된 arbiter → 독립 watchdog → Safety Gate → 모터로만 구성한다.

## 검증 기록

| 범위 | 결과 | 근거와 한계 |
|---|---|---|
| 격리 브라우저·모의 request | PASS | [브라우저 로그](../../local_data/raw_results/manual_control_browser_20261120.log): 키보드/Deadman, 텍스트 포커스, 마우스, 모의 Gamepad, CDP 터치, 375px 폭, 모드·세션·연결 변경 차단. 실물 휴대폰·패드 아님. |
| Fleet 서비스 메서드·모의 DB | 부분 PASS / 차단 발견 | [로그](../../local_data/raw_results/manual_control_server_20261120.log): Lease, 동시 세션, 순서·NaN/Inf, 만료, 연결·세션 변경. `test_known_gap_*`의 OK는 **안전 PASS가 아니라 기존 서버가 Mapping/외부 Nav2 증거 없이 시작을 수락하는 결함의 재현**이다. 서버 프로세스·HTTP 재시작 E2E는 아님. |
| 격리 ROS 216·합성 잠금 Gate | PASS | [로그](../../local_data/raw_results/manual_control_locked_ros_20261120.log): 실제 `ManualWebBench.tick`이 계산한 요청 0.1/0.15, Deadman 해제·만료·Gate 오류의 요청 0, raw 및 locked 출력 전부 0, `/cmd_vel` 발행자 0. Gate는 합성 노드다. |
| 격리 ROS 204·Fleet HTTP·Agent·실제 Gate 코드 | PASS | [로그](../../local_data/raw_results/manual_control_isolated_20261120/bench_http_ros.log). 기존 실험 스크립트를 새 결과 경로에 복사해 실행했다. 기존 서버와 Agent, 모터 잠금으로 remap된 실제 Gate 코드, 합성 LiDAR/odom 사용. viewer·다른 Lease 소유자·두 번째 세션 거부; 전진/회전 요청, 후진 제한, Deadman·입력 만료, odom 중단 및 중복 속도 발행자 확인. `/cmd_vel_raw` 75개 및 `/cmd_vel_locked` 370개 모두 영속도, `/cmd_vel` 발행자 0. 새 브라우저 패널과 실제 HTTP의 결합 시험은 아니다. |
| 기존 Joy ROS 회귀 | PASS | [로그](../../local_data/raw_results/manual_control_joy_ros_20261120.log), 격리 217. 모의 SLAM과 `_motion_permitted` 대체, 실제 Gate·모터 아님. |
| 기존 필터 및 수동 승인 증거 단위 시험 | PASS | [필터](../../local_data/raw_results/manual_control_joystick_20261120.log) 5/5; [Guard](../../local_data/raw_results/manual_control_guard_20261120.log) 1/1. Guard는 SIM-only 실제 클래스의 모의 증거 시험이다. |
| 실제 18190 통합 브라우저, 브라우저→HTTP→Agent→arbiter→Gate | SKIP | 공통 UI 충돌 및 staging arbiter/서버 안전 승인 미완성. |
| Gazebo, 실차 모터 잠금, 실차 수동 주행 | SKIP | 이번 작업에서 실행하지 않았다. |

## 주행 전 차단 조건

현 staging의 `manual_web_bench`는 `real` Agent에서도 입력 요청을 계산하지만 `/cmd_vel_raw`로 영속도만 발행한다. `/cmd_vel` 발행자 존재, Gate 그래프 불일치, 다른 속도 발행자, 오래된 Gate/odom 등에서 차단한다. 지금 상태는 조작기 입력 시험이지 검증된 MANUAL 모드 전환·실차 주행 기능이 아니다. 서버가 Mapping/외부 Nav2 Goal 증거를 강제하지 않는 문제, Agent 측 독립 검증과 arbiter/Watchdog 실제 연결, 실물 E-Stop·watchdog·정지 거리·통신 단절 시험을 해결하기 전에는 모터를 활성화하지 않는다. 실물 E-Stop과 배터리/공간 확인은 현장 운영자 책임으로 별도 확인한다.
