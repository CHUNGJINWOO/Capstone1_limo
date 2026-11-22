# 수동 세션 실패 사례 및 독립 수정 후보 (2026-11-20)

상태: **실차 통합 차단 유지**. 이 문서와 `manual_session_safety_candidate_20261120.patch`는 독립 검토 산출물이다. 후보는 공통 소스에 적용하지 않았다. Claude 작업 종료 후 실제 파일을 다시 대조해야 한다. 기준은 앞선 감사의 staging 및 public이며 staging 공통 파일은 Git 미추적으로 Git diff만으로 확인할 수 없다. 기존 로그는 변경하지 않았다.

## 실패, 원인, 수정 위치, 회귀시험 수용 기준

| 사례 | 기존 결과와 재현 조건 | 원인과 필요한 수정 위치 | PASS 기준 |
|---|---|---|---|
| Mapping | staging FAIL, public에서는 열거 상태 통과. STARTING/MAPPING/PAUSING/PAUSED/SAVING/STOPPING/UNKNOWN에 start | staging `service.py`의 `_manual_bench_robot`에 mapping 검사 없음. `mapping.py`와 Agent 로컬 제어권을 같은 모드 중재에 연결 | 서버와 Agent 각각 모든 비-IDLE·미확인 상태에서 start/sample/pull 거부·세션 폐기. 동시에 Mapping 시작 요청해도 둘 다 승인되지 않음 |
| 활성 Nav2 Goal | staging FAIL, public의 단일 합성 상태는 통과. active_count=1 | staging `service.py` admission에 외부 Goal 검사 없음. Agent 및 Watchdog도 NavigateToPose·FollowPath 전체의 신뢰 가능한 종료 증거 필요 | 두 action 각각 외부 목표 활성/취소 중/결과 미확인에서 거부; 종료 결과의 UUID·서버 세대 확인 및 원자적 제어권 이양 전까지 거부 |
| Nav2 배열 미수신 | staging/public 각 FAIL. `received_status_array=false` | 각 버전 `service.py`의 admission이 관측 여부를 검증하지 않음; Agent 증거와 독립 Watchdog도 보완 필요 | 미수신·누락·잘못된 형식은 명시적 UNKNOWN이고 0과 구별, 서버/Agent/Watchdog 각각 거부 |
| Nav2 오래된 상태 | staging/public 각 FAIL. 합성 `age_s=30` | `service.py`가 샘플의 경과 시간을 반영하지 않음; public은 다른 필드 `last_change_age_s`를 사용하므로 해당 시험을 실제 통신 지연 관측으로 해석하지 않음 | 신뢰 가능한 관측 세대와 수신 단조시각을 결합, 예산 초과·음수·비유한·시계 불확실성 시 거부; 실제 프로듀서 필드로 재현 |
| Nav2 publisher 중복 | staging FAIL; public 합성 시험 통과. publisher=2 | staging `service.py`에서 유일성 미검사; Agent 및 watchdog 각 action graph와 상태 생산자 검증 필요 | 0개, 2개 이상, 서버 교체, graph 불확실 시 권한 폐기. 두 action 각각 시험 |
| 잘못된 Goal 수 | staging FAIL; public 합성 시험 통과. `active_count='error'` | staging `service.py`에서 타입/범위 미검사 | bool·문자열·음수·누락 등 무효값을 UNKNOWN으로 취급해 거부 |
| Offline 후 재사용 | staging FAIL; public의 명시 Offline 거부 후 시험 통과. 250ms 안에 online/재조정이 복원된 모의 조건에서 이전 session/sequence 재전송 | staging `service.py`의 readiness 거부는 예외만 발생시키고 `manual_benches`를 지우지 않음; hello/housekeeping의 일반 movement 무효화도 그 세션을 지우지 않음 | 같은 boot·lease epoch로 즉시 복구해도 옛 session/ticket/seq 거부, 새 session과 새 무장만 가능. 무요청 단절·짧은 단절·lease 만료/이전·양쪽 재시작도 독립 시험 |

추가 독립 FAIL: staging `motion_watchdog.py`의 `idle_proven`은 `nav_active in (None, 0)`을 허용한다. publisher 1개와 합성 빈 cancel 응답만 있을 때 배열 미수신인데 AUTHORIZED로 전환한 기존 재현은 [보존 로그](../../local_data/raw_results/manual_session_watchdog_missing_status_v2_20261120.log)에 있다. 기존 7+2 집계와 별도다. 빈 cancel 답변이나 빈 배열만으로 과거 전달 Goal의 실제 종료를 증명할 수 없다.

## 안전 계약

* 서버와 Agent는 각각 인증된 현재 서버 boot, Agent boot, 연결 세대, 로봇, owner, lease epoch, 제어 모드 세대에 수동 session을 묶는다. ticket이 있는 계약은 ticket을 같은 세대에 결박해 단회 사용하고, session별 sequence는 단조 증가한다. staging의 현재 bench에는 ticket이 없다. 이전 인증정보가 형식상 유효해도 현재 세대와 다르면 거부한다. 새 boot 또는 lease만으로 이전 명령을 재생하지 않는다.
* Offline, stale/UNKNOWN, Agent 또는 서버 재시작, reconnect, lease 만료·양도·세대 변경, 안전 증거 상실, 모드 변경, 입력 만료·Deadman 해제 관찰 시 해당 측은 session·ticket·남은 sample을 즉시 폐기하고 0을 요구한다. 같은 boot/epoch 복구도 새 session·명시적 재무장 필요. 무단 사용자의 잘못된 요청이 다른 사용자의 정상 session을 임의로 취소하지 않게 소유권 검사와 실제 상태 변화에 따른 폐기를 구분한다. 브라우저 중단 시에도 로봇 독립 watchdog과 Gate가 시간 초과로 0을 강제해야 한다.
* `UNKNOWN`(미수신, 지연, 오류, publisher 불명, action 서버 세대 변경)과 유효하게 관측한 `IDLE`은 다르다. NavigateToPose 및 FollowPath 각각에 대해 신뢰 가능한 생산자, action 서버 세대, 활성/전달된 모든 UUID의 종결 결과, 수신 단조시간, 미해결 취소·전달 여부를 확인한다. cancel 요청·빈 응답·빈 배열은 단독 종결 증거가 아니다. 서버의 API 직렬화만으로 외부 ROS action의 원자성을 보장할 수 없으므로 Agent 현장 중재와 외부 goal 차단/감시를 결합하고, 그 보장이 없으면 MANUAL 및 Watchdog 승인을 거부한다.
* Mapping/Fleet Navigation/Manual 중 하나만 같은 로봇의 권한을 가진다. 서버는 인증·lease와 DB/모드 세대 검사 및 전이를 동일 직렬화 경계에서 처리하고, Agent는 자신의 실행기·Mapping·두 Nav2 action·연결/증거 신선도를 독립 확인한다. 전환 중과 상태 UNKNOWN은 어느 쪽의 비영속도 명령도 승인하지 않는다. 이 계약의 실 구현 및 외부 클라이언트와의 원자성은 아직 입증되지 않았다.

## 독립 후보의 범위와 한계

패치는 staging의 `service.py`에 readiness 오류, mission busy, 받아들인 hello, housekeeping의 offline 관측, 만료·변경된 lease/pull 시 세션 폐기를 제안하고, `motion_watchdog.py`의 미수신을 0으로 취급하는 조건만 제거한다. 잠재적으로 운영에 영향을 주는 **부분 후보**다. 실제 공통 파일에 적용해서는 안 된다. 기존 public 경로와 ticket, boot ID가 다른 sample의 복구, Agent에서의 독립 폐기, mode 경쟁, 상태 신선도 및 action 종결 증거는 해결하지 않는다. 상태 배열을 받은 것만으로 모든 goal의 종료가 증명되는 것도 아니다. Watchdog predicate 수정만으로는 stale 상태나 FollowPath를 보호하지 않는다. 적용 여부는 전체 설계·동작·회귀시험을 끝낸 후 결정한다.

복사본 테스트: 모의 서버에서 Offline 복귀 후 이전 session 거부, 두 브라우저 경쟁, 이전 session/epoch/sequence, Deadman/TTL 네 테스트 PASS. 단 250ms 실시간 분산 재연·ticket·완전한 프로세스 경로는 아니다. [로그](../../local_data/raw_results/manual_session_safety_candidate_partial_v2_20261120.log). 격리 ROS 노드 시험은 환경의 네트워크 인터페이스/소켓 제한으로 실행되지 않았고 domain 237 포트 오류도 발생했다. 최초 실행의 Python 경로 오류 및 재시도 오류를 로그로 남겼다. 따라서 ROS 노드 결과는 **SKIP/환경 차단**, PASS가 아니다. 별도 Python의 실제 후보 `idle_proven` 함수만 호출한 시험에서 미수신 거부, 합성 0 허용, 활성·만료 거부를 관찰했다. 이는 ROS 노드나 신뢰된 종료 증거 시험이 아니다. [predicate 로그](../../local_data/raw_results/manual_session_safety_candidate_predicate_20261120.log).

## 통합 순서와 미완료 조건

1. Claude 작업 종료 후 현재 파일/작업트리와 patch의 기준 내용을 직접 재비교한다. `service.py`, `motion_watchdog.py`는 후보와 겹치며, `ownership.py`, `mapping.py`, `agent_node.py`, `manual_web_bench.py`, Agent adapter 및 UI도 통합 변경 충돌 가능성이 있다. 공통 문서는 현재 변경하지 않았고, 통합 확정 후 ARCHITECTURE/CURRENT_STATUS/HANDOFF에 계약, 차단 및 시험 범위를 반영한다.
2. 단일 수동 API와 인증·ticket·boot/epoch/sequence 계약을 합의한다. 서버·Agent·Watchdog의 상호 독립 mode/goal 증거와 원자성, 무응답 시 폐기 및 잠금 출력을 구현한다. candidate patch는 이 과정의 비교 재료일 뿐이다.
3. 위 표 전체, 실제 HTTP/Agent/독립 Watchdog/Gate 경로의 두 브라우저 경쟁·서버/Agent 재시작·재연결·stale/역순·모든 외부 action 및 Mapping 경합·Deadman/입력 만료를 독립 환경에서 재검증한다. Gate 잠금 상태에서 실제 `/cmd_vel` 발행자 0개를 확인하고 모터 경로 비활성 상태를 유지한다. 실제 ROS 실행 및 모바일/UI 시험은 별도로 기록한다.
4. **기존 staging FAIL 7, public FAIL 2 및 Watchdog 추가 FAIL은 실제 코드가 수정·검증되기 전까지 모두 실차 통합 차단 조건으로 유지한다.** 하드웨어 안전과 독립 watchdog/Gate 전 경로 증명이 없으며 실차 수동 구동은 허용되지 않는다.
