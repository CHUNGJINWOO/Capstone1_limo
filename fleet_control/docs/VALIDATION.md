# 검증 요약 (2026-11-12)

이 문서는 **무엇을 어떤 환경에서 확인했는지**와 재현 방법만 적습니다. 원시 로그는 저장소에 넣지 않았습니다. 시뮬레이션(모의 Nav2) 결과를 실차 검증으로 읽지 마세요.

## 1. 환경 구분

| 구분 | 환경 | 이 문서의 결과 |
|---|---|---|
| 모의 환경 | 노트북, 모의 `NavigateToPose` 서버(직선 이동), 임시 데이터·토큰, 독립 포트·ROS 도메인 | 아래 §2–§4의 모든 수치 |
| 실차 | 실제 LIMO, 모터 잠금 상태, 사람 입회 | §5 (상태 수신만) |
| 운영 서버(8080) | 이전 버전 프로세스가 계속 실행 중 | 이번 검사에서 건드리지 않음. 최신 lease·복구 기능은 **적용되지 않음** |

## 2. 자동 검사 결과 (모의 환경, 최종 코드)

| 검사 | 결과 | 확인하는 것 |
|---|---:|---|
| `tests/run_checks.py` | 46/46 (수정 후 연속 6회) | 서버·대시보드 제공, 인증/역할, 모의 로봇 상태·지도, 임무 생성·취소·멱등성·대기열, 모터 잠금 인터록, 서버 재시작 후 보존·재연결, 오프라인 감지 (포트 18080, ROS 도메인 141) |
| `tests/test_dedupe.py` | 14/14 | 같은 `command_id`·`idempotency_key` 재처리, 종결 상태 고정, 다른 로봇의 ACK 거부 |
| `tests/test_auth_reload.py` | 2개 통과 | 토큰 폐기·권한 축소의 핫리로드, 같은 mtime의 원자 교체 감지 |
| `tests/test_recovery.py` | 14개 통과 | 실제 로컬 HTTP·SQLite·AgentCore와 제어 가능한 모의 핸들로 서버/Agent 재시작, 타임아웃, 실행·취소 중 단절, ACK 유실, 오래된·중복 명령, 소유권 불일치, 손상 저널 |
| `tests/test_cancel_results.py` | 14개 통과 | Nav2 취소 서비스 응답과 실제 terminal 결과 구분, 이미 끝난 목표의 취소(`NOTHING_TO_CANCEL`) |
| `tests/test_agent_recovery.py` | 2개 통과 | 운영자 복구 후 Agent가 **자체 Nav2 측정이 일치할 때만** 저널을 닫음 |
| `tests/test_ownership.py` | 37/37 | 제어권 lease(획득·갱신·만료·강제 해제), 소유자 외 생성 시 409, 웹 생성과 RMF lease 획득의 120회 경쟁(동시 성공 0), 증거 기반 운영자 복구 (포트 18083) |
| `rmf_adapter/tests/test_runner_policy.py` | 33/33 | Adapter 정책: 단일 활성 임무, 소유권, 오래된 경로 폐기, 취소 확인, 통신 장애, 고아 임무, lease 펜싱 |
| `rmf_adapter/tests/run_rmf_checks.py` | 59/59 ×3회 연속 | RMF E2E (§3) |

`git diff --check`는 통과했습니다(추적 중인 변경은 루트 `README.md`뿐이며, 새 파일은 공백 오류 검사를 별도로 수행).

### 알려진 불안정성

- `run_checks.py`는 이전에 **한 번 42/46**으로 실패했습니다. 실패한 4개 항목을 기록하지 못했고 **원인은 확인하지 못했습니다.** 이후 게시 준비 중 다른 한 번의 45/46은 지도 PNG가 상태보다 늦게 업로드되는 시험 경합으로 확인했고(서버가 `no_map` 404를 반환하는 75바이트 응답), 시험이 최대 20 s 폴링하도록 고쳤습니다. 수정 후 연속 6회 46/46이었습니다. 42/46이 같은 원인이었는지는 확인하지 못했으므로 **간헐적 문제가 남아 있을 수 있습니다.**
- `limo_msgs`가 없는 환경(저장소만 내려받은 경우)에서는 배터리가 `UNKNOWN`이어야 하며, `run_checks.py`는 `limo_msgs` 유무에 따라 `VOLTAGE_ONLY`/`UNKNOWN` 중 올바른 쪽을 기대하도록 고쳤습니다(백분율을 만들어 내지 않는다는 점은 두 경우 모두 검사).
- 이 수치는 모두 한 대의 노트북, 한 종류의 모의 환경에서 얻었습니다. 반복 횟수는 표에 적은 만큼이며 통계적 신뢰도를 주장하지 않습니다.

## 3. Open-RMF E2E (`run_rmf_checks.py`, 59개 검사)

환경: Open-RMF 2.1.8 격리 설치, 임시 Mission Manager(포트 18090), 모의 로봇 `pcu`, ROS 도메인 151(RMF)/152(로봇). 각 실행 약 2.5–3분. 성공 판정은 "RMF가 임무를 발행 → Mission Manager → 모의 로봇이 실제로 이동 → RMF가 `completed` 보고"를 모두 확인했을 때만 했습니다.

| 영역 | 확인한 것 |
|---|---|
| 지도 정합성 | 5개 waypoint가 Nav2 지도의 free 셀, 4개 lane이 장애물에서 0.2 m 이상, RMF↔Nav2 변환 왕복 오차 < 1e-9 m |
| 등록·임무 | `limo_fleet`/`pcu` 등록, RMF 위치 = 변환된 Nav2 위치, patrol 임무 입찰·배정, 경로 → Mission Manager 임무, 모의 로봇 이동, RMF `completed` |
| 취소 | 긴 구간 실행 중 RMF 취소 → 해당 임무가 로봇 쪽 확인 후 CANCELED, 확인 전 후속 임무 없음, 취소된 목표 재전송 없음, RMF `canceled` |
| 소유권 | RMF가 lease를 가진 동안 웹 생성은 409(대기·실행 중 모두), 임무 미생성. admin 인수 후 RMF 입찰은 `FAILED_TO_ASSIGN`, 웹 임무 종료 후 lease 자동 재획득 |
| 서버 중단 | 서버 강제 종료 후 약 1초 내 `offline` 보고, 단절 중 임무 제출 0, 재시작 후 작업 완료, 목표 중복 없음 |
| Adapter 재시작 | 이전 Adapter의 고아 임무가 종료된 뒤 재등록 |
| Agent 재시작 | `RECONCILIATION_REQUIRED`, 새 이동 임무 차단, 활성 목표 중 복구 거절 → 목표 종료 후 복구 수락(`FAILED`/`RECOVERED_UNVERIFIED_OUTCOME`) → 새 RMF 작업 완료 |
| 중복 실행 | 모의 Nav2가 실제로 받은 목표 수 = 서버가 확인한 navigate 명령 수, 같은 목표의 이중 전송 없음 |
| 정리 | 시험 종료 후 프로세스·포트 잔여 없음 |

Adapter 재시작 후 RMF 고아 작업(§6)은 `tests/repro_adapter_restart.py`로 3회 재현·분석했지만 해결하지 못했습니다.

## 4. 시험 중 발견·수정한 결함 (요약)

- 취소가 임무의 자연 완료와 겹치면 Agent가 취소를 `RECONCILIATION_REQUIRED`로 응답해 이후 모든 목표가 영구 차단됨 → Agent가 직접 종료를 관측한 목표는 `NOTHING_TO_CANCEL`로 처리.
- 서버에서 복구해도 Agent 저널이 새 목표를 거부 → 서버가 복구된 명령 ID를 알리고 Agent가 자체 측정이 일치할 때만 저널을 닫음.
- RMF 입찰 시 1초 캐시된 소유권 확인의 경쟁 → 입찰마다 서버를 조회.
- 토큰 핫리로드가 mtime만 비교해 같은 시각의 원자 교체를 놓침 → inode·ns 시각 비교.
- Nav2 cancel 서비스 응답만으로 취소 완료로 본 문제 → 실제 terminal `STATUS_CANCELED` 확인.

## 5. 실차(LIMO) 검증 범위

2026-11-12, 모터 잠금 상태(`allow_motor_drive` 기본값 false), 사람이 입회한 상태에서 Agent(당시 이전 버전)를 짧게 실행했습니다. Tailscale을 통해 서버가 받은 것:

- 위치(`map` 프레임, TF `map→base_footprint`), 지도, Nav2 상태, LiDAR 상태, 배터리 **전압**(백분율은 보고하지 않음), Safety Gate 상태, 모터 경로 LOCKED(`/cmd_vel` 발행자 없음).
- Depth 카메라는 그 시험에서 꺼져 있어 `NO_DATA`였습니다.
- 임무는 보내지 않았고(`allow_missions=false`) 모터를 구동하지 않았으며 팀 워크스페이스를 수정하지 않았습니다.

실차에서 **검증하지 않은 것**: 임무 실행, Open-RMF를 통한 이동, 서버 재시작 뒤 실차 Agent의 재연결, 장시간 통신 운용, Depth 카메라 상태, 현재 소스 버전의 Agent.

## 6. 미검증·남은 문제

- Gazebo와 Open-RMF 통합, Open-RMF 실차 이동, 다중 로봇 운용·교통 협상, 실제 장시간 통신 운용.
- RMF 고아 작업: Adapter가 재시작되면 이전 작업은 중복 실행되지 않지만 RMF에서 `underway`로 남고 취소에도 응답하지 않습니다.
- 운영자 복구는 활성 Nav2 목표가 스스로 끝나야 가능합니다(Agent가 핸들을 잃은 목표를 취소하는 명령 없음).
- 운영 8080 서버와 실차 Agent는 이전 버전입니다. lease·복구 프로토콜을 적용하려면 서버와 Agent를 함께 계획적으로 업데이트해야 합니다(데이터·저널 백업 필요).
- 통신 장애는 이미 보낸 Nav2 목표의 물리적 정지를 보장하지 않습니다.

## 7. 재현 방법

[README의 테스트 실행](../README.md#테스트-실행)을 따릅니다. 모든 시험은 임시 디렉터리, 독립 포트·ROS 도메인에서 돌며 8080과 실차를 건드리지 않습니다. RMF E2E는 `scripts/install_rmf_isolated.sh`로 만든 격리 환경이 필요합니다.
