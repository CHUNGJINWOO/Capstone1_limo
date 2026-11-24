# Watchdog 속도 Clamp (실차 적용 지점, G2 2026-11-23, 격리 ROS 검증, 미커밋)

**실차 검증 없음.** 모든 시험은 mock 드라이버·격리 ROS·합성 센서다. 실차의 모터 잠금은 그대로다(기본 `allow_motor_drive:=false`).

## 1. 변경 범위 (R2 Watchdog)
- 새 파일 `speed_clamp.py`(순수 로직), `watchdog_speed_link.py`(Agent 쪽), 기존 `motion_watchdog.py`에는 작은 연결부만 추가했다. **기본 꺼짐**(`speed_clamp:=false`)이라 켜지 않으면 R2 동작은 변하지 않는다(R2 66개 시험 통과).
- 켜는 곳은 실차 launch의 Watchdog 노드뿐이다(`speed_clamp: True`, `speed_limit_state_path: ~/.config/limo_fleet/watchdog_speed_limits.json`).
- 보존한 것: 상태기계(LOCKED/AUTHORIZED/FAILSAFE), 단일 출력 잠금 `_publish_lock`, 정지 우선. **clamp는 `_publish_authorized`(상태가 AUTHORIZED이고 종료·정지 요청이 없고 입력이 단일일 때만 도달하는 유일한 비영 발행 지점) 안에서만** 적용된다. 영(0) 발행, LOCKED, FAILSAFE, 종료·STOP 경로는 clamp를 거치지 않는다. 비유한 입력은 기존대로 0.
- 추가한 상태기계 조건 하나: clamp가 켜진 경우 Agent의 `/fleet/speed_limit` 동기화(1.5 s 이내, 같은 Agent boot, 허용된 단일 게시자)가 없으면 `SPEED_LIMIT_NOT_SYNCED`(LOCKED 유지, 운동 중이면 FAILSAFE). 이 기능을 모르는 구버전 Agent면 로봇은 움직이지 않는다(fail-closed).

## 2. 정적 ceiling과 동적 승인 한도의 분리
- 정적 ceiling(파라미터 기본값 = 기존 프로파일): manual 전진 0.10·후진 0.10·각 0.30, auto 전진 0.15·후진 0.10·각 0.50. clamp가 켜져 있으면 항상 적용.
- 동적 한도는 항상 정적 ceiling 이하이고 **낮추는 값만** 있다. 높이는 것은 자동으로 일어나지 않는다.
- scope는 입력 게시자로 결정: `velocity_smoother`→auto, `manual_mapping_teleop`→manual, Agent 자신(mock의 Arbiter 출력)→정적 ceiling만(Arbiter가 scope별로 이미 자른다).
- 실차 launch의 기본 속도 프로파일(`normal`)은 Nav2 목표 속도가 0.15보다 높을 수 있다. 이제 Watchdog이 0.15로 자른다(이전에는 Gate 입력 검증의 0.20 m/s 초과로 Gate가 결함 처리했을 것). 의도된 동작이지만 `normal` 프로파일의 주행 특성이 바뀐다.

## 3. 승인·갱신 규칙 (Watchdog 쪽)
- 입력: `/fleet/speed_limit`(JSON). 허용 조건: 그 토픽의 게시자가 **메시지를 판단하는 순간에** 정확히 1개이고 이름이 `fleet_robot_agent`, 메시지의 `agent_boot_id`가 Watchdog이 보는 heartbeat의 boot와 같음, 값이 유한·bool 아님·최소 이상·정적 ceiling 이하, generation이 정수.
- 낮추는 변경은 즉시 수락. **높이는 변경은 같은 메시지를 0.15 s 이상 간격으로 두 번** 받아야 하고 두 번 모두 게시자가 1개여야 한다. 그래프 갱신 사이에 끼어든 두 번째 게시자가 메시지 한 번으로 높이지 못하게 하려는 것이다(하네스 시험 C4에서 첫 구현은 이를 놓쳤고 수정했다). 같은 binding(서버 boot, Lease 에포크, 계약 세대)에서는 더 높은 generation이거나 낮추는 값이어야 하고, 새 binding은 새 Lease 아래 Agent가 검증한 새 요청으로 본다.
- 메시지가 끊기거나 늦어도 마지막 수락 값을 **풀지 않는다**(`stale`로만 표시). 다른 게시자가 생기면 동기화가 깨져 영(0)이 된다.
- 재시작: 저장 파일(원자 교체)을 **낮은 값 그대로 stale·미확인**으로 읽는다(`restored_from_file_unconfirmed`). 같은 generation으로는 복원 값을 올릴 수 없다. 파일이 없으면 정적 ceiling(한 번도 낮춘 적 없는 경우), 읽을 수 없거나 범위 밖이면 SAFE_FALLBACK(0.05 m/s, 0.15 rad/s). 재시작 직후 Agent 동기화가 올 때까지 Watchdog은 LOCKED라 비영 명령을 내지 않는다.

## 4. Agent 쪽과 서버 계약
- `WatchdogSpeedLink`(5 Hz): 서버 요청을 limiter에 넣고, 현재 Lease·서버 boot·계약 세대에 묶인 승인 값만 `/fleet/speed_limit`로 보낸다(stale 값은 다시 보내지 않음). 확인 대기 중인 후보 값도 보낸다.
- `enforced_by = WATCHDOG`은 Watchdog 상태(최신 1.5 s, 같은 Agent boot, clamp 켜짐·동기화·게시자 정상·결함 없음)가 그럴 때만. 아니면 `null`.
- 적용 확인: Watchdog 상태의 `speed_clamp.applied[scope]`가 같은 generation·request_id·값이고 `stale:false`일 때만 Agent가 `APPLIED`로 올린다. 서버의 `approved=true`는 그 뒤에만. 확인 전에는 `PENDING_ENFORCEMENT`(서버에서는 `PENDING`).
- 서버: `enforced_by`가 `ARBITER` 또는 `WATCHDOG`이면 요청을 받는다. Agent의 `PENDING_ENFORCEMENT`는 거부로 취급하지 않고 20 s 후 `EXPIRED`.

## 5. 가감속·곡률 (격리 ROS 측정)
- **가감속**: clamp는 가감속 제한을 추가하지 않는다. 0 → 0.30 m/s 입력에서 Gate 출력의 첫 비영 샘플이 바로 0.15 m/s였고 연속 샘플 간 최대 변화가 0.150 m/s였다(계단). 경로상 가감속 제한은 Nav2 `velocity_smoother`의 기본 `max_accel`(2.5 m/s²)뿐이다. 기존에 없던 가감속 안전 성능을 검증된 것으로 주장하지 않는다.
- **곡률**: 한 축만 포화되면 곡률이 바뀐다. 입력 (0.30 m/s, 0.30 rad/s)는 ω/v = 1.0 1/m였고 출력 (0.15, 0.30)은 2.0 1/m로 곡률이 두 배가 된다(비례 축소 없음). 포화 상태에서 경로 추종이 어떻게 변하는지는 측정하지 않았다.

## 6. 검증 (격리 ROS·mock, 실차 아님)
- 단위: Watchdog clamp 15, Agent link 7, Agent limiter·Arbiter 18, 서버 API 47(Watchdog 확인 시나리오 포함).
- 실제 launch 추출 하네스 `real_launch_chain_faults_20261123.py` **42/42**(기존 22 + clamp 20):
  - 양성 대조: 0.30 입력이 0.15로 흐름.
  - 한도 0.06에서 출력 0.060.
  - `PENDING_ENFORCEMENT` → `APPLIED` 순서.
  - 상한 초과 요청 거부.
  - 두 번째 `/fleet/speed_limit` 게시자가 생기면 Watchdog 정지, 사라지면 복구되고 0.06 유지.
  - Watchdog 재시작 시 Agent 동기화 전 비영 출력 0, 복원값 0.06(stale), 재동기화 후 0.06 이하.
  - Agent 재시작(새 boot)에서 0.06 유지.
  - Lease 변경 후 자동 복원 없음, 새 요청으로 0.12.
  - 0.25 s마다 한도를 바꾸며 명령을 흘려도 상한 초과 0(302샘플).
- 회귀: R2 66(22+4+23+12+3+2) OK, R1·Gate·Arbiter·mock clamp·bench·cancel·계약·ownership 45 등 70개 파일 OK, mock Arbiter clamp 프로브 18/18, 체인 프로브 29/29.
- 구현 중 발견·수정한 결함 3건:
  1. 링크를 다시 만들 때 이전 publisher가 남아 Watchdog이 게시자 2개로 보고 동기화를 거부했다(`close()`가 publisher를 제거하도록 수정).
  2. 그래프 갱신 사이에 끼어든 두 번째 게시자의 한 번짜리 메시지가 높이는 변경을 통과했다(판단 시점의 그래프 재조회 + 높임은 두 번 필요).
  3. 일부 R2 시험이 `supervisor_fault`를 빈 namespace에 호출해 새 속성에서 실패했다(`getattr`).

## 7. 실차 전 남은 차단 조건 (이 작업으로 해소되지 않음)
- `/limo_driver/cmd_vel` 직접 발행 우회: Gate는 감지만 하고, Watchdog clamp는 그 경로를 거치지 않는다.
- vendor `limo_base`의 입력 리매핑 실확인, STM32 명령 타임아웃, 물리 E-Stop, 제동거리.
- 실제 Watchdog·Agent·Nav2가 실차 하드웨어에서 같은 순서로 동기화되는지.
- `normal` 프로파일에서 clamp된 Nav2의 주행 특성.
- DDS 인증(SROS2 없음): 그래프 식별만 한다.
