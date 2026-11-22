# 신규 DDS 프로세스가 기존 스택의 데이터를 못 받는 문제 — 원인 분석 (2026-11-20)

운영 중인 Robot Agent·18190·실차 스택은 중단·변경하지 않았다. 실험은 구독 전용 프로브(읽기 전용), 로봇 위 임시 프로세스, 노트북 격리 도메인(92·93)만 사용했다. 근본 원인은 **미확정**이다. 확정된 사실과 가설 결과를 구분해 적는다.

## 1. 확정된 사실 (로그: `local_data/raw_results/dds_rootcause_20261120/`)
1. **증상은 `/map` 전용이 아니다.** 단독으로 기동한 신규 프로세스는 `/map`·`/scan`·`/odom`·`/tf`·`/tf_static` **모두 0건**(5/5, `trials_1_alone_vs_warm3.txt`). 발행자 엔드포인트는 0.1~2 s 안에 보인다(Discovery는 동작). 노드 이름은 `_NODE_NAME_UNKNOWN_`.
2. 임시 참여자를 3 s 먼저 띄우면 같은 프로브가 **5/5 전부 수신**(`/map` 9, `/scan` ~78, `/odom` ~390, `/tf` ~600).
3. **원인 단서: 특정 participant ID 하나.** 신규 프로세스는 가장 낮은 빈 ID를 받는다. 현재 빈 ID는 23(UDP 26706/26707)이고, 단독 프로브는 항상 ID 23을 받는다(`ports_alone_vs_warm.txt`). XML 프로파일로 ID를 강제하면: **ID 23 → 0건 (4/4)**, ID 26·27·28~33·40·60·100 → 모두 정상 수신(`forced_participant_id.txt`, `forced_id_sweep.txt`). 즉 "단독 기동이 안 된다"가 아니라 "ID 23 슬롯이 먹통"이다.
4. **전송 계층과 무관.** ID 23은 UDPv4 전용·SHM 전용 모두 0건, ID 26은 둘 다 정상(`forced_id23_transports.txt`). SHM 포트 파일 `fastrtps_port26707`은 존재하지 않는다(`shm_port_files.txt`). ID 23 소켓은 단일(포트당 1개, 수신 큐 0)(`sockets_id23_vs_id27.txt`).
5. **warm-up이 효과 있는 이유(확정):** 임시 참여자가 먼저 ID 23을 차지하고, 실제 프로세스는 다음 빈 ID(26)를 받는다. 포트 캡처로 직접 확인(alone→26706/7, 헬퍼 뒤→26712/13).
6. 로봇 실차 스택 프로세스와 Agent의 RMW 환경(`RMW_IMPLEMENTATION`, `ROS_DOMAIN_ID=77`, `ROS_LOCALHOST_ONLY=1`)은 동일했고 FastDDS XML/환경변수는 없다. `lo`에는 멀티캐스트 239.255.0.1이 있다.

## 2. 가설별 결과
| 가설 | 결과 | 근거 |
|---|---|---|
| slam_toolbox 래치 `/map` QoS 문제 | FAIL | `/scan`(best-effort)·`/odom`·`/tf`도 0건 |
| QoS 비호환 | FAIL | 같은 QoS가 다른 ID에서 정상 |
| RMW/환경 설정 차이 | FAIL | 환경 동일, 다른 ID에서 정상 |
| Agent 코드·`map_save_enabled`·executor 순서 | FAIL | 플래그 없이도, 별도 진단 스크립트도 재현 |
| SHM/UDP 전송 문제 | FAIL | UDP 전용·SHM 전용 모두 ID 23만 실패 |
| 오래된 SHM 포트 파일 | FAIL | ID 23 포트 파일 없음, UDP 전용도 실패 |
| 포트 충돌(다른 소켓) | FAIL | 포트당 소켓 1개 |
| 멀티캐스트(lo) 설정 | FAIL | 다른 ID는 같은 설정에서 정상 |
| **ID 23 슬롯 고유 문제** | **PASS(확정)** | 강제 ID 시험 4/4 실패, 다른 ID 12종 정상 |
| **warm-up = 슬롯 점유 효과** | **PASS(확정)** | 포트 캡처 |
| SIGKILL된 이전 ID 23 보유자가 장수 프로세스에 남긴 stale Discovery 상태 | **미확정** | 타임라인은 일치(원래 Agent를 13:42에 kill -9한 직후부터 첫 실패). 그러나 노트북 격리 랩 2종(짧은/긴 수명 victim, 발행자 포함)에서 재현 실패 → FAIL(재현), 그래프에 유령 노드 없음 |
| UDP 수신 버퍼 오버플로(드롭) | SKIP/미입증 | nav2 프로세스 메타트래픽 소켓에 누적 드롭 3.7M(`udp_drops.txt`; `rmem_max`=212992), 현재 증가 없음. ID 23 소켓 큐는 0이라 직접 원인으로 보기 어려움 |
| 패킷 캡처로 송신측 확인 | SKIP | tcpdump에 root 필요(승인 없이 시도하지 않음) |
| 스택 재시작 후 ID 23이 정상화되는지 | SKIP | 운영 스택 중단이 필요(승인 필요) |

## 3. 재현 절차 (로봇, 읽기 전용: 구독만 하고 아무것도 발행·호출하지 않음)
```bash
scp fleet_control/tests/dds_probe.py fleet_control/tests/dds_trials.sh fleet_control/tests/dds_forced_id.sh pcu@<robot>:/tmp/
ssh pcu@<robot> 'bash /tmp/dds_trials.sh alone 5; bash /tmp/dds_trials.sh warm3 5'   # 단독 0건 vs 헬퍼 뒤 정상
ssh pcu@<robot> 'bash /tmp/dds_forced_id.sh 23 26 40'                                # ID별 수신
```
노트북 격리 랩(SIGKILL 가설): `bash fleet_control/tests/dds_ghost_id_lab.sh 92 3` (긴 victim: `VICTIM_PUBLISHES=1 VICTIM_AGE=25 ... 93 2`).

## 4. 수정 필요성과 실차 적용 위험
- 현재 완화(warm-up)는 **가장 낮은 빈 슬롯이 먹통일 때만** 유효하다. 먹통 슬롯이 둘 이상이거나 헬퍼가 정상 슬롯을 먼저 차지하면 보장되지 않는다(원인 미확정이므로 일반해가 아님).
- **위험:** 로봇의 어떤 ROS 프로세스든(Agent, map_saver, Nav2 서버, Gate·Watchdog 재기동 포함) 재시작 때 슬롯 23을 받으면 에러 없이 먹통이 된다. Agent는 fail-closed(NO_DATA)지만, Watchdog·Gate·맵핑 워커가 같은 상황이면 안전 기능 재기동 시 침묵 실패 가능성을 배제하지 못했다(미시험).
- 제안(적용하지 않음, 승인 필요):
  1. **사전 점검 스크립트화**: 기동 전 `dds_forced_id.sh <빈 ID>`로 슬롯 건강 확인. 먹통 슬롯이 있으면 기동 거부.
  2. 구성요소별 **명시적 participantID**(예: Agent 60, map_saver 61)를 `FASTRTPS_DEFAULT_PROFILES_FILE`로 지정 — 강제 ID 시험에서 다른 ID가 정상이었으나 Agent 전체 경로로는 미시험.
  3. 승인된 시점에 스택 전체를 재시작하고 `dds_forced_id.sh 23` 재시험(정상화되면 stale 상태 가설을 지지, 아니면 기각).
  4. DDS 프로세스에 `kill -9`를 쓰지 않기(현재 Agent가 SIGINT에 종료되지 않는 기존 동작 때문에 `kill -9`가 필요했음 — 별도 개선 과제).
  5. root 권한 패킷 캡처(승인 시)로 송신측이 26707로 보내는지 확인.

## 5. 추가 검증 결과
- **브라우저 클릭→실차 저장 E2E:** 브라우저 클릭부터 저장까지는 **격리 서버(mock Agent+실제 slam_toolbox)에서만** 확인했다. 실차(18190)는 API 시험(`probe_real_save.py` 18/18)만 했고, 실제 브라우저 클릭 E2E는 **미수행**(운영 operator 토큰을 브라우저에 입력해야 해서, 사용자가 직접 하거나 승인이 필요).
- **저장 중 Agent 재시작·토픽 중단(격리 E2E, `probe_save_faults.py` 13/13):** Agent를 SAVING_RASTER 중 SIGKILL → 6.1 s 뒤 `UNKNOWN / robot_offline`, 등록 없음, 재시작 후에도 UNKNOWN 유지(늦은 파일 미등록), Agent 저널은 `INTERRUPTED`, 새 저장은 새 ID로 `REGISTERED`. slam_toolbox를 SAVING_RASTER 중 SIGKILL → `FAILED raster_failed`(원인 문구에 raster와 serialize 실패 모두 표시), 등록 없음, 능력은 사유와 함께 false, `/cmd_vel` 퍼블리셔 0·Gate 출력 0.
- **수동 조작 기능과의 충돌 조사:** 서버 라우트(`/robots/{id}/map-save` vs `/mapping`·수동 bench), 상태 키(`map_save` vs `joystick`·`manual_web_bench`), DB 테이블(`map_save_requests`), Agent 파라미터(`map_save_enabled` vs `manual_*`)는 겹치지 않는다. 공유 지점은 Lease(owner+epoch)와 `agent_node.py`·`service.py`의 조립부뿐이며, 수동 조작 쪽이 이 파일들을 오늘 수정한 흔적은 없다(오늘 수정 파일 목록 확인). 저장은 모터 경로 LOCKED일 때만 가능하므로 수동 주행이 열리면 자동 거부된다. 한 Agent에 `map_save_enabled`와 `manual_web_bench`가 함께 켜질 수 있는지는 막지 않았다(둘 다 이동 없음; 동시 사용은 미시험).
- **회귀 FAIL 1건:** `test_public_manual_arbiter_locked_ros` — `limo_robot_agent.velocity_arbiter`(스테이징에 없는 공개 main 모듈)를 import해서 실패. 파일은 13:42에 수동 조작 작업이 만들었고(`MANUAL_CONTROL_INTEGRATION_AUDIT_20261120.md`가 모듈 부재를 기록), 이번 변경과 무관. 나머지 83건은 통과.

## 6. 다음 단계(웹 자동 맵핑 실차) 차단 조건에 추가
- 위 DDS 슬롯 문제의 원인 규명 또는 사전 점검/ID 고정의 승인·시험. 맵핑·Nav2·Gate·Watchdog 재기동이 필요한 모든 단계에 영향.
