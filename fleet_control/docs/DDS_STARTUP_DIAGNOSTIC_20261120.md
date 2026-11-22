# DDS Participant ID 23 — 추가 조사와 안전한 기동 진단 설계 (2026-11-20)

운영 스택·Agent·18190은 유지했다(Agent PID 39164 그대로, 중단·재시작·Launch 변경·시스템 재시작 없음). 실험은 구독 전용 신규 프로세스, 로봇 위 임시 참여자(사용 후 정리), 노트북 격리 도메인(92~97)만 사용했다. 모터 잠금과 Mapping START 차단은 건드리지 않았다. 근본 원인은 **여전히 미확정**이다. 앞선 보고서(`DDS_NEW_PARTICIPANT_ROOTCAUSE_20261120.md`)와 같은 형식으로 사실/가설/배제 불가를 구분한다. 로그: `local_data/raw_results/dds_rootcause2_20261120/`.

## 1. 확인된 환경
- ROS_DOMAIN_ID=77(스택 프로세스 환경에서 확인), `rmw_fastrtps_cpp` 6.2.11, **Fast DDS(ros-humble-fastrtps) 2.6.12**, fastcdr 1.0.29, `ROS_LOCALHOST_ONLY=1`. FastDDS XML/환경변수 설정 없음(`facts.txt`).
- RTPS 기본 포트식(PB 7400, DG 250, PG 2): 도메인 77 멀티캐스트 메타 26650, 유니캐스트 메타 `26660+2·ID`, 사용자 `26661+2·ID`. **ID 23 = 26706/26707**, ID 26 = 26712/26713. 두 슬롯의 포트 구조 차이는 없다.
- ID 23 포트는 비어 있고 소켓 소유자 없음(`ss`/`/proc/net/udp`, 포트당 소켓 1개). 사용자 ID(구성요소별 ID)별 현재 보유: ID 0~22·24·25는 스택/Agent 등이 사용 중.
- 비정상 종료 흔적: `/var/crash/_opt_ros_humble_bin_ros2.1000.crash`(12:40, `ros2 topic echo --once /stall_detector/status_json`, spin 중 예외) — 다른 작업이 만든 짧은 수명 참여자의 비정상 종료다. 13:42에 내가 Agent를 `kill -9`했고 이후 슬롯 23이 먹통이 됐다. **어느 쪽이 ID 23을 점유했었는지는 증거가 없다.** 모두 가설의 배경일 뿐이다.

## 2. 계층별 결과 (Discovery / Matching / 데이터)
| 계층 | ID 23 | ID 26 | 근거 |
|---|---|---|---|
| 엔드포인트 Discovery(스택 발행자가 보임) | 성공 | 성공 | `layers.txt`: `/scan`·`/tf`·`/map` 발행자 발견 |
| UDP로 수신되는 RTPS 패킷(내장 Discovery 트래픽) | 정상 수신, INFO_DST가 항상 자기 GUID prefix(5~6천 패킷) | 동일 | `strace_rtps_*.txt` (ptrace는 내 프로브 자식만, root 불필요) |
| **스택 → ID 23 사용자 데이터** | **0건**(/scan·/odom·/tf·/tf_static·/map 전부) | 정상 | 5/5 단독, 강제 ID 시험 |
| **신규 참여자 ↔ ID 23 (양방향)** | **정상**: 23↔40, 40↔23, 23↔26 모두 4만~6만 메시지, 발행자 매칭 확인 | 정상 | `layers.txt` ping 시험 |
| 포트에 오는 패킷(참여자 없음) | 없음 — 스택이 ID 23의 옛 Locator로 계속 보내지는 않음 | - | `raw_listen_id23_vs_id70.txt` |

해석(입증 범위 내): ID 23 슬롯 자체(포트·소켓·전송·신규 참여자와의 통신)는 정상이다. **오직 "이미 실행 중이던 스택 프로세스 → ID 23" 방향의 사용자 데이터만 전달되지 않는다.** 스택의 모든 프로세스(라이다, limo_base, slam_toolbox, Nav2 …)가 동일하게 실패하므로 개별 노드 문제가 아니라 장수 프로세스들이 공통으로 가진 ID/Locator 관련 상태를 가리킨다. 그 상태가 무엇인지는 입증하지 못했다. 사용자 데이터의 UDP 패킷이 ID 23 소켓에 도착하는지는 SHM 경로가 기본이라 `strace`로 확인하지 못했다(UDP 전용 모드에서도 정상 슬롯의 사용자 DATA가 `recvfrom`에 보이지 않아 캡처 방식 한계로 판단; SKIP).

## 3. 슬롯별 재현 성공률 (읽기 전용 프로브, 로그 집계)
| Participant ID | 수신 성공 | 비고 |
|---|---|---|
| 23 | **0 / 약 27** (전부 0건) | 단독 5, 강제 ID 4, UDP전용 2, SHM전용 2, 반복 7+2 외 레이어·소켓·strace 시험 |
| 26 | 14 / 15 | 실패 1건은 UDP전용+`strace -e read` 과부하 측정 중 4 s 창에서 /scan 0건(측정 아티팩트로 추정, 미확정) |
| 27 | 7 / 7 | |
| 40 | 7 / 7 | |
| 60 | 5 / 5 | |
| 28~33, 100 | 각 1 / 1 | 스윕 |
추가 관찰: 건강한 슬롯에서도 `/tf`가 일부 실행(8회 중 5회)에서 **0건 또는 수 건에 그치는** 경우가 있었다(`precheck_robot_repeats_v2.txt`). `/tf`는 6개 발행자·버스트성이라 정상 슬롯 판정 신호로 부적합하다. 부분적(주제별) 수신 실패가 ID 23 밖에서도 가끔 일어날 수 있다는 뜻이며, 아직 원인을 모른다.

## 4. 가설 정리
**입증된 사실**: 위 §2~§3, warm-up이 먹통 슬롯을 점유하는 효과(앞선 보고서).
**유력 가설(입증 못 함)**: 과거 ID 23 보유 프로세스의 비정상 종료 후 장수 프로세스들에 남은 슬롯별 Discovery/Locator 상태. 지지: 슬롯 한정, 모든 스택 프로세스에 대칭, 전송 무관, 신규 참여자와는 정상, 시간 순서(13:42 직후 첫 실패). 반증/미재현: 노트북 격리 랩 2종(짧은·긴 수명 victim, 발행자·래치 포함)에서 SIGKILL 후 ID 재사용이 정상이었다. 옛 Locator로의 지속 송신도 없었다(`raw_listen`).
**배제하지 못한 원인**: 장수 프로세스의 UDP 수신 버퍼 오버플로 누적(드롭 370만, 정지 상태; ID 23 소켓 큐는 0이라 직접 증거 없음), Fast DDS 2.6.12 내부의 ID/GUID 관련 버그, ROS_LOCALHOST_ONLY 멀티캐스트 처리, 스택의 장시간 가동으로 생긴 다른 상태. 확정에 필요한 시험은 §8.

## 5. 안전한 기동 진단 설계 (구현·시험 완료, 운영 미배포)
1. **기동 전**: `scripts/dds_slot_precheck.py`(로봇에서 실행, 읽기 전용). 새 노드가 받을 가장 낮은 빈 ID를 구해 그 ID를 강제한 **카나리 참여자**가 필수 토픽(`/scan`, `/odom`)을 구독한다. 각 토픽의 첫 메시지가 `--first-s` 안에 오고, 신선도 창(`--fresh-s`)의 **전·후반 모두** 메시지가 증가해야 `SLOT_OK`. 아니면 `SLOT_DEAD`이고 다음 빈 ID들을 시험해 건강한 `recommended_participant_id`를 보고한다(종료코드 0/3/4). 카나리는 스스로 종료한다(`kill` 안 함: 비정상 종료가 용의 요인이므로).
2. **기동 후**: `scripts/dds_ready_check.py`(노트북, 서버 API만 읽음). 연속 3회 모두 online·state fresh·lidar OK(age<1.5 s)·pose OK·map STATIC/UPDATING이고 `agent_seq`가 증가해야 `READY`. 하나라도 어긋나면 `NOT_READY`, 종료코드 5. 일부 샘플만 나빠도 NOT_READY다.
3. **READY 전이 규칙**: NOT_READY Agent는 READY로 선언하지 않고, Lease·맵핑·저장·이동을 시작하지 않는다. 현재도 Agent는 NO_DATA 시 fail-closed(저장 거부 시험으로 확인)이므로 이 스크립트는 명시적 기동 게이트와 기록 역할이다. 서버·Gate에 NOT_READY를 강제 전이시키는 코드는 **다른 에이전트 영역(Gate·Watchdog)이라 건드리지 않았다**(제안만).
4. `start_robot_agent_field.sh`에 사전 점검을 연결했다(배포 안 함): `NO_HEALTHY_SLOT_FOUND`면 기동 거부(`FORCE_START=1`로만 강행), 기존 warm-up은 **임시 완화**로 남기고 주석·문서에 "해결 아님"으로 표시했다.

### 진단의 오탐·미탐 (실측)
| 시험 | 결과 | 로그 |
|---|---|---|
| 실차 슬롯 23(진짜 먹통) | **10/10 탐지**, 대안 ID 26 건강 10/10 (토픽 `/scan,/odom`) | `precheck_robot_repeats_v3_scan_odom.txt` |
| 격리 랩, 건강한 20 Hz | SLOT_OK 6/6 및 10/10 (오탐 0) | `precheck_lab*.txt` |
| 2 Hz 토픽 | SLOT_OK 3/3 | |
| **0.5 Hz 토픽, fresh-s 2** | **오탐**(건강한 슬롯을 DEAD/NO_HEALTHY로 판정) 3/3; fresh-s 5면 OK | 한계: 창 ≥ 발행주기×2 필요 |
| 토픽 미발행·발행자 없음 | NO_HEALTHY_SLOT_FOUND(정답). **슬롯 먹통과 발행자 부재는 구분 불가** — 모든 대안이 실패하면 슬롯이 아닌 스택 문제로 해석 | |
| `/tf`를 필수에 포함 | **오탐 다수**(건강한 ID 26을 5/8 불합격) → 기본 토픽에서 제외 | `precheck_robot_repeats_v2.txt` |
| 중간에 침묵하는 발행자(stale) | **미확정(SKIP)**: 시험 타이밍이 일정하지 않아 3회 중 2회 통과 — 점검 시점 이후의 정지는 원리상 못 잡는다(TOCTOU). 이를 사후 준비 점검·Agent의 신선도 표시가 담당 | `precheck_lab_v3.txt` |
| 사후 준비 점검 | 단위 10/10(라이다 NO_DATA/정체, pose, map, agent_seq 정체, 오프라인, 한 샘플 불량, 서버 불가, 원격 URL 거부), 실 Agent READY 25/25 및 이전 3. **전체 28회 중 1회 원인 미기록의 일시 NOT_READY** (보수적 거짓 NOT_READY 가능) | `ready_check_25.jsonl` |
추가 미탐 위험: 점검한 슬롯과 실제 노드가 받는 슬롯이 다를 수 있다(점검 후 다른 프로세스가 같은 슬롯을 먼저 점유 — TOCTOU). 사후 준비 점검이 이를 잡는다.

## 6. Participant ID 고정의 위험 검토 (적용하지 않음)
- 장점: 먹통 슬롯 회피를 결정적으로 만들 수 있다. 강제 ID는 포트·GUID 카운터에 반영됨을 확인(`own_prefix` 끝 4바이트 = 강제 ID).
- 위험: (a) ID 충돌 — 이미 쓰는 ID면 참여자 생성이 실패(조용히가 아니라 오류로 드러나는지 Agent 경로 전체는 미시험). (b) 재시작 시 같은 고정 ID를 재사용하므로 **SIGKILL 이후 재사용이 곧 의심 시나리오**(가설 유지)와 겹친다. (c) 구성요소별 고정표 관리, ROS 도구(`ros2 topic echo` 등)는 자동 할당과 섞임. (d) 먹통 슬롯이 늘면 고정 ID도 오염될 수 있음. 따라서 영구 해결로 보지 않고 "사전 점검으로 건강 확인된 ID만, 승인 후 시험" 단계로 둔다.

## 7. 영구 해결 후보와 적용 위험
| 후보 | 기대 | 위험/비고 |
|---|---|---|
| 스택 전체의 계획된 재시작 후 ID 23 재시험(+`kill -9` 금지 운영 규칙) | 장수 프로세스의 stale 상태 가설을 직접 검증·해소 | 승인·모터 잠금 절차 필요, 가설이 틀리면 재발 |
| ID 고정(§6) | 결정적 회피 | 위 위험, 미시험 |
| Fast DDS Discovery Server / 설정 | Discovery를 중앙화해 슬롯 상태 의존 제거 | 전 노드 설정 변경, 큰 변경 |
| RMW 교체(CycloneDDS) 또는 fastrtps 패치 업그레이드 | 버그성 원인이면 해소 | 전 스택 영향, 재검증 대규모 |
| 수신 버퍼(`rmem_max`) 상향 | 드롭 감소(원인이라면) | 원인 불명, root 필요 |
| 진단 게이트만 운영 (현 구현) | 침묵 실패를 막음 | 원인 해결 아님 |

## 8. 승인이 필요한 시험 계획 (실행하지 않음)
1. root `tcpdump -i lo udp port 26706 or 26707` 중 ID 23 프로브 실행 → 스택이 ID 23 사용자 포트로 사용자 DATA를 보내는지, INFO_DST가 자기 prefix인지 확인(송신측 vs 수신측 결함 구분).
2. 같은 캡처에서 UDP 전용 모드 프로브를 사용해 SHM 개입 배제.
3. 계획된 스택 재시작(모터 잠금) 후 `dds_slot_precheck.py`로 슬롯 23 재시험, 재시작 전·후 비교. 재시작 직후 `kill -9` 하나를 스택 외 임시 참여자에 가한 뒤 해당 ID 재사용 시험으로 SIGKILL 가설 직접 검증.
4. Fast DDS 상세 로그(`FASTRTPS_LOG` 등) 활성화로 장수 프로세스의 ID 23 관련 이벤트 수집.

## 9. 운영 서비스에 영향 없는 도입 방법
- 사전 점검은 신규 임시 참여자만 만든다(스택에 쓰기·서비스 호출 없음). 이미 12회 이상 실차에서 운영 중 실행했고 Agent·스택 상태 변화 없음(Agent PID·`agent_seq` 연속).
- 도입 순서: ① 사전 점검과 준비 점검을 수동 실행으로만 사용(기록) → ② 다음 정규 Agent 재기동 때 `start_robot_agent_field.sh`로 사용(로봇에 복사 필요, 현재 미복사) → ③ 서버/Gate 전이 연동은 해당 담당 에이전트와 협의 후 제안 문서로만.

## 10. PASS / FAIL / SKIP
| 항목 | 결과 |
|---|---|
| Fast DDS 2.6.12·도메인 77·포트식 확인 | PASS |
| ID 23 포트·소켓 점검(소유자 없음) | PASS |
| Discovery vs 데이터 분리, 신규↔ID 23 양방향 정상 | PASS |
| 스택→ID 23 사용자 데이터 0건 재현 | PASS(0/약 27) |
| UDP/SHM 공통 실패 조건 재검토(둘 다 ID 23만 실패) | PASS |
| SIGKILL 잔여 상태가 원인 | **미확정**(랩 재현 FAIL, 시간순서 일치) |
| 옛 Locator로의 지속 송신 | FAIL(관측 없음) |
| 사용자 DATA UDP 도착 여부 | SKIP(캡처 한계, root 필요) |
| 슬롯 사전 점검: 실차 먹통 탐지 | PASS(10/10) |
| 슬롯 사전 점검: 오탐 | PASS(20Hz 0/16) / FAIL(0.5 Hz, /tf) — 한계 문서화 |
| 중간 침묵(stale) 탐지 | SKIP/한계(TOCTOU) |
| 준비 점검 단위 10 | PASS |
| 실 Agent 준비 점검 | PASS(25/25, 일시 NOT_READY 1/28 미설명) |
| root 패킷 캡처·스택 재시작 | SKIP(승인 필요, 계획만) |
| 운영 프로세스 종료·실차 Launch 변경·재시작·모터 구동 | 수행 안 함 |

## 11. 다른 에이전트와의 파일 충돌
- 이번 작업은 새 파일(`scripts/dds_*.py`, `tests/dds_*`, `test_dds_ready_check.py`, 본 문서)과 `scripts/start_robot_agent_field.sh`(내가 만든 파일) 수정만 했다. 오늘 수정된 다른 파일 중 `MANUAL_*` 문서·`test_manual_*`·`test_public_manual_arbiter_locked_ros.py`·`manual_session_*`는 수동 조작 작업 소유이며 건드리지 않았다. 추적 중인 저장소 변경 9개와 launch 파일 해시(`1671d282`)는 시작 때와 같다. 공통 문서는 수정하지 않았고 제안만 `COMMON_DOC_UPDATE_PROPOSAL_20261120_MAP_SAVE.md`에 한 줄 추가했다.
