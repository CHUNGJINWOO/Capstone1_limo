# 웹 "현재 SLAM 지도 저장 · 등록" (2026-11-20)

범위: SSH 없이 18190 웹에서 LIMO의 현재 SLAM 지도(PGM/YAML)와 Pose Graph를 저장하고, 학교·건물·층·영역·지도 이름과 함께 Map Registry에 **UNVERIFIED**로 등록한다.
저장 전용이다. 맵핑 시작, 이동, Goal, `/cmd_vel` 경로 활성화, 등록 지도의 로봇 적용은 이 기능에 없다.

## 1. Verified / Not Verified / Planned

| 구분 | 내용 |
|---|---|
| Verified (Mock) | `tests/test_map_save.py` 17, `tests/test_map_save_worker.py` 20 (Agent+서버 loopback, 장애 주입), `tests/test_map_save_panel.mjs` |
| Verified (격리 ROS 2) | 실제 slam_toolbox + map_saver_cli + serialize_map + Agent 프로세스 + Fleet 서버. `probe_save.py` 26/26 (warm-up 변경 후 재실행 포함) |
| Verified (브라우저, 격리 서버) | 카드 활성화 조건, 단계 표시(접수→이미지 저장→등록 완료), 결과 표(파일별 크기·SHA·저장 시각·일치성), 375 px 폭 가로 넘침 없음 |
| Verified (실차, 모터 잠금) | `tests/probe_real_save.py` 18/18, 아래 §5 |
| Not Verified | 실차 Agent **재시작 중 저장** 시나리오(단위·격리에서만), 실차에서의 저장 타임아웃(240 s 총 TTL), 이동 중 저장, 맵핑 중 저장, 등록된 지도의 Nav2/AMCL 사용, 장시간 안정성 |
| Planned | 웹 자동 맵핑 실차 통합(§8 차단 조건) |

## 2. 구조 (기존 구성요소 재사용)
- Fleet 서버 `fleet_server/map_save.py`(신규 `MapSaveMixin`): 전용 상태 기계 `map_save_requests`. 미션 명령 테이블을 쓰지 않으므로 Reconciliation을 유발하지 않는다.
- Agent `limo_robot_agent/map_save_worker.py`(신규) + `agent_node.py`(파라미터 `map_save_enabled`, 기본 false, `allow_missions:=true`와 함께면 기동 거부).
- 등록은 기존 `register_map_bundle` / `_prepare_web_upload`(영역·계보·임계값 검증)를 그대로 사용한다. `register_saved_map.py`와 같은 Registry 규칙이다.
- 상태: QUEUED → ACCEPTED → SAVING_RASTER → SAVING_POSEGRAPH → UPLOADING → REGISTERED. 종료: FAILED / TIMEOUT / UNKNOWN(고정, 늦은 업로드 거부). QUEUED TTL 30 s, 전체 TTL 240 s.
- API: `GET /robots/{id}/map-saves`, `POST /robots/{id}/map-save`(Lease owner+epoch, `idempotency_key`, floor/area/label/note, `threshold_policy` reject|derive), Agent용 `POST /agent/map-save`(poll/accept/stage/fail/bundle).

## 3. 저장 정직성 규칙
1. **실제 저장 완료 확인**: 래스터는 `map_saver_cli` 종료 코드, Pose Graph는 `slam_toolbox serialize_map` 서비스 응답 코드 0으로만 성공 처리한다. 파일이 있다는 것만으로 성공이 아니다.
2. **두 저장은 한 스냅샷이 아니다.** 순차 호출이며 파일별 저장 시작·종료 시각, 크기, SHA-256을 따로 기록한다. 일치성은 라이브 지도 버전·셀 수가 저장 전후 같고 로봇이 정지(변위 ≤0.02 m/0.02 rad)였을 때만 `LIKELY_NOT_PROVEN`, 아니면 `NOT_ESTABLISHED`. `SAME_SNAPSHOT`은 존재하지 않는다.
3. 서버는 Agent manifest의 파일별 크기·SHA-256을 다시 검증한다(`transfer_corrupt` 422, 재시도 가능).
4. `free_thresh` 검사 우회 없음: 기본 거부(`unknown_pixels_misclassified`), `derive`면 원본과 보정본을 분리 등록. Agent는 `--free 0.196`으로 저장하므로 기본 정책으로 통과한다.
5. 지도는 항상 UNVERIFIED, 층 활성 아님. 기존 C103·H107 버전은 수정하지 않는다(시험에서 메타데이터 전후 동일 확인).
6. Agent는 파일을 지우지 않는다. 실패한 저장도 `<state_dir>/map_saves/<save_id>/`에 `status.json`과 함께 남는다. 재시작 시 미완료 저장은 INTERRUPTED로 표시하고 재개·재실행하지 않는다.

## 4. 안전 조건 (실차)
- 저장 가능 조건: 모터 경로 LOCKED(또는 SIM_GATED), SLAM `/map` 수신, `serialize_map` 서비스, `map_saver_cli` 존재. 하나라도 없으면 웹에 사유가 표시되고 요청이 거부된다(격리 시험: SLAM 제거 시 `map_save_unavailable`).
- `allow_motor_drive:=false` 유지, `/cmd_vel` 퍼블리셔 0 확인(시험 전·후), 웹 맵핑 START는 실차에서 계속 거부(Agent에 맵핑 워커 없음 → `worker_unavailable`).

## 5. 실차 모터 잠금 시험 결과 (`local_data/raw_results/map_save_20261120/real/`)
대상: pcu, B1 복사본 서버(18190), 영역 `paichai_h_1f_h107_classroom`, 라벨 `web_save_real_locked`.
- 시도 0(Agent가 지도를 못 받음): 저장 요청이 `map_save_unavailable`로 거부됨. 큐에 남지 않음(`probe_real_save_1_agent_blind.out`).
- 시도 1: `FAILED raster_failed` — `map_saver_cli`: "Failed to spin map subscription". Pose Graph만 저장됨(4,967,449 B), **등록 없음**, 레지스트리 버전 수 불변(2). 실패 원인이 웹에 표시됨(`probe_real_save_2_raster_timeout.out`).
- 시도 2(수정 후): `REGISTERED` UNVERIFIED, 7.0 s. 저장 ID `4b466b0e7a514d14b8a42806de42837b`, 버전 `web_save_real_locked#8bbd94412314`.

| 파일 | 크기 | SHA-256(앞 16자) |
|---|---|---|
| map.yaml | 123 | fc468e2c4a11d880 |
| map.pgm | 30117 | 86a29a806e9705c2 |
| graph.posegraph | 4967449 | 2f0e36a24ae1b6e0 |
| graph.data | 11298 | 9f36e17c694859f0 |

- 로봇 측 파일을 ssh로 다시 해시해 manifest와 일치 확인, 레지스트리 evidence의 파일 해시와도 일치.
- 래스터 저장 3.49 s, Pose Graph 0.05 s, 간격 0.013 s. 라이브 지도 불변, 이동 0.0019 m/0.0035 rad → `LIKELY_NOT_PROVEN`.
- free_thresh 0.196, 보정본 없음. 기존 2개 버전(C103·H107) 메타데이터 불변, 새 버전 정확히 1개 추가. 로봇 측 저장물은 노트북 `real/robot_map_saves/`에 복사.
- 이 지도는 로봇 위치 H107에서 **정지 상태**로 저장한 것이며 known_cells 689인 초기 지도다. 검증된 지도가 아니다.

## 6. 실차에서 발견한 문제 (격리·Mock에서는 재현되지 않았음)
1. **새 DDS 참여자가 slam_toolbox의 래치된 `/map`을 못 받는다.** 아무 것도 없이 단독 기동한 프로세스(방금 기동한 Agent 포함)는 map/lidar/pose가 NO_DATA였고(Agent는 fail-closed로 저장 거부), `map_saver_cli`도 10 s 대기해도 0/3 실패. 다른 임시 참여자를 약 3 s 먼저 띄우면 0.4~1.3 s에 6/6 성공하고 Agent도 정상 수신. **근본 원인은 확정하지 못했다**(SHM 비활성 UDPv4 단독으로는 해결되지 않음). 완화: ① 래스터 저장 시 임시 참여자 선기동(`WARMUP_S=3`), ② Agent 기동은 `scripts/start_robot_agent_field.sh`(임시 참여자 선기동)로 한다. 판정은 여전히 `map_saver_cli` 종료 코드와 파일뿐이다.
2. 서비스 클라이언트를 executor 시작 뒤에 만들면 안 된다고 의심해 `RosSaver`를 executor 시작 전에 만들도록 바꿨다. 이후 실험에서 이것이 NO_DATA의 원인이 아니었음이 확인됐다(Agent 코드 없이도 재현). 변경은 무해하며 순서를 `AgentWiringOrder` 테스트로 고정했다.
3. Agent는 SIGINT로 종료되지 않는 경우가 반복됐다(rclpy 종료 대기). 기존 Agent도 동일했다. 재기동 시 `kill -9`가 필요했으며 이는 이번 변경과 무관한 기존 동작이다.

## 7. 배포 기록
- 로봇 설치 대상: `~/Capstone1_limo/robot_ws` 의 `limo_robot_agent` 패키지만(`colcon build --packages-select limo_robot_agent --base-paths src/limo_robot_agent`).
- 변경 파일 2개: `agent_node.py`(수정), `map_save_worker.py`(신규). 로봇 소스와 노트북 소스는 이 두 파일 외 해시가 동일했음을 확인.
- 백업: 로봇 `~/predeploy_mapsave_agent_20261120_134125.tar.gz`(+sha256), 노트북 `local_data/robot_backups_20261120/`에 복사·검증(토큰 포함 가능성 → 공개 금지).
- 복구: 위 tar를 `robot_ws`에서 풀고 Agent를 `map_save_enabled` 없이 `start_robot_agent_field.sh`로 재기동.
- 18190: B1 복사본 데이터(`local_data/field_run_20261120_b1/data`)를 새 서버 코드로 재기동. 운영 field 데이터(`~/.local/share/robot-sim-projects/limo_field_20261113/data`)는 건드리지 않음. 시험 전 `local_data/field_registry_backups_20261120/b1_data_before_mapsave_*.tar.gz` 백업.
- 다른 에이전트 파일(Gate·Launch·Watchdog 등)은 수정하지 않음. 현재 Agent는 `map_save_enabled:=true`, `allow_missions:=false`로 실행 중.

## 8. 다음 단계 "웹 자동 맵핑 실차 통합"의 필요 수정과 차단 조건
차단(모두 해소 전 No-Go):
1. 실차 맵핑 START 3중 차단(웹 버튼 / Agent 제한 / 서버 `mapping_real_blocked`) — 의도적 해제 설계와 승인 필요.
2. Safety Gate attestation producer, operation/map/localization context producer 미구현(Safety Contract v1에서 fail-closed).
3. Motion Watchdog 패치 02b/03b/04는 제안 상태(미적용). 원본 02/03은 금지.
4. 실차 물리 안전: STM32 통신 타임아웃, 제동거리, E-Stop 확인, 바퀴 오도메트리·USB 허브 끊김, LiDAR 드라이버 정지 이력.
5. 카메라 TF 보정은 실행 중 스택에만 적용(재기동 시 소실). 기동 시 바닥면 자동 보정은 제안만 있음.
6. `limo_scan_normalizer`·obstacle memory가 `capstone_ws/src`에 없음(로봇 `robot_ws`에만).
7. 새 DDS 참여자 수신 문제(§6-1) — 맵핑 워커·Nav2가 새로 뜨는 구성에서는 같은 문제가 재현될 수 있음. 원인 규명 필요.
8. 실차 operator 복구 절차, CONTINUE_MAP 워커, 서버 손실 시 fence 정책.
필요 수정: 위 1~3의 구현, 맵핑 워커의 실차 모드, 저장 기능과 맵핑 워커의 동시 실행 규칙(현재 `map_save_enabled`는 웹 맵핑 워커와 병행하지 않음).
