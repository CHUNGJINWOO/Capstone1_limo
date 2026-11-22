# Common-document update proposal — web map save (2026-11-20)
Common documents were NOT edited for this task (another agent works concurrently). Proposed minimal entries:

- CURRENT_STATUS.md: "웹 지도 저장·등록: Mock·격리 ROS 2·실차(모터 잠금, H107 정지 상태) 검증. 등록된 지도는 UNVERIFIED. 로봇 Agent는 map_save_enabled:=true 로 실행 중. 실차 Agent/래스터 저장은 새 DDS 참여자 수신 문제로 warm-up 참여자가 필요(원인 미확정)."
- CODE_INDEX.md: `fleet_control/server/fleet_server/map_save.py`, `fleet_control/agent/limo_robot_agent/limo_robot_agent/map_save_worker.py`, `static/map_save_panel.js`, `tests/test_map_save*.py|mjs`, `tests/probe_real_save.py`, `tests/web_mapping_isolated/{run_save.sh,probe_save.py}`, `scripts/start_robot_agent_field.sh`.
- VALIDATION.md: 격리 probe_save 26/26, 실차 probe_real_save 18/18(시도 3회 중 마지막; 앞선 2회 실패 원인은 docs/WEB_MAP_SAVE_20261120.md §5–6). 미검증 항목은 같은 문서 §1.
- HANDOFF.md: Agent 재기동은 `start_robot_agent_field.sh`로만(warm-up). 단독 기동 시 NO_DATA가 되며 이 경우 저장은 fail-closed로 거부된다.
- DECISIONS.md: 변경 없음(확정 설계 결정은 사용자 승인 후).

- (추가) DDS 신규 참여자 문제: docs/DDS_NEW_PARTICIPANT_ROOTCAUSE_20261120.md — CURRENT_STATUS/HANDOFF에 "ID 23 슬롯 먹통(미확정 원인), Agent 기동은 warm-up 스크립트 사용" 한 줄 제안.

- (추가 2) DDS 진단: docs/DDS_STARTUP_DIAGNOSTIC_20261120.md — HANDOFF에 "신규 노드 기동 전 scripts/dds_slot_precheck.py, 기동 후 scripts/dds_ready_check.py; warm-up은 임시 완화(해결 아님)" 제안.
