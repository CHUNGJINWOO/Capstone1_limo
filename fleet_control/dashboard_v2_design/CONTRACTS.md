# UI 읽기 데이터 계약 v0.1 — 제안, 현재 API 아님

이 계약은 기존 `OFFICE_INTEGRATION.md`의 provenance 규칙을 구체화합니다. 새 REST 경로나 실시간 서버를 구현하지 않았습니다. `examples/view-contract.json`의 모든 값은 명시적 합성 예시이며 실제 Office/LIMO 자료가 아닙니다.

## 공통 envelope

필수 scope: `environment_id, run_id, source_mode, clock_domain, clock_epoch`. source_mode는LIVE/SIMULATION/REPLAY, 데이터 출처는별도source_kind(REAL_ROS/RMF_OFFICE/SYNTHETIC_FIXTURE 등)입니다. 연결됐다는이유로mode를LIVE로바꾸지않습니다. REPLAY는original_mode,recording_id,original_stamp와playback epoch/rate를추가합니다.

producer는producer_id/boot_id/sequence를갖습니다. sequence와모든nanosecond 값은10진문자열로전송하고JS Number로바꾸지않습니다. 비교는BigInt 또는정수문자열비교로수행합니다. 다른producer의seq/boot를서로비교하지않습니다.

원본stamp는`time.source_stamp_ns`,유형은clock_domain/epoch로판정합니다. `received_at_unix_ms`는수신wall시각이고원본stamp와구분합니다. backend는자기monotonic clock으로`receive_age_ms`를계산해내보내고다른프로세스의monotonic 절대값을비교하지않습니다. clock 일치가확인되지않은경우source_age/latency는null,TIME_UNVERIFIED입니다. TTL은자료형별config로관리하고현재media700ms/adapter5초를임의로모든topic에적용하지않습니다.

UI snapshot은`schema_version, snapshot_id, scope, producer, received_at_unix_ms, resources, robots, missions, timeline, inbox`를포함합니다. snapshot revision은aggregator가관리하고producer sequence와별도입니다. 부분갱신은snapshot_id/base_revision을맞춘경우만적용하고불일치시full snapshot을요청합니다.

## ID와 세션

|키|정의|
|---|---|
|robot_key|canonical JSON 문자열 `[environment_id,fleet_name,robot_name]`. 원본이름은별도유지|
|robot cache scope|robot_key + run_id + clock_epoch. publisher boot는자료별로추가|
|map_context_key|environment/run + map_kind + building/level + map_session + revision|
|mission_key|origin(MM/RMF)/source_instance/run + original_task_id. command_id와별개|
|event_key|source_instance/mission_key/phase_id/event_id/log_entry_id 등원본의계층식별자|
|resource_key|scope + robot_key(공유배경이면null) + resource_kind + producer/boot + resource_id|
|camera frame key|resource_key + stream_id + sequence + source_frame_id + source_stamp_ns|

여러fleet의동명robot,다른building의같은L1,서로다른실험의map이충돌하지않아야합니다. frame_id는원본ROS문자열을보존하고scope를별도필드로둡니다. `base_link`의문자열이같아도다른로봇의TF가같다고해석하지않습니다. producer재시작은해당자료cache를갱신하며세계run이끝나지않았으면robot 정체성을바꾸지않습니다. /clock reset은clock_epoch와종속map/trajectory cache를새로만듭니다.

## 선택 계약

```json
{
  "selection_revision": 17,
  "scope_ref": "office-run-a/epoch-1",
  "robot_key": "[\"office-demo\",\"fleet-a\",\"robot-1\"]",
  "map_context_key": "slam-session-a",
  "mission_key": "rmf-run-a/task-1",
  "origin": "ROBOT_CARD",
  "focus_requested": true
}
```

origin은ROBOT_CARD/MAP_MARKER/MISSION_ROW/INBOX_ROW/LEVEL_PICKER로기록합니다. 선택에대한server POST는없습니다. media_bindings/visible_paths/visible_costmaps는이선택과현재resource index에서계산하는파생값입니다. async decode/GET완료시selection_revision,scope,owner,stream/frame key가그대로인지재검사합니다. 임무와로봇의연결은명시적assignee만사용합니다. 지도조작은focus_requested일때만일어나고view연속성을위해모든센서update마다자동fit하지않습니다.

## 임무 타임라인

Mission 필수: mission_key,origin,source_instance,original_task_id,scope,raw_status,assignee(nullable),time,history_complete,revision. MM과RMF를통합하려면`correlation={mm_mission_id,rmf_booking_id,evidence}`가필요합니다. 없으면별개임무로표시합니다.

TimelineEntry 필수: entry_key,mission_key,phase_id(nullable),event_id(nullable),raw_status,message,source,time,revision. event 종류는STATE_CHANGE/PHASE/EVENT/LOG/ASSIGNMENT/ESTIMATE_UPDATE입니다. 원본에없는필드·시각은null이며대체수신시각은RECEIVED_TIME이라는label을붙입니다. 원본phase/event 트리와로그를유지하고최근500개등조회한계가있으면history_complete=false/truncated_reason을표시합니다.

동일entry_key의동일revision은중복제거,늦은revision은거부합니다. timestamp만으로서로다른log를합치지않습니다. source time이검증됐으면그순서로,아니면수신순서+TIME_UNVERIFIED로표시합니다. wall과sim clock을같은숫자축에정렬하지않습니다. dispatch의finished는배정처리종료로만표시합니다. source간상태가상충하면CONFLICTING_REPORTS로병기하고한쪽을성공으로덮지않습니다.

## 경고 Inbox

Alert 필수: alert_key,source_instance,scope,severity(INFO/WARNING/ERROR/CRITICAL/UNCLASSIFIED),category,robot_key(nullable),mission_key(nullable),message,time,raw_status,revision,lifecycle.

- lifecycle은ACTIVE/RESOLVED/RETRACTED/UNKNOWN이며provider 보고를근거로합니다. TTL초과는STALE이고RESOLVED가아닙니다.
- message/code/원본severity를보존하고UNCLASSIFIED를임의로INFO로낮추지않습니다. 동일code라도다른robot/run이면다른alert입니다.
- `local_read`는사용자의브라우저view 상태이며provider ACK/해결/안전정지와무관합니다. 이번설계는ack/respond/clear command를만들지않습니다. private text/토큰을localStorage에저장하지않고읽음표시를저장한다면opaque alert_key와짧은만료시각만사용합니다.
- 관련robot/task가없으면global경고로표시합니다. source 연결이끊기면원래경고를지우지않고feed STALE을추가합니다.
- 정렬은검증된동일clock timestamp→revision→opaque key이며다른clock은source그룹으로분리합니다. robot/severity/category/lifecycle/read 필터는UI에만적용합니다.

## progress와ETA 표현

```json
{
  "progress": {"value":null,"kind":"UNAVAILABLE","method":null,"source_ref":null,"status":"UNAVAILABLE"},
  "eta": {"remaining_ms":null,"finish_stamp_ns":null,"clock_domain":"ROS_SIM","clock_epoch":1,"kind":"UNAVAILABLE","status":"UNAVAILABLE"}
}
```

kind는REPORTED/ESTIMATE_DERIVED/UNAVAILABLE,progress.value는유효한0..1 또는null입니다. method/source_ref는report field 또는검증한계산식·입력revision을가리킵니다. status는OK/STALE/TIME_UNVERIFIED/INVALID/SIM_PAUSED입니다. task raw_status와progress status를구분합니다. finish_stamp_ns가있더라도clock mapping 미확인시wall시각으로출력하지않습니다. 입력검증·fallback·완료/실패표시정책은DESIGN.md §6을따릅니다.

## 경로와공간 리소스

Path에는layer_kind,robot_key,frame_context,time,source_ref,renderable,unavailable_reason이필수입니다. RMFtrajectory는participant_id/schedule_version/time_horizon/conflicts와time-stamped knots,Nav2plan은원본header/frame과transform evidence,trail은pose_source와분절배열을갖습니다. 없는정보는선택적으로null이며다른자료로대체하지않습니다.

MapContext에는map_kind(BUILDING_LEVEL/SLAM_GRID),building/level(nullable),map_session,revision,source_frame_id,render_frame과registration 상태를둡니다. TransformRegistry는from/to context와stamp/epoch/검증근거를가지며단위는m/rad입니다. label뿐인TF·UNVERIFIED등록은render 허용근거가아닙니다. RGB와Depth의정합은별도camera registration이고map등록으로대체하지않습니다.

## Office 연결 전 계약 변경 절차

실제토픽·QoS·schema 확정→기존OFFICE_INTEGRATION과대조→모르는field를null/UNAVAILABLE로매핑→새run의저장Office fixture→계약·좌표·clock·선택분리검사→승인된독립observer로연결합니다. source_schema_version과mapping_version을반드시기록합니다. 현재예시의합성수치/ID를운영기본값으로복사하지않습니다.
