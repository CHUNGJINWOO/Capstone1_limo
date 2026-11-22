# 공식 소스 기능 확인과 dashboard_lab 비교

아래 공식 경로는 모두 commit c1b13dc90c0f6983970df87f58004e011a322d2f 기준입니다. 코드 존재, 브라우저 fixture 표시, ROS/운영 동작 검증을 구분합니다. 공식 파일은 https://github.com/open-rmf/rmf-web/tree/c1b13dc90c0f6983970df87f58004e011a322d2f 에서 확인할 수 있습니다.

## 구현 확인

|분야|공식 구현·근거|이번 확인|dashboard_lab 기준선과 차이|
|---|---|---|---|
|Dashboard Framework|packages/rmf-dashboard-framework/README.md, src/micro-apps, components/window, examples/demo/main.tsx|소스·Window story 존재|React MicroApp+event bus+Workspace, lab은기존JS 패널구조|
|지도·건물 평면|components/map/map.tsx,image-maker.tsx,layers-controller.tsx|Office 공식이미지+지도 fixture 표시·확대|공식 BuildingMap/level, lab OccupancyGrid·origin yaw·Costmap|
|로봇 위치/상태|map/robot-three.tsx,robots/robot-summary.tsx,robot-table-datagrid.tsx|2대fixture·상태표·요약 표시|공식fleet/robot/level/commission/ETA 풍부; lab source/STALE·Gate 표시 특화|
|Fleet 관리|micro-apps/robots-app.ts,robots/robot-decommission.tsx,API routes/fleets.py|소스만(버튼 명령 미실행)|공식 조회·commission/decommission/reassign 명령, lab읽기만|
|임무 목록·할당|tasks/task-table.tsx,task-form.tsx,task-inspector.tsx,API routes/tasks|표·assignee fixture 표시|공식 dispatch/direct task 생성/취소 포함; labMM목록·관측이력만|
|진행률·이력|tasks/task-timeline.tsx,task-details,task-logs,robot-summary.tsx|3단계timeline·40% estimate표시|lab은event/history리스트, phase/event와ETA심화 필요|
|경로·교통|services/robot-trajectory-manager.ts,trajectory-socket-manager.ts,map/map.tsx|공식trajectory mock 경로 표시|공식 schedule trajectory/time horizon/conflict 구조. labNav2plan/actual trail과다름|
|이벤트·알림|components/alert-manager.tsx,API routes/alerts.py,task logs|소스확인,알림응답 미실행|공식 acknowledge/응답·task log, labseverity/filter/time order·link/stale|
|다중 로봇/층|BuildingMap.levels,Map level-select,RobotData key|2대·L1→L2 copy selector|labcommon-map2대fixture는있으나building/level전용선택은없음|
|인증/역할|services/keycloak.ts,authenticator.ts,admin components,API authenticator/admin/models|소스확인·Admin stories존재,IDP 미실행|공식JWT/OIDC확장·users/roles/permissions; labBearer/cookie viewer/lab|
|통신|services/rmf-api.ts,API fast_io/__init__.py,gateway.py,routes/internal.py|소스확인|공식 REST+Socket.IO+별도trajectory WebSocket+ROS, lab인증GET/미디어WS·nativeTCP|

현재 소스의 RobotSummary 진행률은 estimate_millis/(finish-start) 기반 예상값입니다. 실제 이동 거리 완료율·안전성·정확한 업무 완료율이 아닙니다. Mission assignment는 API/UI가 존재하지만 이번에 scheduler/dispatcher를 실행하지 않았습니다. 과거 임무 데이터 보존·페이지네이션은 API/repository 코드가 있으나 실제 DB 재시작 시험은 하지 않았습니다.

Fleet 관리란 기존fleet/robot 상태·commission 설정 등의 관리입니다. 이 소스만으로 새 Fleet Adapter 배포/등록/운영 설정 전체가 UI에서 완성된다고 주장하지 않습니다. FleetState 수신 자체도 Adapter 등록 확정 증거가 아닙니다.

## 권한 구현의 제한

공식 인증·admin/role/permission 데이터 모델은 존재합니다. 그러나 frontend services/permissions.ts의 canCancelTask는 FIXME와 함께 true를 반환하고, API repositories/tasks.py의 일부 조회에도 authz TODO가 남아 있습니다. demo는 StubAuthenticator를 사용합니다. 따라서 역할별 최소권한이 모든 명령·조회에서 완결됐다고 판단하지 않았습니다. 자체 viewer 읽기 전용 정책을 이 코드로 교체하지 않습니다. UI 버튼 숨김에 의존하지 않고 서버 command API 차단을 유지해야 합니다. 이 연구는 취약점 악용이나 운영 접근 시험이 아닙니다.

## 자체 UI가 유지해야 할 기능

검토한 공식 기본 framework/API 소스에서 RGB·Depth streaming, 정확한frame-key YOLO overlay, metric Depth raw조회, Nav2 occupancy Costmap 패널은 기본 내장 기능으로 확인되지 않았습니다. 플러그인 확장이 불가능하다는 뜻이 아닙니다. 자체 lab의 아래 기능을 보존합니다.

- RGB/Depth 독립 FPS·품질, 공유encoder/latest-only/ACK/TTL, H.264 후보 비교.
- 원자적 YOLO frame_id/stamp/boot 정합 및 오래된 결과 폐기.
- Depth 컬러preview/무손실거리배열 분리·단위미검증 표시.
- Global/Local Costmap·TF/origin/yaw와실제Nav2계획/관측궤적 구분.
- LIVE/SIMULATION/REPLAY 출처,Robot OFFLINE/영상STALE 분리,읽기전용command부재.

공식 Map image-maker는 현재 읽은 코드에서 x/y offset과scale을사용하지만image yaw 적용은확인되지 않았습니다. 임의yaw지도에서공식UI가항상정확하다고가정하지않고 기존자체좌표검증을유지합니다. 공식 Map의 일부level cache는robot명만key로사용하는부분도있으므로 서로다른fleet의동명robot회귀검사가필요합니다. 이번에 upstream코드를수정하지않았습니다.

## 도입 우선순위 — 후속 구현, 이번 미구현

|순위|기능|필요 API/자료|난이도·RMF 의존|검증 방법|
|---|---|---|---|---|
|1|로봇 요약 카드: fleet/robot/층/배터리/임무/last-seen|fleet state,로봇별scope|낮음,RMF정규화만|동명2fleet fixture→Office2대→후일실차읽기|
|2|임무 phase/event 타임라인·이력 탐색|task state/log,booking ID correlation|중간,RMFtask schema|완료/실패/취소기록재생→Office실제보고상태대조|
|3|행 클릭→지도·임무·영상 동일로봇 선택|공통selection event+robot key|낮음,RMF불필요|잘못된robot영상/다른층초점차단|
|4|building/level 선택과 층별지도|BuildingMap,AffineImage,level transform|중간,RMFbuilding schema|회전/원점/다른층fixture·Office landmark 대조|
|5|예상완료시각·예상진행률|task estimate/start/finish,clock domain|중간,task schema|누락·역행·pause·추정변경에서unknown 유지|
|6|경고 inbox·task별로그 연결|alert/task log read API|중간,RMF선택적|중복·순서·stale·새run 분리. 응답API는도입범위밖|
|7|사용자별 패널 배치 저장|local layout config version|중간,RMF불필요|좁은화면·복구·RGB/Depth렌더성능회귀|
|8|교통 schedule trajectory/conflict 읽기레이어|별도trajectory server/participant ID|높음,rmf_visualization·schedule|공식mock→격리Office trajectory→Nav2경로와legend/clock분리|

임무 제출·재할당·decommission·cancel명령,RBAC/SSO전체전환은이번우선도입목록에서제외합니다. 사용자 요청대로실차제어정책을확장하지않습니다. 기존 Mission Manager/Fleet Adapter 유지가전제입니다.

## 재사용 구조·라이선스

공식 repository는Apache-2.0입니다(LICENSE.md,API client/rmf-models package license). 직접 코드/asset을 재배포하면 라이선스 사본·저작권/NOTICE(있는 경우) 보존과 변경 고지가 필요합니다. 외부MUI/React/Three.js/폰트/이미지각각의조건은별도로확인해야하며공식repo라이선스가모든3자asset의조건을대체하지않습니다.

이번에는공식컴포넌트를격리clone에서import했고자체dashboard_lab에복사하지않았습니다. 실험harness에upstream라이선스사본을함께보관합니다. 자체UI에는MicroApp의책임분리,선택이벤트,task phase/read model,layout schema패턴을우선도입하는것이적합합니다. framework전체이전은React/MUI/Three·API client·Socket.IO·RmfApi provider의존성이크므로필요한8개기능을작게추가하는안이현재구조에맞습니다.
