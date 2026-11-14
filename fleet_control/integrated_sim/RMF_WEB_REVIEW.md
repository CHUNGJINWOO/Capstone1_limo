# RMF Web 검토 (문서 조사만, 설치·실행하지 않음)

**결론: 이번 통합에서는 RMF Web을 쓰지 않고 기존 Mission Manager 대시보드를 유지합니다.** 아래는 공개 문서(open-rmf/rmf-web README, api-server README, dashboard framework README/micro-apps)를 읽은 결과이며, 실행해서 확인한 것이 아닙니다. 읽지 못한 페이지(dashboard README, 문서 사이트의 api-server 페이지는 404)에 대해서는 판단하지 않았습니다.

| 항목 | 조사 결과 | 이 프로젝트에 대한 의미 |
|---|---|---|
| 지원 버전 | README는 "Ubuntu 26.04, ROS 2 Lyrical, 최신 Open-RMF"를 지원 대상으로 적고 다른 배포판은 해당 브랜치(`jazzy` 등)를 쓰라고 함. Humble은 언급되지 않음. rmf_api_msgs/rmf_fleet_adapter 버전 고정은 찾지 못함 | 우리는 Humble + Open-RMF 2.1.8 격리 설치. Humble 호환은 **미확인**(공식 지원 범위 밖으로 보임) |
| 실행 요건 | API 서버(Python, 기본 메모리 SQLite, PostgreSQL 등 선택) 포트 8000, 대시보드 개발 서버 5173 / Docker 3000, `ghcr.io/open-rmf/rmf-web/*` 이미지(`--network host`). Node는 pnpm LTS | 이 노트북: Node 12(너무 오래됨), 사용자가 Docker 소켓 접근 권한 없음(`permission denied`) → **별도 환경 설치를 이번에 하지 않음**. 시스템 변경 없이 하려면 Docker 그룹/Node 18+ 환경이 먼저 필요 |
| 상태 수집 | Fleet Adapter가 `server_uri`(`http://localhost:8000/_internal`)로 상태를 보냄. 우리 Adapter는 이미 `--rmf-api-uri` 웹소켓(RMF API 서버 대용)에 task/fleet 상태를 보냄 | 이론상 같은 경로에 붙일 수 있으나 우리 시험은 자체 WebSocket sink로 대체. 실제 API 서버와의 메시지 호환은 **미검증** |
| 임무 생성·조회·취소 | 엔드포인트 경로는 문서에서 확인하지 못함(서버 `/docs`에서 확인해야 함). RMF 표준 task API(`task_api_requests`)는 우리 통합에서 이미 사용(생성·취소 검증됨) | 임무 생성/취소는 현재 RMF 표준 task API + Mission Manager로 동작 |
| 지도 | 평면도는 traffic-editor/rmf_site의 `.building.yaml`(벽 필요) 기반 | 우리는 Nav2 저장 지도(PGM)와 RMF 그래프 JSON을 씀. Web용 building 파일 변환 작업이 필요(**미구현**) |
| 영상·YOLO 패널 | 기본 제공 없음. MicroApp(`createMicroApp`)으로 새 패널을 만들 수 있음 | 하이브리드(임무는 RMF Web, 영상/YOLO는 자체 패널)는 가능해 보이나 React 패널 개발·CORS/토큰 처리가 필요. **미구현** |
| 인증 | OpenID Connect(Keycloak 등) 공급자 필요, 개발 로그인 `admin/admin`이 언급됨(기본 구성은 미확인) | 대시보드의 토큰·역할 모델과 별개. 통합하려면 인증 설계 필요 |

## 판단

- 현재 구조(Fleet Adapter → Mission Manager가 로봇의 유일한 명령 경로, 제어권 lease, 증거 기반 복구)는 RMF Web이 대신할 수 없는 부분입니다. RMF Web은 RMF 쪽 임무/로봇 **조회·생성 UI**를 제공할 수 있을 뿐이며 Mission Manager의 lease·복구 정책을 모릅니다.
- 따라서 **자체 대시보드 유지**가 현실적입니다. RMF Web 도입은 (1) Humble 호환 확인(또는 지원 배포판으로 이전), (2) 별도 컨테이너/Node 환경, (3) 평면도 변환, (4) 영상 패널 MicroApp 개발이 선행 과제입니다. 선택한다면 임무 관리 보조 화면으로만 붙이고 기존 Adapter/중앙 관제를 대체하지 않는 하이브리드가 안전합니다.
