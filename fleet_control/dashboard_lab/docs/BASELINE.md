# 오프라인 기준선과 비교 범위

기준선 ID: `dashboard-lab-offline-20261113-v1`. 브랜치 `codex/dashboard-visualization`의 현재 미커밋 파일을 로컬 기준선으로 보존합니다. commit/tag/PR은 만들지 않았습니다. [해시 목록](evidence/baseline-20261113.sha256.json)은 이 문서 정리 전에 존재하던 코드·테스트·문서·계측 근거를 기록하며 비밀·캐시·스크린샷·raw 대용량 파일은 포함하지 않습니다. 해시 목록은 복구용 파일 백업 자체가 아닙니다. 기존 작업 폴더를 삭제하면 복구되지 않습니다.

## 기존 dashboard와 dashboard_lab

|항목|기존 dashboard|dashboard_lab 기준선|
|---|---|---|
|위치|fleet_control/dashboard|fleet_control/dashboard_lab 별도 추가|
|실행|기존18210/18211은 변경하지 않음|개선18310, 비교18311, native18313|
|공유 코드|기존 지도·좌표 검증/인증 기반|dashboard.core와 기존 baseline server 구현을 import하므로 완전 독립 복제는 아님|
|비교 화면|기존 polling UI|baseline_web 복사본에 표시 계측만 추가|
|작업/관제 UI|기존 상태 중심 화면|임무 필터·이력, 이벤트 필터/정렬, 다중 로봇·목적지·궤적|
|미디어|gateway/UI polling, Depth 주기 제한|latest-only 공유 encoder, atomic frame metadata, ACK, H.264/WebRTC 후보|
|거리|기존 센서 기록/표시|컬러 preview와 무손실 배열 분리, key별 거리 조회|
|입력|기존 구성별 upstream|기본 합성 spatial 데이터+저장 Gazebo RGB; 기록 REPLAY; LIVE 미설정|
|권한|기존 정책 유지|viewer/lab 분리, 제어 API 없음|
|실제 RMF/Office|별도 기존 Adapter/실험|아직 Office 연결 안 됨; 등록 fixture 또는 선택적 snapshot|

`dashboard_lab`는 Mission Manager/Fleet Adapter/주행·안전 제어를 대체하지 않습니다. 기존 dashboard49개 원본 파일 해시 보존 결과는 [preservation.json](evidence/preservation.json)에 남아 있습니다.

## 고정된 측정 결과와 한계

이전 검증 결과: 새 Python25+JS6=31 PASS, 기존 visualization31+geometry6+live_fusion6+dedupe14+RMF runner policy33=90 PASS. 이번 문서 정리에서 재실행하지 않았습니다. 향후 소스나 의존성 변경 후 이전 PASS를 현재 실행 결과로 재사용하지 않습니다.

|단일 전경 브라우저 탭|기존|개선 JPEG/TCP|
|---|---:|---:|
|RGB 실제 표시 FPS|1.96|11.90|
|Depth 실제 표시 FPS|1.00|7.99|
|RGB 생성→표시 P95/P99|373/436ms|38/40ms|

같은 저장 JPEG 및 CPU 사전계산 YOLO 입력입니다. 실제 카메라 노출·실시간 GPU 추론·LIMO 네트워크·모니터 광학 지연은 포함하지 않습니다. SIMULATION 공간 메시지는 ROS bridge 스키마 객체이며 이번 lab에서 Gazebo/ROS middleware를 실행한 것은 아닙니다. 기록의 RGB·실차 지도/Depth는 동시 RGB-D가 아닙니다.

H.264/TCP는 별도 native7초 시험에서 balanced JPEG172.91kB/s→66.55kB/s(약62% 절감). 같은 화질 PSNR 검사는 하지 않았습니다. 브라우저 WebRTC P95약72ms로 JPEG보다 느렸습니다. native 수신기와 브라우저 결과를 섞지 않습니다.

대역폭은 payload 기준이며 IP/TLS/전체 NIC 사용량이 아닙니다. CPU19.04%는 최적화 서버 한 코어100% 기준이고 baseline1.77%와 기능 수가 다릅니다. 최대RSS225.36MiB는 서버 프로세스만입니다. LIMO와 브라우저 전체 부하는 미측정입니다. GPU0%/674MiB는 전체 장치 순간 표본으로 lab 귀속값이 아닙니다.

장애 조건은 프로그램 수준 모델입니다. RTT400은 순방향200ms 주입,150kbps는 스트림·클라이언트별 서비스 제한이며 NIC 공유 cap이 아닙니다. TCP transfer unit 손실 모델과 실제 RTP datagram 손실 시험을 구분합니다. 심한 제한에서 FPS 목표 미달, 다중·비활성 탭 약1FPS 사례가 있었습니다. 최종 수치는 약11초/131프레임 표본이므로 P99와 장시간 안정성을 확정하지 않습니다. 수동 서버 재시작 시간을 포함한15.71초 복구는 자동 daemon 결과가 아닙니다.

상세 조건·단계별·장애 표는 [RESULTS](RESULTS.md), 원시 자료는 private state의 results/20261113에 있습니다. 기존 결과를 변경하지 말고 다음 Office 측정은 새 run ID·새 파일에 기록합니다.

## 보류한 항목

대규모 UI 추가, 실제 RGB-D 배포, Collision Monitor C·Nav2·Safety Gate 수정, Office 실행/Adapter 재기동, 임무 제출·취소·motor API는 하지 않습니다. 다음 단계는 [Office API 계약](OFFICE_INTEGRATION.md)의 읽기 전용 변환과 검증입니다.
