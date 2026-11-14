# 자체 대시보드 차기 설계 공간

설계 ID: `dashboard-view-v0.1-design-20261113`.

현재 dashboard_lab 기준선과 RMF Web 연구 결과를 보존하면서 8개 기능의 자체 도입 설계만 준비했습니다. 새 UI, normalizer, 서버, Office bridge는 구현·실행하지 않았습니다. 실제 LIMO 접속도 하지 않았습니다. Git 브랜치 변경·commit/push/PR·기존 프로세스 변경은 없습니다.

|구분|이번 상태|
|---|---|
|도입 설계|완료: 8개 기능·선택 연동·경로/좌표·시간/ID·progress/ETA 정책|
|데이터 계약|초안 v0.1 완료: timeline·Inbox·scope·resource ownership|
|합성 데이터|계약 예시 JSON 작성, 정적 정합 확인12/12 PASS|
|합성 UI/서비스 구현|미착수. 예시가 UI에 연결돼 동작한 결과 아님|
|실제 RMF/Office 자료 검증|미착수. Claude 인수 조건 확정 후 진행|
|실제 LIMO 검증|미실시|
|라이선스|공식 Apache-2.0 및 직접 패키지12개 선언 확인; transitive/폰트/자산 배포 검토 보류|

## 파일 안내

- [8개 기능·선택 상태·좌표·ETA 설계](DESIGN.md)
- [타임라인·Inbox·ID/session·시간 계약](CONTRACTS.md)
- [코드 재사용·라이선스 검토](REUSE_LICENSES.md)
- [합성 계약 예시](examples/view-contract.json)
- [예시 정합 검사 범위와 결과](evidence/example-audit.json)
- [설치됐던 패키지의 버전·license 선언](evidence/dependency-licenses.json)
- [기준선·연구 결과 보존 검사](evidence/preservation.json)

## 상위 기준과 후속 작업

기존 [OFFICE_INTEGRATION](../dashboard_lab/docs/OFFICE_INTEGRATION.md)과 [RMF Web 연구](../rmf_web_research/README.md)를 유지합니다. Office 실행환경·토픽/type/QoS/clock/인증/소유 관계가 확정된 뒤 이 계약의 매핑을 개정하고, 독립 구현 공간에서 합성 UI → 저장 Office 자료 → 읽기 전용 실시간 연결 순으로 검증합니다. 계약 JSON은 제안 형식이며 현재 `/api/state`의 구현 응답이 아닙니다.

이번에는 기존31개 신규 검사/90개 회귀를 다시 실행하지 않았습니다. 기존11.90FPS/7.99FPS 및 JPEG P95약38ms는 이전 조건의 성능 기준이며 이 설계의 측정 결과가 아닙니다. 새 기능 구현 후 같은 조건의 회귀·성능 비교가 필요합니다. 라이선스 검사도 전체 배포물 승인으로 확대 해석하지 않습니다.
