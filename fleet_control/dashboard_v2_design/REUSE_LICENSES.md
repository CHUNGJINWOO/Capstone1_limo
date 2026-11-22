# 공식 코드 재사용·라이선스 검토 범위

## 현재 상태

설계만 추가했고 이 실험 폴더나 dashboard_lab에 공식 UI 코드를 새로 복사하거나 runtime dependency를 설치하지 않았습니다. 기존 rmf_web_research의 공식 clone/Storybook harness와 Apache 라이선스 사본은 보존합니다. 참고 revision은 c1b13dc90c0f6983970df87f58004e011a322d2f입니다.

## 확인한 선언

공식 저장소 LICENSE.md는 Apache-2.0입니다. 이전 실험에 실제 설치된 package.json을 읽어 다음 선언을 확인했습니다. 전체 전이 의존성·폰트·asset 라이선스를 모두 검토했다는 뜻은 아닙니다. 세부 목록과 package.json 해시는 evidence/dependency-licenses.json에 있습니다.

|패키지|설치 버전|package.json 선언|
|---|---|---|
|api-client / rmf-models|0.3.0|Apache-2.0|
|rxjs|7.8.1|Apache-2.0|
|keycloak-js|25.0.2|Apache-2.0|
|react / react-dom|18.3.1|MIT|
|@mui/material / @mui/icons-material|5.16.1|MIT|
|three|0.166.1|MIT|
|react-grid-layout|1.4.4|MIT|
|storybook|10.5.8|MIT (개발 도구)|
|@fontsource/roboto|4.5.8|MIT (패키지 선언; 폰트 파일 조건 별도 확인 필요)|

Roboto metadata는 Google Fonts attribution URL과 font v30을 가리킵니다. 설치 패키지의 루트 LICENSE 파일은 확인되지 않았으므로 패키지 선언만으로 폰트 파일까지 재배포 승인으로 판단하지 않습니다. Office 도면·로고·로봇 아이콘도 개별 asset 출처/고지 확인 전 자체 제품에 복사하지 않습니다. 현재 설계는 기존 폰트·아이콘·JS/Canvas를 우선 사용해 새 의존성을 최소화합니다.

## 실제 코드 복사 시 필요한 기록

1. `reuse-manifest`에 upstream URL/commit/path/hash/license, 복사 대상 path, 변경 요약, 유지한 copyright/NOTICE 경로, transitive dependency 목록을 기록합니다.
2. Apache-2.0 LICENSE 사본, 원본의 저작권·특허·상표·귀속 고지를 보존합니다. upstream에 배포 대상 관련 NOTICE가 있으면 해당 귀속 내용을 전달하고, 변경한 파일에는 변경 사실을 명확히 남깁니다. 브랜드/로고 사용 권한을 라이선스로 자동 추정하지 않습니다.
3. MIT 의존성의 배포물에는 해당 copyright·permission notice를 보존합니다. 번들링 과정에서 LICENSE가 사라지지 않도록 배포 artifact를 검사합니다.
4. 필요 기능만 import하더라도 실제 번들에 포함되는 dependency/asset을 기준으로 목록을 생성합니다. direct package 선언 표만으로 전이 의존성 검토를 끝내지 않습니다.
5. 변경 소스·빌드/lock hash와 notices를 함께 검토하고 새 실험 공간의 smoke/회귀를 실행합니다. 기존 저장소 publication policy와 개인정보/토큰 검토를 거친 뒤에만 배포를 별도로 판단합니다. 이번 작업은 commit/push/PR/배포를 하지 않습니다.

## 권장 재사용 방식

1순위는 MicroApp의 책임 분리·선택 이벤트·읽기 모델 패턴을 현재 JS 구조에 맞춰 설계하는 것입니다. 공식 구현을 직접 복사하는 경우에만 해당 파일과 의존성을 명시적으로 가져옵니다. React Framework 전체 이전은 보류합니다. StubAuthenticator, canCancelTask의 임시 허용 구현, 제어 API는 재사용하지 않습니다. 기존 viewer/lab 정책과 서버의 명령 차단을 유지합니다.

아래는 **향후 고지 양식**이며 아직 해당 기능을 자체 대시보드에 복사했다는 주장이 아닙니다.

```text
Component: Open-RMF Web [copied component/file names]
Upstream: https://github.com/open-rmf/rmf-web
Revision: [exact commit]
License: Apache License 2.0 — see licenses/Apache-2.0.txt
Original notices: [preserved source headers / NOTICE if applicable]
Local modifications: [date, paths, factual change summary]
Included dependencies/assets: [versioned inventory and notice locations]
```

보류: 전체 lockfile SBOM, 실제 배포 bundle의 license inventory, 폰트 원문·Office 도면/아이콘 provenance, 복사할 파일의 구체 목록. 복사 대상이 확정되지 않은 상태에서 모두 완료로 기록하지 않습니다.
