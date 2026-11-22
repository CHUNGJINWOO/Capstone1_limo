# Spatial Monitor 데이터 인터페이스 요청 (제안 · 미구현)

2026-11-16. 이 문서는 담당자 검토용 요청이며 서버, Agent, Gate, RMF 코드를 변경하거나 계약을 확정하지 않는다.

## 현재 확인

Fleet `/api/v1/dashboard`는 선택 로봇의 pose, map version/meta, localization 및 mission goal을 제공한다. `/api/v1/robots/{id}/map.png`는 지도 스냅샷이다. Fleet에는 `/plan`, 영상 Depth, 건물·층의 로봇 소속이나 인증된 attestation 조회 API가 없다. 별도 `dashboard/bridge_node.py`는 `/plan`, `/local_plan` (`nav_msgs/Path`)과 `/camera/depth/image_raw` (`sensor_msgs/Image`)를 읽고 별도 dashboard gateway의 `/api/depth` 및 visualization 상태를 제공한다. 별도 RGB/AI host는 독립 토큰의 `/api/v1/status`, `/api/v1/frame.jpg`를 제공한다. 이 시스템들을 Fleet UI에서 같은 출처로 취급할 근거는 없다.

실차 조사 기록 `fleet_control/dashboard/docs/REAL_LIMO.md`: depth 16UC1 수신 사례가 있으나 0.001 m 배율은 가정이고 장치 정확도 미검증; 무효 값 0. RGB-D 정합 미검증. `/plan` 실차 표본은 미수신. 실제 센서·경로 지원 완료의 증거가 아니다.

## 요청할 계약

1. 읽기 전용 공간 데이터 게이트웨이에 robot_id, agent/bridge 세션, source timestamp, receive timestamp, frame_id, map_version_id, source age 및 명시적인 stale/unknown을 포함한다. 계획 경로는 실제 `/plan`·`/local_plan`만 제공하며 지도·TF·버전 불일치 또는 시각 증거 부족이면 그리지 않는다. 주행 궤적의 세션 경계와 보존 기간을 명시한다.
2. Depth는 원본 encoding (`16UC1` / `32FC1`), 단위·확정된 scale과 근거, invalid 표기, 수신/원본 시각, 표시 범위와 컬러맵 및 미보정 경고를 명시한다. 프레임 미수신/오래됨/재연결은 정상 영상과 구분한다. 가정값을 실측 거리로 표현하지 않는다.
3. Registry의 건물·층·버전과 로봇의 실제 사용 지도·Localization evidence를 서버가 구분해 제공한다. 층 선택은 조회 전용이며 map_server/AMCL/운용 모드 변경을 의미하지 않는다. 건물 전체 보기에는 각 층 도면과 상대 변환/정합 증거가 필요하다.
4. 게이트웨이별 독립 viewer 자격증명, 허용 origin/CSP, 접근 제어·토큰 수명·전송 보호·오류 정책을 설계한다. Fleet 제어 토큰을 영상 호스트에 보내지 않는다. 연결 그래프에는 건물 간 출입구·엘리베이터/계단 전환과 접근 제약, 좌표 변환, 검증 상태가 필요하다. 그래프가 없으면 경로를 생성하지 않는다.
5. OccupancyGrid의 유효 점유 영역 bounding box가 필요하다. 현 Fit to Floor는 실제 occupancy metadata의 해상도·origin·yaw 및 전체 이미지 경계를 사용하며, 픽셀별 유효 영역 추출은 지원하지 않는다. 경로가 없어 Fit to Route는 비활성이다.
