# 실제 LIMO 재연결 시 확인할 사항 — 이번에는 실행하지 않음

이번 작업은 SSH, 실시간 ROS 토픽 조회, 실차 프로세스 변경, 모터 구동 없이 노트북에서만 검증했습니다. 기존 실차용 소스/Agent/Depth/Nav2/Gate/자동 맵핑 파일을 수정하지 않았습니다.

1. **기존 담당 작업과 상태 확인:** Claude의 현장 작업 완료 여부, 활성 publisher/프로세스 소유권, 모터 잠금·비상정지 절차를 먼저 확인합니다. 새 코드 배포/Agent 재시작이 필요하면 범위·위험을 보고하고 별도 승인을 받습니다.
2. **읽기 전용 입력 계약:** 기존 sim/real bridge envelope와 실제 camera frame/encoding/CameraInfo/QoS/source clock을 대조합니다. loopback lab의 SYNTHETIC/REPLAY source를 LIVE로 이름만 바꿔 연결 성공으로 보고하지 않습니다.
3. **RGB sender 최소 적용:** 별도 사용자 폴더·별도 프로세스에서 최신 한 슬롯, resize480px/q65, 우선10~12FPS를 시험합니다. 기존 raw ROS camera 주기와 Nav2/Gate 데이터 주기는 변경하지 않습니다. 실제 YOLO worker의 매 프레임 추론을 다시 포함해야 합니다. 이번 lab의 사전계산 결과를 그대로 운영 검출로 쓰지 않습니다.
4. **Depth 분리:** 관제 preview320px/5~8FPS부터 시작하고 거리용 네이티브 데이터를 별도 보존합니다. DaBai16UC1의 실제 단위/스케일과 유효 범위를 확인하기 전 거리 정확도를 주장하지 않습니다. RGB-D registration과3D 객체 위치는 별도 검증입니다.
5. **자원 예산:** camera/SLAM/Nav2/Gate watchdog 주기와 callback 지연, 실차 CPU/RAM/USB/네트워크를 추가 전후로 측정합니다. 이번 노트북 평균19% CPU 표본을 LIMO 추가 부하로 환산하지 않습니다. 과부하가 있으면 관제 FPS/해상도부터 낮추고 안전 센서 주기를 낮추지 않습니다.
6. **좌표와 세션:** 실제 map origin/resolution/yaw, local rolling origin, TF stamp, 경로 frame, 로봇/map session이 일치하는지 확인합니다. 다른 로봇의 map 프레임을 이름만 같다고 합치지 않습니다.
7. **성능:** 단일 전경 표시 탭과 다중/비활성 탭을 분리해 실제 표시 FPS, P95/P99, drop/queue/clock uncertainty를 측정합니다. 원본 camera capture→동일 프레임 결과→브라우저까지 측정하며 replay 생성 지연과 구분합니다.
8. **장애:** 프로그램 수준 지연/단절부터 시작합니다. 실차 Wi-Fi/Tailscale을 끊거나 변경하지 않습니다. 오래된 frame·YOLO 결과가 거부되고 새로운 boot 상태가 분리되는지 확인합니다. 미디어 장애가 주행 제어로 전파되지 않도록 독립 프로세스/큐를 유지합니다.
9. **연결 방식 결정:** 현재 로컬 비교에서는 JPEG/TCP가 더 낮은 지연, H.264는 더 적은 전송량, WebRTC는 추가 playout 지연을 보였습니다. 현장 uplink/브라우저/CPU에서 다시 비교한 뒤 선택합니다. WebRTC 도입을 이유로 새 외부 STUN/TURN이나 공개 포트를 임의로 만들지 않습니다.
10. **중앙 관제:** Mission Manager GET과 Adapter 관측 heartbeat/snapshot을 실제 소스에 연결하고 미설정은 UNAVAILABLE로 남깁니다. 임무 제출·원격 정지·Fleet Adapter 제어는 이 시각화 작업 범위가 아닙니다.

모터 잠금·읽기 전용 검증과 승인된 배포가 끝나기 전 새 lab을 실차에 자동 실행시키는 서비스/SSH 스크립트는 만들지 않았습니다.
