# 11월 18일 지도 수집 사전 점검 — 2026-11-17

## 결론

실차 접속·기동은 하지 않았다. 현재 웹 SLAM은 SIM_ONLY이고 실차 웹 녹화도 차단되어 있어 **내일 웹 기반 실차 지도 수집이 가능하다고 확정할 수 없다.** 기존 배포본의 물리 확인·전원·USB·E-Stop·watchdog 조건은 유지한다. 실차 접속 및 기동은 별도 승인 후 현장 담당자의 확인이 필요하다.

## 코드·문서 근거

- `real_auto_mapping_ready_20261113` launch는 LiDAR, base, TF/odom, SLAM 및 Nav2를 함께 포함하고 탐색은 별도 시작이다. scan normalizer 활성 시 `/scan`을 244빔 `/scan_slam`으로 바꾸어 SLAM에 사용한다. Depth 사용은 해당 launch 설정과 장치에 달려 있다. 이는 현재 실차 설치 상태를 증명하지 않는다.
- 준비본은 `/cmd_vel_locked` 소프트웨어 경로, 지도 raster 및 가능한 posegraph 저장을 포함한다. 웹 SIM SLAM이 별도로 기동되면 기존 SLAM/map 및 TF와 충돌할 수 있고, 별도 rosbag 기동은 중복 기록·용량 문제를 일으킨다. 현재 graph 검사와 동일 root flock은 외부 프로세스 부재나 레이스 방지를 증명하지 않는다. 불명 시 시작하지 않는다.
- 제조사의 [LIMO Pro 매뉴얼](https://github.com/agilexrobotics/limo_pro_doc/blob/master/Limo%20Pro%20user%20manual%28EN%29.md)은 배터리가 차체, 컴퓨터, 센서에 전원을 공급한다고 설명한다. 이 자료와 기존 로컬 기록에서 **모터 에너지만 차단한 채 센서 전원을 유지하는 승인된 방법을 확인하지 못했다.** 제조사와 안전 담당자 확인이 필요하며 임의 배선 변경은 금지한다.
- 웹 recorder가 `run_recorder.py`에 고정 `--extra /scan_slam`을 전달하도록 수정했다. 임의 토픽 명령 API는 추가하지 않았다. `/tf_static` QoS 및 실제 로봇에서의 메시지 수신은 아직 확인되지 않았다.

## 결과

- 격리 ROS 실제 recorder: 합성 `/scan_slam` `sensor_msgs/msg/LaserScan` **5건 기록** 및 타입/QoS를 지정한 실제 재생 수신 PASS. `/tmp/limo-scan-slam-20261117-b/{bag-info.log,play.log,echo.log}`. 첫 시험 `...-a`는 구독자의 토픽 타입 자동 탐색 실패로 재생 수신 FAIL; bag 기록 5건은 성공했다.
- recorder 단위 회귀 5개 PASS. 웹 녹화 최초 시험 `.../limo-nov18-web-20261117-a`는 합성 `/scan` 단발 발행이 구독 준비보다 빨라 bag 메시지 0건으로 FAIL이었다. 여러 번 발행한 `.../limo-nov18-web-20261117-b`에서는 브라우저 시작·중지·nonzero scan·Agent 재시작 UNKNOWN은 PASS였으나, 마지막 Agent 종료 코드가 **-11**로 종료 안정성은 FAIL이다. 테스트 프로그램은 이 종료 코드를 실패로 전파하지 않았으므로 프로세스 exit 0을 전체 PASS로 해석하지 않는다. 실차 데이터·물리 잠금·실제 장치별 토픽/QoS·Depth 및 장시간 데이터는 SKIP.
- Agent SIGSEGV 선행 조사에서는 sandbox DDS interface 제한이 동반된 종료 6회 중 1회 및 별도 8회 중 1회 SIGSEGV; faulthandler는 executor의 `rclpy.signals` guard condition 정리 스택을 기록했다. 이번 추가 sandbox 재현 6회는 정상 종료해 재현되지 않았다 (`/tmp/limo-nov18-executor-20261117-a/result.json`). 이전 loopback 허용 비교 8회도 정상 종료했으나 원인 미확정이며 이전 실패는 유지한다. [상세](REAL_LOCKED_PREPARATION_20261117.md).

## 이후 현장 점검 (별도 실차 접속 승인 후에만)

1. 제조사·현장 안전 담당자와 물리 차단/센서 전원 양립, E-Stop·watchdog 및 전원·USB 이상 이력의 해소 여부를 확인한다. 확인 전 기동하지 않는다.
2. 실제 배포본·설정의 파일 hash, 프로세스 소유자, ROS domain과 기존 SLAM/Nav2/map publisher 및 기록기를 확인한다. 다른 프로세스를 임의 종료하거나 UNKNOWN을 강제 해제하지 않는다.
3. 승인된 저장 디렉터리의 `df -h -- <경로>`, `df -i -- <경로>`로 여유를 점검하고 예상 수집량·임시 복원 공간을 산정한다. 실제 Agent의 `recording_root`, 지도·posegraph root 및 노트북 보관 위치는 아직 미확인이다.
4. `ros2 node list`, `ros2 topic list -t`, `ros2 topic info --verbose /scan_slam`, `ros2 param get /slam_toolbox scan_topic` 등은 해당 ROS graph 접근에 대한 별도 승인 후 읽기 점검에만 사용한다. CLI daemon이 시작될 수 있다. `/scan`, `/scan_slam`, `/tf`, `/tf_static`, odom 및 Depth 토픽의 타입·QoS·주기와 frame 연결을 현장에서 확인한다.
5. 실제 저장/종료 절차는 배포본 소유자와 안전 담당자가 현장 조건을 확인한 뒤 확정한다. 지도·posegraph의 생성과 재로드, rosbag 정상 종료·`ros2 bag info` 메시지 수 및 무결성을 확인하고 원본을 보존한다. 서버 ACK나 software stop을 물리 정지 증거로 취급하지 않는다.

실행 가능한 현장 지도 수집 절차는 아직 승인·검증되지 않았다. 이전 수집 실험의 성공을 이번 시스템의 실차 준비 완료로 확대하지 않는다. 공통 문서 대신 본 독립 기록을 남긴다.
