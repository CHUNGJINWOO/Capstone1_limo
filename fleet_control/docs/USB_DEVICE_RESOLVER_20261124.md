# USB Device Resolver 구현 및 검증 문서 (2026-11-24)

## 1. 목적 및 범위
- 학교 소유 LIMO의 OS(`/etc/udev/rules.d/`), USB 전원 관리, EEPROM/펌웨어, 벤더 드라이버(`limo_base`, `ydlidar_ros2_driver`), systemd 서비스를 **변경하지 않고**, `limo_capstone` 패키지 내부에서 USB 시리얼 포트 번호 및 물리 포트 경로 변경에 안전하게 대응합니다.
- **물리 장애와 포트 변경의 엄격한 구분**: USB 허브(`1a40:0101`) 자체가 커널에 미열거되는 물리/전원 장애는 소프트웨어로 복구할 수 없으며, 이 경우 `BASE_HUB_NOT_ENUMERATED`로 fail-closed 차단합니다.
- **미구현(범위 외)**: 웹 관제 하드웨어 진단 UI, 자동 USB 전원 제어/리셋, 자동 드라이버 재시작, 자동 모터 재무장, Agent Harness M1.

## 2. 구현 파일 (통합 브랜치 `integration/capstone1-field-release-20261124`, `01761cd4` 후속 커밋)
- `capstone_ws/src/limo_capstone/limo_capstone/usb_device_resolver.py`: Python 표준 라이브러리 전용 읽기 전용·fail-closed USB 시리얼 탐색 모듈 및 CLI.
- `capstone_ws/src/limo_capstone/launch/limo_integrated_auto_mapping_real.launch.py`: `use_usb_resolver`(기본 `true`), `usb_resolve_timeout_sec`(기본 `5.0`), `usb_resolve_interval_sec`(기본 `0.5`) 인자 및 `OpaqueFunction(resolve_serial_drivers)` 최소 연결.
  - 기존 `/dev/ttylimo`·`/dev/ydlidar` 정상 경로일 때 기존과 동일하게 동작하며, `limo_base` 명령 토픽 리매핑(`/limo_driver/cmd_vel`)과 Watchdog·Gate 경로를 전혀 변경하지 않습니다.
  - `limo_base`에는 `port_name`(basename), `ydlidar_ros2_driver_node`에는 `SetParameter(name='port', value=<전체 경로>)`만 전달합니다.
- `capstone_ws/src/limo_capstone/test/test_usb_device_resolver.py`: 합성 sysfs fixture 기반 단위 시험 14종.

## 3. 장치 식별 및 Fail-Closed 정책
1. `/dev/ttylimo`, `/dev/ydlidar` 고정 링크를 우선 확인하되, 링크 대상의 실제 USB 토폴로지(`10c4:ea60` + 부모 허브 `1a40:0101` 유무)가 역할과 일치하는지 검증합니다.
2. `ttyUSB` 번호 순서, VID/PID/Serial(`10c4:ea60`, `0001`), 부모 허브 VID/PID 단일 조건만으로 역할을 확정하지 않습니다.
3. 전체 `10c4:ea60`가 3개 이상이거나, `1a40:0101` 허브가 2개 이상이거나, 역할별 후보가 중복되거나, 고정 링크가 잘못된 장치를 가리키면 `AMBIGUOUS_CP2102_CANDIDATES` 또는 `PORT_COLLISION`으로 즉시 fail-closed 차단합니다.
4. 지연 인식에 대해서는 제한된 재시도(`resolve_with_retry`, 기본 최대 `5.0 s`, 간격 `0.5 s`)만 수행하고, 드라이버 실행 직전 `verify_before_start`로 장치 지문(실제 경로·USB 경로·VID/PID·조상 허브)을 재검증해 변경 시 `DEVICE_CHANGED_BEFORE_START`로 차단합니다.
5. 오류 코드: `BASE_HUB_NOT_ENUMERATED`, `BASE_PORT_NOT_FOUND`, `LIDAR_PORT_NOT_FOUND`, `AMBIGUOUS_CP2102_CANDIDATES`, `PORT_COLLISION`, `PORT_PERMISSION_DENIED`, `DEVICE_CHANGED_BEFORE_START`.

## 4. Launch 인자 및 탐색 대기 시간 설정 / 안전 한계
- `use_usb_resolver` (기본 `true`): USB 토폴로지 기반 fail-closed 장치 검증 활성화.
  - **`use_usb_resolver:=false` 주의**: 안전 검사를 우회하여 기존 고정 기본값(`port_name:='ttylimo'`, `port:='/dev/ydlidar'`)으로 드라이버를 직접 기동하는 **비상·하위 호환 전용 경로**입니다. 장치 미연결·오연결·권한 오류를 사전에 차단하지 못하므로 현장 운영에서는 기본값 `true` 유지를 권장합니다.
- `usb_resolve_timeout_sec` (기본 `5.0`): USB 장치 열거 지연을 기다리는 최대 제한 시간(초). 무한 대기 및 자동 USB reset은 금지되어 있습니다.
- `usb_resolve_interval_sec` (기본 `0.5`): 재탐색 폴링 간격(초).
  - 예: `ros2 launch limo_capstone limo_integrated_auto_mapping_real.launch.py allow_motor_drive:=false usb_resolve_timeout_sec:=8.0 usb_resolve_interval_sec:=0.5`
- **장치 분리·재연결 시 자동 운동 재개 금지**: 운용 중 USB가 분리되거나 재연결되어도 `limo_base`·드라이버 자동 재시작이나 Gate/Watchdog 자동 재무장을 수행하지 않습니다. 기존 Safety Gate(`FAILSAFE_SENSOR_TIMEOUT` 래치)와 Watchdog 고장 대응 및 운영자 수동 재승인·재기동 절차를 그대로 유지합니다.

## 5. 사용 예시 (읽기 전용 진단 CLI)
```bash
python3 capstone_ws/src/limo_capstone/limo_capstone/usb_device_resolver.py --timeout 5.0 --interval 0.5 --json
```

## 6. 검증 결과
- **합성 sysfs 단위 시험 (`test_usb_device_resolver.py`)**: **14/14 PASS (0.134s)** — 정상 고정 링크, `ttyUSB0`/`ttyUSB1` 스왑, 고정 링크 부재, 물리 USB 포트 변경, 동일 VID/PID/Serial 미구분, 3번째 CP2102 추가, 동일 모델 부모 허브 중복, 잘못된/충돌 고정 링크, Base 허브 미열거, 허브 있음/포트 없음, 지연 인식(제한 재시도·타임아웃), 권한 오류(즉시 실패), 분리·재연결 및 실행 직전 변경, CLI JSON 종료 코드.
- **실제 ROS 2 Humble LaunchService + Mock 드라이버 통합 시험 (`test_real_launch_usb_resolver_ros2.py`)**: `OpaqueFunction(resolve_serial_drivers)` 및 `SetParameter(port)`의 실제 파라미터 전달(`port_name`, `port`), `/limo_driver/cmd_vel` 리매핑 유지, 미인식/식별 불확실 시 드라이버 미기동(fail-closed), `use_usb_resolver:=false` 우회 동작 검증.
- **실차 읽기 전용 프로브 (`pcu@100.124.58.112`, stdin 실행·파일 미저장)**: `errors=["BASE_HUB_NOT_ENUMERATED"]`, `lidar_candidate_ttys=["ttyUSB0"]`, `base=null`, `exit=2` — 오할당 0건 확인.
- **미검증**: 실제 LIMO에 설치 및 `1a40:0101` 허브 복구 후 실기 런타임 검증 (배포 승인 대기). **`REAL_MOTION_READY: BLOCKED` 유지.**
