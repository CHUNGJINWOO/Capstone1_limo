# LIMO Pro 실물 로봇 적용 전 사전 배포 점검표 (Pre-Deployment Checklist)

> [!CAUTION]
> **절대 원칙: 시뮬레이션 성공 결과만으로 실물 안전을 보증하지 않는다.**
> Gazebo Fortress 시뮬레이션 환경은 이상적인 강체 접촉, 정밀한 타임스탬프, 제한된 마찰 모델을 가정한다.
> 실물 LIMO Pro 하드웨어 배포 시에는 바닥면 마찰 계수 변동, 휠 백래시, IMU 열 드리프트, 센서 통신 지연, 배터리 전압 강하 등 예측 불가능한 물리적 요인이 존재하므로, 반드시 본 점검표의 모든 항목을 물리적으로 검증한 후 단계적으로 투입해야 한다.

---

## 1. 사전 배포 안전 위험 매트릭스 (Pre-Deployment Risk Matrix)

| 위험도 | 위험 요소 (Hazard) | 예상 영향 | 필수 완화 및 예방 조치 (Mitigation) | 검증 방법 |
| :---: | :--- | :--- | :--- | :--- |
| **CRITICAL** | **미감지 장애물 후진 충돌** | 전방 충돌 후 자율 후진으로 후방 보행자/의자 격돌 | `enable_auto_reverse = false` 강제 적용, 영구 래칭 | 파라미터 파일 및 드라이런 확인 |
| **CRITICAL** | **소프트웨어 멈춤 시 폭주** | ROS 2 데몬 크래시 시 최종 속도 명령 유지 | 물리 비상정지(E-Stop) 리모컨 상시 파지, 1m 이내 감시 | 통신 단절 시 하드웨어 E-Stop 테스트 |
| **HIGH** | **유리벽/투명 파티션 투과** | 2D LiDAR 레이저 투과로 벽면 관통 충돌 | Stall Detector 가상 범퍼 활성화, 정적 지도에 유리벽 마킹 | 유리벽 전방 0.5m 접근 시 감속/정지 확인 |
| **HIGH** | **좁은 문틀 코너 깎기** | DWB 플래너의 급회전으로 후륜 문틀 충돌 | `inflation_radius: 0.20m`, 모서리 여유 마진 확대 | 문틀 서행 통과($0.15\text{ m/s}$) 실측 |
| **MEDIUM** | **휠 슬립에 의한 위치 발산** | 미끄러운 바닥에서 휠 오도메트리 급격한 누적 오차 | F4/F5_improved LiDAR ICP 융합 오도메트리 사용 | 직선 5m 주행 후 오도메트리 잔여 오차 측정 |
| **MEDIUM** | **타임스탬프 비동기(Clock Skew)** | 온보드 PC와 센서 간 시간 불일치로 TF 에러 | Chrony/PTP 기반 마스터-슬레이브 시간 동기화 ($< 5\text{ms}$) | `chronyc tracking` 확인 |

---

## 2. 하드웨어 및 기구 점검 (Phase 1: Hardware & Mechanical)

- [ ] **1.1 차체 외관 및 기구 체결**
  - [ ] 4륜 서스펜션 스프링 및 힌지 유격 없음 확인.
  - [ ] 조향 서보 링키지(Ackermann 조향 링크)의 중립 정렬 및 백래시 점검.
  - [ ] 전륜/후륜 고무 타이어의 마모 상태 및 좌우 공기압/접지 균일성 확인.
- [ ] **1.2 배터리 및 전원 계통**
  - [ ] 3S Li-ion 배터리 전압 측정: **`11.8 V 이상`** (완충 $12.6\text{V}$, $11.1\text{V}$ 이하 주행 금지).
  - [ ] 메인 전원 스위치 및 내부 DC-DC 컨버터(12V $\to$ 5V/19V) 발열 및 커넥터 체결 상태 점검.
- [ ] **1.3 물리 비상정지 장치 (Hardware E-Stop)**
  - [ ] 상단 물리 비상정지 버튼 기계적 동작 확인 (누름 시 모터 전원 물리적 차단, 회전 시 복귀).
  - [ ] 무선 E-Stop 리모컨 페어링 상태 및 비상 정지 신호 수신 반경(최소 10m) 확인.

---

## 3. 센서 보정 및 정적 변환 (Phase 2: Sensor Calibration & TF)

- [ ] **2.1 2D LiDAR (EAI T-mini Pro / YDLidar)**
  - [ ] 광학 커버 오염 및 지문 세척 완료.
  - [ ] 차체 수평계 기준 수평 장착 각도($\pm 0.5^\circ$ 이내) 및 지면 높이($0.16\text{ m}$) 체결 확인.
  - [ ] 실시간 스캔 주파수 확인:
    ```bash
    ros2 topic hz /scan  # 10.0 Hz ± 0.2 Hz 확인
    ```
- [ ] **2.2 온보드 IMU (HI226 / MPU6050)**
  - [ ] 로봇을 완벽히 수평한 정지 바닥에 정치한 상태에서 자이로 영점 바이어스 보정:
    ```bash
    # 각속도 잔여 바이어스 확인: |wz| <= 0.005 rad/s
    ros2 topic echo /imu/data --field angular_velocity
    ```
- [ ] **2.3 휠 엔코더 및 조향 칼리브레이션**
  - [ ] 수동으로 $1.0\text{ m}$ 직진 이동 시 `/odom` 토픽의 이동 거리 측정 ($1.00\text{ m} \pm 0.02\text{ m}$).
  - [ ] 제자리 또는 완만한 회전 시 조향각 피드백과 실제 차륜 각도 일치 확인.
- [ ] **2.4 TF 변환 트리 무결성 검증**
  - [ ] `base_footprint -> base_link -> laser_link` 정적 변환 정상 여부:
    ```bash
    ros2 run tf2_ros tf2_echo base_footprint laser_link
    ```

---

## 4. 소프트웨어 및 안전 파라미터 (Phase 3: Software Configuration)

- [ ] **3.1 Stall Detector 안전 락 확인**
  - [ ] `enable_auto_reverse: false` 설정 확인 (자율 후진 원천 봉쇄).
  - [ ] `require_operator_unlatch: true` 설정 확인.
  - [ ] 수동 래치 해제 서비스 테스트:
    ```bash
    ros2 service call /stall_detector/reset_latch std_srvs/srv/Trigger "{}"
    ```
- [ ] **3.2 Nav2 속도 및 가속도 제한**
  - [ ] 실물 배치 초기에는 최대 선속도를 $v_{\text{max}} = 0.25\text{ m/s}$, 각속도 $\omega_{\text{max}} = 0.50\text{ rad/s}$로 보수적 제한.
  - [ ] 비상 제동 감속도 가속 한계 상향 (`acc_lim_x: 1.5 m/s^2`).
- [ ] **3.3 Costmap 및 장애물 여유 마진**
  - [ ] 로봇 풋프린트: `[[-0.16, -0.11], [0.16, -0.11], [0.16, 0.11], [-0.16, 0.11]]`.
  - [ ] `inflation_layer.inflation_radius: 0.22 m` (차폭 $0.22\text{m}$ 통과 안전거리 확보).
  - [ ] `cost_scaling_factor: 8.0`.

---

## 5. 실물 주행 투입 2인 1조 현장 운영 절차 (Phase 4: Live Deployment SOP)

- [ ] **4.1 인력 배치 및 역할 분담**
  - **운영자 1 (Base Station Console)**: 내비게이션 목표 지정, RViz2 모니터링, 토픽 상태 및 오도메트리 진단.
  - **안전 감시자 2 (Safety Watcher)**: **물리 E-Stop 리모컨을 손에 쥐고 로봇 후방 1m 이내에서 상시 보행 추종**.
- [ ] **4.2 단계별 실물 검증 주행 (Staged Rollout)**
  1. **Stage 1 (개방 공간 5m 직선 주행)**: 장애물이 없는 넓은 공간에서 $5\text{m}$ 목표 이동 및 제동 안정성 검증.
  2. **Stage 2 (단일 90도 코너링)**: 복도 진입 전 단일 모퉁이 회전 반경 및 차체 후미 간섭 검증.
  3. **Stage 3 (문틀 및 협소 통로 저속 통과)**: 목표 통로를 $0.15\text{ m/s}$ 저속으로 수동 통과 후 자율주행 실행.
  4. **Stage 4 (유리벽 가상 범퍼 접촉 모의 시험)**: 의도적으로 유리벽 전방에서 정체 유발 시 `STALL_LATCHED` 즉각 발동 및 구동 정지 확인.
