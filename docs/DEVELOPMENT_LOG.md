## 2026-09-30 — LIMO 작업 기록

### 1. 작업 환경

- 로봇: AgileX LIMO
- PC: LIMO onboard PC
- OS: Ubuntu 22.04
- ROS 2: Humble
- Workspace: `~/agilex_ws`

---

### 2. Repository 및 프로젝트 문서 정리

LIMO ROS 2 패키지는 다음과 같이 구성되어 있다.

- `limo_base`
- `limo_car`
- `limo_description`
- `limo_msgs`

LIMO ROS 2 패키지의 주요 구성요소를 확인하였다.

- LIMO base driver
- Serial communication
- TF 관련 코드
- LIMO message
- Ackermann model
- Gazebo simulation resources
- RViz configuration
- LiDAR 관련 launch 구성

#### 프로젝트 문서 정리

기존 프로젝트 문서를 검토하고 계획과 실제 개발 기록을 분리하기 위해 문서 구조를 정리하였다.

현재 주요 문서:

- `docs/PROJECT_PLAN.md`
- `docs/DEVELOPMENT_LOG.md`

`PROJECT_PLAN.md`는 프로젝트의 목표와 개발 단계를 기록한다.

`DEVELOPMENT_LOG.md`는 실제로 수행한 개발과 검증 결과를 기록한다.

---

### 3. Ackermann 모델 수정

`limo_car/gazebo/ackermann_with_sensor.xacro`를 수정하였다.

기존 파일에는 Ackermann 모델을 실제로 생성하는 `xacro:limo_ackermann` 인스턴스가 없었다.

다음 내용을 추가하였다.

```xml
<xacro:limo_ackermann/>
```

이를 통해 센서가 포함된 Ackermann 모델 구성에서 Ackermann 모델 인스턴스가 생성되도록 수정하였다.

---

### 4. 개발 기록 원칙

캡스톤 개발 과정에서 실제로 확인하지 않은 기능은 완료된 것으로 기록하지 않는다.

개발 기록은 다음 원칙을 따른다.

- 실제 실행한 명령만 기록한다.
- 실제 확인한 결과만 완료로 기록한다.
- 계획과 구현 결과를 구분한다.
- 오류가 발생하면 원인과 해결 과정을 기록한다.
- 코드 변경과 테스트 결과를 함께 기록한다.
- 실제 주행 및 센서 검증 결과는 해당 환경에서 직접 확인한 후 기록한다.

---

# 5. LIMO Base 및 센서 실기기 검증

## 5.1 ROS 2 환경 설정

LIMO 관련 작업을 시작하기 전에 ROS 2와 workspace를 source한다.

```bash
source /opt/ros/humble/setup.bash
source ~/agilex_ws/install/setup.bash
```

현재 환경에서는 다음 경고가 발생한다.

```text
not found: "/home/pcu/agilex_ws/install/limo_car/share/limo_car/local_setup.bash"
```

이 경고가 출력되더라도 이번 작업에서는 LIMO Base와 센서 Topic이 실제로 동작하는 것을 확인하였다.

---

## 5.2 LIMO Base 실행

기존 기본 설정은 `/dev/ttylimo`를 사용했으나 해당 장치가 존재하지 않았다.

따라서 실제 확인된 USB 포트인 `/dev/ttyUSB1`을 명시적으로 지정하여 실행하였다.

```bash
ros2 launch limo_base limo_base.launch.py port_name:=ttyUSB1
```

실행 결과:

```text
Loading parameters:
- port name: ttyUSB1
- odom frame name: odom
- base frame name: base_link

connet the serial port:'/dev/ttyUSB1'
Open the serial port:'/dev/ttyUSB1'
```

별도 터미널에서:

```bash
ros2 node list
```

결과:

```text
/limo_base_node
```

따라서 `/dev/ttyUSB1`을 이용한 LIMO Base 실행을 확인하였다.

---

## 5.3 LIMO Base Topic 확인

LIMO Base가 실행된 상태에서:

```bash
ros2 topic list
```

다음 Topic이 생성되는 것을 확인하였다.

```text
/cmd_vel
/imu
/limo_status
/parameter_events
/rosout
/tf
/tf_static
/wheel/odom
```

따라서 LIMO Base가 /dev/ttyUSB1을 통해 실행되고 있으며, /imu, /wheel/odom, /cmd_vel, /limo_status 등의 ROS 2 Topic이 생성되는 것을 확인하였다.

---

# 6. Wheel Odometry 확인

다음 명령으로 한 번만 데이터를 출력하였다.

```bash
ros2 topic echo /wheel/odom --once
```

`--once` 옵션을 사용하면 Topic 데이터를 계속 출력하지 않고 한 번만 출력할 수 있어 센서 값을 개별적으로 확인할 때 편리하다.

확인된 주요 Frame:

```text
frame_id: odom
child_frame_id: base_link
```

실제 측정값을 여러 번 확인한 결과 위치 값이 변화하였다.

예:

```text
x: 0.059886...
y: -0.029865...
```

다시 측정:

```text
x: 0.022201...
y: -0.249523...
```

따라서 `/wheel/odom`이 동일한 값을 반복해서 내보내는 것이 아니라 실제 LIMO의 상태 변화에 따라 값이 변화하는 것을 확인하였다.

---

# 7. IMU 확인

다음 명령으로 IMU 데이터를 한 번 출력하였다.

```bash
ros2 topic echo /imu --once
```

확인된 Frame:

```text
frame_id: imu_link
```

Orientation, Angular velocity, Linear acceleration 등의 데이터가 출력되었다.

예:

```text
linear_acceleration:
  x: 0.02
  y: -0.22
  z: 10.03
```

여러 번 측정했을 때 Orientation 및 일부 IMU 값이 변화하는 것을 확인하였다.

## 7.1 IMU 주기 확인

```bash
ros2 topic hz /imu
```

측정 결과:

```text
average rate: 약 99.9 Hz
```

따라서 `/imu`가 약 **100 Hz**로 publish되는 것을 확인하였다.

---

# 8. TF 확인

## 8.1 주요 TF 구조 확인

다음 명령으로 TF Tree를 확인하였다.

```bash
ros2 run tf2_tools view_frames
```

생성된 PDF 파일을 확인하였다.

```bash
ls -l frames_*.pdf
```

주요 확인 결과:

odom
└── base_link
    └── imu_link

base_link
└── laser_frame

---

## 8.2 odom → base_link 확인

```bash
ros2 run tf2_ros tf2_echo odom base_link
```

실제 Translation과 Rotation 값이 출력되는 것을 확인하였다.

예:

```text
Translation:
[0.045, -0.053, 0.000]
```

따라서 다음 TF가 정상적으로 존재하는 것을 확인하였다.

```text
odom → base_link
```

---

## 8.3 base_link → imu_link 확인

`view_frames` 결과에서 다음 관계를 확인하였다.

```text
base_link
└── imu_link
```

따라서 LIMO Base와 IMU Frame의 TF 관계를 확인하였다.

---

# 9. YDLiDAR T-mini Plus 확인

## 9.1 USB 장치 확인

다음 명령으로 USB 장치를 확인하였다.

```bash
ls -l /dev/ttyUSB*
```

결과:

```text
/dev/ttyUSB0
/dev/ttyUSB1
```

USB 장치 정보를 확인한 결과 `/dev/ttyUSB0`에서 YDLiDAR T-mini Plus가 연결되는 것을 확인하였다.

다음 명령으로 USB 장치 정보를 확인하였다.

```bash
lsusb | grep -i -E "CP210|Silicon|YDLIDAR"
```

Silicon Labs CP210x UART Bridge가 확인되었다.

---

## 9.2 LiDAR Driver 실행

현재 환경에서 YDLiDAR Driver는 다음과 같이 실행하였다.

```bash
source /opt/ros/humble/setup.bash
source ~/agilex_ws/install/setup.bash

ros2 launch ydlidar_ros2_driver ydlidar_launch.py \
params_file:=/home/pcu/agilex_ws/src/ydlidar_ros2_driver/params/tmini.yaml
```

실제 확인 과정에서 `/dev/ttyUSB0`으로 YDLiDAR T-mini Plus가 연결되는 것을 확인하였다.

---

## 9.3 YDLiDAR 정상 연결 확인

정상적으로 연결된 경우 다음 메시지가 출력되었다.

```text
Lidar successfully connected [/dev/ttyUSB0:230400]
```

실제 T-mini Plus 장치 정보도 확인하였다.

```text
Model: Tmini Plus
Firmware version: 1.3
Hardware version: 1
Serial: 2024000400170358
```

또한 다음 메시지를 확인하였다.

```text
Lidar running correctly! The health status good
Successed to start scan mode
Scan Frequency: 10.00Hz
Sample Rate: 4.00K
Now lidar is scanning...
```

따라서 YDLiDAR T-mini Plus가 정상적으로 연결되고 Scan Mode가 시작되는 것을 확인하였다.

---

# 10. /scan LaserScan 확인

LiDAR Driver가 실행된 상태에서 다음 명령을 사용하였다.

```bash
ros2 topic echo /scan --once
```

실제 LaserScan 메시지를 확인하였다.

주요 내용:

```text
frame_id: laser_frame
angle_min: 약 -π
angle_max: 약 π
range_min: 0.03
range_max: 12.0
```

`ranges` 배열에도 실제 거리 데이터가 존재하였다.

예:

```text
5.315
5.302
5.239
...
0.259
0.245
...
```

따라서 다음 구조가 실제로 동작하는 것을 확인하였다.

```text
YDLiDAR
    ↓
YDLiDAR Driver
    ↓
/scan
    ↓
sensor_msgs/msg/LaserScan
```

---

# 11. base_link → laser_frame TF 확인

LiDAR Driver가 실행된 상태에서 다음 명령을 사용하였다.

```bash
ros2 run tf2_ros tf2_echo base_link laser_frame
```

최종적으로 다음 변환을 확인하였다.

```text
Translation:
[0.000, 0.000, 0.020]

Rotation:
[0.000, 0.000, 0.000, 1.000]
```

따라서 다음 TF 관계를 확인하였다.

```text
base_link
└── laser_frame
```

---

# 12. RViz LaserScan 확인

RViz2를 실행하였다.

```bash
rviz2
```

RViz에서:

```text
Add
→ by topic
→ /scan
→ LaserScan
```

을 추가하였다.

현재 실제 검증에서는 **LaserScan을 추가했지만 좌표평면에 LiDAR Scan 점이 표시되지 않았다.**

현재까지 확인한 내용:

```text
/scan Topic 존재
      ↓
실제 LaserScan 데이터 존재
      ↓
frame_id = laser_frame
      ↓
base_link → laser_frame TF 존재
      ↓
RViz에서 /scan → LaserScan 추가
      ↓
좌표평면에 Scan 표시 안 됨
```

따라서 현재 단계에서는 LiDAR 자체 또는 `/scan` 생성 문제로 단정하지 않고 RViz의 Fixed Frame, QoS, TF 연결 등의 가능성을 추가로 확인할 필요가 있다.

이 문제는 다음 작업에서 계속 확인한다.

---

# 13. 발생한 오류 및 확인 내용

## 13.1 `/dev/ttylimo` 없음

기본 LIMO Base 실행:

```bash
ros2 launch limo_base limo_base.launch.py
```

오류:

```text
Failed to open: '/dev/ttylimo'
```

확인 결과 실제 USB 장치는:

```text
/dev/ttyUSB0
/dev/ttyUSB1
```

따라서 실제 장치를 지정하여:

```bash
ros2 launch limo_base limo_base.launch.py port_name:=ttyUSB1
```

로 실행하였다.

### 해결 결과

- `/limo_base_node` 정상 실행
- `/imu` 생성 확인
- `/wheel/odom` 생성 확인
- `/cmd_vel` 생성 확인
- `/limo_status` 생성 확인

---

## 13.2 LiDAR Driver 실행 실패

일부 설정 및 포트 조합에서는 다음 오류가 발생하였다.

```text
Error, cannot retrieve Lidar health code -1
Fail to get baseplate device information!
Failed to start scan mode -1
```

해당 실행에서는 Driver가 종료되었다.

반대로 올바른 장치 및 설정에서는:

```text
Lidar successfully connected
Lidar running correctly! The health status good
Successed to start scan mode
Now lidar is scanning...
```

까지 확인하였다.

따라서 오류가 발생한 실행 결과와 정상 실행 결과를 구분해서 기록하였다.

---

## 13.3 `/scan`은 존재하지만 RViz에 표시되지 않음

현재까지 확인한 내용:

- `/scan` Topic 존재
- 실제 LaserScan 데이터 존재
- `frame_id: laser_frame`
- `base_link → laser_frame` TF 존재
- RViz에서 `/scan → LaserScan` 추가

하지만 좌표평면에 Scan 점이 표시되지 않았다.

따라서 **LiDAR 자체 또는 `/scan` 생성 문제로 단정하지 않고 RViz의 Fixed Frame / QoS / TF 연결 문제와 구분하여 기록하였다.**

---

# 14. 현재 검증 환경의 실행 절차

## 터미널 1 — LIMO Base

```bash
source /opt/ros/humble/setup.bash
source ~/agilex_ws/install/setup.bash

ros2 launch limo_base limo_base.launch.py port_name:=ttyUSB1
```

실행 후 별도 터미널에서:

```bash
ros2 node list
```

확인:

```text
/limo_base_node
```

---

## 터미널 2 — YDLiDAR Driver

```bash
source /opt/ros/humble/setup.bash
source ~/agilex_ws/install/setup.bash

ros2 launch ydlidar_ros2_driver ydlidar_launch.py \
params_file:=/home/pcu/agilex_ws/src/ydlidar_ros2_driver/params/tmini.yaml
```

---

## 터미널 3 — 센서 및 TF 확인

### Topic 목록

```bash
ros2 topic list
```

### Wheel Odometry

```bash
ros2 topic echo /wheel/odom --once
```

### IMU

```bash
ros2 topic echo /imu --once
```

### IMU 주기

```bash
ros2 topic hz /imu
```

### LiDAR

```bash
ros2 topic echo /scan --once
```

### TF 전체 확인

```bash
ros2 run tf2_tools view_frames
```

### Odometry TF

```bash
ros2 run tf2_ros tf2_echo odom base_link
```

### LiDAR TF

```bash
ros2 run tf2_ros tf2_echo base_link laser_frame
```

---

## 터미널 4 — RViz

```bash
rviz2
```

RViz에서:

```text
Add
→ by topic
→ /scan
→ LaserScan
```

현재는 Scan 점이 표시되지 않는 상태이므로 Fixed Frame, QoS, TF 등을 추가 확인해야 한다.

---

# 15. 작업 결과

- [v] LIMO ROS 2 패키지 구조 확인
- [v] 프로젝트 문서 구조 정리
- [v] `PROJECT_PLAN.md`와 `DEVELOPMENT_LOG.md`의 역할 분리
- [v] `ackermann_with_sensor.xacro`의 Ackermann 모델 인스턴스 수정
- [v] `/dev/ttyUSB0`에서 **YDLiDAR T-mini Plus 확인**
- [v] **YDLiDAR 정상 연결 및 Scan Mode 시작 확인**
- [v] **`/scan` LaserScan 실제 데이터 확인**
- [v] `/dev/ttyUSB1`을 이용해 **LIMO Base 실행**
- [v] **`/limo_base_node` 정상 실행 확인**
- [v] **`/wheel/odom` 실제 데이터 확인 및 값 변화 확인**
- [v] **`/imu` 실제 데이터 확인**
- [v] **`/imu` 약 100 Hz publishing 확인**
- [v] **odom → base_link TF 확인**
- [v] **base_link → imu_link TF 확인**
- [v] **base_link → laser_frame TF 확인**
- [v] **RViz에서 /scan LaserScan Display 추가 확인**
- [ ] **RViz 좌표평면에서 LaserScan 점 표시 — 미해결**

### 다음 작업

1. RViz의 Fixed Frame 확인
2. `/scan` QoS 확인
3. TF 연결 상태 재확인
4. RViz에서 LaserScan 표시 문제 해결
5. LiDAR와 LIMO Base를 동시에 실행한 상태에서 RViz 재확인
6. 이후 SLAM Toolbox 실행 및 실내 지도 생성 단계로 진행
