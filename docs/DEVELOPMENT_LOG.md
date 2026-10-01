## 2026-09-30 — LIMO 작업 기록

### 1. 작업 환경

* 로봇: AgileX LIMO
* PC: LIMO onboard PC
* OS: Ubuntu 22.04
* ROS 2: Humble
* Workspace: `~/agilex_ws`

LIMO onboard PC에서 ROS 2 Humble 환경을 구성하고 실제 LIMO에 연결된 센서와 ROS 2 Topic을 직접 확인하였다.

[출처: 실기기 검증]

---

### 2. Repository 및 프로젝트 문서 정리

LIMO ROS 2 패키지 구조를 확인하였다.

주요 패키지는 다음과 같다.

* `limo_base`
* `limo_car`
* `limo_description`
* `limo_msgs`
* `ydlidar_ros2_driver`

LIMO ROS 2 패키지의 주요 구성요소를 확인하였다.

* LIMO base driver
* Serial communication
* TF 관련 코드
* LIMO message
* Ackermann model
* Gazebo simulation resources
* RViz configuration
* LiDAR 관련 launch 구성

프로젝트 문서에서는 LIMO의 센서와 TF 구조 및 이후 SLAM, Localization, Navigation2로 이어지는 개발 흐름을 정의하고 있다.

[출처: 교재]

---

### 3. Ackermann 모델 수정

`limo_car/gazebo/ackermann_with_sensor.xacro`를 확인하였다.

기존 파일에서 Ackermann 모델 인스턴스가 실제로 생성되도록 다음 내용을 추가하였다.

```xml
<xacro:limo_ackermann/>
```

이를 통해 센서가 포함된 Ackermann 모델 구성에서 Ackermann 모델 인스턴스가 생성되도록 수정하였다.

[출처: 실기기 검증]

---

### 4. ROS 2 환경 설정

LIMO 관련 작업을 시작하기 전에 다음 명령으로 ROS 2와 workspace를 source하였다.

```bash
source /opt/ros/humble/setup.bash
source ~/agilex_ws/install/setup.bash
```

현재 workspace에서는 다음 경고가 발생하였다.

```text
not found: "/home/pcu/agilex_ws/install/limo_car/share/limo_car/local_setup.bash"
```

해당 경고가 발생하였지만 ROS 2 명령 및 LIMO 관련 노드 실행은 진행되었다.

[출처: 실기기 검증]

---

### 5. USB 장치 확인

다음 명령으로 연결된 USB Serial 장치를 확인하였다.

```bash
ls -l /dev/ttyUSB*
```

확인 결과:

```text
/dev/ttyUSB0
/dev/ttyUSB1
```

USB 장치의 상세 정보를 확인하였다.

```bash
ls -l /dev/serial/by-path/
```

확인된 장치 경로:

```text
pci-0000:00:14.0-usb-0:3:1.0-port0 -> ../../ttyUSB0
pci-0000:00:14.0-usb-0:7.4:1.0-port0 -> ../../ttyUSB1
```

또한 USB 장치가 Silicon Labs CP2102 USB-to-UART Bridge Controller임을 확인하였다.

[출처: 실기기 검증]

---

### 6. LIMO Base 실행

LIMO Base Driver를 실행하였다.

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

실행 결과:

```text
/limo_base_node
```

따라서 실제 LIMO onboard PC에서 `/dev/ttyUSB1`을 이용하여 LIMO Base Node가 실행되는 것을 확인하였다.

[출처: 교재 + 실기기 검증]

---

### 7. LIMO Base Topic 확인

LIMO Base가 실행된 상태에서 다음 명령으로 Topic을 확인하였다.

```bash
ros2 topic list
```

주요 Topic:

```text
/cmd_vel
/imu
/limo_status
/wheel/odom
/tf
/tf_static
```

LIMO Base가 정상적으로 실행되면서 IMU, Wheel Odometry 및 제어 관련 Topic이 생성되는 것을 확인하였다.

[출처: 실기기 검증]

---

### 8. Wheel Odometry 확인

다음 명령으로 Wheel Odometry 데이터를 확인하였다.

```bash
ros2 topic echo /wheel/odom --once
```

확인된 주요 Frame:

```text
frame_id: odom
child_frame_id: base_link
```

실제 측정 과정에서 위치 값이 변화하는 것을 확인하였다.

따라서 `/wheel/odom` Topic이 존재하고 실제 LIMO의 주행 상태를 반영하는 Odometry 데이터가 출력되는 것을 확인하였다.

[출처: 교재 + 실기기 검증]

---

### 9. IMU 확인

다음 명령으로 IMU 데이터를 확인하였다.

```bash
ros2 topic echo /imu --once
```

확인된 Frame:

```text
frame_id: imu_link
```

Orientation, Angular velocity, Linear acceleration 등의 데이터가 출력되는 것을 확인하였다.

[출처: 실기기 검증]

---

### 10. IMU 주기 확인

다음 명령으로 IMU Publish 주기를 확인하였다.

```bash
ros2 topic hz /imu
```

측정 결과:

```text
average rate: 약 99.9 Hz
```

따라서 IMU가 약 100 Hz로 Publish되는 것을 확인하였다.

[출처: 실기기 검증]

---

### 11. TF Tree 확인

다음 명령으로 전체 TF Tree를 확인하였다.

```bash
ros2 run tf2_tools view_frames
```

생성된 PDF 파일을 확인하였다.

```bash
ls -l frames_*.pdf
```

주요 TF 구조:

```text
odom
└── base_link
    ├── imu_link
    └── laser_frame
```

LIMO의 센서들은 각각 자신의 Frame을 가지며 TF를 통해 `base_link`와 연결된다.

[출처: 교재 + 실기기 검증]

---

### 12. odom → base_link TF 확인

다음 명령으로 Odometry와 LIMO Base 사이의 TF를 확인하였다.

```bash
ros2 run tf2_ros tf2_echo odom base_link
```

실제 Translation과 Rotation 값이 출력되는 것을 확인하였다.

따라서 다음 TF 관계가 정상적으로 존재하는 것을 확인하였다.

```text
odom → base_link
```

[출처: 교재 + 실기기 검증]

---

### 13. base_link → imu_link TF 확인

TF Tree에서 다음 관계를 확인하였다.

```text
base_link
└── imu_link
```

따라서 LIMO Base와 IMU Frame 사이의 TF 관계가 구성되어 있음을 확인하였다.

[출처: 교재 + 실기기 검증]

---

### 14. YDLiDAR Tmini Plus 확인

YDLiDAR를 연결한 후 USB Serial 장치를 확인하였다.

```bash
ls -l /dev/ttyUSB*
```

YDLiDAR는 다음 장치에서 확인되었다.

```text
/dev/ttyUSB0
```

YDLiDAR Driver의 설정 파일은 다음과 같이 사용하였다.

```text
/home/pcu/agilex_ws/src/ydlidar_ros2_driver/params/Tmini.yaml
```

주요 설정:

```yaml
port: "/dev/ttyUSB0"
frame_id: "laser_frame"
baudrate: 230400
sample_rate: 4
scan_frequency: 10
range_min: 0.02
range_max: 12.0
```

[출처: 실기기 검증]

---

### 15. YDLiDAR Driver 실행

다음 명령으로 YDLiDAR Driver를 실행하였다.

```bash
source /opt/ros/humble/setup.bash
source ~/agilex_ws/install/setup.bash

ros2 launch ydlidar_ros2_driver ydlidar_launch.py \
params_file:=/home/pcu/agilex_ws/src/ydlidar_ros2_driver/params/Tmini.yaml
```

정상 실행 시 다음과 같은 메시지를 확인하였다.

```text
Lidar successfully connected [/dev/ttyUSB0:230400]
Lidar running correctly! The health status good
Successed to start scan mode
Scan Frequency: 10.00Hz
Sample Rate: 4.00K
Now lidar is scanning...
```

또한 실제 연결된 LiDAR 모델 정보를 확인하였다.

```text
Model: Tmini Plus
Firmware version: 1.3
Hardware version: 1
Serial: 2024000400170358
```

[출처: 실기기 검증]

---

### 16. /scan LaserScan 확인

LiDAR Driver가 실행된 상태에서 다음 명령으로 LaserScan 데이터를 확인하였다.

```bash
ros2 topic echo /scan --once
```

주요 데이터:

```text
frame_id: laser_frame
angle_min: 약 -π
angle_max: 약 π
range_min: 약 0.03
range_max: 12.0
```

`ranges` 배열에 실제 거리 데이터가 존재하는 것을 확인하였다.

따라서 다음 구조가 정상적으로 동작함을 확인하였다.

```text
YDLiDAR
    ↓
YDLiDAR Driver
    ↓
/scan
    ↓
sensor_msgs/msg/LaserScan
```

[출처: 교재 + 실기기 검증]

---

### 17. base_link → laser_frame TF 확인

다음 명령으로 LiDAR Frame의 TF를 확인하였다.

```bash
ros2 run tf2_ros tf2_echo base_link laser_frame
```

확인 결과:

```text
Translation:
[0.000, 0.000, 0.020]

Rotation:
[0.000, 0.000, 0.000, 1.000]
```

따라서 다음 관계를 확인하였다.

```text
base_link
└── laser_frame
```

[출처: 교재 + 실기기 검증]

---

### 18. RViz2에서 LaserScan 확인

RViz2를 실행하였다.

```bash
rviz2
```

RViz2에서:

```text
Add
→ By topic
→ /scan
→ LaserScan
```

을 추가하였다.

LaserScan 데이터를 RViz2에서 확인하고 LiDAR의 주변 환경이 표시되는 것을 확인하였다.

[출처: 교재 + 실기기 검증]

---

### 19. SLAM을 이용한 실내 Mapping

LiDAR `/scan`, Wheel Odometry, TF가 정상적으로 구성된 상태에서 SLAM을 실행하여 실내 환경의 지도를 생성하였다.

SLAM 과정에서는 로봇을 실제 공간에서 이동시키면서 LiDAR Scan과 Odometry를 이용하여 주변 환경을 누적하였다.

RViz2에서 로봇이 이동함에 따라 주변 공간이 지도 형태로 확장되는 것을 확인하였다.

생성된 지도에서는 벽과 장애물이 Occupancy Grid 형태로 표시되었다.

[출처: 교재 + 실기기 검증]

---

### 20. 작업 실행 순서

#### 터미널 1 — LIMO Base

```bash
source /opt/ros/humble/setup.bash
source ~/agilex_ws/install/setup.bash

ros2 launch limo_base limo_base.launch.py port_name:=ttyUSB1
```

[출처: 실기기 검증]

#### 터미널 2 — YDLiDAR

```bash
source /opt/ros/humble/setup.bash
source ~/agilex_ws/install/setup.bash

ros2 launch ydlidar_ros2_driver ydlidar_launch.py \
params_file:=/home/pcu/agilex_ws/src/ydlidar_ros2_driver/params/Tmini.yaml
```

[출처: 실기기 검증]

#### 터미널 3 — SLAM

```bash
source /opt/ros/humble/setup.bash
source ~/agilex_ws/install/setup.bash

ros2 launch slam_toolbox online_async_launch.py \
use_sim_time:=false \
slam_params_file:=/tmp/limo_slam_async.yaml
```

단, 위 `slam_toolbox` 실행은 **교재의 Cartographer 실행 명령이 아니라 10/01 실제 환경에서 사용한 SLAM Toolbox 실행 방식**이다.

교재 기준 SLAM 구성은 Cartographer를 사용한다.

[출처: 교재 + 실기기 검증]

#### 터미널 4 — RViz2

```bash
source /opt/ros/humble/setup.bash
source ~/agilex_ws/install/setup.bash

rviz2
```

RViz2에서 다음 Display를 확인한다.

```text
/scan → LaserScan
/map → Map
/TF → TF
```

[출처: 교재 + 실기기 검증]

---

### 21. 2026-09-30 작업 결과

* [v] LIMO ROS 2 패키지 구조 확인
* [v] 프로젝트 문서 구조 확인
* [v] Ackermann 모델 인스턴스 수정
* [v] USB Serial 장치 확인
* [v] `/dev/ttyUSB1` LIMO Base 연결 확인
* [v] `/limo_base_node` 실행 확인
* [v] `/wheel/odom` 데이터 확인
* [v] `/imu` 데이터 확인
* [v] IMU 약 100 Hz 확인
* [v] `odom → base_link` TF 확인
* [v] `base_link → imu_link` TF 확인
* [v] `/dev/ttyUSB0` YDLiDAR 연결 확인
* [v] YDLiDAR Tmini Plus 정상 동작 확인
* [v] `/scan` LaserScan 데이터 확인
* [v] `base_link → laser_frame` TF 확인
* [v] RViz2 LaserScan 확인
* [v] SLAM 실행 및 실내 지도 생성 확인

[출처: 실기기 검증]

---

### 22. 다음 단계

2026-09-30에는 LIMO Base, IMU, Wheel Odometry, LiDAR, TF 및 SLAM까지 기본적인 센서 기반 Mapping 흐름을 확인하였다.

다음 단계에서는 생성된 지도를 기반으로 Localization과 Navigation2를 구성한다.

전체 개발 흐름:

```text
LiDAR / IMU / Odometry
        ↓
       TF
        ↓
       SLAM
        ↓
       Map
        ↓
      AMCL
        ↓
  Navigation2
        ↓
Global / Local Planning
        ↓
    경로 추종
        ↓
동적 장애물 회피
        ↓
    자율 순찰
```

[출처: 교재 + 프로젝트 문서]

# 2026-10-01 — LIMO 작업 기록

## 1. 작업 목표

2026-09-30 작업에서 확인하지 못했던 RViz의 LiDAR Scan 표시 문제를 계속 확인하였다.

이번 작업에서는 다음 항목을 순서대로 재검증하였다.

- YDLiDAR USB 포트 확인
- YDLiDAR 설정 파일 확인
- LIMO Base 정상 실행 여부 확인
- YDLiDAR Driver 정상 실행 여부 확인
- `/scan` Topic 및 LaserScan 데이터 확인
- `/scan` QoS 확인
- LIMO 및 LiDAR 관련 TF 확인
- RViz에서 LaserScan 시각화 확인
- RViz 확대 후 실제 Scan 점 표시 여부 확인
- 로봇을 움직였을 때 Scan이 변화하는지 확인

WeGo 교재에서는 LiDAR Driver 실행 후 `/scan` Topic을 확인하고, `/scan`을 `sensor_msgs/msg/LaserScan`으로 RViz에서 시각화하는 절차를 설명한다.

---

# 2. 작업 환경

- 로봇: AgileX LIMO
- PC: LIMO onboard PC
- OS: Ubuntu 22.04
- ROS 2: Humble
- Workspace: `~/agilex_ws`

---

# 3. USB 장치 및 LIMO Base 상태 확인

## 3.1 USB 장치 확인

다음 명령으로 현재 연결된 USB Serial 장치를 확인하였다.

```bash
ls -l /dev/ttyUSB*
```

확인 결과:

```text
/dev/ttyUSB0
/dev/ttyUSB1
```

두 장치 모두 Silicon Labs CP210x UART Bridge로 확인되었다.

```bash
lsusb | grep -i -E "CP210|Silicon|YDLIDAR"
```

확인 결과:

```text
Bus 003 Device 009: ID 10c4:ea60 Silicon Labs CP210x UART Bridge
Bus 003 Device 005: ID 10c4:ea60 Silicon Labs CP210x UART Bridge
```

따라서 USB Serial 장치가 두 개 연결되어 있으며, 장치 종류만으로는 `/dev/ttyUSB0`과 `/dev/ttyUSB1`을 구분할 수 없는 상태였다.

---

# 4. YDLiDAR 설정 파일 확인

`ydlidar_ros2_driver`의 params 디렉터리에 이름이 비슷한 두 개의 설정 파일이 존재하는 것을 확인하였다.

```text
Tmini.yaml
tmini.yaml
```

두 파일의 `port` 설정을 확인하였다.

## 4.1 Tmini.yaml

```bash
grep -n -E "port|portname|device" \
~/agilex_ws/src/ydlidar_ros2_driver/params/Tmini.yaml
```

결과:

```text
3:    port: /dev/ttyUSB0
8:    device_type: 0
19:    support_motor_dtr: false
```

따라서 `Tmini.yaml`은 다음 포트를 사용하도록 설정되어 있다.

```text
/dev/ttyUSB0
```

## 4.2 tmini.yaml

```bash
grep -n -E "port|portname|device" \
~/agilex_ws/src/ydlidar_ros2_driver/params/tmini.yaml
```

결과:

```text
3:    port: "/dev/ttyUSB1"
7:    device_type: 0
```

따라서 `tmini.yaml`은 다음 포트를 사용하도록 설정되어 있다.

```text
/dev/ttyUSB1
```

### 확인 결과

두 파일의 이름은 대소문자가 다르지만 서로 다른 파일이며, LiDAR Serial Port 설정도 다르다.

```text
Tmini.yaml
    ↓
/dev/ttyUSB0

tmini.yaml
    ↓
/dev/ttyUSB1
```

따라서 여러 사람이 작업하면서 두 설정 파일이 함께 존재하게 된 경우, 어떤 파일을 launch에 지정하는지에 따라 서로 다른 USB 포트를 사용하게 된다.

이번 작업에서는 이 차이가 LiDAR 연결 문제를 확인하는 데 중요한 요소였다.

---

# 5. ROS 2 환경 설정

새 터미널에서 ROS 2와 LIMO workspace를 source하였다.

```bash
source /opt/ros/humble/setup.bash
source ~/agilex_ws/install/setup.bash
```

다음 경고가 반복적으로 출력되었다.

```text
not found: "/home/pcu/agilex_ws/install/limo_car/share/limo_car/local_setup.bash"
```

현재 작업에서는 이 경고가 출력된 상태에서도 ROS 2 명령 및 LIMO Base, LiDAR Driver 실행을 진행할 수 있었다.

해당 workspace의 `limo_car` 설치 경로 문제는 별도의 정리 대상이며, 이번 LiDAR 검증과는 구분하여 기록한다.

---

# 6. LIMO Base 정상 실행 확인

LIMO Base는 `/dev/ttyUSB1`을 사용하여 실행하였다.

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

이후 별도 터미널에서:

```bash
ros2 node list
```

확인 결과:

```text
/limo_base_node
```

따라서 `/dev/ttyUSB1`을 이용한 LIMO Base가 정상적으로 실행되는 것을 확인하였다.

---

# 7. LIMO Base Topic 확인

```bash
ros2 topic list
```

확인 결과:

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

따라서 LIMO Base 실행 후 다음 센서 및 상태 Topic이 정상적으로 생성되는 것을 확인하였다.

- `/imu`
- `/wheel/odom`
- `/cmd_vel`
- `/limo_status`

---

# 8. Wheel Odometry 재확인

```bash
ros2 topic echo /wheel/odom --once
```

확인 결과:

```text
frame_id: odom
child_frame_id: base_link
```

실제 출력 예:

```text
position:
  x: 0.000465927095539424
  y: -7.319367706042427e-06
  z: 0.0
```

Orientation 및 Twist 데이터도 정상적으로 출력되었다.

현재 측정에서는 LIMO가 정지해 있어 선속도와 각속도는 다음과 같이 출력되었다.

```text
linear:
  x: 0.0
  y: 0.0
  z: 0.0

angular:
  x: 0.0
  y: 0.0
  z: 0.0
```

따라서 LIMO Base에서 `/wheel/odom` 데이터가 정상적으로 publish되는 것을 재확인하였다.

---

# 9. IMU 재확인

```bash
ros2 topic echo /imu --once
```

확인된 Frame:

```text
frame_id: imu_link
```

확인된 데이터 예:

```text
orientation:
  x: 0.0
  y: 0.0
  z: -0.007853900888711334
  w: 0.9999691576447897
```

Linear acceleration:

```text
x: -0.01
y: -0.13
z: 9.98
```

따라서 `/imu`에서 Orientation, Angular Velocity, Linear Acceleration 등의 데이터가 정상적으로 출력되는 것을 재확인하였다.

---

# 10. YDLiDAR Driver 실행 문제 재확인

처음에는 `tmini.yaml`을 사용하여 LiDAR Driver를 실행하였다.

```bash
ros2 launch ydlidar_ros2_driver ydlidar_launch.py \
params_file:=/home/pcu/agilex_ws/src/ydlidar_ros2_driver/params/tmini.yaml
```

`/dev/ttyUSB1`에 연결하도록 설정된 `tmini.yaml`을 사용한 결과:

```text
Lidar successfully connected [/dev/ttyUSB1:230400]
```

까지 연결되었지만 이후 다음 오류가 발생하였다.

```text
Error, cannot retrieve Lidar health code -2
Fail to get baseplate device information!
Failed to start scan mode -2
```

결과적으로 Driver가 종료되었다.

따라서 단순히 Serial Port가 열리는 것만으로 LiDAR가 정상 동작한다고 판단할 수 없으며, 실제 Health 확인 및 Scan Mode 시작까지 확인해야 한다.

---

# 11. Tmini.yaml을 이용한 LiDAR Driver 재실행

앞서 확인한 설정 파일 중 `Tmini.yaml`은 `/dev/ttyUSB0`을 사용하도록 설정되어 있었다.

따라서 다음 명령으로 Driver를 실행하였다.

```bash
ros2 launch ydlidar_ros2_driver ydlidar_launch.py \
params_file:=/home/pcu/agilex_ws/src/ydlidar_ros2_driver/params/Tmini.yaml
```

정상 연결 결과:

```text
Lidar successfully connected [/dev/ttyUSB0:230400]
```

이후 Health 상태도 정상적으로 확인되었다.

```text
Lidar running correctly! The health status good
```

장치 정보:

```text
Current Lidar Model Code 151
Baseplate device info
Firmware version: 1.3
Hardware version: 1
Model: Tmini Plus
Serial: 2024000400170358
```

Scan Frequency:

```text
Current scan frequency: 10.00Hz
```

이후 다음 메시지를 확인하였다.

```text
Successed to start scan mode
Scan Frequency: 10.00Hz
Fixed Size: 430
Sample Rate: 4.00K
Successed to check the lidar
Now lidar is scanning...
```

따라서 이번 작업에서는 다음 조건에서 YDLiDAR T-mini Plus가 정상적으로 동작하는 것을 확인하였다.

```text
Tmini.yaml
    ↓
/dev/ttyUSB0
    ↓
YDLiDAR T-mini Plus
    ↓
Health status good
    ↓
Scan Mode 시작
    ↓
Now lidar is scanning...
```

---

# 12. `/scan` Topic 확인

LiDAR Driver가 정상적으로 실행된 상태에서:

```bash
ros2 topic list | grep scan
```

결과:

```text
/scan
```

따라서 `/scan` Topic이 정상적으로 생성된 것을 확인하였다.

---

# 13. `/scan` LaserScan 데이터 확인

다음 명령으로 LaserScan 데이터를 한 번 확인하였다.

```bash
ros2 topic echo /scan --once
```

확인 결과:

```text
frame_id: laser_frame
```

주요 값:

```text
angle_min: -3.1415927410125732
angle_max: 3.1415927410125732
angle_increment: 0.014646119438111782
time_increment: 0.0002510226331651211
scan_time: 0.09990700334310532
range_min: 0.029999999329447746
range_max: 12.0
```

`ranges` 배열에도 실제 거리 값이 존재하였다.

예:

```text
4.020999908447266
4.0269999504089355
4.058000087738037
...
2.7780001163482666
2.805000066757202
...
5.545000076293945
5.551000118255615
...
1.996000051498413
1.9620000123977661
...
```

또한 `intensities` 배열에도 실제 값이 존재하였다.

따라서 YDLiDAR에서 실제 측정값이 들어오고 있으며 `/scan`을 통해 `sensor_msgs/msg/LaserScan` 데이터가 publish되고 있음을 확인하였다.

이는 WeGo 교재에서 설명하는 LiDAR 데이터 확인 절차와 동일한 방향의 검증이다. 교재에서는 `/scan`의 Interface Type을 `sensor_msgs/msg/LaserScan`으로 설명하며 `ranges[]`를 각 Laser에서 측정한 거리 데이터로 설명한다.

---

# 14. `/scan` Topic QoS 확인

다음 명령으로 `/scan` Publisher 정보를 확인하였다.

```bash
ros2 topic info /scan -v
```

확인 결과:

```text
Type: sensor_msgs/msg/LaserScan

Publisher count: 1

Node name: ydlidar_ros2_driver_node
Node namespace: /
Topic type: sensor_msgs/msg/LaserScan
Endpoint type: PUBLISHER
```

QoS:

```text
Reliability: BEST_EFFORT
History (Depth): UNKNOWN
Durability: VOLATILE
Lifespan: Infinite
Deadline: Infinite
Liveliness: AUTOMATIC
```

따라서 `/scan`의 Publisher는 `BEST_EFFORT` Reliability를 사용하는 것을 확인하였다.

---

# 15. TF Frame 확인

이번 작업에서 확인한 주요 Frame은 다음과 같다.

```text
odom
base_link
imu_link
laser_frame
```

LIMO Base와 관련하여:

```text
odom
└── base_link
    └── imu_link
```

LiDAR와 관련하여:

```text
base_link
└── laser_frame
```

구조를 확인하였다.

---

# 16. RViz LaserScan 시각화 재확인

RViz2를 실행하였다.

```bash
rviz2
```

WeGo 교재에서는 RViz에서:

```text
Add
→ by topic
→ /scan:LaserScan
```

을 추가한 후 LiDAR 데이터를 시각화하도록 설명한다.

또한 교재에서는 Fixed Frame을 센서 Frame으로 변경하고 LaserScan의 Reliability Policy를 Best Effort로 설정하도록 설명한다.

이번 환경에서는 실제 `/scan`의 Frame이 다음과 같았다.

```text
frame_id: laser_frame
```

따라서 RViz에서 `/scan` LaserScan Display를 추가하고 다음 설정을 확인하였다.

```text
Topic: /scan
Reliability Policy: Best Effort
```

LaserScan 관련 표시 설정은 다음과 같이 조정하였다.

```text
Size: 0.01
Style: Flat Squares
```

---

# 17. RViz에서 Scan이 보이지 않았던 원인 확인

처음 RViz에서 LaserScan Display를 추가했을 때 좌표평면에 Scan 점이 보이지 않는 것처럼 보였다.

하지만 `/scan` Topic 자체는 정상적으로 존재했고:

```text
/scan
    ↓
sensor_msgs/msg/LaserScan
    ↓
frame_id = laser_frame
```

실제 거리 데이터도 존재했다.

또한:

```text
base_link → laser_frame
```

TF도 확인된 상태였다.

따라서 LiDAR Driver나 `/scan` 생성 자체의 문제라고 단정하지 않고 RViz의 화면 표시 범위와 설정을 추가로 확인하였다.

RViz의 좌표평면을 확대하여 확인한 결과 **LaserScan 점이 실제로 표시되는 것을 확인하였다.**

따라서 2026-09-30에 기록했던:

```text
RViz에 LaserScan을 추가했으나 좌표평면에서 Scan 점이 표시되지 않음
```

문제는 이번 작업에서 **화면 확대 후 정상 표시되는 것을 확인하여 해결하였다.**

---

# 18. LIMO 움직임에 따른 LaserScan 변화 확인

RViz에서 LaserScan이 표시된 상태에서 LIMO의 위치 및 주변 환경을 움직여 확인하였다.

그 결과 좌표평면에 표시되는 LaserScan 점들이 변화하는 것을 확인하였다.

따라서 현재 LiDAR 데이터는 단순히 고정된 값이 출력되는 것이 아니라 실제 주변 환경의 거리 변화에 따라 Scan 결과가 변화하는 것을 확인하였다.

즉:

```text
YDLiDAR
    ↓
실제 주변 거리 측정
    ↓
YDLiDAR Driver
    ↓
/scan
    ↓
LaserScan
    ↓
RViz
    ↓
주변 환경 표시
```

전체 데이터 흐름을 실제 환경에서 확인하였다.

---

# 19. 문제 및 해결 과정

## 19.1 `tmini.yaml` 사용 시 LiDAR Scan Mode 시작 실패

### 문제

다음 설정:

```text
tmini.yaml
port: /dev/ttyUSB1
```

으로 Driver를 실행했을 때:

```text
Lidar successfully connected [/dev/ttyUSB1:230400]
```

까지는 확인되었지만:

```text
Error, cannot retrieve Lidar health code -2
Fail to get baseplate device information!
Failed to start scan mode -2
```

가 발생하였다.

### 확인

설정 파일을 확인한 결과:

```text
Tmini.yaml
    /dev/ttyUSB0

tmini.yaml
    /dev/ttyUSB1
```

로 서로 다른 포트를 사용하고 있었다.

### 해결

`Tmini.yaml`을 사용하여:

```bash
ros2 launch ydlidar_ros2_driver ydlidar_launch.py \
params_file:=/home/pcu/agilex_ws/src/ydlidar_ros2_driver/params/Tmini.yaml
```

실행하였다.

결과:

```text
Lidar successfully connected [/dev/ttyUSB0:230400]
Lidar running correctly! The health status good
Successed to start scan mode
Now lidar is scanning...
```

정상적으로 Scan Mode가 시작되었다.

---

## 19.2 `/scan`이 처음 확인되지 않음

초기에는 다음 명령에서:

```bash
ros2 topic info /scan -v
```

다음 결과가 발생하였다.

```text
Unknown topic '/scan'
```

또한:

```bash
ros2 topic hz /scan
```

에서:

```text
WARNING: topic [/scan] does not appear to be published yet
```

가 출력되었다.

이는 당시 YDLiDAR Driver가 정상적으로 Scan Mode까지 실행되지 않은 상태와 관련하여 확인하였다.

이후 올바른 설정 파일을 사용하여 Driver가 정상적으로 Scan Mode에 진입한 뒤:

```bash
ros2 topic list | grep scan
```

결과:

```text
/scan
```

이 확인되었다.

---

## 19.3 RViz에서 LaserScan이 보이지 않음

### 문제

RViz에서 `/scan → LaserScan`을 추가했지만 처음에는 좌표평면에 Scan 점이 보이지 않았다.

### 확인한 내용

```text
/scan Topic 존재
↓
LaserScan 데이터 존재
↓
frame_id = laser_frame
↓
base_link → laser_frame TF 존재
↓
RViz LaserScan Display 존재
↓
QoS = Best Effort
```

### 해결

RViz 좌표평면을 확대하여 확인하였다.

확대 후 LaserScan 점이 정상적으로 표시되는 것을 확인하였다.

또한 LIMO의 움직임에 따라 Scan 점이 변화하는 것을 확인하였다.

따라서 해당 문제는 LiDAR 데이터 자체의 문제라기보다 RViz 화면에서 Scan 표시 위치와 범위를 확인하는 과정에서 발생한 것으로 기록한다.

---

# 20. 교재 기준과 현재 실기기 환경의 차이

WeGo 교재에서는 다음과 같이 LiDAR Driver와 RViz 시각화를 설명한다.

```text
LiDAR Driver
↓
/scan
↓
sensor_msgs/msg/LaserScan
↓
RViz
```

교재의 예제 명령은:

```bash
ros2 launch ydlidar_ros2_driver ydlidar.launch.py
```

이며 RViz에서 `/scan:LaserScan`을 추가하고 Fixed Frame과 Best Effort 설정을 확인하도록 안내한다.

현재 LIMO onboard PC의 실제 환경에서는 Driver launch 파일이 다음과 같았다.

```bash
ros2 launch ydlidar_ros2_driver ydlidar_launch.py
```

또한 실제 LaserScan의 Frame은:

```text
laser_frame
```

이었다.

따라서 이후 기록에서는 **교재의 명령어와 실제 프로젝트 환경에서 사용하는 명령어를 구분하여 기록한다.**

교재의 설명을 그대로 실제 환경에 적용한다고 가정하지 않고, 실제 실행 결과를 기준으로 검증한다.

---

# 21. 현재 실기기 실행 순서

## 터미널 1 — LIMO Base

```bash
source /opt/ros/humble/setup.bash
source ~/agilex_ws/install/setup.bash

ros2 launch limo_base limo_base.launch.py port_name:=ttyUSB1
```

정상 실행 확인:

```bash
ros2 node list
```

예상:

```text
/limo_base_node
```

---

## 터미널 2 — YDLiDAR Driver

현재 정상 동작을 확인한 설정:

```text
Tmini.yaml
→ /dev/ttyUSB0
```

실행:

```bash
source /opt/ros/humble/setup.bash
source ~/agilex_ws/install/setup.bash

ros2 launch ydlidar_ros2_driver ydlidar_launch.py \
params_file:=/home/pcu/agilex_ws/src/ydlidar_ros2_driver/params/Tmini.yaml
```

정상 실행 시 확인할 메시지:

```text
Lidar successfully connected
Lidar running correctly! The health status good
Successed to start scan mode
Now lidar is scanning...
```

---

## 터미널 3 — 센서 확인

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

### LiDAR Topic

```bash
ros2 topic list | grep scan
```

### LiDAR 데이터

```bash
ros2 topic echo /scan --once
```

### LiDAR Topic 정보

```bash
ros2 topic info /scan -v
```

### TF

```bash
ros2 run tf2_tools view_frames
```

또는:

```bash
ros2 run tf2_ros tf2_echo odom base_link
```

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

확인할 설정:

```text
Topic: /scan
Reliability Policy: Best Effort
Style: Flat Squares
Size: 0.01
```

Scan이 보이지 않을 경우:

1. RViz 화면 확대
2. Fixed Frame 확인
3. LaserScan의 Topic 확인
4. Reliability Policy 확인
5. `header.frame_id` 확인
6. TF 연결 확인

순서로 확인한다.

---

# 22. 작업 결과

- [v] `/dev/ttyUSB0`, `/dev/ttyUSB1` USB 장치 확인
- [v] 두 USB 장치가 Silicon Labs CP210x UART Bridge임을 확인
- [v] `Tmini.yaml`과 `tmini.yaml`이 서로 다른 파일임을 확인
- [v] `Tmini.yaml`은 `/dev/ttyUSB0`을 사용함을 확인
- [v] `tmini.yaml`은 `/dev/ttyUSB1`을 사용함을 확인
- [v] `/dev/ttyUSB1`을 이용한 LIMO Base 정상 실행 확인
- [v] `/limo_base_node` 정상 실행 확인
- [v] `/wheel/odom` 실제 데이터 확인
- [v] `/imu` 실제 데이터 확인
- [v] `tmini.yaml` 사용 시 LiDAR Health / Scan Mode 시작 실패 확인
- [v] `Tmini.yaml` 사용 시 YDLiDAR T-mini Plus 정상 연결 확인
- [v] YDLiDAR Health status good 확인
- [v] YDLiDAR Scan Mode 시작 확인
- [v] Scan Frequency 10 Hz 확인
- [v] `/scan` Topic 생성 확인
- [v] `/scan`의 `sensor_msgs/msg/LaserScan` Type 확인
- [v] `/scan` 실제 거리 데이터 확인
- [v] `/scan` Publisher QoS가 Best Effort임을 확인
- [v] `laser_frame` 확인
- [v] `odom`, `base_link`, `imu_link`, `laser_frame` Frame 확인
- [v] `base_link → laser_frame` TF 확인
- [v] RViz에서 `/scan → LaserScan` 추가 확인
- [v] RViz LaserScan Reliability Policy를 Best Effort로 확인
- [v] LaserScan 표시 설정 확인
- [v] RViz 화면 확대 후 LaserScan 점이 실제로 표시되는 것을 확인
- [v] LIMO 움직임에 따라 RViz의 LaserScan이 변화하는 것을 확인
- [v] 2026-09-30에 확인했던 RViz LaserScan 미표시 문제 해결

---

# 23. 현재 상태 및 다음 작업

현재까지 실기기에서 다음 데이터 흐름을 확인하였다.

```text
YDLiDAR T-mini Plus
        ↓
/dev/ttyUSB0
        ↓
YDLiDAR Driver
        ↓
/scan
        ↓
sensor_msgs/msg/LaserScan
        ↓
laser_frame
        ↓
base_link
        ↓
RViz2
        ↓
주변 환경 Scan 표시
```

또한 LIMO Base에서는:

```text
LIMO Base
    ├── /wheel/odom
    │       ↓
    │   odom → base_link
    │
    └── /imu
            ↓
        imu_link
```

구조를 확인하였다.

다음 단계에서는 LiDAR와 Odometry/IMU가 정상적으로 연결된 현재 상태를 기반으로 **SLAM Toolbox를 이용한 실내 Mapping**을 진행한다.

SLAM 작업에 들어가기 전에 기존 환경을 불필요하게 변경하지 않고, 현재 확인된 다음 조건을 유지한다.

- LIMO Base: `/dev/ttyUSB1`
- YDLiDAR: `/dev/ttyUSB0`
- YDLiDAR 설정: `Tmini.yaml`
- LiDAR Topic: `/scan`
- LiDAR Frame: `laser_frame`
- Odometry Frame: `odom`
- Robot Base Frame: `base_link`
- IMU Frame: `imu_link`

SLAM 실행 후에는 실제로 생성된 `/map`, `map → odom` 관계, RViz Map 표시 및 로봇 이동에 따른 지도 갱신 여부를 각각 확인하고 기록한다.
```
