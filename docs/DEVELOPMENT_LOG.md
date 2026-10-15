# 2026-09-30 — LIMO 온보드 환경 및 LiDAR / LIMO Base 점검

## 1. 작업 환경

- 로봇: AgileX LIMO
- 작업 컴퓨터: LIMO onboard PC
- 계정: `pcu`
- OS: Ubuntu 22.04.5
- ROS 2: Humble
- Workspace: `~/agilex_ws`

LIMO onboard PC에 연결된 하드웨어를 기준으로 ROS 2 구동 환경, LIMO Base, YDLiDAR T-mini Plus의 통신 및 장치 인식 상태를 단계별로 점검하였다.

ROS 2 및 LIMO 관련 기능은 온보드 환경에서 직접 실행하여 터미널 출력과 토픽 통신 상태를 통해 정상 동작 여부를 확인하였다.

---

## 2. ROS 2 환경 및 Workspace 확인

ROS 2 Humble과 LIMO 관련 workspace를 확인하였다.

```bash
source /opt/ros/humble/setup.bash
source ~/agilex_ws/install/setup.bash
```

현재 workspace:

```text
~/agilex_ws
```

주요 LIMO 관련 패키지:

```text
limo_base
limo_car
limo_description
limo_msgs
ydlidar_ros2_driver
```

ROS 2 환경을 source할 때 다음 경고가 반복적으로 출력되었다.

```text
not found: "/home/pcu/agilex_ws/install/limo_car/share/limo_car/local_setup.bash"
```

해당 경고는 패키지 설치 경로 미생성으로 인한 것으로 확인하였으며, 하드웨어 통신 점검에 직접적인 영향이 없어 워크스페이스 환경 설정은 기존 상태를 유지하였다.

---

## 3. USB 장치 확인

LIMO onboard PC에서 USB Serial 장치를 확인하였다.

```bash
ls -l /dev/ttyUSB*
```

확인된 장치:

```text
/dev/ttyUSB0
/dev/ttyUSB1
```

물리적인 USB 경로도 확인하였다.

```bash
ls -l /dev/serial/by-path/
```

확인 결과:

```text
pci-0000:00:14.0-usb-0:3:1.0-port0 -> ../../ttyUSB0
pci-0000:00:14.0-usb-0:7.4:1.0-port0 -> ../../ttyUSB1
```

두 장치 모두 Silicon Labs CP2102 USB-UART 계열 장치로 확인하였다.

---

## 4. LIMO Base 연결 확인

LIMO Base의 Serial 포트를 실제 USB 장치에 맞춰 지정하였다.

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

실행 후:

```bash
ros2 node list
```

결과:

```text
/limo_base_node
```

따라서 `/dev/ttyUSB1`을 이용하여 LIMO Base Node가 실행되는 것을 확인하였다.

교재에서는 `limo_base`가 LIMO 하부 MCU와 Serial 통신을 수행하고 ROS 2 Topic 형태로 데이터를 제공하는 구조를 설명하며, 기본 실행은 `ros2 launch limo_base limo_base.launch.py`를 사용한다.

---

## 5. LIMO Base Topic 확인

LIMO Base 실행 후 Topic을 확인하였다.

```bash
ros2 topic list
```

확인된 주요 Topic:

```text
/cmd_vel
/imu
/limo_status
/tf
/tf_static
/wheel/odom
```

교재에서는 LIMO Driver 실행 후 `cmd_vel`, `imu`, `limo_status`, `odom` 등의 Topic을 통해 LIMO 제어 및 상태 정보를 확인할 수 있도록 설명하고 있다.

현재 실제 환경에서는 `/wheel/odom`이라는 Topic 이름이 확인되었다.

---

## 6. IMU 확인

다음 명령으로 IMU 데이터를 확인하였다.

```bash
ros2 topic echo /imu --once
```

실제 `sensor_msgs/msg/Imu` 메시지가 출력되었다.

확인된 Frame:

```text
frame_id: imu_link
```

또한 다음과 같은 데이터가 출력되었다.

```text
angular_velocity:
  x: ...
  y: ...
  z: ...

linear_acceleration:
  x: ...
  y: ...
  z: ...
```

교재에서도 LIMO Driver 실행 후 `/imu` Topic을 통해 `sensor_msgs/msg/Imu` 데이터를 확인하는 방법을 제시한다.

---

## 7. Wheel Odometry 확인

다음 명령으로 Wheel Odometry Topic을 확인하였다.

```bash
ros2 topic info /wheel/odom --verbose
```

결과:

```text
Type: nav_msgs/msg/Odometry
Publisher count: 1
Node name: limo_base_node
```

따라서 `limo_base_node`가 `/wheel/odom`을 Publisher로 가지고 있는 것을 확인하였다.

이후 실제 데이터 출력 여부도 확인하였다.

```bash
ros2 topic echo /wheel/odom --once
```

초기 확인 과정에서는 데이터 출력이 바로 확인되지 않는 경우가 있었으므로, 당시에는 Publisher 존재와 실제 데이터 수신을 구분하여 기록하였다.

교재에서는 LIMO의 Odometry가 Encoder 데이터를 이용하여 현재 속도와 위치 정보를 나타내는 것으로 설명한다.

---

## 8. LIMO TF 확인

다음 명령으로 `odom`과 `base_link` 사이의 TF를 확인하였다.

```bash
ros2 run tf2_ros tf2_echo odom base_link
```

실제 Translation 및 Rotation 값이 출력되는 것을 확인하였다.

따라서 다음 TF 관계가 존재하는 것을 확인하였다.

```text
odom
└── base_link
```

LIMO의 TF 구조와 `odom` 기준 좌표 관계는 교재의 TF 실습 및 LIMO TF 구성에서 다루고 있다.

---

## 9. YDLiDAR T-mini Plus 연결 확인

YDLiDAR ROS 2 Driver가 이미 workspace에 존재하는 것을 확인하였다.

```text
~/agilex_ws/src/ydlidar_ros2_driver
```

YDLiDAR 설정 파일을 확인하고 실제 LiDAR 포트를 `/dev/ttyUSB0`으로 지정하여 테스트하였다.

실제 장치 정보:

```text
Model: Tmini Plus
Firmware version: 1.3
Hardware version: 1
Serial: 2024000400170358
```

따라서 현재 LIMO에 연결된 LiDAR가 YDLIDAR T-mini Plus임을 실제 장치 로그에서 확인하였다.

---

## 10. LiDAR Driver 실행

다음 명령으로 YDLiDAR Driver를 실행하였다.

```bash
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

따라서 다음 단계까지 정상 동작을 확인하였다.

```text
/dev/ttyUSB0
      ↓
YDLiDAR T-mini Plus
      ↓
ydlidar_ros2_driver
      ↓
scan mode
```

---

## 11. /scan LaserScan 확인

LiDAR Driver 실행 후:

```bash
ros2 topic echo /scan --once
```

명령으로 실제 LaserScan 메시지를 확인하였다.

주요 정보:

```text
frame_id: laser_frame
angle_min: 약 -π
angle_max: 약 π
range_min: 0.03
range_max: 12.0
scan_time: 약 0.1초
```

또한 `ranges` 배열에서 실제 거리 값이 존재하는 것을 확인하였다.

예:

```text
0.232
0.244
0.303
0.527
0.972
4.417
...
```

따라서 LiDAR가 실제 거리 데이터를 `/scan`으로 Publishing하는 것을 확인하였다.

교재에서는 YDLiDAR Driver 실행 후 `/scan`으로 `sensor_msgs/msg/LaserScan` 데이터가 나오며, `ranges[]`가 각 Laser에서 검출된 장애물의 거리라고 설명한다.

---

## 12. base_link → laser_frame TF 확인

YDLiDAR launch 구성에서 Static Transform Publisher가 실행되며 다음 TF를 생성하는 것을 확인하였다.

```text
base_link
└── laser_frame
```

Translation:

```text
x = 0
y = 0
z = 0.02
```

Rotation:

```text
0 0 0 1
```

실행 로그에서도 다음 내용을 확인하였다.

```text
from 'base_link' to 'laser_frame'
```

따라서 LiDAR의 `laser_frame`이 LIMO의 `base_link`와 연결되어 있는 것을 확인하였다.

---

## 13. RViz2에서 LaserScan 확인

RViz2를 실행하였다.

```bash
rviz2
```

다음 순서로 LaserScan을 추가하였다.

```text
Add
→ By topic
→ /scan
→ LaserScan
```

교재에서는 Lidar Data를 RViz에서 확인하기 위해 `/scan`을 LaserScan으로 추가하고, Fixed Frame과 Reliability Policy를 설정하도록 안내한다.

특히 교재에서는 LaserScan의 Reliability Policy를 `Best Effort`로 변경하도록 설명한다.

---

## 14. 09/30 작업 결과

확인된 결과:

```text
[x] LIMO onboard PC에서 ROS 2 Humble 환경 확인
[x] ~/agilex_ws workspace 확인
[x] LIMO 관련 ROS 2 패키지 확인
[x] /dev/ttyUSB0 확인
[x] /dev/ttyUSB1 확인
[x] /dev/ttyUSB0 = YDLiDAR T-mini Plus 확인
[x] /dev/ttyUSB1 = LIMO Base 연결 확인
[x] LIMO Base Node 실행 확인
[x] /imu 확인
[x] /wheel/odom Publisher 확인
[x] odom → base_link TF 확인
[x] YDLiDAR 정상 연결 확인
[x] LiDAR health 정상 확인
[x] LiDAR scan mode 시작 확인
[x] /scan LaserScan 데이터 확인
[x] base_link → laser_frame TF 확인
[ ] RViz LaserScan 표시 문제 완전 해결
[ ] SLAM 지도 생성
```

09/30 점검 결과 **LIMO Base와 LiDAR의 하드웨어 연결 및 센서 데이터 수신 단계까지 정상 동작을 확인**하였으며, RViz 표시 개선 및 SLAM 맵핑은 후속 작업으로 이관하였다.

---

## 15. 실행 명령어 정리

### 터미널 1 — LIMO Base

```bash
source /opt/ros/humble/setup.bash
source ~/agilex_ws/install/setup.bash

ros2 launch limo_base limo_base.launch.py port_name:=ttyUSB1
```

### 터미널 2 — YDLiDAR

```bash
source /opt/ros/humble/setup.bash
source ~/agilex_ws/install/setup.bash

ros2 launch ydlidar_ros2_driver ydlidar_launch.py \
params_file:=/home/pcu/agilex_ws/src/ydlidar_ros2_driver/params/Tmini.yaml
```

### 터미널 3 — 확인

```bash
ros2 node list
ros2 topic list
ros2 topic echo /imu --once
ros2 topic echo /wheel/odom --once
ros2 topic echo /scan --once
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo base_link laser_frame
```

### 터미널 4 — RViz

```bash
rviz2
```

# 2026-10-01 — LIMO Base / LiDAR / RViz / SLAM 연동 점검

## 1. 작업 목적 및 환경

- 로봇: AgileX LIMO
- 작업 컴퓨터: LIMO onboard PC
- 계정: `pcu`
- Workspace: `~/agilex_ws`

09/30에 확인한 LIMO Base와 YDLiDAR T-mini Plus를 동시 구동하여 센서 파이프라인(Base/LiDAR → TF → `/scan` → RViz → SLAM)이 정상 동작하는지 점검하였다.

특히 RViz의 LaserScan QoS 비호환 문제 해결과 SLAM Toolbox 노드 연동 상태를 집중 확인하였다.

```text
LIMO Base
    +
YDLiDAR
    ↓
TF
    ↓
/scan
    ↓
RViz
    ↓
SLAM
```

---

## 2. ROS 2 환경 설정

```bash
source /opt/ros/humble/setup.bash
source ~/agilex_ws/install/setup.bash
```

source 과정에서 다음 경고가 반복적으로 출력되었다.

```text
not found: "/home/pcu/agilex_ws/install/limo_car/share/limo_car/local_setup.bash"
```

해당 워크스페이스 경고는 유지한 상태로 센서 연동 및 SLAM 노드 테스트를 진행하였다.

---

## 3. LIMO Base 실행

LIMO Base는 실제 LIMO 본체에 연결된 `/dev/ttyUSB1`을 사용하였다.

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

이후:

```bash
ros2 node list
```

결과:

```text
/limo_base_node
```

따라서 LIMO Base Node가 실제 onboard PC에서 실행되는 것을 확인하였다.

---

## 4. Wheel Odometry 및 IMU 확인

LIMO Base 실행 후 `/wheel/odom`을 확인하였다.

```bash
ros2 topic info /wheel/odom --verbose
```

결과:

```text
Type: nav_msgs/msg/Odometry
Publisher count: 1
Node name: limo_base_node
Reliability: RELIABLE
```

또한 `/imu` Topic에서 실제 IMU 데이터가 출력되는 것을 확인하였다.

교재에서는 LIMO의 Odometry가 Encoder 기반으로 현재 속도와 위치를 나타내며, `/imu`에서는 IMU의 자세와 각속도 및 가속도 데이터를 제공한다고 설명한다.

---

## 5. YDLiDAR T-mini Plus 실행

다음 명령으로 LiDAR Driver를 실행하였다.

```bash
source /opt/ros/humble/setup.bash
source ~/agilex_ws/install/setup.bash

ros2 launch ydlidar_ros2_driver ydlidar_launch.py \
params_file:=/home/pcu/agilex_ws/src/ydlidar_ros2_driver/params/Tmini.yaml
```

정상 실행 결과:

```text
Lidar successfully connected [/dev/ttyUSB0:230400]
Lidar running correctly! The health status good
Successed to start scan mode
Scan Frequency: 10.00Hz
Sample Rate: 4.00K
Now lidar is scanning...
```

실제 장치:

```text
Model: Tmini Plus
Firmware version: 1.3
Hardware version: 1
Serial: 2024000400170358
```

---

## 6. /scan 데이터 확인

```bash
ros2 topic echo /scan --once
```

실제 LaserScan 데이터가 출력되는 것을 확인하였다.

```text
frame_id: laser_frame
angle_min: -3.141592...
angle_max: 3.141592...
scan_time: 0.100...
range_min: 0.03
range_max: 12.0
```

`ranges` 배열에도 실제 거리 데이터가 존재하였다.

따라서 LiDAR 하드웨어와 ROS 2 Driver 및 `/scan` Publishing은 정상적으로 동작하는 것으로 확인하였다.

---

## 7. /scan QoS 문제 확인

RViz 실행 시 다음 경고가 출력되었다.

```text
New publisher discovered on topic '/scan',
offering incompatible QoS.
No messages will be sent to it.
Last incompatible policy:
RELIABILITY_QOS_POLICY
```

다음 명령으로 `/scan`의 QoS를 확인하였다.

```bash
ros2 topic info /scan --verbose
```

Publisher:

```text
Node name: ydlidar_ros2_driver_node
Reliability: BEST_EFFORT
```

초기 RViz Subscriber:

```text
Reliability: RELIABLE
```

따라서 `/scan` Publisher와 RViz Subscriber의 Reliability Policy가 서로 맞지 않는 문제가 발생한 것을 확인하였다.

교재에서도 LiDAR 데이터를 RViz에서 시각화할 때 LaserScan의 Reliability Policy를 `Best Effort`로 설정하도록 안내한다.

---

## 8. RViz LaserScan QoS 수정

RViz의 LaserScan Display에서 Reliability Policy를 변경하였다.

```text
RELIABLE
    ↓
BEST_EFFORT
```

이후 `/scan`의 QoS가 다음과 같이 일치하는 것을 확인하였다.

```text
/scan Publisher
Reliability: BEST_EFFORT

RViz Subscriber
Reliability: BEST_EFFORT
```

따라서 이전의 QoS incompatibility 문제를 해결하였다.

---

## 9. RViz LaserScan 표시 확인

QoS 수정 후 RViz에서 LiDAR 벽과 주변 환경이 표시되는 것을 확인하였다.

따라서 다음 구조가 정상적으로 연결된 것을 확인하였다.

```text
YDLiDAR T-mini Plus
        ↓
/scan
        ↓
LaserScan
        ↓
RViz
```

그러나 RViz에서 LaserScan이 표시되는 것과 SLAM 지도가 정상적으로 생성되는 것은 별개의 단계이므로 이를 구분하였다.

---

## 10. SLAM Toolbox 실행

10/01에는 SLAM Toolbox를 사용하여 실제 지도 생성 가능 여부를 확인하였다.

실행에 사용한 명령:

```bash
ros2 launch slam_toolbox online_async_launch.py \
use_sim_time:=false \
slam_params_file:=/tmp/limo_slam_async.yaml
```

실행 중:

```bash
ros2 node list | grep slam
```

에서:

```text
/slam_toolbox
```

가 확인되었다.

한때 동일한 `/slam_toolbox` 노드가 두 개 실행되는 상황도 확인하였다.

```text
/slam_toolbox
/slam_toolbox
```

프로세스 확인:

```bash
ps -ef | grep '[s]lam_toolbox'
```

동일한 `online_async_launch.py`가 두 번 실행된 것을 확인하였으며, 이후 중복 실행을 정리하였다.

최종적으로:

```bash
ros2 node list | grep slam
```

에서:

```text
/slam_toolbox
```

하나만 확인하였다.

---

## 11. SLAM Toolbox의 /scan 연결 확인

다음 명령으로 `/scan` 연결 상태를 확인하였다.

```bash
ros2 topic info /scan --verbose
```

확인 결과:

```text
Publisher:
ydlidar_ros2_driver_node
Reliability: BEST_EFFORT
```

SLAM Toolbox Subscriber:

```text
slam_toolbox
Reliability: BEST_EFFORT
```

따라서 현재 `/scan`의 QoS는 SLAM Toolbox와 호환되는 것을 확인하였다.

---

## 12. SLAM Toolbox Node 구성 확인

```bash
ros2 node info /slam_toolbox
```

결과에서 다음 Subscriber를 확인하였다.

```text
/map
/parameter_events
/scan
/slam_toolbox/feedback
```

Publisher로:

```text
/map
/map_metadata
/pose
/slam_toolbox/graph_visualization
/slam_toolbox/scan_visualization
/tf
```

등이 존재하는 것을 확인하였다.

따라서 SLAM Toolbox Node 자체가 실행되고 `/scan`을 Subscriber로 가지고 있는 것은 확인하였다.

---

## 13. 실제 /scan 데이터 확인

다음 명령으로 실제 `/scan` 데이터를 확인하였다.

```bash
ros2 topic echo /scan --once
```

실제 메시지:

```text
frame_id: laser_frame
angle_min: -3.141592...
angle_max: 3.141592...
scan_time: 0.100...
range_min: 0.03
range_max: 12.0
```

`ranges` 배열에 실제 거리 값이 존재하였다.

따라서 현재 단계에서 확인된 것은:

```text
LiDAR 하드웨어        정상
LiDAR Driver          정상
/scan Topic           정상
/scan 거리 데이터     정상
SLAM Toolbox Node     실행 확인
SLAM Toolbox /scan     구독 확인
```

이다.

---

## 14. /map 확인

SLAM Toolbox 실행 후 다음 명령을 사용하였다.

```bash
ros2 topic echo /map --once
```

그러나 실제 점유격자 지도 데이터(`/map`)가 정상적으로 발행되는 것은 확인되지 않았다.

현 시점의 상태는 다음과 같이 정리한다.

```text
SLAM Toolbox 실행        [확인]
/scan 구독               [확인]
/map Publisher 등록      [확인]
실제 map 데이터 생성     [미확인]
이동에 따른 지도 확장    [미확인]
```

**SLAM 매핑 성공으로 판정하지 않으며, 파라미터 및 TF 연동 문제를 후속 해결 과제로 분류한다.**

---

## 15. 현재 SLAM 문제 상태

현재 LiDAR 센서 데이터 수신 자체는 정상 상태이다.

```text
YDLiDAR
   ↓
/scan
   ↓
LaserScan
   ↓
SLAM Toolbox
```

위 데이터 흐름까지는 확인되었다.

그러나 LIMO 이동 시 RViz 상에서 주변 공간이 확장되며 지도 형태가 생성되는 현상은 관측되지 않았다.

따라서 현재 문제는 LiDAR 하드웨어나 `/scan` 토픽 부재가 아닌, SLAM Toolbox 내부 처리 및 좌표계 매핑 과정의 문제로 판단된다.

추가로 확인해야 할 항목은:

```text
1. SLAM Toolbox parameter
2. SLAM Toolbox의 TF 요구사항
3. laser_frame 관련 TF
4. odom → base_link 변화
5. SLAM Toolbox가 사용하는 tracking frame
6. /map 실제 publish 상태
7. RViz Map Display 설정
```

이다.

---

## 16. 교재와 현재 SLAM 방식의 차이

중요한 점은 교수님 교재에서 설명하는 LIMO SLAM 방식과 현재 실험한 방식이 다르다는 것이다.

교재의 LIMO SLAM:

```text
Cartographer SLAM
```

실행 순서:

```bash
ros2 launch wego teleop_launch.py
```

```bash
ros2 launch wego cartographer_launch.py
```

지도 작성 후:

```bash
ros2 run nav2_map_server map_saver_cli -f map
```

교재에서는 이 과정을 통해:

```text
map.pgm
map.yaml
```

이 생성된다고 설명한다.

따라서 **교재의 Cartographer 구성을 비교 기준선으로 설정하고, 향후 SLAM 구현 시 이를 우선 대조한다.**

10/01에 진행한 `slam_toolbox` 실험은 교재 실습과 별개로 진행된 파이프라인 구성 실험이므로, 개발 기록에서 이를 명확히 구분하여 관리한다.

---

## 17. 10/01 작업 결과

```text
[x] LIMO onboard PC에서 ROS 2 Humble 실행
[x] /dev/ttyUSB1 LIMO Base 연결 확인
[x] /limo_base_node 실행
[x] /imu 확인
[x] /wheel/odom Publisher 확인
[x] odom → base_link TF 확인
[x] /dev/ttyUSB0 YDLiDAR T-mini Plus 확인
[x] LiDAR health 정상
[x] LiDAR scan mode 정상
[x] /scan LaserScan 실제 데이터 확인
[x] base_link → laser_frame TF 확인
[x] RViz2 실행
[x] /scan LaserScan Display 확인
[x] /scan QoS incompatibility 확인
[x] RViz Reliability Policy를 BEST_EFFORT로 변경
[x] RViz에서 LaserScan 표시 확인
[x] SLAM Toolbox 실행 확인
[x] SLAM Toolbox의 /scan Subscriber 확인
[x] 중복 실행된 SLAM Toolbox 프로세스 확인 및 정리
[ ] SLAM Toolbox에서 정상적인 /map 생성 확인
[ ] 이동에 따른 지도 확장 확인
[ ] 지도 저장
```

---

## 18. 실행 명령어 정리

### 터미널 1 — LIMO Base

```bash
source /opt/ros/humble/setup.bash
source ~/agilex_ws/install/setup.bash

ros2 launch limo_base limo_base.launch.py port_name:=ttyUSB1
```

### 터미널 2 — YDLiDAR T-mini Plus

```bash
source /opt/ros/humble/setup.bash
source ~/agilex_ws/install/setup.bash

ros2 launch ydlidar_ros2_driver ydlidar_launch.py \
params_file:=/home/pcu/agilex_ws/src/ydlidar_ros2_driver/params/Tmini.yaml
```

### 터미널 3 — SLAM Toolbox 실험

```bash
source /opt/ros/humble/setup.bash
source ~/agilex_ws/install/setup.bash

ros2 launch slam_toolbox online_async_launch.py \
use_sim_time:=false \
slam_params_file:=/tmp/limo_slam_async.yaml
```

단, `/tmp/limo_slam_async.yaml`은 임시 경로 파일로 재부팅 후 소실되었으므로 당시 실행 기록으로만 보존하며, 영구 파라미터 파일 확보 전까지 재실행 명령으로 사용하지 않는다.

### 터미널 4 — RViz

```bash
source /opt/ros/humble/setup.bash
source ~/agilex_ws/install/setup.bash

rviz2
```

RViz에서:

```text
Add
→ By topic
→ /scan
→ LaserScan
```

Reliability Policy:

```text
BEST_EFFORT
```

---

## 19. 다음 작업

현재 가장 중요한 것은 SLAM Toolbox 자체를 무작정 수정하는 것이 아니라 **교재의 LIMO SLAM 구성을 기준선으로 다시 비교하는 것이다.**

교재 기준:

```text
LIMO 통합 Driver
       ↓
LiDAR / IMU / Odometry
       ↓
Cartographer
       ↓
Occupancy Grid
       ↓
/map
       ↓
map.pgm + map.yaml
```

현재 실험:

```text
LIMO Base
       +
YDLiDAR
       ↓
/scan
       ↓
SLAM Toolbox
       ↓
/map 생성 미확인
```

따라서 다음 작업에서는 먼저 교재의:

```text
wego teleop_launch.py
wego cartographer_launch.py
limo_lds_2d.lua
occupancy_grid_launch.py
```

구성을 실제 `~/agilex_ws`에 존재하는 패키지와 비교한다.

그 후 교재 방식으로 실행할 수 있는지 확인하고, 실제 LIMO에서 지도 생성이 되는지 확인한다.

지도 생성이 실제로 확인된 경우에만:

```bash
ros2 run nav2_map_server map_saver_cli -f map
```

을 사용하여 지도 저장 단계로 진행한다.


# 2026-10-13 — Gazebo Classic 기반 LIMO SLAM 시뮬레이션 실험 및 매핑 평가

## 1. 작업 목적 및 시뮬레이션 환경

- 로봇 모델: AgileX LIMO (Gazebo 시뮬레이션 모델)
- 작업 환경: 개발 PC (`~/simulator-research/limo_sim_ws`)
- 시뮬레이션 플랫폼: Gazebo Classic 11 + ROS 2 Humble
- 대상 월드: `enclosed_room.world` (약 5m × 5m 밀폐 공간)
- SLAM 패키지: `slam_toolbox` (`async_slam_toolbox_node`)
- 검증 근거 (시뮬레이션 로그 및 결과 파일):
  - 실험 실행 로그: `~/simulator-research/limo_sim_ws/experiments/exp_slam_mapping_20261013_212244.log`
  - 시뮬레이션 TF 트리 기록: `frames_2026-10-13_19.38.47.gv`
  - 생성된 지도 파일: `~/simulator-research/limo_sim_ws/results/enclosed_room_slam_map_20261013_212244.pgm/yaml`

10/01의 LIMO 하드웨어 점검에 이어, 개발 PC의 Gazebo Classic 시뮬레이션 환경에서 LIMO 모델과 SLAM Toolbox를 연동한 지도 작성 실험을 수행하고 주행 제어 및 맵 생성 상태를 점검하였다.

---

## 2. 시뮬레이션 TF 상태 확인

실험 초기화 단계에서 확인된 시뮬레이션 좌표 변환 상태:
- `map → odom`: SLAM Toolbox 정상 브로드캐스트 확인 (오차 x=-0.000, y=-0.000, z=-0.001)
- `odom → base_footprint`: Gazebo 오도메트리 플러그인 (약 50.2Hz)
- `base_footprint → base_link`: 정적 변환
- `base_link → laser_link`: 라이다 프레임 정적 변환
- `base_footprint → front/rear_wheel_link`: 휠 링크 조인트 상태 (약 60.2Hz)

---

## 3. SLAM 매핑 주행 실험 및 타임아웃 관측

`test_slam_mapping.py`를 통해 지정된 주행 패턴을 실행하고 SLAM 갱신 상태를 평가하였다.

### 3.1 주행 단계별 실행 결과
- **Step 2.1 (제자리 360도 회전, 목표 6.28 rad):**
  - 회전 속도 0.4 rad/s로 주행 중 5.43 rad만 누적된 상태에서 제한 시간(25.0초)이 초과되어 **회전 타임아웃으로 실패 (`SIM_TIMEOUT (25.0s, accumulated: 5.43 rad)`)**.
- **Step 2.2 (직진 0.8m) 및 Step 2.3 (역회전):**
  - Step 2.1 실패에 따라 안전을 위해 이후 주행 단계는 **중단 (Aborted)** 처리됨.

### 3.2 지도 품질 및 기하학적 형태 분석
- 생성된 점유격자 지도 크기: 98 × 98 cells (4.90m × 4.90m, 해상도 0.05m)
- 영역별 셀 분포:
  - 미탐색 영역 (Unknown, -1): **9,443셀 (98.3%)**
  - 자유 공간 (Free, 0~25): 160셀 (1.7%)
  - 점유 벽면 (Occupied, 65~100): **1셀 (0.0%)**
- 방 치수 검출:
  - 검출된 벽면 영역(점유 셀 ≥ 65 기준): X span 0.00m, Y span 0.00m (Ground Truth: 약 5.0m)
  - 치수 오차: dX = 500.0cm, dY = 500.0cm
- 판정: 회전 주행이 완료되지 못하고 스캔 데이터 누적이 부족하여 방의 형상이 거의 표현되지 못한 미완성 지도로 확인됨.

---

## 4. 지도 파일 저장 및 유효성 확인

주행 실패로 지도 품질은 미흡했으나, 실험 스크립트의 지도 파일 생성 및 헤더 유효성 확인 절차는 정상 완료됨을 확인하였다.
- 저장 파일: `enclosed_room_slam_map_20261013_212244.pgm` (9,617 Bytes), `enclosed_room_slam_map_20261013_212244.yaml` (151 Bytes)
- 확인 내용: PGM 매직 넘버(`P5`), 크기(98×98), 픽셀 페이로드 및 YAML 메타데이터 정상 판독 완료.
- 주의: **지도 파일 저장 절차 성공이 SLAM 지도 품질이나 완성도를 의미하지 않음**을 구분하여 기록함.

---

## 5. 10/13 작업 결과 요약

```text
[x] Gazebo Classic 기반 LIMO 시뮬레이션 구동 및 SLAM Toolbox 연동 확인
[x] 시뮬레이션 환경 내 odom → base_footprint → base_link → laser_link TF 확인
[!] 360도 제자리 회전 주행 타임아웃(25초 초과, 5.43 rad) 및 이후 주행 중단
[!] 생성된 시뮬레이션 지도의 98.3%가 미탐색 영역으로 남음 (품질 불량)
[x] 시뮬레이션 맵 파일(.pgm, .yaml) 생성 및 파일 헤더 유효성 확인 완료
```

# 2026-10-15 — 자율 매핑 런치 파일 구조 분석 및 현장 보고 사항

## 1. 런치 파일 구조 및 TF 실행 순서 분석 (확인된 사실)

- 분석 근거 (코드 및 설정 파일):
  - 저장소 루트 메모 파일: `2026_10_15` (12:52 KST 작성)
  - 대상 파일: `limo_auto_mapping.launch.py`, `explore.yaml`

### 1.1 분석 배경
- 현재 런치 파일은 `base_link → base_footprint` 간 정적 변환을 제공함.
- `explore.yaml`은 `robot_base_frame: base_link`로 설정되어 있으나, Nav2 스택의 일부 노드는 `base_footprint`를 참조함.
- `explore_node`가 static TF 노드보다 먼저 실행될 경우, 탐색 노드가 시작 시점에 필요한 TF를 즉시 참조하지 못하고 대기하거나 초기 오류/경고를 기록할 수 있음.

### 1.2 제안된 조치 내용
- 탐색 노드(`explore_node`)를 시작하기 전에 `base_link → base_footprint` 정적 TF 노드가 먼저 구동되어 변환을 안정적으로 제공하도록 런치 파일 내 실행 순서를 변경 배치하는 방안 도출.

---

## 2. 현장 매핑 및 지도 저장 관련 보고 사항 (로컬 증거 미확보 / 확인 필요)

- **보고된 내용:**
  - 런치 순서 조정 후 미래관 3층 현장 구역에서 매핑을 진행하고, 지도를 저장하여 `MC_3F_10_15` (`MC_3F_10_15.pgm`, `MC_3F_10_15.yaml`) 파일명으로 보관 중이라는 보고가 있었음.
- **현황 및 확인 필요 사항:**
  - 현재 로컬 개발 머신 및 본 Git 저장소 내에는 `MC_3F_10_15` 실물 지도 파일 및 해당 주행/저장 실행 로그가 존재하지 않음.
  - 증거가 없다는 이유만으로 작업 자체를 단정적으로 배제하지 않으며, **실제 LIMO 온보드 PC 환경에서 주행 여부, 실행 로그, 맵 파일 존재 여부를 추가 확인해야 할 항목**으로 분리하여 관리함.
  - 실제 런치 순서 변경 적용 여부와 그에 따른 TF 경고 개선 효과 역시 별도 실행 로그를 통해 추가 확인이 필요함.

---

## 3. 10/15 작업 결과 요약

```text
[x] limo_auto_mapping.launch.py 내 explore_node와 static_tf 실행 순서 문제 분석 (메모 확인)
[x] 탐색 노드 시작 전 static TF 선행 구동 배치 방안 도출
[?] 런치 파일 실행 순서 변경의 실제 적용 및 TF 경고 개선 효과 (실행 로그 미확보 / 확인 필요)
[?] 현장 구역(미래관 3층) 주행 및 MC_3F_10_15 지도 파일 저장 (로컬 증거 미확보 / 온보드 확인 필요)
[!] 자율 탐색 중 주행 정지 및 목표 반복(Goal Oscillation) 문제의 해결 여부 (미해결)
```