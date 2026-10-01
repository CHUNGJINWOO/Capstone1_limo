# 2026-09-30 — LIMO 실기기 환경 및 LiDAR / LIMO Base 확인

## 1. 작업 환경

- 로봇: AgileX LIMO
- 작업 컴퓨터: LIMO onboard PC
- 사용자: `pcu`
- OS: Ubuntu 22.04.5
- ROS 2: Humble
- Workspace: `~/agilex_ws`

이번 작업에서는 LIMO onboard PC에 연결된 실제 하드웨어를 기준으로 ROS 2 환경, LIMO Base, YDLiDAR T-mini Plus의 연결 상태를 단계적으로 확인하였다.

ROS 2와 LIMO 관련 기능은 실제 하드웨어에서 실행하고 터미널 출력 및 ROS 2 Topic을 통해 동작 여부를 확인하였다.

[출처: 교재 / 실기기 검증]

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

해당 경고는 확인하였으나 이번 작업에서는 workspace 환경 자체를 수정하지 않았다.

[출처: 교재 / 실기기 검증]

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

[출처: 실기기 검증]

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

[출처: 교재 / 실기기 검증]

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

[출처: 교재 / 실기기 검증]

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

[출처: 교재 / 실기기 검증]

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

[출처: 교재 / 실기기 검증]

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

[출처: 교재 / 실기기 검증]

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

[출처: 실기기 검증]

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

[출처: 실기기 검증]

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

[출처: 교재 / 실기기 검증]

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

[출처: 실기기 검증]

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

[출처: 교재 / 실기기 검증]

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

따라서 09/30의 실제 작업은 **LIMO Base와 LiDAR의 실기기 연결 및 ROS 2 데이터 확인 단계까지 완료한 것**으로 기록한다.

[출처: 실기기 검증]

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

[출처: 실기기 검증]

# 2026-10-01 — LIMO Base / LiDAR / RViz / SLAM 연동 검증

## 1. 작업 목적

09/30에 확인한 LIMO Base와 YDLiDAR T-mini Plus를 동시에 실행하고,

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

구조가 실제 LIMO onboard PC에서 동작하는지 확인하였다.

이번 작업에서는 특히 RViz의 LaserScan QoS 문제와 SLAM Toolbox 실행 상태를 확인하였다.

[출처: 실기기 검증]

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

이번 작업에서는 해당 workspace 경고 자체를 수정하지 않았다.

[출처: 실기기 검증]

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

[출처: 교재 / 실기기 검증]

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

[출처: 교재 / 실기기 검증]

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

[출처: 실기기 검증]

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

[출처: 교재 / 실기기 검증]

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

[출처: 교재 / 실기기 검증]

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

[출처: 교재 / 실기기 검증]

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

[출처: 실기기 검증]

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

[출처: 실기기 검증]

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

[출처: 실기기 검증]

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

[출처: 실기기 검증]

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

[출처: 실기기 검증]

---

## 14. /map 확인

SLAM Toolbox 실행 후 다음 명령을 사용하였다.

```bash
ros2 topic echo /map --once
```

그러나 실제 지도 데이터가 정상적으로 출력되는 것까지 확인하지 못하였다.

따라서 이번 작업에서는:

```text
SLAM Toolbox 실행        [확인]
/scan 구독               [확인]
/map Publisher 존재      [확인]
실제 map 생성            [미확인]
정상적인 지도 확장        [미확인]
```

으로 기록한다.

**현재 단계에서 SLAM 또는 Mapping이 성공했다고 기록하지 않는다.**

[출처: 실기기 검증]

---

## 15. 현재 SLAM 문제 상태

현재 LiDAR 데이터 자체는 정상적으로 들어오고 있다.

```text
YDLiDAR
   ↓
/scan
   ↓
LaserScan
   ↓
SLAM Toolbox
```

까지는 확인되었다.

하지만 LIMO가 이동할 때 RViz에서 이전처럼 주변 공간이 지속적으로 확장되면서 지도 형태가 생성되는 현상은 현재 확인되지 않았다.

따라서 현재 문제는 단순히 LiDAR 하드웨어나 `/scan` 데이터가 없는 문제로 단정할 수 없다.

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

[출처: 실기기 검증]

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

따라서 **교재를 기준으로 삼는 현재 프로젝트에서는 향후 SLAM 구현 시 교재의 Cartographer 구성을 먼저 확인하는 것이 기준선이 된다.**

현재 10/01에 실행한 `slam_toolbox`는 교재의 LIMO SLAM 실습을 그대로 적용한 것이 아니라 별도로 진행한 실험이므로, 개발 기록에서는 이를 구분한다.

[출처: 교재 / 실기기 검증]

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

[출처: 실기기 검증]

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

단, `/tmp/limo_slam_async.yaml`은 현재 재부팅 이후 존재하지 않는 것이 확인되었으므로, 위 명령은 **당시 사용했던 실행 방식 기록**으로만 남긴다. 현재 동일한 명령을 그대로 재실행할 수 있다고 단정하지 않는다.

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

[출처: 교재 / 실기기 검증]

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

그 후 교재 방식으로 실행할 수 있는지 확인하고, 실제 LIMO에서 지도 생성이 되는지 검증한다.

지도 생성이 실제로 확인된 경우에만:

```bash
ros2 run nav2_map_server map_saver_cli -f map
```

을 사용하여 지도 저장 단계로 진행한다.

[출처: 교재 / 실기기 검증]