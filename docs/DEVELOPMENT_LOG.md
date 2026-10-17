# 2026-09-30 — LIMO 온보드 환경 및 LiDAR / LIMO Base 점검

## 1. 작업 환경

- 로봇: AgileX LIMO
- 작업 컴퓨터: LIMO onboard PC
- 계정: `pcu`
- OS: Ubuntu 22.04.5
- ROS 2: Humble
- Workspace: `~/agilex_ws`

> [!NOTE]
> 09/30 및 10/01 기록은 학교 소유 LIMO 로봇에 기존부터 구성되어 있던 기본 환경(`~/agilex_ws`)과 패키지를 활용하여 하드웨어 통신 및 센서 상태를 점검한 초기 기록입니다.

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

## 2. 현장 자율 매핑 및 실내 지도 저장 결과

- **수행 내용:**
  - 런치 파일의 static TF 선행 구동 배치 구조를 적용하여, LIMO 실차를 이용해 미래관 3층 현장 구역에서 자율 매핑을 수행함.
- **지도 저장 상태 (LIMO 온보드 PC):**
  - 사용자 보고에 따르면 매핑 후 `MC_3F_10_15.pgm`과 `MC_3F_10_15.yaml`이 LIMO 온보드 PC에 생성되어 보관 중임.
  - 사용자가 제공한 Map Saver 출력에는 `764 × 809` cells, `0.05 m/pix`와 `Map saved successfully`가 기록되어 있음. 출력 경로는 당시 `/home/pcu/map_limo.pgm` 및 `/home/pcu/map_limo.yaml`이며, 이후 `MC_3F_10_15`로 이름을 변경했다고 보고함.
  - 파일이 온보드 PC에 있다는 사실은 사용자 보고를 기준으로 기록함. 현재 실제 파일 경로와 YAML의 `image` 참조는 별도 확인이 필요하며, 지도 품질과 실내 구조 정합성도 아직 검증되지 않음.
- **Git 저장소 동기화 상태:**
  - 해당 지도 파일은 현재 LIMO 온보드 환경에 위치하며, 본 로컬 개발 머신 및 GitHub 원격 저장소(`Capstone1_limo`)에는 아직 동기화(커밋)되지 않은 상태임. 향후 온보드 환경에서 지도 파일을 추출하여 본 저장소로 동기화할 예정임.

---

## 3. 10/15 작업 결과 요약

```text
[x] limo_auto_mapping.launch.py 내 explore_node와 static_tf 실행 순서 문제 분석 (메모 확인)
[x] 탐색 노드 시작 전 static TF 선행 구동 배치 방안 도출
[x] 현장 구역(미래관 3층) 주행 및 MC_3F_10_15 지도 생성 완료 (LIMO 온보드 PC 보관 중)
[-] 지도 파일의 본 Git 저장소 커밋 및 동기화 (진행 예정)
[!] 자율 탐색 중 주행 정지 및 목표 반복(Goal Oscillation) 문제의 해결 여부 (미해결 과제)
```

# 2026-10-16 — `limo_capstone` 패키지 빌드 오류 수정 및 검증

## 1. 문제 및 원인 분석 (확인된 사실)

- **대상 파일:** `capstone_ws/src/limo_capstone/setup.py`
- **증상:** `capstone_ws` 클린 빌드 사전 점검 중 `limo_capstone` 패키지 빌드 단계에서 다음 오류와 함께 실패함:
  ```text
  error: can't copy 'maps/install': doesn't exist or not a regular file
  ```
- **원인:**
  - `capstone_ws/src/limo_capstone/maps/` 내부에 과거 빌드 실행으로 생성된 `build/`, `install/`, `log/` 하위 디렉터리가 존재함(Git 추적에서는 `.gitignore`로 제외됨).
  - `setup.py` 18행의 `data_files` 설정이 `(os.path.join('share', package_name, 'maps'), glob('maps/*'))`로 작성되어 있어, 일반 지도 파일뿐 아니라 하위 디렉터리(`maps/build`, `maps/install`, `maps/log`)까지 복사 대상으로 포함되면서 `setuptools` 파일 복사 단계에서 오류가 발생함.

---

## 2. 수정 내용

- `capstone_ws/src/limo_capstone/setup.py` 18행에서 `os.path.isfile()`을 사용하여 `maps/` 하위의 일반 파일만 수집하도록 최소 수정함:
  ```python
  (os.path.join('share', package_name, 'maps'), [f for f in glob('maps/*') if os.path.isfile(f)]),
  ```

---

## 3. 검증 및 실제 결과

1. **Python 문법 검사:**
   ```bash
   python3 -m py_compile capstone_ws/src/limo_capstone/setup.py
   ```
   - 오류 없이 통과 확인.
2. **분리 경로(`/tmp`) 단일 패키지 클린 빌드 검증:**
   ```bash
   source /opt/ros/humble/setup.bash
   cd /home/cjinw/school/2026-2/Capstone1_limo/capstone_ws

   colcon --log-base /tmp/capstone1_limo_limo_capstone_log build \
     --build-base /tmp/capstone1_limo_limo_capstone_build \
     --install-base /tmp/capstone1_limo_limo_capstone_install \
     --packages-select limo_capstone
   ```
   - 실행 결과: `Finished <<< limo_capstone [0.68s]`, `Summary: 1 package finished`로 빌드 성공 확인.
3. **설치 경로 내 지도 파일 포함 확인:**
   - `/tmp/capstone1_limo_limo_capstone_install/limo_capstone/share/limo_capstone/maps/` 경로에 `MC_3F_10_15.{pgm,png,yaml}`, `MC_2F_10_07.{pgm,yaml}`, `map_limo.{pgm,yaml}`, `demo_map.{pgm,png,yaml}`, `domi_map.{pgm,png,yaml}` 총 13개 지도 파일과 `launch/`, `config/` 파일이 정상 설치됨을 확인한 후 `/tmp` 임시 빌드 산출물을 정리함.

---

## 4. 10/16 작업 결과 요약

```text
[x] capstone_ws 워크스페이스 Git 통합 및 초기 커밋(54699d0) 완료
[x] limo_capstone/setup.py 내 maps/* 디렉터리 포함 오류 원인 확인 및 os.path.isfile() 필터 적용
[x] python3 -m py_compile 문법 검사 및 colcon build (--packages-select limo_capstone) 성공 확인
[x] 설치 경로(share/limo_capstone/maps) 내 MC_3F_10_15 등 지도 파일 13개 정상 설치 확인
[!] robot_localization, ydlidar_ros2_driver, astra_camera 시스템/SDK 의존성 미설치 (별도 검토 과제)
```

# 2026-10-16 — 문제 C: 자동 매핑 조기 정지(Premature Goal Succeeded) 원인 규명 및 파라미터 조정

## 1. 문제 현상 및 배경

- **현상:** `limo_auto_mapping.launch.py` 실행 시 로봇이 물리적으로 이동하지 않고 즉시 다음 로그를 출력하며 탐색이 조기 정지됨:
  ```text
  [controller_server] Received a goal, begin computing control effort.
  [controller_server] Reached the goal!
  [bt_navigator] Goal succeeded
  ```
  당시 로그에 기록된 목표 좌표: `(-0.79, -0.63)`

---

## 2. 원인 정밀 분석 (C++ 소스 및 수치 기반)

1. **프론티어 탐색 패키지 및 파라미터 구조 확인:**
   - 사용 패키지: `explore_lite` (v1.0.0, `capstone_ws/src/m-explore-ros2/explore`)
   - 파라미터 명칭: `min_frontier_size` (타입: `float`, 단위: **미터[m]**)
   - `frontier_search.cpp` 79-80행:
     ```cpp
     if (new_frontier.size * costmap_->getResolution() >= min_frontier_size_) {
       frontier_list.push_back(new_frontier);
     }
     ```
     `size`는 셀 수, `getResolution()`은 셀당 미터(0.05m/cell)이므로 `min_frontier_size`의 단위는 명백히 미터(m)임.

2. **파라미터 역전 현상 (`min_frontier_size < xy_goal_tolerance`):**
   - 기존 설정:
     - `explore.yaml`: `min_frontier_size: 0.20` (0.20m = 20cm, 5cm 해상도 기준 4개 셀)
     - `nav2_explore_params.yaml`: `general_goal_checker.xy_goal_tolerance: 0.25` (0.25m = 25cm)
   - 분석:
     - `min_frontier_size`(0.20m)가 목표 도달 판정 반경 `xy_goal_tolerance`(0.25m)보다 작았음.
     - `explore.yaml`의 `potential_scale: 3.0` 가중치로 인해 로봇 바로 근처의 미소 프론티어(4셀 노이즈, 거리 < 0.25m)가 최우선 목표로 선택됨.
     - 목표 좌표 `(-0.79, -0.63)` 수신 즉시 Nav2 `controller_server`(`SimpleGoalChecker`)는 현재 로봇 위치가 이미 목표 허용 반경(0.25m) 내에 있다고 판단하여 물리적 바퀴 구동 없이 즉시 `Reached the goal!` 및 `Goal succeeded`를 반환함.

3. **`explore_lite`의 `SUCCEEDED` 시 블랙리스트 미등록 구조:**
   - `explore.cpp` 384-407행:
     - `ResultCode::ABORTED` 시에만 `frontier_blacklist_`에 추가되고, `ResultCode::SUCCEEDED` 시에는 블랙리스트에 추가되지 않음.
     - 따라서 제자리 성공 판정을 받은 동일/인접 노이즈 프론티어가 다음 주기에도 계속 유효한 상태로 남아있거나, 차단되지 않은 채 `All frontiers traversed/tried out, stopping.` 경고와 함께 탐색이 비정상 조기 종료됨.

---

## 3. 최소 수정 내용

1. **`capstone_ws/src/limo_capstone/config/explore.yaml` (13행):**
   - `min_frontier_size: 0.20` → `min_frontier_size: 0.50`
   - 근거: LIMO Pro 차폭(약 0.22m)보다 크고 `explore_lite` 공식 기본값인 0.50m(10개 셀)로 설정하여 20cm 수준의 미소 음영 노이즈를 배제하고 실질적인 주행 통로만 프론티어로 인식하도록 함.

2. **`capstone_ws/src/limo_capstone/config/nav2_explore_params.yaml` (138, 167행):**
   - `general_goal_checker.xy_goal_tolerance: 0.25` → `0.15`
   - `FollowPath.xy_goal_tolerance: 0.25` → `0.15`
   - 근거: LIMO Pro 차체 규격(전폭 22cm, 반경 11cm)에 적합하게 목표 도달 반경을 15cm로 정밀화하여, `min_frontier_size(0.50m) >> xy_goal_tolerance(0.15m)` (3.33배)의 확실한 마진을 확보하고 조기 도착 판정을 원천 차단함.

---

## 4. 검증 결과 및 실행 명령

1. **YAML 문법 및 유효성 검사:**
   ```bash
   python3 -c "import yaml; yaml.safe_load(open('capstone_ws/src/limo_capstone/config/explore.yaml')); yaml.safe_load(open('capstone_ws/src/limo_capstone/config/nav2_explore_params.yaml')); print('YAML syntax OK')"
   ```
   - 결과: `YAML syntax OK`

2. **`limo_capstone` 패키지 빌드 및 설치 경로 반영 확인:**
   ```bash
   source /opt/ros/humble/setup.bash
   cd capstone_ws
   colcon build --packages-select limo_capstone
   ```
   - 결과: `Finished <<< limo_capstone [0.71s]`, `Summary: 1 package finished`
   - `capstone_ws/install/limo_capstone/share/limo_capstone/config/` 내 `explore.yaml` 및 `nav2_explore_params.yaml`에 수정값(`min_frontier_size: 0.50`, `xy_goal_tolerance: 0.15`)이 정상 반영됨을 확인.

3. **`explore_lite` 패키지 C++ 클린 빌드 검증:**
   ```bash
   colcon build --packages-select explore_lite explore_lite_msgs
   ```
   - 결과: `Summary: 2 packages finished [14.8s]` (C++17 컴파일 성공 확인)

4. **코드 포맷 및 공백 검사:**
   ```bash
   git diff --check
   ```
   - 결과: 공백 오류 및 문법 이슈 없이 0개 경고 통과.

---

## 5. 미검증 항목 및 향후 과제

- **Fortress 시뮬레이션 상의 런타임 탐색 주행 (미검증):**
  - 실차용 `limo_auto_mapping.launch.py`는 `base_link -> base_footprint` static TF를 발행하도록 작성되어 있으나, Fortress 시뮬레이션 모델은 `odom -> base_footprint -> base_link`를 자체 발행하므로 TF 순환 충돌(Problem E)이 발생함.
  - 시뮬레이션 환경에서 자동 매핑을 안전하게 검증하기 위해서는 `limo_nav2_sim.launch.py`와 같이 시뮬레이션 전용 매핑 런치 파일 분리가 필요함.
- **실물 LIMO Pro 온보드 주행 (미검증):**
  - 실물 하드웨어 미연결 상태이며 안전 수칙에 따라 실제 로봇 구동은 보류됨. 온보드 배포 후 실내 환경에서 최종 검증 필요.

---

# 2026-10-16 — 문제 C 해결: explore_lite 목표점(frontier->middle) 최소 수정 및 Fortress 시뮬레이션 210초 장시간 검증

## 1. 검증 배경 및 목적

- **목적:** Fortress 시뮬레이션에서 발생하던 문제 C(조기 정지 및 동일 목표 초고속 무한 루프)의 근본 원인을 해소하기 위해 `explore.cpp`의 목표점 선택 로직을 수정한 후, 3~5분간의 장시간 자율 탐색 런타임 안정성과 지도 확장 여부를 검증함.
- **검증 환경:** Ubuntu 22.04 LTS / ROS 2 Humble / Ignition Fortress / FastDDS (Domain ID 30)
- **실행 런치:** `limo_auto_mapping_sim.launch.py` (시뮬레이션 전용 자동 매핑 런치)
- **모니터링 시간:** 총 210초 (3분 30초)

## 2. 코드 수정 내용 및 원인

- **수정 대상 파일:** `capstone_ws/src/m-explore-ros2/explore/src/explore.cpp` (268행)
- **수정 내용:**
  ```diff
  - geometry_msgs::msg::Point target_position = frontier->centroid;
  + geometry_msgs::msg::Point target_position = frontier->middle;
  ```
- **수정 이유:**
  - 5m × 5m 방 환경에서 360° LiDAR가 방 내부 전체를 Free Space로 인식함에 따라, 미지의 영역(NO_INFORMATION)과의 경계 셀들은 방 둘레 4개 벽면에 고리 형태로 형성됨.
  - 기존 `frontier->centroid`는 둘레 셀들의 산술 평균 좌표이므로, 실제 벽면이 아닌 방 한가운데(로봇 발치 앞 `(0.27, 0.03)`)에 맺히게 됨.
  - 로봇이 20cm만 전진해도 무게중심과의 거리가 `xy_goal_tolerance`(0.08m) 이내로 진입하여 Nav2가 `Reached the goal!`을 반환하고, 실제 벽면의 프론티어는 지워지지 않아 동일 가상 좌표에 대해 초당 40회씩 1,000회 이상 무한 성공 루프를 도는 문제 C가 발생함.
  - `frontier->middle`은 로봇 위치에서 가장 가까운 실제 프론티어 격자 셀(Closest point on frontier, 벽면 방향 약 2~2.5m)이므로, 로봇을 실제 미탐색 경계로 주행시켜 센서로 프론티어를 실질적으로 해소하도록 수정함.

## 3. 210초 장시간 시뮬레이션 검증 결과

1. **반복 목표 현상 완전 해소:**
   - 기존 1,000회 이상 반복되던 무한 루프 횟수: **0회** (완전 소멸).
2. **목표 좌표 순차 전이 (3개 목표):**
   - 1차 목표: `(-0.41, -0.46)`
   - 2차 목표: `(0.78, -0.61)`
   - 3차 목표: `(-1.17, 1.31)`
3. **로봇의 실제 주행:**
   - 시작 위치 `(0.00, 0.00)`에서 `(-0.12, -0.01)`을 거쳐 최종 `(0.222, -0.995)`까지 **1m 이상 횡단 주행** 수행.
4. **지도 크기 및 셀 통계 변화 (실측 데이터):**
   - 지도 크기: $97 \times 97$ ($4.85\text{m} \times 4.85\text{m}$) $\rightarrow$ **$108 \times 108$ ($5.40\text{m} \times 5.40\text{m}$)** 확장.
   - 미탐색(UNKNOWN, -1) 셀: 초기 **7,832개 (83.2%)** $\rightarrow$ 최종 **1,755개 (15.0%)** (68.2%p 대폭 감소).
   - 자유(FREE, 0) 공간 셀: 초기 **1,569개 (16.7%)** $\rightarrow$ 최종 **8,823개 (75.6%)** (4.5배 이상 증가).
   - 장애물(OCCUPIED, 100) 셀: 초기 **8개** $\rightarrow$ 최종 **1,086개** (방 4개 외벽 전면 매핑 완료).
5. **탐색 완료 자동 감지:**
   - 3차 목표 주행 중 방 둘레의 모든 프론티어가 해소되자, `explore_node`가 스스로 탐색 종료를 선언함:
     ```text
     [explore-10] [WARN] [explore_node]: No frontiers found, stopping.
     [explore-10] [INFO] [explore_node]: Exploration stopped.
     [bt_navigator-6] [INFO] [bt_navigator]: Goal canceled
     [controller_server-2] [INFO] [controller_server]: Goal was canceled. Stopping the robot.
     ```

## 4. Progress Checker 경고 및 Recovery 동작 분석

- **현상:** 1차 목표 수행 초기(t = 55s ~ 87s) 동안 `controller_server`에서 `Failed to make progress` 에러가 총 6회 기록됨.
- **원인 분석:**
  - 1차 목표 `(-0.41, -0.46)`가 로봇 후방에 생성되어 제자리 180° 회전 및 방향 정렬을 수행함.
  - Nav2 `SimpleProgressChecker`는 선형 변위($\Delta x, \Delta y$)만 검사하며 각도 변화는 검사하지 않음 (`required_movement_radius: 0.5m`, `movement_time_allowance: 10.0초`).
  - 로봇이 제자리 회전하는 10초 동안 선형 이동 거리가 0.5m 미만이어서 매 10초마다 타임아웃 경고가 발생함.
- **Recovery 결과:**
  - `behavior_server`의 `backup`(후진) 복구 동작이 1회 실행되어 성공(`backup completed successfully`) 후 공간을 확보함.
  - 이후 2차, 3차 목표로 전이되며 정상 주행을 재개하였으므로 시스템 고착으로 이어지지 않음.

## 5. 미검증 항목 및 향후 과제

1. **실물 LIMO Pro 온보드 환경 검증 (미검증):**
   - 실차 하드웨어 미연결 상태로, 모터 토크 및 슬립, 실제 라이다 반사 특성 하에서의 자율 탐색은 온보드 배포 후 실내 환경에서 검증 필요.
2. **원점 복귀(`return_to_init`) 동작 (미검증):**
   - `explore_lite` 소스 코드(`explore.cpp` 72, 115, 452행)에 `return_to_init` 기능이 구현되어 있으나, 현재 기본값(`false`)으로 비활성화되어 있어 이번 실험에서는 최종 위치에서 안전 정지함.
3. **Progress Checker 파라미터 최적화 (향후 과제):**
   - 제자리 회전 시 불필요한 실패 경고를 방지하기 위해 `movement_time_allowance`를 10.0초에서 15.0~20.0초로 완화하는 튜닝 검토 필요.

---

# 2026-10-17 — Phase 12-L/N/O Explore 블랙리스트 검증 및 시뮬레이션 결과

## 1. Phase 12-L: `explore.cpp` 블랙리스트 판정 좌표 기준 통일 및 빌드

- **수정 내용:** [`capstone_ws/src/m-explore-ros2/explore/src/explore.cpp`](file:///home/cjinw/school/2026-2/Capstone1_limo/capstone_ws/src/m-explore-ros2/explore/src/explore.cpp)에서 블랙리스트 조회(`goalOnBlacklist`) 좌표를 `frontier.centroid`에서 `frontier.middle`로 통일함.
- **수정 위치 (총 2곳):**
  1. `visualizeFrontiers()` ([`explore.cpp:196`](file:///home/cjinw/school/2026-2/Capstone1_limo/capstone_ws/src/m-explore-ros2/explore/src/explore.cpp#L196)):
     ```diff
     - m.color = goalOnBlacklist(frontier.centroid) ? red : blue;
     + m.color = goalOnBlacklist(frontier.middle) ? red : blue;
     ```
  2. `makePlan()` 후보 필터링 람다 ([`explore.cpp:258`](file:///home/cjinw/school/2026-2/Capstone1_limo/capstone_ws/src/m-explore-ros2/explore/src/explore.cpp#L258)):
     ```diff
     - return !goalOnBlacklist(f.centroid);
     + return !goalOnBlacklist(f.middle);
     ```
- **수정 목적:**
  - `makePlan()`의 목표 전송(`target_position = frontier->middle`, [`explore.cpp:268`](file:///home/cjinw/school/2026-2/Capstone1_limo/capstone_ws/src/m-explore-ros2/explore/src/explore.cpp#L268)) 및 블랙리스트 저장(`frontier_blacklist_.push_back(target_position)`, [`explore.cpp:284`](file:///home/cjinw/school/2026-2/Capstone1_limo/capstone_ws/src/m-explore-ros2/explore/src/explore.cpp#L284), [`explore.cpp:392`](file:///home/cjinw/school/2026-2/Capstone1_limo/capstone_ws/src/m-explore-ros2/explore/src/explore.cpp#L392))은 `frontier->middle` 좌표를 사용하는 반면, 기존 조회부는 `frontier.centroid`를 비교하여 발생하던 좌표 기준 불일치 가능성을 제거함.
- **빌드 검증:** `colcon build --packages-select explore_lite` 실행 결과 `explore_lite` 패키지 빌드 성공 및 설치 경로(`capstone_ws/install/explore_lite/lib/explore_lite/explore`) 반영 확인.

## 2. Phase 12-N: RPP + `frontier.middle` 정상 주행 시뮬레이션 검증

- **검증 환경:** Ignition Fortress 시뮬레이터 + `limo_auto_mapping_sim.launch.py` (`use_sim_time:=true`, `nav2_sim_params.yaml` RPP 컨트롤러 적용).
- **순차 목표 전송 및 선점 전이 (총 4회 목표 전송, 약 12.13초 주행):**
  - 3초 주기 재계획(`planner_frequency: 0.33 Hz`) 과정에서 새 목표가 전송되면서 이전 첫 세 목표는 `bt_navigator`의 선점(`Received goal preemption request`)으로 처리되어 주행이 속행됨.
  1. `(-0.00, -0.00)` $\rightarrow$ `(-0.41, -0.46)` (`1792212609.102`, 2차 목표 전송으로 선점)
  2. `(0.17, -0.03)` $\rightarrow$ `(1.03, 0.89)` (`1792212612.153`, 3차 목표 전송으로 선점)
  3. `(0.46, -0.07)` $\rightarrow$ `(-1.24, -1.69)` (`1792212615.183`, 4차 목표 전송으로 선점)
  4. `(0.52, 0.00)` $\rightarrow$ `(1.66, 1.56)` (`1792212618.213`, 주행 중 프론티어 완전 해소로 취소 `Goal was canceled`)
- **`/cmd_vel` 및 `/odom` 주행 통계:**
  - `/cmd_vel` (263 샘플): `linear.x` 범위 `0.0000 ~ 0.2600 m/s`, `angular.z` 범위 `-0.5778 ~ +0.5778 rad/s`, 후진 명령(`linear.x < 0`) **0회**, 정상 주행 중 순수 제자리 회전(`|linear.x| < 0.005 && |angular.z| > 0.05`) **0회** (단, 탐색 종료 직후 감속 구간의 2샘플/`0.1 s` 동안만 선속도가 먼저 `0.0`에 도달함).
  - `/odom` (585 샘플): 초기 `(-0.0000, -0.0000)` $\rightarrow$ 최종 `(1.2053, -1.6699)`, 누적 이동 거리(`path_len`) **`2.1797 m`**, 순 변위(`net_disp`) **`2.0594 m`**.
- **지도 셀 통계 및 탐색 종료:**
  - `/map` 셀 변화: 초기 `103 × 99` (`UNKNOWN 742 [7.3%]`, `FREE 8,789 [86.2%]`, `OCCUPIED 666`) $\rightarrow$ 최종 `108 × 99` (`UNKNOWN 721 [6.7%]`, `FREE 9,005 [84.2%]`, `OCCUPIED 966`).
  - 탐색 완료: 모든 프론티어 해소 후 `found 0 frontiers` $\rightarrow$ `No frontiers found, stopping.` $\rightarrow$ `Exploration stopped.` 순으로 정상 종료(`Failed to make progress` **0회**, `BackUp`/`Spin` **0회**).
- **한계 (미검증 사항):**
  - **개별 목표 도달 여부 (미검증):** 이번 실험은 탐색 중 주행과 정상적인 탐색 종료를 검증한 것이며, 네 개 목표의 개별 도달 성공을 검증한 것은 아님. 로그에서 개별 목표의 `SUCCEEDED` 결과는 한 번도 확인되지 않았으며, 첫 세 목표의 최종 액션 `ResultCode`도 로그에 기록되지 않았음(코드 구조상 새 목표 전송 시 `active_goal_id_`가 교체되어 이전 목표 결과 콜백을 무시함).
  - Phase 12-N에서는 모든 목표가 정체 없이 정상 주행으로 해소되어 `Adding current goal to black list`가 **0회** 발생하였으므로, **블랙리스트 타임아웃 분기는 Phase 12-N에서 검증되지 않았음**.

## 3. Phase 12-O: 무이동 정체 조건에서의 블랙리스트 타임아웃 분기 검증

- **검증 설계 (임시 설정):**
  - 프로젝트 파일을 수정하지 않고 임시 YAML(`progress_timeout: 3.0`, `planner_frequency: 1.0`, `movement_time_allowance: 60.0`, `velocity_smoother`의 `max_velocity`/`min_velocity: [0.0, 0.0, 0.0]`)을 사용하여 Nav2 목표를 `ACTIVE` 상태로 유지한 채 로봇 이동만 차단함.
- **타임아웃 및 블랙리스트 등록 확인:**
  - 첫 목표 수락 시각 `sim_t = 9.167 s` (`wall_t = 1792214458.650 s`) 이후 무이동 상태가 지속되자, `sim_t = 12.988 s` (`wall_t = 1792214462.649 s`, 경과 **`3.821 s`**)에 `[explore_node]: Adding current goal to black list` 로그가 출력됨.
- **`middle` vs `centroid` 좌표 차이 및 마커/필터 동작 확인:**
  - 당시 검출된 단일 프론티어(`id=0`, `n=823` points)의 실제 좌표는 `middle = (-0.409, -0.457)`, `centroid = (0.272, 0.028)`로 두 좌표 간 차이가 $|\Delta x| = 0.681\,\text{m} > 0.25\,\text{m}$, $|\Delta y| = 0.485\,\text{m} > 0.25\,\text{m}$ (`tolerance = 0.25 m` 초과)였음.
  - 블랙리스트 등록 직후 `/explore/frontiers`의 동일 프론티어 마커 색상이 `BLUE` (`seq=5`, `r=0, b=1`)에서 **`RED` (`seq=6`, `sim_t = 13.013 s`, `r=1, b=0`)**로 즉시 전환되었으며, `goalOnBlacklist(f.middle)` 필터에 의해 재선택되지 않고 `All frontiers traversed/tried out, stopping.` $\rightarrow$ `Exploration stopped.`로 정상 종료됨.
- **한계 및 산출물 보관 위치:**
  - `/cmd_vel_nav` (82 샘플)에서는 RPP 주행 명령(`linear_x = 0.191822 m/s`, `angular_z = -0.577778 rad/s`)이 생성되었으나, `velocity_smoother` 제한으로 최종 `/cmd_vel` (101 샘플)은 전 구간 `0.000000`, `/odom` (258 샘플) 이동 거리는 `0.000000 m`였음. 따라서 **Phase 12-O는 블랙리스트 타임아웃 분기·마커 색상 전환·후보 필터링을 검증한 테스트이며 실제 주행은 검증하지 않았음**.
  - Phase 12-N 및 Phase 12-O의 임시 테스트 설정, 런타임 로그, CSV 파일(`phase12n_*`, `phase12o_*`)은 Antigravity `scratch` 디렉토리(`/home/cjinw/.gemini/antigravity/brain/0e39003d-5210-4560-840d-5c8e6dd6055f/scratch/`)에 보관되어 있으며 Git 저장소에는 포함하지 않음.

## 4. Phase 12-P1: scratch 다중 Frontier 초기 검출 검증

- **검증 설계 (임시 SDF 및 설정):**
  - 기존 월드(`enclosed_room_fortress.sdf`)와 프로젝트 설정을 수정하지 않고, `scratch` 디렉토리 내 임시 월드(`phase12p_multi_frontier.sdf`)를 구성하여 5m $\times$ 5m 방 중앙에 동쪽 가림벽(`(1.5, 0.0, 0.5)`, 크기 `2.0 × 0.15 × 1.0 m`)과 서쪽 가림벽(`(-1.5, 0.0, 0.5)`, 크기 `2.0 × 0.15 × 1.0 m`, 중앙 통로 폭 `1.0 m`)을 배치함.
  - 임시 설정(`phase12p_explore.yaml`: `planner_frequency: 0.2 Hz`, `progress_timeout: 300.0 s`; `phase12p_nav2_sim_params.yaml`: `velocity_smoother` 속도 `0.0` 고정)으로 초기 프론티어 분할 검출만 단독 확인함.
- **검증 결과 (사실):**
  - XML 파싱 통과 및 Fortress 시뮬레이션 월드 정상 로드를 확인함.
  - 시작 위치(`(0, 0)`)에서 2개의 독립된 프론티어(`found 2 frontiers`)가 검출됨:
    1. **프론티어 A (북쪽):** `size = 332 cells`, `middle = (-0.409, 0.543)`, `centroid = (0.107, 0.858)`, `cost = -16.646600`, 마커 색상 `BLUE`.
    2. **프론티어 B (남쪽):** `size = 322 cells`, `middle = (-0.359, -0.507)`, `centroid = (0.316, -0.829)`, `cost = -16.146949`, 마커 색상 `BLUE`.
  - 텔레메트리 확인: `/cmd_vel` 29샘플 전부 `0.000000`, `/odom` 144샘플 동안 위치 및 yaw 변화 없음(이동 거리 `0.000000 m`).
  - Git 저장소 파일은 변경하지 않았으며 모든 임시 파일(`phase12p_*`)은 `scratch` 디렉토리에만 생성함.
- **해석 및 한계:**
  - 다중 프론티어 검출 조건 및 두 프론티어 `middle` 간 이격($|\Delta y| = 1.050\,\text{m} > 0.25\,\text{m}$)은 확인되었으나, 실제 로봇 주행과 목표 전환은 P1에서 검증하지 않았음.
  - 임시 테스트 월드에서의 프론티어 개수가 실제 환경의 탐색 성능을 보장하지는 않음.

## 5. Phase 12-P2: 다중 프론티어 블랙리스트 처리 및 A $\rightarrow$ B 목표 전환 알고리즘 검증

- **검증 설계:**
  - Phase 12-P1의 임시 다중 프론티어 월드(`phase12p_multi_frontier.sdf`)와 속도 제한 설정(`velocity_smoother` 출력 `0.0` 고정)에서 `planner_frequency: 1.0 Hz`, `progress_timeout: 3.0 s`를 적용하여 목표 A 타임아웃 후 차순위 목표 B로의 전환 흐름을 검증함.
- **시간순 상태 전이 및 검증 결과 (사실):**
  1. **초기 프론티어 검출 및 목표 A 선택 (`sim_t = 9.443 s`, `wall_t = 1792218711.123 s`):**
     - A(`middle = (-0.409, 0.543)`, `BLUE`)와 B(`middle = (-0.359, -0.507)`, `BLUE`) 두 개가 검출되었고, 두 프론티어의 `middle` y좌표 차이는 `1.050 m`(블랙리스트 판정 허용 오차 `0.25 m` 초과)였음.
     - 최소 비용 프론티어인 목표 A(`(-0.41, 0.54)`)가 전송 및 수락됨.
  2. **목표 A 타임아웃 및 블랙리스트 등록 (`sim_t = 12.452 s`, `wall_t = 1792218714.134 s`):**
     - 무이동 상태에서 약 `3.009 s` 경과 후 `progress_timeout`이 발생하여 `[explore_node]: Adding current goal to black list`가 기록됨.
  3. **마커 색상 개별 전환 및 목표 B 선택 (`sim_t = 12.465 s`, `wall_t = 1792218714.146 ~ 1792218714.155 s`):**
     - `/explore/frontiers`(`seq=5`)에서 프론티어 A는 **`RED`**(`r=1, b=0`)로 전환되고 프론티어 B는 **`BLUE`**(`r=0, b=1`)를 유지함.
     - 조기 `All frontiers traversed` 종료 없이 즉시 차순위 프론티어 B(`(-0.36, -0.51)`)가 선택·전송 및 수락됨.
  4. **목표 B 유지 및 A 재선택 방지 (`sim_t = 12.465 ~ 15.448 s`, `seq=6, 7, 8`):**
     - 목표 B로 전환될 때 `last_progress_`가 갱신되어 즉시 연쇄 타임아웃되지 않고 후속 1.0 Hz 재계획 주기 동안 목표 B가 유지되었으며, 블랙리스트에 등록된 A는 재선택되지 않음.
  5. **목표 B 타임아웃 및 전체 후보 소진 종료 (`sim_t = 16.445 ~ 16.468 s`, `wall_t = 1792218718.135 s`):**
     - 목표 B 전환 후 약 `3.980 s` 경과 시점에 목표 B도 타임아웃으로 블랙리스트에 추가(`Adding current goal to black list`)됨.
     - `/explore/frontiers`(`seq=10`, `sim_t = 16.468 s`)에서 A와 B 모두 **`RED`**로 전환된 뒤 `All frontiers traversed/tried out, stopping.` $\rightarrow$ `Exploration stopped.`가 출력되며 탐색이 종료됨.
     - 탐색 종료 시 활성 목표 B에 대해 취소(`Goal was canceled`)가 수행됨.
- **텔레메트리 및 액션 결과 (사실):**
  - `/cmd_vel_nav` (142 샘플): 목표 A 구간(`seq=1~62`)에서 `linear_x = 0.167735 m/s, angular_z = +0.577778 rad/s`(북쪽 조향), 목표 B 전환 후 구간(`seq=63~141`)에서 `linear_x = 0.171919 m/s, angular_z = -0.577778 rad/s`(남쪽 조향)가 생성됨.
  - `/cmd_vel` (162 샘플): `velocity_smoother` 제한에 의해 전 구간 `0.000000`.
  - `/odom` (405 샘플): 위치 및 yaw 변화 없음, 누적 이동 거리 `0.000000 m`.
  - 액션 종료 상태: 목표 A는 목표 B 전송 시 선점(`Received goal preemption request`)되었고, 목표 B는 탐색 종료 시 취소(`Goal was canceled`)되었으며, 두 목표 모두 `SUCCEEDED`에는 도달하지 않았음.
- **해석 및 한계:**
  - `progress_timeout` 발생 $\rightarrow$ 블랙리스트 등록 $\rightarrow$ 해당 프론티어 후보 제외(`goalOnBlacklist(f.middle)`) $\rightarrow$ 차순위 프론티어 선택(A $\rightarrow$ B) $\rightarrow$ 모든 후보 소진 시 탐색 종료의 알고리즘 흐름이 정상 동작함을 검증함.
  - 단, `/cmd_vel_nav`는 `velocity_smoother` 이전의 Nav2 컨트롤러 출력이므로 실제 로봇 구동 증거가 아니며, 최종 구동 토픽인 `/cmd_vel`은 `0`이었고 실제 이동 거리도 `0.000000 m`였음.
  - 또한 `All frontiers traversed/tried out, stopping.`은 모든 후보 프론티어가 블랙리스트 등으로 소진되어 중단됨을 뜻하며, 맵 전체 탐색 완료(`No frontiers found, stopping.`)와는 구별됨.
  - 실제 주행 상태에서 목표 B로 이동하는 동작, 개별 목표의 `SUCCEEDED` 도달, 그리고 `reachedGoal()`의 `ResultCode::ABORTED` 콜백에 의한 블랙리스트 분기는 이번 테스트에서 검증하지 않았음.
  - 관련 임시 파일 및 로그(`phase12p_*`, `phase12p2_*`)는 모두 Antigravity `scratch` 디렉토리(`/home/cjinw/.gemini/antigravity/brain/0e39003d-5210-4560-840d-5c8e6dd6055f/scratch/`)에 보관되어 있으며 Git 저장소에는 포함하지 않음.

## 6. 후속 작업 우선순위 (진도 보고서 및 중간 발표 대비)

1. **1순위: 맵 파일 로딩/저장 경로 및 포맷 검증**
   - 저장된 맵 파일(`.yaml`, `.pgm` 등)의 경로, 상대/절대 참조, 메타데이터 및 `map_server` 로딩 절차를 점검하여 맵 로딩 실패 원인을 재현·해결함.
2. **2순위: 정지 및 탐색 종료 원인 진단 로그·확인 절차 정리**
   - `/cmd_vel_nav`(컨트롤러 출력)와 `/cmd_vel`(최종 로봇 입력)의 구분, 그리고 프론티어 완전 해소 종료(`No frontiers found`)와 후보 소진 종료(`All frontiers traversed/tried out`)의 구분 기준을 체크리스트화함.
3. **3순위: 빌드 및 런타임 반영 여부 사전 점검 절차 정리**
   - 수정한 소스 코드 및 YAML 파라미터가 `colcon build` 설치본(`install/`)과 실제 실행 중인 노드에 반영되었는지 실행 전 확인하는 절차를 표준화함.
4. **4순위: 속도 제한을 해제한 상태에서 A $\rightarrow$ B 전환 후 실제 주행 통합 테스트**
   - 목표 A가 차단되거나 실패한 뒤 목표 B로 전환하여 실제 `/cmd_vel` 구동 및 `/odom` 주행이 이어지는지 통합 검증함.
5. **5순위: 진도 보고서 및 중간 발표 PPT용 결과·근거 표 정리**
   - Phase 12-L/N/O/P1/P2의 문제 원인, 해결 내역, 정량 검증 지표, 남은 한계를 보고서 및 발표 자료용 표로 정리함.

---

## 7. Phase 14: 시뮬레이션 환경 Nav2 주행 검증 및 실차 지도(MC_3F_10_15) 분리 검증

- **검증 배경 및 목적:**
  - 실차 취득 지도(`MC_3F_10_15`, 미래관 3층 $38.2\text{m} \times 40.45\text{m}$)와 Gazebo Fortress 가상 환경(5m $\times$ 5m 사각 방)을 분리하여 독립적으로 검증함.
  - 가상 환경과 형상이 일치하는 시뮬레이션용 지도(`phase12s_map.yaml`) 기반의 Nav2 전체 스택 주행 검증 및 `MC_3F_10_15`의 단독 `map_server` 로딩 무결성을 확인함.

### 1) 시뮬레이션용 지도 기반 Nav2 주행 검증 (`phase14_nav_verify.sh`)
- **실행 환경:** Gazebo Fortress (`enclosed_room_fortress.sdf`, 5m $\times$ 5m 방), `ROS_DOMAIN_ID=50`, `ROS_LOCALHOST_ONLY=1`.
- **지도 선택:** `phase12s_map.yaml` (해상도 0.05m, $100 \times 100$ cells, $5.0\text{m} \times 5.0\text{m}$, 원점 `[-2.50, -2.51, 0]`).
- **단계별 검증 결과:**
  1. **센서 및 TF 입력:** `/clock`(19 샘플), `/scan`(`laser_link`), `/odom`(`odom` $\rightarrow$ `base_footprint`) 정상 확인.
  2. **Nav2 Lifecycle:** 6개 노드(`map_server`, `amcl`, `controller_server`, `planner_server`, `bt_navigator`, `velocity_smoother`) 전원 `ACTIVE` 진입 성공.
  3. **초기 포즈 및 TF:** `/initialpose` 주입 직후 TF `map -> base_link` 정상 해석 완료 (`trans=(-0.006, 0.028)`).
  4. **액션 및 경로 계획:** `/navigate_to_pose` 액션 서버 연결 성공, 목표 `(0.80, 0.00)` 전송 및 수락, 글로벌 경로 5개 경유점 생성.
  5. **제어 명령 및 실제 이동:**
     - RPP 컨트롤러 속도 명령 `/cmd_vel` 출력: 최대 `vx = 0.2600 m/s`, 유효 명령 16샘플.
     - 로봇 위치 변화 (Odom): 시작 `(-0.0000, -0.0000)` $\rightarrow$ 종료 `(0.8491, -0.0169)`.
     - 이동 거리: 순 변위 `0.8492 m`, 누적 주행 거리 `0.8505 m`.
     - 목표 도달 오차: Odom 기준 목표 도달 오차 `0.0519 m`로 허용 오차(`xy_goal_tolerance = 0.08 m`) 이내 안착. (단, AMCL 최종 추정 오차는 `0.2378 m`로 0.08m 기준 미달).
  6. **액션 최종 상태:** `GoalStatus.STATUS_SUCCEEDED` (코드 4).
  7. **판정 및 한계 (Phase 15-A 정정 반영):**
     - **`SUCCESS (조건부)`**: 액션 성공 및 Odom 0.85m 실제 주행은 검증되었으나, AMCL 최종 위치 오차가 0.238m로 허용치를 초과하였으며 Costmap 토픽 갱신 직접 확인 및 `/map_metadata`는 당시 미검증 상태였음 (Phase 15에서 완전 검증됨).

### 2) 실차 건물 지도(`MC_3F_10_15`) 로딩 및 메타데이터 무결성 검증 (`phase14_map_mc3f_verify.py`)
- **파일 무결성:** `MC_3F_10_15.pgm`(618,091 바이트, P5 바이너리, $764 \times 809$ 격자), `MC_3F_10_15.yaml` 문법 정상.
- **물리적 공간 크기:** Width $764 \times 0.05\text{m} = 38.20\text{m}$, Height $809 \times 0.05\text{m} = 40.45\text{m}$, 원점 `[-13.70, -30.80, 0.00]`.
- **격자 통계 (Phase 15-A 정정 반영):**
  - 원본 PGM 픽셀 값: 점유(0) 10,272개(1.66%), 미지(205) 524,520개(84.86%), 자유(254) 83,284개(13.47%).
  - 실제 발행된 `/map` (free_thresh 0.25 적용 시): 점유 10,272개, 자유 607,804개, 미지 0개 (미지 영역이 자유 공간으로 오인식됨을 규명).
- **Map Server Lifecycle:** `unconfigured` $\rightarrow$ `inactive` $\rightarrow$ `active [3]` 전환 성공.
- **토픽 발행 검증:** `/map` 수신 확인 (원점 `[-13.7, -30.8]`).
- **시뮬레이션 정합 한계 규명:**
  - 5m $\times$ 5m 시뮬레이션 방의 LiDAR 스캔과 38m 규모 실차 지도의 장애물 간 최단 유클리드 오차 전수 측정 결과, 평균 오차 70.5cm (최대 2.41m)로 물리적 공간 불일치에 기인한 필연적 불합치임을 확인.

---

## 8. Phase 15: MC_3F_10_15 전용 Gazebo SDF 월드 자동 생성 및 LiDAR 정합·Nav2 종합 검증

- **검증 배경 및 목적:**
  - 실차 지도(`MC_3F_10_15`)의 점유 격자로부터 1:1 좌표계가 완전히 일치하는 Gazebo Fortress 전용 월드(`mc3f_10_15.sdf`)를 자동 생성하고 시뮬레이션 정합 및 Nav2 주행을 종합 검증함.

### 1) 환경 구축 및 월드 자동 생성
1. **월드 생성기 (`map_to_world.py`):**
   - 2D 점유 픽셀(10,272개)을 수평 우선 탐욕 병합 알고리즘으로 3,732개의 충돌 박스로 압축 생성.
   - 지도 원점 `[-13.7, -30.8]`, 해상도 0.05m를 그대로 적용하여 Gazebo world 좌표계와 ROS 2 `map` 프레임을 1:1 일치시킴.
   - 100% 비트 단위 동일한 재현성(Bit-for-bit reproducibility) 확인.
2. **시뮬레이션 기동 런치 (`limo_mc3f_sim.launch.py`):**
   - 복도 내 안전 스폰 위치 `(-1.07, -0.68)`, 복도 방향 헤딩 `yaw = 2.23 rad` (128°) 지정.
   - URDF 로봇 모델 및 파라미터 브리지 연동.
3. **`free_thresh` 비교 검증 (`verify_free_thresh_comparison.py`):**
   - 원본 YAML (`free_thresh = 0.25`): 점유 10,272개(1.66%), 자유 607,804개(98.34%), 미지 0개(0.00%).
   - 시뮬레이션 사본 (`free_thresh = 0.15`): 점유 10,272개(1.66%), 자유 83,284개(13.47%), 미지 524,520개(84.86%).
   - 원본 실차 파일은 보존하고, 시뮬레이션 Nav2에는 사본(`MC_3F_10_15_sim.yaml`)을 적용하여 미지 영역 추적 유지.

### 2) LiDAR-to-Map 정량적 정합 오차 검증 결과 (347개 유효 빔)
- **평균 오차 (Mean error):** `0.0296 m (2.96 cm)` (Phase 14 기준 70.47 cm 대비 **95.8% 대폭 감소**)
- **중앙값 오차 (Median error):** `0.0000 m (0.00 cm)`
- **최대 오차 (Max error):** `0.1414 m (14.14 cm)` (Phase 14 기준 240.96 cm 대비 대폭 감소)
- **10cm 이내 정상 정합 비율:** **`99.14%`** (잠정 기준 >= 80.0% 초과 달성)
- **20cm 이내 정상 정합 비율:** **`100.00%`**
- **50cm 초과 이상치 비율:** **`0.00%`**
- **정합 판정:** **`PASS`** (지도와 시뮬레이션 월드 간의 기하학적 정합 완료).

### 3) Nav2 전체 스택 및 Costmap 직접 검증
- **Lifecycle 6개 노드:** `map_server`, `amcl`, `controller_server`, `planner_server`, `bt_navigator`, `velocity_smoother` 전원 `ACTIVE (id=3)` 확인.
- **Global Costmap (`/global_costmap/costmap`):** Frame `'map'`, 764×809 크기, 장애물 10,373셀, 인플레이션 47,795셀 정상 갱신 확인.
- **Local Costmap (`/local_costmap/costmap`):** Frame `'odom'`, 60×60 크기, 장애물 144셀, 인플레이션 1,363셀 정상 갱신 확인.
- **AMCL 공분산:** 초기 `0.353`에서 관측 후 `0.276`으로 감소하며 입자 수렴 확인.

### 4) 복도 내 다중 목표 주행 결과
1. **Goal 1 (단거리 1.0m NW: `x=-1.68, y=0.11`):**
   - 로봇 복도 따라 순 변위 `0.832 m` 실제 주행.
   - 최종 AMCL 추정 오차 `0.235 m`.
   - 35초 타임아웃 도달 시 잔여 거리 0.23m로 `xy_goal_tolerance`(0.08m) 이내 미세 정렬 전 종료됨 (Status 6).
2. **Goal 2 (중거리 2.0m NW: `x=-2.29, y=0.90`):**
   - 로봇 복도 따라 추가 순 변위 `0.977 m` 주행 (누적 이동 거리 `1.809 m`).
   - 최종 AMCL 추정 오차 `0.477 m`.
   - 복도 축을 따라 총 1.8m 이상의 물리적 자율 이동 달성 확인.

### 5) 시뮬레이션 RTF 및 안정성
- **RTF (Real-Time Factor):** 3,732개 충돌 박스 초기화 시 지연 후 안정 동작 확인.
- **안정성:** Gazebo Fortress 및 Nav2 전체 스택 정상 기동 및 프로세스 종료 확인 (Exit code 0).

### 6) 한계 및 다음 단계
- 본 시뮬레이션 월드는 2D 점유 지도 격자에서 생성된 모델이므로, 실제 건물의 재질, 유리창 투과, 실제 문 개폐 상태와의 일치성을 증명하지는 않음.
- 실물 LIMO Pro를 이용한 미래관 3층 현장 검증이 필수적임.

---

## 9. Phase 15-B: Nav2 시험 하네스 오류 분석 및 원인 규명

- **조사 배경:**
  - Phase 15의 초기 다중 목표 주행 시험에서 두 목표 모두 Nav2 액션 상태 `6` (`STATUS_ABORTED`)으로 종료되었고, Odom 기준 오차가 2.51m 및 4.16m로 보고된 원인을 규명함.

### 1) 시험 하네스 액션 상태 6의 실제 원인 규명
- **하네스 코드 분석 (`phase15_full_sim_verification.py` 419~424행):**
  - 시험 스크립트에서 호스트 경과 시간 `elapsed > 35.0` 도달 시 `gh.cancel_goal_async()`를 호출하면서 변수 `status`에 `GoalStatus.STATUS_ABORTED (6)`를 직접 대입하는 하드코딩 오류가 존재함을 확인.
- **Nav2 내부 로그 (`phase15_nav2.log`) 대조:**
  - `[bt_navigator]: [navigate_to_pose] [ActionServer] Client requested to cancel the goal. Cancelling.`
  - `[bt_navigator]: Goal canceled`
  - Nav2 내부의 플래너나 컨트롤러가 실패한 것이 아니라, 외부 시험 스크립트가 35초 제한 도달로 취소를 요청하고 상태를 6으로 강제 기록했던 것임.

### 2) Odom 오차(2.51m, 4.16m) 좌표계 직접 감산 결함 규명
- **계산식 분석 (444행):**
  - `odom_err = math.hypot(end_odom['x'] - g['x'], end_odom['y'] - g['y'])`
  - `end_odom`은 로봇 스폰 시 `(0, 0)`으로 초기화되는 **`odom` 프레임** 좌표이고, 목표 `g`는 **`map` 프레임** 좌표임.
  - 서로 다른 좌표계를 TF 변환 없이 직접 뺄셈함으로써 발생한 스크립트 수식 오류(수학적 허위 오차)였음을 규명함. 실제 로봇의 이동 오차는 AMCL 및 TF 기준 잔여 거리(Goal 1: 0.235m, Goal 2: 0.477m)로 평가되어야 함.

---

## 10. Phase 15-C: Nav2 단거리 목표(Goal 1) 재검증 결과

- **검증 환경:**
  - 월드: `mc3f_10_15.sdf` (3,732개 충돌 박스, 복도 스폰: `x=-1.07, y=-0.68, yaw=2.23 rad = 127.8°`)
  - 지도: `MC_3F_10_15_sim.yaml` (`free_thresh: 0.15`, 원본 지도는 보존)
  - DDS 격리: `ROS_DOMAIN_ID=51`, `ROS_LOCALHOST_ONLY=1`
  - 내비게이션 파라미터: 기존 `nav2_sim_params.yaml` 변경 없음 (`xy_goal_tolerance: 0.08m` 유지)
  - 대상 목표: **Goal 1 단독** (`x=-1.68, y=0.11, yaw=2.23 rad`, 직선 거리 0.998m)
  - 하네스 보정: 호스트 65초 안전 제한 적용, 임의 상태 주입 제거, TF 및 AMCL Map 프레임 분리 측정

### 1) 액션 수행 및 시간 측정 결과
- **액션 종료 상태:** Nav2 액션 서버가 **`STATUS_SUCCEEDED` (4)**를 반환하며 자연 정상 종료 (`[controller_server]: Reached the goal!`, `[bt_navigator]: Goal succeeded`).
- **외부 타임아웃 / 취소:** 발생하지 않음 (`timeout_triggered: False`).
- **소요 시간:** 호스트 실제 시간 **`52.90 s`**, 시뮬레이션 시간(`/clock`) **`6.74 s`** (평균 RTF `0.127x`).

### 2) 위치 오차 및 주행량 정량 측정 결과
1. **물리적 이동량 (Odom 프레임):**
   - 로봇 순 물리 주행 거리: **`0.9970 m`** (스폰에서 목표까지의 직선 거리 `0.9980 m` 대비 **99.9%** 이동).
   - Odom 종료 좌표: `(0.9950, 0.0642)`.
2. **TF 연속 갱신 기준 (`map -> base_link`, stamp: 7.480s):**
   - 최종 TF 추정 위치: `(-1.5666, 0.0277, yaw=2.3489 rad = 134.6°)`.
   - 유클리드 위치 오차: **`0.1401 m (14.01 cm)`**.
   - 헤딩 각도 오차: **`6.81° (0.1189 rad)`** (yaw_tolerance 14.3° 이내 만족).
3. **AMCL 파티클 필터 기준 (stamp: 5.800s):**
   - 최종 AMCL 추정 위치: `(-1.4737, -0.0769, yaw=2.2789 rad = 130.6°)`.
   - 유클리드 위치 오차: **`0.2784 m (27.84 cm)`**.
   - 헤딩 각도 오차: **`2.80° (0.0489 rad)`** (yaw_tolerance 14.3° 이내 만족).
4. **Nav2 Controller 피드백 잔여 거리 (종료 시점):**
   - `distance_remaining`: **`0.117 m (11.7 cm)`**.

### 3) 성공 판정 및 측정값 해석 주의사항
- **허용 오차 충족 여부 구분:**
  - Nav2 `controller_server` 내부에서는 자체 goal checker 판정에 따라 `Reached the goal!` 및 액션 `STATUS_SUCCEEDED` (4)를 반환함.
  - 그러나 TF 기준 최종 위치 오차는 **`14.01 cm`**로, 설정된 `xy_goal_tolerance` (**`8.0 cm`**)보다 약 6cm 큼.
  - 따라서 **"Nav2 액션 성공(`STATUS_SUCCEEDED`)"과 "독립적인 8cm 위치 정확도 기준 충족"을 동일시하지 않고 구분하여 기록함.**
- **AMCL vs TF 측정 시각 불일치 및 가설:**
  - AMCL 측정 시각은 `5.800 s`이고 TF 측정 시각은 `7.480 s`로 서로 다름. 두 오차는 동일 시각에 측정된 값이 아니므로 직접 비교할 수 없음.
  - AMCL 파티클 갱신이 종료 약 1.6초 전(`5.800 s`)에 멈춘 원인에 대해, `nav2_sim_params.yaml`의 `update_min_d: 0.25` (25cm) 설정으로 인해 마지막 20cm 극저속 감속 구간의 이동량이 25cm에 미달하여 추가 갱신이 트리거되지 않았을 가능성이 가설로 제기됨 (추후 추가 입증 필요).
- **Goal 2 미검증 명시:**
  - Phase 15-C에서는 Goal 1 단독 재검증만 수행하였으며, **Goal 2(`x=-2.29, y=0.90`)는 아직 성공 여부가 검증되지 않은 미검증 상태**임.
- **물리적 환경 검증 한계 유지:**
  - 본 월드는 2D 점유 지도 격자에서 생성된 기하학적 시뮬레이션 환경이며, 실제 미래관 3층 건물의 물리적 건축 치수, 벽체 재질, 센서 노이즈 환경과의 일치성을 입증한 것은 아님. 실물 LIMO Pro를 이용한 현장 검증이 필수적임.
