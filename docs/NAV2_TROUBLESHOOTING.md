# Nav2 자율주행 검증

## 목적

Nav2가 지도 위에서 목표점으로 이동하는 과정을 센서 입력, TF, localization, costmap, planner, controller, 속도 명령으로 나누어 검증한다. 현재 문서는 검증 계획과 확인 기준을 정리한 것이며, 아래 항목을 모두 구현 또는 실기기 검증했다는 뜻은 아니다.

## 데이터 및 제어 흐름

```text
/scan + TF + map + localization
              ↓
         Nav2 Costmaps
              ↓
        Global Planner
              ↓
        Local Controller
              ↓
           /cmd_vel
              ↓
           LIMO Base
```

## 문제를 분리하는 기준

| 관찰 현상 | 먼저 확인할 항목 | 확인 방법 |
|---|---|---|
| 목표를 보내도 경로가 만들어지지 않음 | map, localization, TF, costmap | RViz에서 지도·로봇 pose·costmap을 확인하고 Nav2 로그를 수집 |
| 경로는 있지만 로봇이 움직이지 않음 | controller 상태, velocity command, LIMO Base | Nav2 action 상태와 `/cmd_vel`, Base 노드 상태를 각각 확인 |
| 장애물 앞에서 경로가 끊김 | 장애물 관측, sensor TF, costmap layer/inflation | `/scan`과 costmap에 장애물이 반영되는지 비교 |
| 로봇 위치가 지도와 어긋남 | map–odom–base TF, localization | TF와 localization pose를 확인하고 초기 pose를 다시 검증 |
| 목표를 반복해서 취소하거나 재전송함 | action 상태, planner/controller 결과, recovery, goal tolerance | 시간순 로그와 action 상태를 수집해 최초 실패 지점을 찾음 |

표는 진단 방향을 제시하며 특정 원인이 현재 문제의 원인이라고 단정하지 않는다.

## 검증 순서

1. **입력:** `/scan`, odometry, TF가 지속적으로 발행되는지 확인한다.
2. **지도 및 localization:** 지도 파일을 로드하고 로봇 pose가 지도와 일치하는지 확인한다.
3. **Costmap:** RViz에서 global/local costmap이 올바른 위치에 장애물을 표시하는지 확인한다.
4. **Global planning:** 정지 상태에서 안전한 목표점을 지정하고 경로 생성 여부를 확인한다.
5. **Controller:** 생성된 경로를 따라가는 동안 controller 상태와 `/cmd_vel`을 확인한다.
6. **장애물 반응:** 정적 장애물부터 시험하고, 동적 장애물은 안전한 통제 환경에서 별도로 검증한다.
7. **성능 기록:** 목표 도착 여부, 실패 유형, 소요 시간, 재시도 횟수와 환경을 기록한다.

## 현재 상태

- SLAM과 지도 저장은 [SLAM 및 지도 작성](SLAM_MAPPING.md)에 기록된 범위까지만 확인되었다.
- Nav2의 목표점 도착, 안정적인 경로 추종, 장애물 회피 성공률은 이 문서를 작성하는 시점에 증거가 확보되지 않았으므로 **미검증**으로 둔다.
- 실기기 테스트 전에는 속도 제한과 비상 정지 방법을 확인하고, 로봇 주변을 통제한다.

## 실험 결과 기록 템플릿

### 문제
무슨 현상이 어떤 조건에서 발생했는가?

### 원인 분석
로그·토픽·TF·costmap 등으로 확인된 사실은 무엇이며, 원인 가설은 무엇인가?

### 해결 조치
변경한 파라미터나 코드, 실행 명령은 무엇인가?

### 검증
동일 조건에서 문제가 재현되는가? 변경 후 무엇이 달라졌는가?

### 한계 및 다음 작업
아직 확인하지 못한 조건과 다음 실험을 기록한다.
