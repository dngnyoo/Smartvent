# Gas Avoidance Navigation System

GoPiGo3 로봇의 가스 감지 기반 장애물 회피 시스템 가이드

## 개요

이 시스템은 MQ-2 가스 센서로 유해 가스를 감지하면 해당 위치를 가상 장애물로 설정하여
Nav2가 자동으로 그 지역을 피해 경로를 재계획합니다.

```
┌─────────────────────────────────────────────────────────────┐
│                    시스템 아키텍처                           │
├─────────────────────────────────────────────────────────────┤
│                                                              │
│  ┌──────────┐    ┌─────────────────┐    ┌───────────────┐   │
│  │ MQ-2     │───▶│ /gas_detection  │───▶│ Gas Obstacle  │   │
│  │ Sensor   │    │ (Float32)       │    │ Node          │   │
│  └──────────┘    └─────────────────┘    └───────┬───────┘   │
│                                                  │           │
│                                                  ▼           │
│  ┌──────────┐    ┌─────────────────┐    ┌───────────────┐   │
│  │ Saved    │───▶│ Nav2            │◀───│ /gas_obstacles│   │
│  │ Map      │    │ (Costmap)       │    │ (OccupancyGrid)   │
│  └──────────┘    └────────┬────────┘    └───────────────┘   │
│                           │                                  │
│                           ▼                                  │
│                  ┌─────────────────┐                        │
│                  │ 탈출구로 이동    │                        │
│                  │ (가스 영역 회피)  │                        │
│                  └─────────────────┘                        │
│                                                              │
└─────────────────────────────────────────────────────────────┘
```

## 파일 구조

```
gopigo3_driver/
├── gopigo3_driver/
│   ├── gas_obstacle_node.py    # 가스 감지 → 장애물 변환
│   └── gas_simulator_node.py   # 테스트용 시뮬레이터
├── config/
│   └── nav2_params.yaml        # gas_layer 추가됨
└── launch/
    └── navigation.launch.py    # gas_obstacle 노드 포함
```

## 빠른 시작

### 1. 빌드

```bash
cd ~/nav2_ws
colcon build --packages-select gopigo3_driver
source install/setup.bash
```

### 2. 실행 (테스트 모드)

**터미널 1: 네비게이션 시작**
```bash
ros2 launch gopigo3_driver navigation.launch.py map:=/home/ubuntu/maps/my_room
```

**터미널 2: 가스 시뮬레이터 실행**
```bash
ros2 run gopigo3_driver gas_simulator
```

**터미널 3: Foxglove 접속**
- Foxglove에서 목표 지점 설정
- 가스 시뮬레이터에서 'g' 키를 눌러 가스 감지 트리거
- Nav2가 자동으로 경로 재계획

### 3. 시뮬레이터 키보드 명령

| 키 | 동작 |
|----|------|
| `g` | 현재 위치에 가스 감지 트리거 (1회) |
| `t` | 연속 가스 감지 모드 토글 |
| `c` | 모든 가스 장애물 삭제 |
| `q` | 종료 |

## 센서 코드 연동 방법

MQ-2 센서 코드에서 아래 토픽을 publish하면 됩니다.

### 옵션 1: Float32 농도 값 (권장)

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class MQ2SensorNode(Node):
    def __init__(self):
        super().__init__('mq2_sensor')
        self.publisher = self.create_publisher(Float32, '/gas_detection', 10)

    def publish_reading(self, concentration_ppm: float):
        msg = Float32()
        msg.data = concentration_ppm  # 예: 350.0
        self.publisher.publish(msg)
```

### 옵션 2: Bool 감지 플래그 (간단)

```python
from std_msgs.msg import Bool

# use_bool_topic 파라미터를 true로 설정해야 함
self.publisher = self.create_publisher(Bool, '/gas_detected', 10)

msg = Bool()
msg.data = True  # 가스 감지됨
self.publisher.publish(msg)
```

## 파라미터 설정

### Gas Obstacle Node 파라미터

| 파라미터 | 기본값 | 설명 |
|---------|--------|------|
| `gas_threshold` | 300.0 | 위험 판단 농도 (ppm) |
| `obstacle_radius` | 0.5 | 가상 장애물 반경 (m) |
| `inflation_radius` | 0.3 | 추가 안전 거리 (m) |
| `zone_timeout` | 300.0 | 장애물 유지 시간 (초) |
| `use_bool_topic` | false | Bool 토픽 사용 여부 |

### 파라미터 변경 방법

**방법 1: Launch 파일에서 수정**

`navigation.launch.py` 에서:
```python
gas_obstacle = Node(
    package='gopigo3_driver',
    executable='gas_obstacle',
    parameters=[{
        'gas_threshold': 200.0,  # MQ-2 센서에 맞게 조정
        'obstacle_radius': 0.8,
        ...
    }]
)
```

**방법 2: 커맨드라인에서 수정**
```bash
ros2 run gopigo3_driver gas_obstacle --ros-args \
    -p gas_threshold:=200.0 \
    -p obstacle_radius:=0.8
```

## 토픽 및 서비스

### 토픽

| 토픽 | 타입 | 방향 | 설명 |
|------|------|------|------|
| `/gas_detection` | Float32 | 입력 | 가스 농도 (ppm) |
| `/gas_detected` | Bool | 입력 | 가스 감지 플래그 |
| `/gas_obstacles` | OccupancyGrid | 출력 | 가상 장애물 맵 |
| `/gas_zones` | MarkerArray | 출력 | Foxglove 시각화 |
| `/gas_alert` | Bool | 출력 | 가스 경고 알림 |

### 서비스

| 서비스 | 타입 | 설명 |
|--------|------|------|
| `/clear_gas_obstacles` | Empty | 모든 가스 장애물 삭제 |

## Foxglove에서 시각화

1. **가스 영역 시각화**
   - Topic: `/gas_zones`
   - 3D 패널에서 MarkerArray로 표시
   - 노란색~빨간색 실린더로 가스 농도 표시

2. **Costmap 확인**
   - Topic: `/global_costmap/costmap`
   - 가스 영역이 장애물(빨간색)로 표시됨

## 테스트 시나리오

### 시나리오 1: 기본 회피 테스트

1. 네비게이션 시작
2. Foxglove에서 목표 지점 설정
3. 로봇이 이동 중일 때 가스 시뮬레이터에서 'g' 키
4. 로봇이 현재 위치를 피해 새 경로로 이동하는지 확인

### 시나리오 2: 다중 가스 영역 테스트

1. 여러 위치에서 'g' 키를 눌러 다중 가스 영역 생성
2. 로봇이 모든 영역을 피해 경로를 찾는지 확인

### 시나리오 3: 실제 센서 연동

1. MQ-2 센서 노드가 `/gas_detection` 토픽에 publish
2. 실제 가스 감지 시 자동 회피 확인

## 문제 해결

### 가스 장애물이 costmap에 표시되지 않음

```bash
# 토픽이 publish되는지 확인
ros2 topic echo /gas_obstacles

# gas_obstacle 노드 상태 확인
ros2 node info /gas_obstacle
```

### TF 오류 발생

```bash
# TF 트리 확인
ros2 run tf2_tools view_frames

# base_footprint → map 변환 가능한지 확인
ros2 run tf2_ros tf2_echo map base_footprint
```

### 경로 재계획이 안됨

Nav2의 costmap이 gas_layer를 정상적으로 구독하는지 확인:
```bash
ros2 param get /global_costmap/global_costmap plugins
# ["static_layer", "obstacle_layer", "gas_layer", "inflation_layer"] 출력되어야 함
```

## MQ-2 센서 연동 예시 (I2C)

MQ-2 센서 코드에 추가할 내용:

```python
#!/usr/bin/env python3
"""MQ-2 Gas Sensor ROS2 Node"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
# I2C 라이브러리 (예: smbus2)
import smbus2

class MQ2SensorNode(Node):
    def __init__(self):
        super().__init__('mq2_sensor')

        # ROS2 Publisher
        self.gas_pub = self.create_publisher(Float32, '/gas_detection', 10)

        # I2C 설정 (주소는 실제 센서에 맞게 수정)
        self.bus = smbus2.SMBus(1)
        self.sensor_addr = 0x48  # 예시 주소

        # 10Hz로 센서 읽기
        self.timer = self.create_timer(0.1, self.read_sensor)

    def read_sensor(self):
        try:
            # I2C에서 가스 농도 읽기 (실제 프로토콜에 맞게 수정)
            data = self.bus.read_i2c_block_data(self.sensor_addr, 0x00, 2)
            concentration = (data[0] << 8) | data[1]

            # ROS2 토픽으로 publish
            msg = Float32()
            msg.data = float(concentration)
            self.gas_pub.publish(msg)

        except Exception as e:
            self.get_logger().error(f'Sensor read error: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = MQ2SensorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

## 다음 단계 (향후 구현)

- [ ] 탈출구 위치 미리 저장
- [ ] 다중 탈출구 중 최적 경로 선택
- [ ] 가스 확산 모델링 (시간에 따른 장애물 크기 변화)
- [ ] 비상 모드 자동 진입 (알람 연동)
