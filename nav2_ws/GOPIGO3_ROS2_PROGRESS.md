# GoPiGo3 ROS2 Setup Progress

**날짜**: 2025-12-07

## 완료된 작업

### 1. 하드웨어 설정
- ✅ I2C 활성화 (`/boot/firmware/config.txt`)
- ✅ SPI 활성화
- ✅ 사용자 권한 설정 (i2c, spi, gpio, dialout 그룹)
- ✅ GoPiGo3 보드 감지 확인 (I2C 주소 0x08)

### 2. 소프트웨어 설치
- ✅ GoPiGo3 Python 라이브러리 설치 (`~/GoPiGo3/`)
- ✅ DI_Sensors 라이브러리 설치
- ✅ pigpio 소스 컴파일 및 설치
- ✅ spidev Python 라이브러리
- ✅ ROS2 Humble (이미 설치됨)
- ✅ xacro 패키지

### 3. ROS2 gopigo3_driver 패키지 생성
**위치**: `~/nav2_ws/src/gopigo3_driver/`

**구조**:
```
gopigo3_driver/
├── gopigo3_driver/
│   ├── __init__.py
│   ├── gopigo3.py (GoPiGo3 라이브러리 복사본)
│   └── gopigo3_driver_node.py (ROS2 드라이버 노드)
├── launch/
│   └── gopigo3_bringup.launch.py
├── urdf/
│   └── gopigo3.urdf.xacro
├── setup.py
└── package.xml
```

**기능**:
- `/cmd_vel` 토픽 구독하여 모터 제어
- `/odom` 토픽 발행 (오도메트리)
- `/joint_states` 발행
- `odom` → `base_footprint` TF 발행
- robot_state_publisher로 URDF 시각화

### 4. 로봇 모델 (URDF)
- ✅ base_footprint, base_link
- ✅ 왼쪽/오른쪽 휠 (continuous joint)
- ✅ 앞/뒤 캐스터 휠
- ✅ LiDAR 마운트 (laser_frame)

### 5. 빌드
```bash
cd ~/nav2_ws
source /opt/ros/humble/setup.zsh
colcon build --packages-select gopigo3_driver --symlink-install
source install/setup.zsh
```

## 현재 상황

### 통신 상태
- **GoPiGo3 보드와 통신**: ✅ 정상
  - 제조사: Dexter Industries
  - 보드: GoPiGo3
  - 하드웨어 버전: 3.x.x
  - 펌웨어 버전: 1.0.0
  - 배터리 전압: 10.4V

- **ROS2 드라이버**: ✅ 정상 작동
  - `/cmd_vel` 토픽 수신 정상
  - 모터 속도 계산 정상 (예: `dps_left=868.12, dps_right=868.12`)

### 문제: 모터가 움직이지 않음 ❌

**증상**:
- PWM/DPS 명령을 보내도 모터가 전혀 회전하지 않음
- 엔코더 값이 변하지 않음 (항상 Left: -11, Right: 6)

**테스트 결과**:
```python
# DPS로 제어
GPG.set_motor_dps(GPG.MOTOR_LEFT, 300)
GPG.set_motor_dps(GPG.MOTOR_RIGHT, 300)
# 결과: 엔코더 변화 없음

# PWM으로 제어
GPG.set_motor_power(GPG.MOTOR_LEFT, 50)  # 50% 파워
GPG.set_motor_power(GPG.MOTOR_RIGHT, 50)
# 결과: 엔코더 변화 없음
```

**가능한 원인**:
1. 모터 케이블이 GoPiGo3 보드의 MA/MB 포트에 제대로 연결되지 않음
2. 모터 드라이버 칩 손상
3. 배터리→모터 전원 공급 회로 문제
4. 펌웨어 버전 문제 (1.0.0은 초기 버전)

## 다음 단계

### 1. 하드웨어 점검 (최우선)
- [ ] GoPiGo3 보드의 **MA** (왼쪽 모터)와 **MB** (오른쪽 모터) 포트에 모터 케이블이 확실히 연결되어 있는지 확인
- [ ] 케이블을 뽑았다가 다시 단단히 끼우기
- [ ] GoPiGo OS 사용 시 모터가 작동했었는지 확인

### 2. 펌웨어 업데이트
```bash
cd ~/GoPiGo3/Firmware
./gopigo3_flash_firmware.sh
```

### 3. 모터 직접 테스트
- 모터 케이블을 GoPiGo3 보드에서 분리
- 배터리 +/- 단자를 모터 와이어에 직접 연결
- 모터가 회전하는지 확인 → 모터 자체 정상 여부 판단

## ROS2 드라이버 실행 방법

### 시스템 시작 시
```bash
# 1. pigpio 데몬 시작 (필수!)
sudo pigpiod

# 2. ROS2 환경 설정
source ~/nav2_ws/install/setup.zsh

# 3. GoPiGo3 드라이버 실행
ros2 launch gopigo3_driver gopigo3_bringup.launch.py
```

### 새 터미널에서 텔레옵
```bash
source ~/nav2_ws/install/setup.zsh
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**키 조작**:
- `i`: 전진
- `k`: 정지
- `,`: 후진
- `j`: 좌회전
- `l`: 우회전

## 주요 ROS2 토픽

```bash
# /cmd_vel 확인
ros2 topic echo /cmd_vel

# /odom 확인
ros2 topic echo /odom

# 토픽 목록
ros2 topic list
```

## 로봇 사양

- **휠 간격 (wheel_base)**: 0.117m
- **휠 지름 (wheel_diameter)**: 0.066m
- **엔코더**: 360 degrees per rotation
- **발행 주기**: 20Hz

## 참고 파일

- **드라이버 노드**: [gopigo3_driver_node.py](src/gopigo3_driver/gopigo3_driver/gopigo3_driver_node.py)
- **Launch 파일**: [gopigo3_bringup.launch.py](src/gopigo3_driver/launch/gopigo3_bringup.launch.py)
- **URDF**: [gopigo3.urdf.xacro](src/gopigo3_driver/urdf/gopigo3.urdf.xacro)
- **설정**: [setup.py](src/gopigo3_driver/setup.py)

## 문제 해결 팁

### pigpiod 실행 확인
```bash
ps aux | grep pigpiod
```

### GoPiGo3 보드 통신 테스트
```bash
sudo python3
```
```python
import sys
sys.path.insert(0, '/home/ubuntu/GoPiGo3/Software/Python')
import gopigo3

GPG = gopigo3.GoPiGo3()
print("Manufacturer:", GPG.get_manufacturer())
print("Battery voltage:", GPG.get_voltage_battery())
```

### I2C 디바이스 확인
```bash
sudo i2cdetect -y 1
# 0x08에 GoPiGo3 보드가 보여야 함
```

---

## 2025-12-12 작업 내역

### 수동 맵핑 및 Nav2 네비게이션 시스템 구현

#### 1. 목표
- 수동 맵핑 (teleop 사용)으로 맵 생성
- 맵 저장 시 타임스탬프 파일명 (YYYYMMDD_HHMMSS)
- 시작/종료 위치 JSON 저장
- Nav2로 네비게이션
- Foxglove에서 클릭으로 목표점 지정

#### 2. 새로 생성된 노드들

##### map_saver_node.py
- 맵핑 시작 시 시작 위치 기록
- 맵 저장 시 종료 위치 기록
- slam_toolbox `serialize_map` 서비스 호출
- 타임스탬프 기반 파일명 자동 생성
- JSON으로 위치 정보 저장

**사용법:**
```bash
# 맵핑 시작과 함께 실행 (수동 저장 모드)
ros2 run gopigo3_driver map_saver --ros-args -p auto_save:=false

# 맵핑 완료 후 저장
ros2 service call /save_map std_srvs/srv/Trigger
```

**JSON 출력 형식:**
```json
{
  "map_file": "/home/ubuntu/maps/20251212_172520",
  "timestamp": "2025-12-12T17:25:20",
  "exploration_time_seconds": 300,
  "start_position": {
    "map_frame": {"x": 0.0, "y": 0.0, "yaw": 0.0}
  },
  "end_position": {
    "map_frame": {"x": 5.2, "y": -1.3, "yaw": 1.57}
  }
}
```

##### go_to_start_node.py
- JSON에서 end_position 로드하여 initial pose 설정
- JSON에서 start_position 로드하여 목표 설정
- Nav2 NavigateToPose 액션으로 자동 이동

**사용법:**
```bash
ros2 run gopigo3_driver go_to_start --ros-args -p map_file:=/home/ubuntu/maps/20251212_172520
```

#### 3. 발생한 문제들 및 해결

##### 문제 1: TF 타임스탬프 불일치
**증상:**
```
Extrapolation Error: Requested time 1765578664 but earliest data is at time 1765578665
Transform data too old when converting from map to odom
Message Filter dropping message: frame 'laser_frame' - timestamp earlier than transform cache
```

**원인:**
- Foxglove에서 보낸 goal의 타임스탬프가 TF 캐시보다 과거
- transform_tolerance 값이 너무 작음 (0.1~0.2초)

**해결:** `nav2_params.yaml`과 `slam_toolbox_localization.yaml` 수정
| 파라미터 | 변경 전 | 변경 후 |
|---------|--------|--------|
| bt_navigator/transform_tolerance | 1.0 | 2.0 |
| FollowPath/transform_tolerance | 1.0 | 2.0 |
| behavior_server/transform_tolerance | 0.1 | 2.0 |
| collision_monitor/transform_tolerance | 0.2 | 2.0 |
| slam_toolbox/transform_timeout | 0.2 | 1.0 |
| slam_toolbox/tf_buffer_duration | 30.0 | 60.0 |

##### 문제 2: 라즈베리 파이 성능 부족
**증상:**
```
[ekf_node] Failed to meet update rate! Took 0.15s
Timed out while waiting for action server to acknowledge goal request
```

**원인:**
- EKF, Nav2 주파수가 Pi4 성능에 비해 너무 높음
- 모든 노드가 과부하로 타임아웃 발생

**해결:** 주파수 및 타임아웃 조정

**ekf.yaml:**
| 파라미터 | 변경 전 | 변경 후 |
|---------|--------|--------|
| frequency | 15.0 Hz | 10.0 Hz |

**nav2_params.yaml:**
| 파라미터 | 변경 전 | 변경 후 |
|---------|--------|--------|
| controller_frequency | 3.0 Hz | 2.0 Hz |
| local_costmap update | 2.0 Hz | 1.0 Hz |
| local_costmap publish | 1.0 Hz | 0.5 Hz |
| global_costmap update | 1.0 Hz | 0.5 Hz |
| planner_frequency | 5.0 Hz | 1.0 Hz |
| behavior_server cycle | 10.0 Hz | 5.0 Hz |
| velocity_smoother | 20.0 Hz | 10.0 Hz |
| bt_loop_duration | 100 ms | 200 ms |
| default_server_timeout | 60 s | 120 s |
| wait_for_service_timeout | 5000 ms | 10000 ms |

**slam_toolbox_localization.yaml:**
| 파라미터 | 변경 전 | 변경 후 |
|---------|--------|--------|
| map_update_interval | 5.0 s | 10.0 s |
| minimum_time_interval | 0.5 s | 1.0 s |

#### 4. setup.py 업데이트
새 entry points 추가:
```python
'map_saver = gopigo3_driver.map_saver_node:main',
'go_to_start = gopigo3_driver.go_to_start_node:main',
```

#### 5. 워크플로우

##### 수동 맵핑
```bash
# 터미널 1: SLAM 시작
ros2 launch gopigo3_driver slam.launch.py

# 터미널 2: 맵 세이버 시작
ros2 run gopigo3_driver map_saver --ros-args -p auto_save:=false

# 터미널 3: 텔레옵
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# 맵핑 완료 후 저장
ros2 service call /save_map std_srvs/srv/Trigger
```

##### Nav2 네비게이션
```bash
# 저장된 맵으로 네비게이션 시작
ros2 launch gopigo3_driver navigation.launch.py map:=/home/ubuntu/maps/20251212_172520

# (선택) 시작 위치로 자동 이동
ros2 run gopigo3_driver go_to_start --ros-args -p map_file:=/home/ubuntu/maps/20251212_172520
```

##### Foxglove 목표 클릭
- navigation.launch.py에 goal_relay 노드가 포함됨
- Foxglove에서 맵 클릭 시 `/move_base_simple/goal` → goal_relay → `/goal_pose` → Nav2

#### 6. 현재 상태
- 맵 저장: ✅ 성공 (`/home/ubuntu/maps/20251212_172520`)
- Nav2 네비게이션: 🔄 테스트 중 (성능 최적화 적용 필요)
- Foxglove 목표 클릭: 🔄 테스트 중

#### 7. 다음 단계
1. 로봇에서 빌드:
   ```bash
   cd ~/nav2_ws && colcon build --packages-select gopigo3_driver
   source install/setup.bash
   ```

2. 수정된 설정으로 Nav2 재시작:
   ```bash
   ros2 launch gopigo3_driver navigation.launch.py map:=/home/ubuntu/maps/20251212_172520
   ```

3. Foxglove에서 목표 클릭 테스트
