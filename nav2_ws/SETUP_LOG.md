# GoPiGo3 ROS2 Nav2 Setup Log

## Project Overview
- **Hardware**: GoPiGo3 + Raspberry Pi 4 + BNO055 IMU + RPLidar A1M8
- **Software**: Ubuntu 22.04 + ROS2 Humble
- **Goal**: SLAM and Nav2 autonomous navigation
- **Visualization**: Foxglove on MacBook (instead of RViz2)

---

## Step 1: GoPiGo3 Hardware Test - COMPLETED

### Key Setup
1. **pigpio**: Ubuntu 22.04 ARM64에서 소스 빌드 필요
   ```bash
   git clone https://github.com/joan2937/pigpio.git && cd pigpio && make && sudo make install
   ```

2. **GPIO 23 Power Keep-Alive** (CRITICAL!)
   - GoPiGo3 모터 작동에 필수
   - `gpg3_power.service` 설치하여 부팅 시 자동 설정

### Test Results
- Battery: **12.01V**, Motor Test: **SUCCESS**

---

## Step 2: IMU (BNO055) Test - COMPLETED

### DI_Sensors Library Fix
`~/DI_Sensors/Python/di_sensors/BNO055.py` 수정 필요:
```bash
sed -i 's/import di_i2c/import di_sensors.dexter_i2c as di_i2c/' BNO055.py
sed -i 's/di_i2c.DI_I2C/di_i2c.Dexter_I2C/' BNO055.py
cd ~/DI_Sensors/Python && sudo python3 setup.py install
```

### I2C Addresses
- **0x08**: GoPiGo3 board
- **0x28/0x29**: BNO055 IMU

---

## Step 3: LiDAR Test - COMPLETED

### Setup
```bash
# USB 권한
sudo usermod -aG dialout ubuntu
sudo chmod 666 /dev/ttyUSB0

# rplidar_ros 설치
cd ~/nav2_ws/src && git clone -b ros2 https://github.com/Slamtec/rplidar_ros.git
cd ~/nav2_ws && colcon build --packages-select rplidar_ros
```

### Test
```bash
ros2 launch rplidar_ros rplidar_a1_launch.py
```
- `/scan` 토픽 발행, 360° 스캔 정상 작동

---

## Step 4: ROS2 Integration Test - COMPLETED

### 4-1. IMU Node I2C 에러 수정

**Problem**: Software I2C에서 `[Errno 5] Input/output error` 발생

**Solution** (`bno055_imu_node.py`):
```python
import threading

class BNO055IMUNode(Node):
    I2C_READ_DELAY = 0.01  # 10ms
    MAX_RETRIES = 3

    def __init__(self):
        self.last_calib_status = None  # Must be before _init_imu
        self.i2c_lock = threading.Lock()
        self._init_imu()

    def publish_imu_data(self):
        if not self.i2c_lock.acquire(blocking=False):
            return  # Skip if busy
        try:
            # Add delays between I2C reads
            time.sleep(self.I2C_READ_DELAY)
        finally:
            self.i2c_lock.release()
```

**Key Changes**: `threading.Lock()`, 10ms 딜레이, publish_rate 50Hz → 10Hz

### 4-2. IMU 캘리브레이션 스크립트

**File**: `~/nav2_ws/src/gopigo3_driver/scripts/calibrate_imu.py`
- 시각적 진행률 바 (█░░ 1/3)
- Gyro: 정지 → Accel: 기울이기 → Mag: 8자 패턴
- 실패 시 개별 재시도 기능

### 4-3. 센서 버스 독립성 확인
- **IMU**: I2C (RPI_1SW)
- **LiDAR**: USB Serial (/dev/ttyUSB0)
- **GoPiGo3**: SPI (/dev/spidev0.1)

→ I2C 에러 없이 안정적으로 동작!

---

## Step 5: Foxglove Setup - COMPLETED

### 5-1. Foxglove Bridge 설치 (Raspberry Pi)
```bash
sudo apt install ros-humble-foxglove-bridge
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
```

### 5-2. Zsh 환경 설정
```bash
# ~/.zshrc에 추가
source /opt/ros/humble/setup.zsh
source ~/nav2_ws/install/setup.zsh
```

### 5-3. 전체 노드 실행 순서
```bash
# Terminal 1: GoPiGo3 Driver
ros2 run gopigo3_driver gopigo3_driver

# Terminal 2: IMU Node
ros2 run gopigo3_driver bno055_imu

# Terminal 3: Robot State Publisher (TF 브로드캐스트)
ros2 run robot_state_publisher robot_state_publisher \
  --ros-args -p robot_description:="$(xacro ~/nav2_ws/src/gopigo3_driver/urdf/gopigo3.urdf.xacro)"

# Terminal 4: LiDAR (frame_id 맞춤!)
ros2 launch rplidar_ros rplidar_a1_launch.py frame_id:=laser_frame

# Terminal 5: Foxglove Bridge
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
```

### 5-4. Foxglove Studio 연결 (MacBook)
1. Foxglove Studio 실행
2. **Open connection** → `ws://172.20.10.4:8765` (Raspberry Pi IP)
3. **3D Panel** 추가 → Fixed frame: `odom`, Display frame: `base_footprint`

### 5-5. TF Frame 이슈 해결
**Problem**: LiDAR 기본 frame_id `laser` vs URDF의 `laser_frame` 불일치

**Solution**: LiDAR 실행 시 `frame_id:=laser_frame` 파라미터 추가

### 시각화 결과
- `/imu/data`: IMU 데이터 실시간 스트리밍 ✓
- `/scan`: LiDAR 360° 스캔 3D 패널에 표시 ✓
- `/tf`: robot_state_publisher에서 12개 transform 브로드캐스트 ✓

---

## Key Learnings

1. **GPIO 23**: 없으면 GoPiGo3 모터 작동 안 함
2. **pigpio**: Ubuntu 22.04 ARM64에서 소스 빌드 필요
3. **DI_Sensors**: import 문 수정 필요
4. **Software I2C**: 느리고 불안정 → 딜레이 + threading lock 필수
5. **Non-blocking lock**: `lock.acquire(blocking=False)`로 데드락 방지
6. **BNO055 캘리브레이션**: 전원 끄면 초기화됨
7. **센서 버스 독립**: IMU/LiDAR/GoPiGo3 동시 사용 가능
8. **TF frame_id 일치**: LiDAR frame_id와 URDF 정의가 일치해야 함
9. **robot_state_publisher**: URDF에서 TF tree 브로드캐스트에 필수
10. **Foxglove Bridge**: ROS2 ↔ WebSocket 변환, RViz2 없이 원격 시각화 가능

---

## Commands Quick Reference

```bash
# 서비스 확인
sudo systemctl status pigpiod gpg3_power

# 하드웨어 테스트
python3 -c "import sys; sys.path.insert(0,'/home/ubuntu/GoPiGo3/Software/Python'); import gopigo3; print(f'Battery: {gopigo3.GoPiGo3().get_voltage_battery():.2f}V')"

# IMU 캘리브레이션
python3 ~/nav2_ws/src/gopigo3_driver/scripts/calibrate_imu.py

# ROS2 노드 실행 (전체)
ros2 run gopigo3_driver gopigo3_driver      # Terminal 1: GoPiGo3
ros2 run gopigo3_driver bno055_imu          # Terminal 2: IMU
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(xacro ~/nav2_ws/src/gopigo3_driver/urdf/gopigo3.urdf.xacro)"  # Terminal 3: TF
ros2 launch rplidar_ros rplidar_a1_launch.py frame_id:=laser_frame  # Terminal 4: LiDAR
ros2 launch foxglove_bridge foxglove_bridge_launch.xml  # Terminal 5: Foxglove
ros2 launch slam_toolbox online_async_launch.py # Terminal 6: SLAM (선택)

# 토픽 확인
ros2 topic list
ros2 topic echo /imu/data
```

---

*Last Updated: December 2025*
