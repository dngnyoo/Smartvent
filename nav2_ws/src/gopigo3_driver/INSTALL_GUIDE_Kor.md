# GoPiGo3 Nav2 Navigation Package 설치 가이드

이 가이드는 GoPiGo3 로봇에서 수동 SLAM 맵핑 및 Nav2 자율주행 기능을 설치하고 사용하는 방법을 설명합니다.

---

## 목차
1. [사전 요구사항](#1-사전-요구사항)
2. [패키지 다운로드](#2-패키지-다운로드)
3. [의존성 설치](#3-의존성-설치)
4. [빌드 및 설정](#4-빌드-및-설정)
5. [설치 테스트](#5-설치-테스트)
6. [기존 기능과 통합](#6-기존-기능과-통합)
7. [사용 방법](#7-사용-방법)
8. [토픽 및 TF 인터페이스](#8-토픽-및-tf-인터페이스)
9. [문제 해결](#9-문제-해결)
10. [Foxglove 시각화 (Windows/Mac/Linux)](#10-foxglove-시각화-windowsmaclinux)
11. [설치 완료 체크리스트](#11-설치-완료-체크리스트)

---

## 1. 사전 요구사항

### 하드웨어
- GoPiGo3 로봇
- Raspberry Pi 4 (4GB 이상 권장)
- RPLidar A1M8 (또는 호환 LiDAR)
- BNO055 IMU (선택사항)

### 소프트웨어
- Ubuntu 22.04 (Raspberry Pi용)
- ROS2 Humble
- Python 3.10+

---

## 2. 패키지 다운로드

### 방법 A: Git Clone (권장)
```bash
cd ~
git clone https://github.com/WonbumSohn/Robotics_SmartVent.git
```

### 방법 B: 특정 폴더만 복사
필요한 폴더:
- `nav2_ws/src/gopigo3_driver/` - ROS2 패키지
- `maps/` - 샘플 맵 (선택사항)

---

## 3. 의존성 설치

### ROS2 패키지 설치
```bash
sudo apt update
sudo apt install -y \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-slam-toolbox \
  ros-humble-robot-localization \
  ros-humble-rplidar-ros \
  ros-humble-foxglove-bridge \
  ros-humble-tf2-ros \
  ros-humble-tf2-tools \
  ros-humble-tf2-geometry-msgs \
  ros-humble-xacro \
  ros-humble-joint-state-publisher \
  ros-humble-robot-state-publisher \
  ros-humble-teleop-twist-keyboard
```

### Python 의존성
```bash
pip3 install spidev
```

### GoPiGo3 라이브러리 (없는 경우)
```bash
cd ~
git clone https://github.com/DexterInd/GoPiGo3.git
cd GoPiGo3/Software/Python
sudo python3 setup.py install
```

### pigpio 데몬 설정
```bash
# pigpio가 없는 경우 설치
sudo apt install -y pigpio

# 부팅 시 자동 시작 설정
sudo systemctl enable pigpiod
sudo systemctl start pigpiod
```

### m-explore-ros2 설치 (자율 탐색용, 선택사항)
```bash
cd ~/nav2_ws/src
git clone -b humble https://github.com/robo-friends/m-explore-ros2.git
```

### udev 규칙 설정 (LiDAR 장치)
```bash
# RPLidar udev 규칙 생성
echo 'KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0666", GROUP:="dialout", SYMLINK+="rplidar"' | sudo tee /etc/udev/rules.d/99-rplidar.rules

# udev 규칙 리로드
sudo udevadm control --reload-rules
sudo udevadm trigger

# 필요한 그룹에 사용자 추가
sudo usermod -a -G dialout $USER
sudo usermod -a -G spi $USER
sudo usermod -a -G gpio $USER
```

> **참고:** 그룹 변경 후 재로그인 또는 재부팅이 필요합니다.

---

## 4. 빌드 및 설정

### 4.1 기존 워크스페이스에 통합하는 경우

기존에 `~/nav2_ws`가 있다면:
```bash
# gopigo3_driver 패키지만 복사
cp -r ~/Robotics_SmartVent/nav2_ws/src/gopigo3_driver ~/nav2_ws/src/

# 빌드
cd ~/nav2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select gopigo3_driver --symlink-install
source install/setup.bash
```

### 4.2 새로 설치하는 경우

```bash
# 워크스페이스 생성
mkdir -p ~/nav2_ws/src
cd ~/nav2_ws/src

# 패키지 복사
cp -r ~/Robotics_SmartVent/nav2_ws/src/gopigo3_driver .

# 의존성 설치
cd ~/nav2_ws
rosdep install --from-paths src --ignore-src -r -y

# 빌드
source /opt/ros/humble/setup.bash
colcon build --packages-select gopigo3_driver --symlink-install
```

### 4.3 환경 설정

사용하는 쉘에 따라 설정:

```bash
# 현재 쉘 확인
echo $SHELL
# /bin/zsh 또는 /usr/bin/zsh → zsh 사용
# /bin/bash 또는 /usr/bin/bash → bash 사용
```

**zsh 사용 시:**
```bash
echo "source /opt/ros/humble/setup.zsh" >> ~/.zshrc
echo "source ~/nav2_ws/install/setup.zsh" >> ~/.zshrc
source ~/.zshrc
```

**bash 사용 시:**
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source ~/nav2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 4.4 맵 폴더 설정

```bash
# 맵 저장 폴더 생성
mkdir -p ~/maps

# 샘플 맵 복사 (선택사항)
cp ~/Robotics_SmartVent/maps/* ~/maps/
```

### 4.5 GoPiGo3 캘리브레이션 파일 확인

```bash
# 캘리브레이션 파일이 있는지 확인
ls -la /home/ubuntu/Dexter/gpg3_config.json

# 없으면 기본 설정으로 생성
mkdir -p /home/ubuntu/Dexter
cat > /home/ubuntu/Dexter/gpg3_config.json << 'EOF'
{
  "wheel-diameter": 66.5,
  "wheel-base-width": 117,
  "ticks": 6,
  "motor_gear_ratio": 120
}
EOF
```

---

## 5. 설치 테스트

빌드 완료 후, 다음 테스트를 순서대로 진행하여 설치가 정상인지 확인합니다.

### 5.1 시스템 요구사항 확인

```bash
# Ubuntu 버전 확인 (22.04 필요)
lsb_release -a

# ROS2 Humble 설치 확인
ros2 --version

# Python3 확인
python3 --version
```

### 5.2 GoPiGo3 하드웨어 통신 확인

```bash
# SPI 장치 확인
ls /dev/spidev*
# 예상 출력: /dev/spidev0.0  /dev/spidev0.1

# GoPiGo3 Python 테스트
python3 -c "import gopigo3; gpg = gopigo3.GoPiGo3(); print('Manufacturer:', gpg.get_manufacturer()); print('Board:', gpg.get_board()); print('Voltage:', gpg.get_voltage_battery())"
# 예상 출력: Manufacturer: Dexter Industries, Board: GoPiGo3 등
```

### 5.3 LiDAR 연결 확인

```bash
# LiDAR 장치 확인
ls -la /dev/rplidar
# 또는
ls -la /dev/ttyUSB*
```

### 5.4 ROS2 패키지 확인

```bash
# 패키지 목록에서 gopigo3_driver 확인
ros2 pkg list | grep gopigo3
# 예상 출력: gopigo3_driver

# 실행 가능한 노드 확인
ros2 pkg executables gopigo3_driver
```

### 5.5 기본 Bringup 테스트

```bash
# 터미널 1: 로봇 드라이버 실행
ros2 launch gopigo3_driver gopigo3_bringup.launch.py

# 터미널 2: 토픽 확인
ros2 topic list
# 확인해야 할 토픽들:
# /odom, /odometry/filtered, /imu/data, /scan, /tf, /tf_static

# 오도메트리 데이터 확인
ros2 topic echo /odometry/filtered --once
```

### 5.6 모터 동작 테스트

```bash
# Bringup 실행 상태에서 터미널 2에서 실행
# 앞으로 이동 명령
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}, angular: {z: 0.0}}" --once

# 로봇이 약간 앞으로 움직여야 함
# 정지 명령
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}" --once
```

### 5.7 TF 트리 확인

```bash
# Bringup 실행 상태에서
ros2 run tf2_tools view_frames

# 또는 실시간 확인
ros2 run tf2_ros tf2_echo odom base_footprint
```

---

## 6. 기존 기능과 통합

### 6.1 TF 프레임 구조

이 패키지는 다음 TF 트리를 사용합니다:
```
map
 └── odom (EKF 또는 slam_toolbox가 발행)
      └── base_footprint
           └── base_link
                ├── left_wheel
                ├── right_wheel
                ├── laser_frame
                └── imu_link
```

### 6.2 충돌 방지 체크리스트

다른 노드와 함께 사용할 때 확인하세요:

| 항목 | 이 패키지 | 확인 사항 |
|-----|----------|----------|
| `odom → base_footprint` TF | EKF가 발행 | 다른 노드에서 중복 발행 금지 |
| `/cmd_vel` 토픽 | Nav2가 발행 | 텔레옵과 동시 사용 시 충돌 주의 |
| `/scan` 토픽 | LiDAR 드라이버 필요 | rplidar_ros 등 별도 실행 필요 |
| `/imu/data` 토픽 | BNO055 노드 필요 | IMU 없으면 EKF 설정 수정 필요 |

### 6.3 다른 노드와 함께 실행

**예시: LiDAR와 함께 실행**
```bash
# 터미널 1: LiDAR
ros2 launch rplidar_ros rplidar_a1_launch.py

# 터미널 2: Navigation
ros2 launch gopigo3_driver navigation.launch.py map:=/home/ubuntu/maps/20251212_172520
```

### 6.4 Launch 파일 통합

기존 launch 파일에 통합하려면:
```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    gopigo3_driver_dir = get_package_share_directory('gopigo3_driver')

    # Nav2 Navigation 포함
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gopigo3_driver_dir, 'launch', 'navigation.launch.py')
        ),
        launch_arguments={'map': '/home/ubuntu/maps/your_map'}.items()
    )

    return LaunchDescription([
        # 기존 노드들...
        navigation_launch,
    ])
```

---

## 7. 사용 방법

### 7.1 수동 맵핑 (새 맵 생성)

```bash
# 터미널 1: SLAM 시작
ros2 launch gopigo3_driver slam.launch.py

# 터미널 2: 맵 세이버 (저장 준비)
ros2 run gopigo3_driver map_saver --ros-args -p auto_save:=false

# 터미널 3: 텔레옵으로 로봇 조종
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**키보드 조작:**
- `i`: 전진
- `k`: 정지
- `,`: 후진
- `j`: 좌회전
- `l`: 우회전

**맵핑 완료 후 저장:**
```bash
ros2 service call /save_map std_srvs/srv/Trigger
```

맵 파일 저장 위치: `~/maps/YYYYMMDD_HHMMSS.data`, `.posegraph`, `_positions.json`

### 7.2 저장된 맵으로 자율주행

```bash
# Nav2 + 로컬라이제이션 시작
ros2 launch gopigo3_driver navigation.launch.py \
  map:=/home/ubuntu/maps/20251212_172520
```

### 7.3 목표 지점 설정 방법

**방법 1: Foxglove에서 클릭**
1. Foxglove Studio 실행
2. 맵 패널에서 원하는 위치 클릭
3. `/move_base_simple/goal` → `goal_relay` → `/goal_pose` → Nav2

**방법 2: 커맨드라인**
```bash
ros2 topic pub /goal_pose geometry_msgs/PoseStamped "{
  header: {frame_id: 'map'},
  pose: {
    position: {x: 1.0, y: 0.5, z: 0.0},
    orientation: {w: 1.0}
  }
}"
```

**방법 3: 시작 위치로 자동 복귀**
```bash
ros2 run gopigo3_driver go_to_start \
  --ros-args -p map_file:=/home/ubuntu/maps/20251212_172520
```

---

## 8. 토픽 및 TF 인터페이스

### 주요 토픽

| 토픽 | 타입 | 방향 | 설명 |
|-----|------|-----|------|
| `/cmd_vel` | `Twist` | 입력 | 로봇 속도 명령 |
| `/odom` | `Odometry` | 출력 | 휠 오도메트리 |
| `/odometry/filtered` | `Odometry` | 출력 | EKF 융합 오도메트리 |
| `/scan` | `LaserScan` | 입력 | LiDAR 데이터 |
| `/imu/data` | `Imu` | 입력 | IMU 데이터 |
| `/goal_pose` | `PoseStamped` | 입력 | Nav2 목표 위치 |
| `/move_base_simple/goal` | `PoseStamped` | 입력 | Foxglove 목표 (릴레이됨) |
| `/map` | `OccupancyGrid` | 출력 | 맵 데이터 |

### 제공되는 노드

| 노드 | 실행 명령 | 설명 |
|-----|----------|------|
| `gopigo3_driver` | `ros2 run gopigo3_driver gopigo3_driver` | 모터/오도메트리 드라이버 |
| `bno055_imu` | `ros2 run gopigo3_driver bno055_imu` | IMU 드라이버 |
| `map_saver` | `ros2 run gopigo3_driver map_saver` | 맵 저장 노드 |
| `go_to_start` | `ros2 run gopigo3_driver go_to_start` | 시작점 복귀 노드 |
| `goal_relay` | `ros2 run gopigo3_driver goal_relay` | Foxglove 목표 릴레이 |

---

## 9. 문제 해결

### 9.1 "pigpiod not running" 오류
```bash
sudo pigpiod
# 또는
sudo systemctl start pigpiod
```

### 9.2 SPI 통신 오류

```
[ERROR] No SPI response
```

**해결책:**
```bash
# SPI 활성화 확인
sudo raspi-config
# Interface Options > SPI > Enable

# pigpiod 시작
sudo pigpiod

# 재부팅
sudo reboot
```

### 9.3 GoPiGo3 보드 통신 실패
```bash
# I2C 확인 (0x08에 응답해야 함)
sudo i2cdetect -y 1

# Python에서 테스트
sudo python3
>>> import sys
>>> sys.path.insert(0, '/home/ubuntu/GoPiGo3/Software/Python')
>>> import gopigo3
>>> GPG = gopigo3.GoPiGo3()
>>> print(GPG.get_manufacturer())
```

### 9.4 LiDAR 권한 오류

```
[ERROR] Cannot open serial port
```

**해결책:**
```bash
sudo chmod 666 /dev/ttyUSB0
# 또는 udev 규칙 다시 적용
sudo udevadm control --reload-rules
sudo udevadm trigger
```

### 9.5 TF 타임스탬프 오류
```
Extrapolation Error: Requested time X but earliest data is at time Y
```
**해결:** 이미 config 파일에 `transform_tolerance: 2.0`으로 설정되어 있습니다.
Foxglove 사용 시 `goal_relay` 노드가 타임스탬프를 자동 수정합니다.

### 9.6 Nav2 서버 타임아웃
```
Timed out while waiting for action server
```
**해결:** Raspberry Pi 4 성능에 맞게 이미 주파수가 최적화되어 있습니다.
그래도 문제가 있으면:
```yaml
# nav2_params.yaml
bt_navigator:
  default_server_timeout: 180  # 120 → 180
  wait_for_service_timeout: 15000  # 10000 → 15000
```

### 9.7 EKF 업데이트 실패
```
Failed to meet update rate!
```
**해결:** `config/ekf.yaml`에서 frequency 낮추기:
```yaml
frequency: 8.0  # 10.0 → 8.0
```

### 9.8 패키지를 찾을 수 없음

```
Package 'gopigo3_driver' not found
```

**해결책:**
```bash
# zsh 사용시
source ~/nav2_ws/install/setup.zsh

# bash 사용시
source ~/nav2_ws/install/setup.bash

# 또는 다시 빌드
cd ~/nav2_ws && colcon build
```

### 9.9 토픽/TF 확인 명령어
```bash
# 토픽 목록
ros2 topic list

# 오도메트리 확인
ros2 topic echo /odom --once

# TF 트리 확인
ros2 run tf2_ros tf2_echo map base_footprint

# TF 트리 시각화
ros2 run tf2_tools view_frames
```

---

## 10. Foxglove 시각화 (Windows/Mac/Linux)

Foxglove를 사용하면 별도의 PC에서 로봇의 맵, 위치, LiDAR 스캔 등을 실시간으로 볼 수 있습니다.

### 10.1 Foxglove 설치

**방법 1: 웹 브라우저 (설치 불필요)**
- https://app.foxglove.dev 접속
- Chrome, Edge, Safari 등 지원

**방법 2: 데스크톱 앱**
- https://foxglove.dev/download
- Windows, Mac, Linux용 설치 파일 다운로드

### 10.2 로봇 연결

1. 로봇(라즈베리파이)에서 bringup 실행:
   ```bash
   ros2 launch gopigo3_driver gopigo3_bringup.launch.py
   ```
   - `foxglove_bridge`가 자동으로 포트 8765에서 시작됨

2. Foxglove에서 연결:
   - **Open connection** 클릭
   - **Foxglove WebSocket** 선택
   - 주소 입력: `ws://[라즈베리파이_IP]:8765`
   - 예: `ws://192.168.1.100:8765`

### 10.3 라즈베리파이 IP 확인

라즈베리파이에서:
```bash
hostname -I
```

### 10.4 네트워크 요구사항

| 장치 | 요구사항 |
|-----|---------|
| 라즈베리파이 (로봇) | WiFi 연결 |
| PC (Foxglove) | 같은 WiFi 네트워크 |

### 10.5 Foxglove 패널 추가

연결 후 다음 패널을 추가하여 시각화:

| 패널 | 토픽 | 설명 |
|-----|------|------|
| Map | `/map` | 2D 맵 |
| 3D | `/scan`, TF | LiDAR 스캔 + 로봇 위치 |
| Image | `/camera/image_raw` | 카메라 (있는 경우) |
| Raw Messages | 원하는 토픽 | 토픽 데이터 확인 |

### 10.6 Foxglove에서 목표 지점 설정

1. 3D 또는 Map 패널에서 **Publish** 도구 선택
2. 토픽: `/move_base_simple/goal` 또는 `/goal_pose`
3. 맵에서 원하는 위치 클릭
4. `goal_relay` 노드가 Nav2로 전달

---

## 11. 설치 완료 체크리스트

설치 완료 후 다음 항목을 모두 확인하세요:

- [ ] Ubuntu 22.04 및 ROS2 Humble 설치 확인
- [ ] GoPiGo3 Python 테스트 통과 (`python3 -c "import gopigo3; ..."`)
- [ ] LiDAR 장치 인식됨 (`/dev/rplidar` 또는 `/dev/ttyUSB*`)
- [ ] `ros2 pkg list | grep gopigo3` 출력 확인
- [ ] `ros2 launch gopigo3_driver gopigo3_bringup.launch.py` 에러 없이 실행
- [ ] `/odom`, `/odometry/filtered`, `/imu/data`, `/scan` 토픽 발행 확인
- [ ] `cmd_vel`로 로봇 움직임 테스트 완료
- [ ] TF 트리 정상 (`ros2 run tf2_tools view_frames`)
- [ ] (선택) Foxglove 연결 테스트

---

## 라이선스

MIT License

## 문의

GitHub Issues: https://github.com/WonbumSohn/Robotics_SmartVent/issues
