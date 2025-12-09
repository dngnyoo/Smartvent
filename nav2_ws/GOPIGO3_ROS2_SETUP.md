# GoPiGo3 ROS2 설정 가이드

## 1. GoPiGo3 Python 라이브러리 설치

GoPiGo OS 없이 Ubuntu에서 GoPiGo3 하드웨어를 제어하려면 GoPiGo3 Python 라이브러리를 설치해야 합니다.

```bash
# GoPiGo3 저장소 클론
cd ~
git clone https://github.com/DexterInd/GoPiGo3.git

# Python 라이브러리 설치
cd ~/GoPiGo3/Software/Python
sudo python3 setup.py install

# I2C 활성화 (라즈베리파이에서)
sudo raspi-config
# Interface Options -> I2C -> Enable 선택

# 사용자를 i2c 그룹에 추가
sudo usermod -a -G i2c $USER
sudo usermod -a -G spi $USER

# 재부팅
sudo reboot
```

## 2. GoPiGo3 연결 테스트

재부팅 후 GoPiGo3 보드와 통신 확인:

```bash
cd ~/GoPiGo3/Software/Python/Examples
python3 easy_Distance_Sensor.py  # 또는 다른 예제 실행
```

## 3. ROS2 GoPiGo3 패키지 옵션

### 옵션 A: 기존 ROS2 GoPiGo3 패키지 사용

```bash
cd ~/gopigo_mount/src
git clone https://github.com/slowrunner/ROS2-GoPiGo3.git

# 또는 다른 community 패키지
git clone https://github.com/ros-gopigo/gopigo3_node.git
```

### 옵션 B: 직접 ROS2 노드 작성

GoPiGo3를 NeuronBot2처럼 사용하려면 커스텀 노드가 필요합니다.

## 4. 필요한 ROS2 노드 구조

GoPiGo3를 SLAM과 Nav2에서 사용하려면 다음 기능을 제공하는 노드가 필요합니다:

### 필수 노드:
1. **gopigo3_driver** - GoPiGo3 하드웨어 인터페이스
   - 구독: `/cmd_vel` (geometry_msgs/Twist)
   - 발행: `/odom` (nav_msgs/Odometry)
   - 발행: `/tf` (odom -> base_link 변환)
   - 발행: `/joint_states` (sensor_msgs/JointState) - 선택사항

2. **robot_state_publisher** - URDF 기반 TF 발행
   - GoPiGo3 URDF 파일 필요

### 선택사항:
- 거리 센서가 있다면: `/scan` (sensor_msgs/LaserScan) 발행
- IMU가 있다면: `/imu` (sensor_msgs/Imu) 발행

## 5. 전원 공급 방법

**반드시 GoPiGo3 보드에 전원을 연결하세요:**

```
배터리팩 → GoPiGo3 보드 → 라즈베리파이
```

라즈베리파이에 직접 전원을 연결하면:
- ❌ GoPiGo3 보드에 전원이 공급되지 않음
- ❌ 모터 제어 불가능
- ❌ 인코더 읽기 불가능

## 6. GoPiGo3와 NeuronBot2 비교

| 기능 | NeuronBot2 | GoPiGo3 (필요한 작업) |
|------|-----------|---------------------|
| 모터 제어 | neuronbot2_driver | gopigo3_driver 작성 필요 |
| Odometry | ✅ | gopigo3_driver에서 구현 |
| LiDAR | RPLiDAR 패키지 | 별도 LiDAR 필요 (RPLiDAR 등) |
| URDF | ✅ | GoPiGo3 URDF 작성 필요 |
| Gazebo 시뮬레이션 | ✅ | 선택사항 |

## 7. 다음 단계

1. GoPiGo3 Python 라이브러리 설치 및 테스트
2. ROS2 gopigo3_driver 노드 작성 또는 기존 패키지 사용
3. GoPiGo3 URDF 파일 작성
4. LiDAR 센서 추가 및 ROS2 드라이버 설치 (RPLiDAR 등)
5. SLAM 및 Nav2 테스트

## 참고 자료

- GoPiGo3 GitHub: https://github.com/DexterInd/GoPiGo3
- GoPiGo3 Python API: https://gopigo3.readthedocs.io/
- ROS2 GoPiGo3 예제: https://github.com/slowrunner/ROS2-GoPiGo3
