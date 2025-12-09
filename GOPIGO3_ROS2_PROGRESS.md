# GoPiGo3 ROS2 Setup Progress

**최근 업데이트**: 2025-12-08
**시작일**: 2025-12-07

## 현재 상태

### 완료 ✅
- **모터 제어**: GPIO 23 Power Keep-Alive 문제 해결, 텔레옵 제어 성공
- **IMU 센서**: BNO055 설정 완료, 캘리브레이션 3/3 달성 (0.7% 정확도)
- **ROS2 드라이버**: gopigo3_driver 패키지 구현 완료

### 진행 중 🔄
- **LiDAR**: RPLidar A1M8 테스트 예정

### 보류 ⏸️
- **RGB 카메라**: Ubuntu 22.04 CSI 지원 한계로 보류 (USB 웹캠으로 대체 가능)

---

## 빠른 시작 가이드

### 전제 조건 (최초 1회)

```bash
# Power Management 서비스 확인
sudo systemctl status gpg3_power.service

# 설치되지 않았다면
sudo cp ~/gpg3_power.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable gpg3_power.service
sudo systemctl start gpg3_power.service
```

### 일반 사용

```bash
# 1. IMU 캘리브레이션 (부팅 후 필수, 2-3분)
sudo python3 ~/calibrate_imu.py

# 2. ROS2 드라이버 실행
cd ~/nav2_ws
source /opt/ros/humble/setup.zsh
source install/setup.zsh
ros2 launch gopigo3_driver gopigo3_bringup.launch.py

# 3. 텔레옵 (새 터미널)
ros2 run teleop_twist_keyboard teleop_twist_keyboard
# i=전진, k=정지, ,=후진, j=좌회전, l=우회전
```

### 로봇 사양
- **휠 간격**: 0.117m
- **휠 지름**: 0.066m
- **엔코더**: 360 deg/rotation
- **센서**: BNO055 9-DOF IMU (I2C 0x28), VL53L0X 거리 센서 (I2C 0x29)

---

## 🎯 주요 문제 해결 기록

### 1. GPIO 23 Power Keep-Alive (2025-12-07)

**문제**: 모터가 명령은 받지만 실제로 회전하지 않음

**근본 원인**: GoPiGo3 하드웨어는 GPIO 23이 HIGH 상태여야 RPi가 "켜져 있다"고 인식하고 모터에 전원 공급

**해결**:
1. `gpg3_power.service` 설치 (부팅 시 GPIO 23 자동 관리)
2. ROS2 드라이버에 GPIO 23 초기화 추가 (백업)

```python
# gopigo3_driver_node.py
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(23, GPIO.OUT)
GPIO.output(23, True)  # CRITICAL!
```

**교훈**: 하드웨어 요구사항 간과 금지, 공식 OS 분석의 중요성

---

## 📊 IMU 센서 설정 (2025-12-08)

### 센서 사양

**DEXTER IMU Sensor (BNO055)**
- **칩셋**: Bosch BNO055 9-DOF IMU
- **I2C 주소**: 0x28
- **연결**: Software I2C (GPIO 2, 3)
- **출력**: Euler angles, Quaternion, 가속도, 자이로, 자력계

### 설치 요약

**문제**: `ModuleNotFoundError: No module named 'di_i2c'`

**해결**:
```bash
# 1. 시스템 패키지
sudo apt-get install -y python3-dev python3-smbus i2c-tools libi2c-dev

# 2. Python 의존성
sudo pip3 install python-periphery smbus2 numpy curtsies

# 3. DI_Sensors 설치
cd ~/DI_Sensors/Python
sudo python3 setup.py install

# 4. 호환성 래퍼 생성
cd ~/DI_Sensors/Python/di_sensors
sed -i '12s/import di_i2c/from . import di_i2c/' BNO055.py
```

**di_i2c.py 래퍼**:
```python
# ~/DI_Sensors/Python/di_sensors/di_i2c.py
from .dexter_i2c import Dexter_I2C, Dexter_I2C_RPI_1SW

DI_I2C = Dexter_I2C
DI_I2C_RPI_1SW = Dexter_I2C_RPI_1SW
```

### I2C 장치 확인

```bash
i2cdetect -y 1
#      0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
# 00:          -- -- -- -- -- 08 -- -- -- -- -- -- --
# 20: -- -- -- -- -- -- -- -- 28 29 -- -- -- -- -- --

# 0x08: GoPiGo3 보드
# 0x28: BNO055 IMU
# 0x29: VL53L0X 거리 센서
```

### 캘리브레이션

**대화형 스크립트**: `~/calibrate_imu.py`

```bash
sudo python3 ~/calibrate_imu.py
```

**과정**:
1. **Gyroscope (5초)**: 평평한 곳에 완전 정지
2. **Accelerometer**: 6개 면 각 3초씩 (바닥, 뒤집기, 앞/뒤/좌/우 세우기)
3. **Magnetometer (15-20초)**: 공중에서 8자 회전, 금속 물체 멀리

**결과** (3/3 달성):
```
System: ✅ 3/3
Gyro:   ✅ 3/3
Accel:  ✅ 3/3
Mag:    ✅ 3/3
```

**정확도 검증**:
- **가속도계**: 9.74 m/s² ≈ 지구 중력 9.81 m/s² (오차 0.7%)
- **자이로스코프**: ~0.06 °/s (정지 상태 정확)
- **자력계**: -40.9 μT (정상 범위 -60~60 μT)

### 테스트 코드

**기본 테스트**: `~/test_imu.py`

```python
#!/usr/bin/env python3
import sys
sys.path.insert(0, '/home/ubuntu/DI_Sensors/Python')
from di_sensors.inertial_measurement_unit import InertialMeasurementUnit

imu = InertialMeasurementUnit(bus="RPI_1SW")

cal_status = imu.BNO055.get_calibration_status()  # System, Gyro, Accel, Mag (0-3)
mag = imu.read_magnetometer()        # X, Y, Z (μT)
gyro = imu.read_gyroscope()          # X, Y, Z (°/s)
accel = imu.read_accelerometer()     # X, Y, Z (m/s²)
euler = imu.read_euler()             # Heading, Roll, Pitch (°)
temp = imu.read_temperature()        # 온도 (°C)
quaternion = imu.read_quaternion()   # X, Y, Z, W
```

### 중요 참고사항

**캘리브레이션 유지**:
- ✅ 전원 켜진 동안 유지
- ❌ 재부팅 시 다시 필요
- ⚠️ 금속 물체 근처 영향 주의

**SLAM 활용**:
- 휠 오도메트리 슬립 보정
- 정확한 자세 추정 (Roll, Pitch, Yaw)
- LiDAR/카메라와 센서 융합
- IMU Preintegration (VSLAM/VINS)

### 트러블슈팅

**캘리브레이션이 3/3 미달**:
- **Gyro**: 완전 정지, 평평한 곳
- **Accel**: 더 많은 면 시도 (최소 2-3개)
- **Mag**: 8자 회전 크게, 금속 멀리

**온도 0°C 표시**:
- 캘리브레이션 전 센서 초기화 중, 완료 후 정상

---

## 📷 카메라 문제 및 해결 (2025-12-08)

### CSI 카메라 감지 실패

**하드웨어**: Raspberry Pi Camera Module V2 (Sony IMX219 8MP)

**문제 현상**:
```bash
vcgencmd get_camera
# supported=1 detected=0
```

- 물리적 연결 정상 확인
- `/boot/firmware/config.txt`에 `start_x=1`, `gpu_mem=128` 추가 후 재부팅해도 실패

### GoPiGo OS vs Ubuntu 비교

**핵심 발견**: **GoPiGo OS = Raspberry Pi OS (Debian 기반)**

#### 증거

1. **`picamera` 라이브러리 사용**:
   - Face Detection.ipynb에서 `import picamera` 확인
   - `picamera`는 Raspberry Pi OS 전용 라이브러리
   - Raspberry Pi GPU 펌웨어와 직접 통신

2. **GUI 환경**:
   - LXDE 데스크톱 (`lxpanel`, `pcmanfm`)
   - Chromium 브라우저
   - Raspberry Pi OS 기본 스택

3. **카메라 감지**:
   - `/home/pi/Dexter/detected_camera.txt`: "Camera is detected"
   - `vcgencmd get_camera` 정상 작동

#### 기술적 차이

| 항목 | Raspberry Pi OS (GoPiGo) | Ubuntu 22.04 |
|------|-------------------------|--------------|
| **기반** | Debian + RPi 펌웨어 | 표준 Ubuntu ARM64 |
| **CSI 카메라** | ✅ 완벽 (`picamera`, `libcamera`) | ❌ 제한적 |
| **카메라 라이브러리** | `picamera` (레거시) | `libcamera` (불완전) |
| **vcgencmd** | ✅ 완전 작동 | ⚠️ 제한적 |
| **GPU 펌웨어** | VideoCore IV 완전 통합 | 표준 커널만 |

**Raspberry Pi OS의 카메라 처리**:
```python
# GoPiGo OS (Raspberry Pi OS)
import picamera  # GPU 펌웨어 직접 제어

with picamera.PiCamera() as camera:
    camera.capture('image.jpg')  # ✅ 작동!
```

**Ubuntu의 한계**:
```python
# Ubuntu 22.04
import picamera  # ❌ 설치 불가 (RPi OS 전용)

import cv2
cap = cv2.VideoCapture(0)  # ❌ CSI 카메라 미감지
```

#### 근본 원인

**Raspberry Pi OS (GoPiGo)**:
- VideoCore IV GPU가 카메라 직접 제어
- `/boot/config.txt`의 `start_x=1`이 GPU 펌웨어 활성화
- CSI 카메라가 GPU 메모리에서 직접 처리

**Ubuntu 22.04**:
- Raspberry Pi 펌웨어 미통합
- CSI를 V4L2 드라이버로 감지 실패
- `libcamera` 있지만 RPi Camera V2 지원 불완전

### 해결: Intel RealSense D435i

**프로젝트 요구사항**: 자율주행 + 사람 인식/식별

**선택**: Intel RealSense D435i (RGB-D 카메라)

**선택 이유**:
- **RGB + Depth**: 사람까지 거리 측정
- **IMU 내장**: BNO055와 센서 퓨전
- **ROS2 완벽 지원**: `realsense2_camera` 공식 패키지
- **사람 인식 최적화**: 얼굴 인식 + 거리 정보
- **장애물 회피 강화**: Depth로 LiDAR 보완

#### 스펙

**Intel RealSense D435i**
- 가격: $279-$329 (Amazon)
- RGB: 1920x1080 @ 30fps
- Depth: 1280x720 @ 30fps (최대 10m)
- FOV: 87° × 58° × 95°
- IMU: Bosch BMI055 (Gyro + Accel)
- USB: USB 3.1 Gen 1
- 구매: [Amazon - Intel RealSense D435i](https://www.amazon.com/s?k=intel+realsense+d435i)

#### ROS2 통합

**설치**:
```bash
# RealSense SDK
sudo apt-get install ros-humble-realsense2-camera
sudo apt-get install ros-humble-realsense2-description

# Python 라이브러리
pip3 install pyrealsense2

# 사람 인식 라이브러리
pip3 install face-recognition        # 얼굴 인식
pip3 install ultralytics             # YOLOv8 사람 검출
pip3 install deep-sort-realtime      # 사람 추적
```

**Launch 예제**:
```python
# realsense_launch.py
Node(
    package='realsense2_camera',
    executable='realsense2_camera_node',
    parameters=[{
        'enable_color': True,
        'enable_depth': True,
        'enable_gyro': True,
        'enable_accel': True,
        'align_depth.enable': True,
        'depth_module.profile': '1280x720x30',
        'rgb_camera.profile': '1920x1080x30',
    }]
)
```

#### 사람 인식 시스템

**1. 사람 검출 (YOLO)**:
```python
import ultralytics
model = ultralytics.YOLO('yolov8n.pt')

results = model(rgb_frame)
for box in results.boxes:
    if box.cls == 0:  # person class
        x, y = box.xywh[:2]
        distance = depth_frame.get_distance(int(x), int(y))
        print(f"Person at {distance:.2f}m")
```

**2. 얼굴 인식**:
```python
import face_recognition

# 사전 등록된 얼굴 DB
known_faces = {
    "Alice": face_encoding_alice,
    "Bob": face_encoding_bob,
}

# 실시간 인식
face_encodings = face_recognition.face_encodings(rgb_image)
for encoding in face_encodings:
    matches = face_recognition.compare_faces(
        known_faces.values(), encoding
    )
    name = list(known_faces.keys())[matches.index(True)]
    print(f"Recognized: {name}")
```

**3. 사람 추적 (DeepSORT)**:
```python
from deep_sort_realtime.deepsort_tracker import DeepSort

tracker = DeepSort(max_age=30)
tracks = tracker.update_tracks(detections, frame=rgb_image)

for track in tracks:
    if track.is_confirmed():
        print(f"Tracking Person ID: {track.track_id}")
```

### 시스템 아키텍처

```
┌─────────────────────────────────────────────────┐
│              GoPiGo3 Robot System               │
├─────────────────────────────────────────────────┤
│                                                 │
│  센서 레이어:                                     │
│  ┌──────────┐  ┌─────────┐  ┌──────────────┐  │
│  │ RPLidar  │  │ BNO055  │  │  RealSense   │  │
│  │  A1M8    │  │  IMU    │  │    D435i     │  │
│  │ (Serial) │  │ (I2C)   │  │  (USB 3.0)   │  │
│  └────┬─────┘  └────┬────┘  └──────┬───────┘  │
│       │             │               │           │
│       └─────────────┴───────────────┘           │
│                     │                           │
│  처리 레이어:         │                           │
│              ┌──────▼──────┐                    │
│              │   ROS2 Hub  │                    │
│              │  (Humble)   │                    │
│              └──────┬──────┘                    │
│                     │                           │
│       ┌─────────────┼─────────────┐            │
│       │             │             │            │
│  ┌────▼────┐  ┌─────▼─────┐  ┌───▼────────┐  │
│  │  SLAM   │  │   Nav2    │  │   Person   │  │
│  │ Toolbox │  │  Stack    │  │Recognition │  │
│  └─────────┘  └───────────┘  └────────────┘  │
│                                               │
│  출력: 2D Map + 자율주행 + "Alice at 2.3m"     │
└───────────────────────────────────────────────┘
```

### 대안: 저예산 옵션

#### Logitech C920 ($60-80)

**장점**:
- 저렴 ($60-80)
- 검증된 ROS2 호환
- 얼굴 인식 가능

**단점**:
- Depth 정보 없음
- 장애물 회피 LiDAR만 의존

**사용**:
```bash
sudo apt-get install ros-humble-usb-cam
ros2 run usb_cam usb_cam_node_exe --ros-args \
  -p video_device:=/dev/video0 \
  -p framerate:=30.0 \
  -p image_width:=1920 \
  -p image_height:=1080
```

### 카메라 비교표

| 제품 | 가격 | 해상도 | FOV | Depth | IMU | ROS2 | 추천도 |
|------|------|--------|-----|-------|-----|------|--------|
| **Logitech C920** | $70 | 1080p | 78° | ❌ | ❌ | ✅ | ⭐⭐⭐⭐ |
| **Logitech C930e** | $130 | 1080p | 90° | ❌ | ❌ | ✅ | ⭐⭐⭐⭐ |
| **RealSense D435i** | $299 | 1080p | 87° | ✅ | ✅ | ✅✅✅ | ⭐⭐⭐⭐⭐ |

### 최종 BOM (Bill of Materials)

| 항목 | 제품 | 가격 | 상태 |
|------|------|------|------|
| 메인 카메라 | Intel RealSense D435i | $299 | 구매 예정 |
| LiDAR | RPLidar A1M8 | $99 | 구매 예정 |
| IMU | DEXTER BNO055 | - | ✅ 보유 |
| 로봇 플랫폼 | GoPiGo3 | - | ✅ 보유 |
| 컴퓨팅 | Raspberry Pi 4 | - | ✅ 보유 |
| **총합** | | **$398** | |

### 성능 예상

**RealSense D435i 기준**:
- **맵핑**: LiDAR + Depth fusion → 정확도 향상
- **장애물 회피**: LiDAR (2D) + Depth (3D) → 완벽 회피
- **사람 인식**: 30fps @ 1080p, 실시간 얼굴 인식
- **거리 측정**: RGB 픽셀 → Depth → 정확한 거리 (오차 ±2%)
- **총 처리**: ~50ms (RPi 4 기준)

---

## 다음 단계

1. **RPLidar A1M8 연결 및 테스트**
2. **LiDAR ROS2 드라이버 설정**
3. **Nav2 + SLAM Toolbox 통합**
4. **맵핑 및 네비게이션 테스트**

---

## 참고 파일

- **ROS2 드라이버**: [gopigo3_driver_node.py](nav2_ws/src/gopigo3_driver/gopigo3_driver/gopigo3_driver_node.py)
- **IMU 테스트**: [test_imu.py](test_imu.py)
- **IMU 캘리브레이션**: [calibrate_imu.py](calibrate_imu.py)
- **카메라 테스트**: [test_camera.py](test_camera.py)
- **Power Service**: [gpg3_power.service](gpg3_power.service)
