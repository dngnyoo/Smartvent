# GoPiGo3 ROS2 Setup Progress

**최근 업데이트**: 2025-12-08
**시작일**: 2025-12-07

## 요약

### 완료된 작업 ✅
- I2C/SPI 활성화, 권한 설정, GoPiGo3 보드 감지 (I2C 0x08)
- ROS2 gopigo3_driver 패키지 생성 (`~/nav2_ws/src/gopigo3_driver/`)
- URDF 모델, launch 파일, robot_state_publisher 설정
- 커스텀 SPI 코드 제거 → 공식 GoPiGo3 라이브러리 사용으로 변경
- Motor limits 설정 추가 (PWM=100%, DPS=1000)

### 주요 수정사항 (2025-12-08)
```python
# 공식 GoPiGo3 라이브러리 사용
import sys
sys.path.insert(0, '/home/ubuntu/GoPiGo3/Software/Python')
import gopigo3
self.gpg = gopigo3.GoPiGo3()

# 모터 limits 설정 (필수!)
self.gpg.set_motor_limits(self.gpg.MOTOR_LEFT + self.gpg.MOTOR_RIGHT, 100, 1000)
```

## 빠른 시작 가이드

### 1. pigpiod 시작 (한 번만)
```bash
sudo pigpiod  # SPI 핀 초기화용, 재부팅 후 1회 실행
```

### 2. 빌드 (수정 후 필요 시)
```bash
cd ~/nav2_ws
source /opt/ros/humble/setup.zsh
colcon build --packages-select gopigo3_driver --symlink-install
source install/setup.zsh
```

### 3. 드라이버 실행
```bash
ros2 launch gopigo3_driver gopigo3_bringup.launch.py
```

### 4. 텔레옵 (새 터미널)
```bash
source ~/nav2_ws/install/setup.zsh
ros2 run teleop_twist_keyboard teleop_twist_keyboard
# i=전진, k=정지, ,=후진, j=좌회전, l=우회전
```

## 참고
- **드라이버**: [gopigo3_driver_node.py](src/gopigo3_driver/gopigo3_driver/gopigo3_driver_node.py)
- **로봇 사양**: 휠 간격 0.117m, 휠 지름 0.066m, 엔코더 360 deg/rotation
- **pigpiod 자동 시작**: [setup_pigpiod_service.sh](setup_pigpiod_service.sh) 실행

---

## 진행 상황 (2025-12-08 계속)

### 추가 문제 발견: 모터가 여전히 움직이지 않음 ❌

**상황**:
1. ✅ ROS2 드라이버가 공식 GoPiGo3 라이브러리 사용
2. ✅ pigpiod 실행됨
3. ✅ GoPiGo3 보드와 통신 정상
4. ✅ `/cmd_vel` 토픽 수신하고 DPS 계산 정상
5. ❌ 모터가 움직이지 않음, 엔코더 값 변화 없음

**진단 결과**:

### 1단계: 모터 상태 확인
```python
# 초기 상태
print("Left status:", GPG.get_motor_status(GPG.MOTOR_LEFT))
print("Right status:", GPG.get_motor_status(GPG.MOTOR_RIGHT))
# 출력: [0, -128, 0, 0]  # -128 = MOTOR_FLOAT (모터 비활성화)
```

### 2단계: Motor Limits 설정 추가 ✅
**발견**: GoPiGo3 라이브러리의 `reset_all()` 메소드가 모터 limits를 `(0, 0)`으로 설정함
- `set_motor_limits(0, 0)` → 모터가 명령을 받아도 작동하지 않음

**해결책**: 드라이버 초기화 시 모터 limits 설정 추가
```python
# gopigo3_driver_node.py의 __init__에 추가
self.gpg.set_motor_limits(self.gpg.MOTOR_LEFT + self.gpg.MOTOR_RIGHT, 100, 1000)
```

**테스트 결과**:
```python
# Motor limits 설정 후
GPG.set_motor_limits(GPG.MOTOR_LEFT + GPG.MOTOR_RIGHT, 100, 1000)
GPG.set_motor_power(GPG.MOTOR_LEFT + GPG.MOTOR_RIGHT, 50)
time.sleep(2)
print("Left status:", GPG.get_motor_status(GPG.MOTOR_LEFT))
# 출력: [0, 50, 0, 0]  # PWM 50% 명령 수신됨
print("Left encoder:", GPG.get_motor_encoder(GPG.MOTOR_LEFT))
# 출력: 0  # 엔코더 여전히 변화 없음 ❌
```

### 3단계: 모터 상태 분석
- **모터 상태 플래그**: `[flags, pwm, encoder_offset, dps]`
  - `[0, -128, 0, 0]` → 모터 FLOAT 상태 (비활성화)
  - `[0, 50, 0, 0]` → PWM 50% 명령 수신됨
- **문제**: 명령은 GoPiGo3 보드에 전달되지만, 실제 모터가 회전하지 않음
- **엔코더**: 계속 0으로 고정 (물리적 움직임 없음)

### 4단계: 전압 확인
```python
print("Battery:", GPG.get_voltage_battery(), "V")  # 12.096V (정상)
print("5V:", GPG.get_voltage_5v(), "V")            # 5.135V (정상)
```
→ 전원 공급 정상

### 5단계: 펌웨어 정보
```python
print("Firmware:", GPG.get_version_firmware())  # 1.0.0 (초기 버전)
print("Hardware:", GPG.get_version_hardware())  # 3.x.x
```

### 현재 상태 요약

**작동하는 것**:
- ✅ I2C 통신 (0x08 주소로 GoPiGo3 보드 인식)
- ✅ SPI 통신 (배터리 전압, 펌웨어 버전 읽기 성공)
- ✅ 명령 전달 (모터 상태가 FLOAT → PWM 50%로 변경됨)
- ✅ GoPiGo3 OS에서 모터 정상 작동 확인됨

**작동하지 않는 것**:
- ❌ 모터가 실제로 회전하지 않음
- ❌ 엔코더 값 변화 없음 (항상 0)

### 가능한 원인 (우선순위 순)

1. **GoPiGo3 OS vs Ubuntu 22.04 환경 차이**
   - GoPiGo3 OS: Raspbian 기반, Python 3.7, 특별한 초기화 스크립트 가능성
   - Ubuntu 22.04: Python 3.10, 다른 시스템 설정
   - **검증 필요**: GoPiGo3 라이브러리 버전 비교

2. **펌웨어 버전 문제**
   - 현재: 1.0.0 (초기 버전)
   - 업데이트 시도 실패 (경로 문제: `/home/pi` vs `/home/ubuntu`)

3. **하드웨어 연결 문제** (가능성 낮음)
   - GoPiGo3 OS에서 작동함 → 하드웨어 자체는 정상
   - 모터 케이블이 MA/MB 포트에 연결되어 있음 (GoPiGo3 OS에서 확인됨)

4. **모터 드라이버 초기화 누락**
   - GoPiGo3 보드 펌웨어가 특정 초기화 시퀀스를 기대할 수 있음
   - GoPiGo3 OS의 부팅 스크립트나 서비스 확인 필요

### 다음 진단 단계

#### 우선순위 1: GoPiGo3 OS와 비교
```bash
# GoPiGo3 OS로 부팅 후
head -20 ~/GoPiGo3/Software/Python/gopigo3.py | grep version

# Ubuntu 22.04에서
head -20 ~/GoPiGo3/Software/Python/gopigo3.py | grep version
```

#### 우선순위 2: GoPiGo3 OS에서 motor limits 테스트
```python
import sys
sys.path.insert(0, '/home/pi/GoPiGo3/Software/Python')
import gopigo3
import time

GPG = gopigo3.GoPiGo3()

# Limits 설정 없이 테스트
print("=== Before set_motor_limits ===")
GPG.set_motor_dps(GPG.MOTOR_LEFT, 300)
time.sleep(2)
print("Encoder:", GPG.get_motor_encoder(GPG.MOTOR_LEFT))
GPG.set_motor_dps(GPG.MOTOR_LEFT, 0)

# Limits 설정 후 테스트
print("\n=== After set_motor_limits ===")
GPG.set_motor_limits(GPG.MOTOR_LEFT, 100, 1000)
GPG.set_motor_dps(GPG.MOTOR_LEFT, 300)
time.sleep(2)
print("Encoder:", GPG.get_motor_encoder(GPG.MOTOR_LEFT))
GPG.set_motor_dps(GPG.MOTOR_LEFT, 0)
```

#### 우선순위 3: GoPiGo3 OS 초기화 스크립트 확인
```bash
# GoPiGo3 OS에서
systemctl list-units | grep gopigo
cat /etc/rc.local
ls /etc/systemd/system/ | grep gopigo
```

### 참고: 수정된 드라이버 코드
[gopigo3_driver_node.py](src/gopigo3_driver/gopigo3_driver/gopigo3_driver_node.py) Line 49-57:
```python
# CRITICAL: Set motor limits BEFORE using motors
# Without this, motors will not respond to commands!
try:
    # Set PWM limit to 100% and DPS limit to 1000
    self.gpg.set_motor_limits(self.gpg.MOTOR_LEFT + self.gpg.MOTOR_RIGHT, 100, 1000)
    self.get_logger().info('Motor limits set: PWM=100%, DPS=1000')
except Exception as e:
    self.get_logger().error(f'Failed to set motor limits: {e}')
    raise
```

### 미해결 과제

**핵심 질문**: 왜 GoPiGo3 보드가 PWM 명령을 받아도 모터에 전원을 공급하지 않는가?

**가설**:
1. Ubuntu 22.04 환경에서 GoPiGo3 라이브러리의 초기화가 불완전
2. GoPiGo3 OS에만 존재하는 특별한 설정/서비스가 필요
3. 펌웨어 1.0.0의 버그 또는 호환성 문제

**다음 작업**: GoPiGo3 OS 환경 분석 및 라이브러리 버전 비교
