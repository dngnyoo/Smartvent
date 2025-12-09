# GoPiGo OS Analysis & Ubuntu Fix Plan

**작성일**: 2025-12-08
**목적**: GoPiGo OS 분석을 통해 파악한 Ubuntu 22.04 모터 작동 불가 문제의 근본 원인과 해결 방법

---

## 🎯 문제의 근본 원인 (ROOT CAUSE)

### **GPIO 23번 핀 - Power Keep-Alive Signal**

GoPiGo3 하드웨어는 **GPIO 23번 핀이 HIGH 상태**여야만 라즈베리파이가 "켜져 있다"고 인식합니다.

```python
# /home/pi/Dexter/GoPiGo3/Software/gopigo3_power.py
GPIO.setup(23, GPIO.OUT)
GPIO.output(23, True)  # 이것이 핵심!
```

**주석 내용 (gopigo3_power.py Line 11-15):**
```
GPIO 23 needs to remain low impedance (output) set to a HIGH state.
If GPIO 23 gets left floating (high impedance) the GoPiGo3 assumes
the RPi has shut down fully.
SW should never write GPIO 23 to LOW or set it as an INPUT.
```

### 왜 이것이 문제인가?

**GoPiGo OS:**
- ✅ 부팅 시 `gpg3_power.service` 자동 시작
- ✅ `gopigo3_power.py`가 백그라운드에서 GPIO 23을 HIGH로 유지
- ✅ GoPiGo3가 "RPi 켜짐" 인식 → **모터에 전원 공급**

**Ubuntu 22.04 (현재):**
- ❌ `gpg3_power.service` 없음
- ❌ GPIO 23이 초기화되지 않음 (floating 상태)
- ❌ GoPiGo3가 "RPi 꺼짐" 판단 → **모터에 전원 차단**
- ❌ SPI 명령은 받지만 모터가 실제로 작동하지 않음!

**이것이 바로 Ubuntu에서 다음과 같은 증상이 나타나는 이유입니다:**
- SPI/I2C 통신 정상 (배터리 전압, 펌웨어 버전 읽기 가능)
- 모터 명령 전달 정상 (모터 상태 변경됨)
- 하지만 모터가 실제로 회전하지 않음
- 엔코더 값 항상 0

---

## 📊 GoPiGo OS 전체 분석

### 1. 디렉토리 구조

```
/home/pi/
├── .pypaths                          # Python 패키지 경로 추적
├── Desktop/
│   ├── gopigo3_control_panel.desktop
│   ├── gopigo3_calibration.desktop
│   └── Troubleshooting_Start.desktop
└── Dexter/                           # 메인 GoPiGo 디렉토리
    ├── gpg3_config.json              # 로봇 물리 파라미터
    ├── detected_robot.txt            # "GoPiGo3"
    ├── GoPiGo3/                      # GoPiGo3 소프트웨어 저장소
    │   ├── Install/
    │   │   ├── install.sh
    │   │   ├── update_gopigo3.sh
    │   │   ├── gpg3_power.service    # 🔥 Power management 서비스
    │   │   ├── gpg3_power.sh         # 서비스 시작 스크립트
    │   │   ├── antenna_wifi.service
    │   │   └── antenna_wifi.sh
    │   ├── Software/
    │   │   ├── gopigo3_power.py      # 🔥 GPIO 23 관리 데몬
    │   │   └── Python/
    │   │       ├── gopigo3.py        # v1.3.2
    │   │       ├── easygopigo3.py
    │   │       └── setup.py
    │   └── Firmware/
    │       ├── GoPiGo3_Firmware_1.0.0.bin
    │       └── gopigo3_flash_firmware.sh
    ├── DI_Sensors/
    ├── PivotPi/
    └── lib/
        ├── Dexter/script_tools/
        ├── wiringPi/
        └── openocd/
```

### 2. 중요 설정 파일

#### A. 로봇 물리 파라미터 (`/home/pi/Dexter/gpg3_config.json`)
```json
{
  "wheel-diameter": 66.5,
  "wheel-base-width": 117,
  "ticks": 6,
  "motor_gear_ratio": 120
}
```

#### B. Python 경로 추적 (`/home/pi/.pypaths`)
```
/usr/local/lib/python3.7/dist-packages/gopigo3-1.2.0-py3.7.egg
/root/.local/lib/python3.7/site-packages/gopigo3-1.2.0-py3.7.egg
```

### 3. Systemd 서비스

#### A. Power Management Service

**파일:** `/etc/systemd/system/gpg3_power.service`
```ini
[Unit]
Description=GoPiGo3 Power Service

[Service]
Type=idle
ExecStart=/usr/bin/env bash /home/pi/Dexter/GoPiGo3/Install/gpg3_power.sh

[Install]
WantedBy=multi-user.target
```

**실행 내용 (`gopigo3_power.sh`):**
```bash
# gopigo3_power.py가 이미 실행 중인지 확인
# 실행 중이 아니면 시작
sudo python3 $REPO_PATH/Software/gopigo3_power.py
```

**핵심 코드 (`gopigo3_power.py`):**
```python
import RPi.GPIO as GPIO
import time
import os

GPIO.setmode(GPIO.BCM)
GPIO.setup(22, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)  # Shutdown button

# 🔥 이것이 핵심!
GPIO.setup(23, GPIO.OUT)
GPIO.output(23, True)  # GoPiGo3에 "RPi 켜짐" 신호 전송

# 무한 루프로 GPIO 23 HIGH 유지 + 셧다운 버튼 모니터링
while True:
    if GPIO.input(22):
        os.system("shutdown now -h")
    time.sleep(0.1)
```

#### B. WiFi Indicator Service

**파일:** `/etc/systemd/system/antenna_wifi.service`
- WiFi LED 제어 (연결: 청록색, 미연결: 빨간색)
- 모터 작동과는 무관

### 4. SPI 통신 설정

**GoPiGo3 라이브러리 초기화 (`gopigo3.py`):**
```python
import spidev
import pigpio

# SPI 설정
GPG_SPI = spidev.SpiDev()
GPG_SPI.open(0, 1)              # Bus 0, Device 1 (CS1)
GPG_SPI.max_speed_hz = 500000   # 500 kHz
GPG_SPI.mode = 0b00             # Mode 0
GPG_SPI.bits_per_word = 8

# GPIO 핀을 SPI ALT0 모드로 설정 (pigpiod 필요!)
pi_gpio = pigpio.pi()
pi_gpio.set_mode(9, pigpio.ALT0)   # MISO
pi_gpio.set_mode(10, pigpio.ALT0)  # MOSI
pi_gpio.set_mode(11, pigpio.ALT0)  # SCLK
```

**부트 설정 (`/boot/config.txt`):**
```bash
dtparam=spi=on
```

**커널 모듈 (`/etc/modules`):**
```
spi-dev
```

### 5. 설치 프로세스

**주요 단계 (`update_gopigo3.sh`):**
1. 시스템 패키지 설치
   ```bash
   sudo apt-get install python3-pip python3-numpy python3-curtsies pigpio
   ```

2. pigpiod 활성화
   ```bash
   sudo systemctl enable pigpiod
   sudo systemctl start pigpiod
   ```

3. SPI 활성화
   ```bash
   # /boot/config.txt 수정
   dtparam=spi=on

   # 모듈 로드
   echo "spi-dev" >> /etc/modules
   ```

4. Python 패키지 시스템 전체 설치
   ```bash
   cd /home/pi/Dexter/GoPiGo3/Software/Python
   sudo python3 setup.py install
   ```

5. Power management 서비스 설치
   ```bash
   sudo cp Install/gpg3_power.service /etc/systemd/system/
   sudo systemctl daemon-reload
   sudo systemctl enable gpg3_power.service
   sudo systemctl start gpg3_power.service
   ```

### 6. 라이브러리 버전

**GoPiGo OS와 Ubuntu 22.04 동일:**
- `gopigo3.py` version: **1.3.2**

**중요:** 라이브러리 버전은 같지만 시스템 설정이 다름!

---

## 🔧 Ubuntu 22.04에서 해야 할 작업

### 우선순위 1: GPIO 23 활성화 (즉시!)

**빠른 테스트 (임시 해결):**
```python
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(23, GPIO.OUT)
GPIO.output(23, True)

# 이제 모터 테스트!
import sys
sys.path.insert(0, '/home/ubuntu/GoPiGo3/Software/Python')
import gopigo3
GPG = gopigo3.GoPiGo3()
GPG.set_motor_limits(GPG.MOTOR_LEFT + GPG.MOTOR_RIGHT, 100, 1000)
GPG.set_motor_dps(GPG.MOTOR_LEFT, 300)
# 이제 모터가 작동해야 함!
```

### 우선순위 2: pigpiod 자동 시작 확인

```bash
# 상태 확인
sudo systemctl status pigpiod

# 활성화 (부팅 시 자동 시작)
sudo systemctl enable pigpiod
sudo systemctl start pigpiod
```

### 우선순위 3: Power Management 서비스 설치

#### 방법 A: GoPiGo OS 파일 복사 (추천)

```bash
# GoPiGo OS에서 Ubuntu로 파일 복사
# gopigo3_power.py
sudo cp ~/GoPiGo3/Software/gopigo3_power.py /home/ubuntu/GoPiGo3/Software/

# 서비스 파일 (경로 수정 필요)
sudo cp ~/Dexter/GoPiGo3/Install/gpg3_power.service /etc/systemd/system/
sudo cp ~/Dexter/GoPiGo3/Install/gpg3_power.sh /home/ubuntu/GoPiGo3/Install/

# 경로 수정
sudo sed -i 's|/home/pi|/home/ubuntu|g' /etc/systemd/system/gpg3_power.service
sudo sed -i 's|/home/pi|/home/ubuntu|g' /home/ubuntu/GoPiGo3/Install/gpg3_power.sh

# 서비스 활성화
sudo systemctl daemon-reload
sudo systemctl enable gpg3_power.service
sudo systemctl start gpg3_power.service

# 상태 확인
sudo systemctl status gpg3_power.service
```

#### 방법 B: 직접 생성

**파일 1: `/home/ubuntu/GoPiGo3/Software/gopigo3_power.py`**
```python
#!/usr/bin/env python3
import RPi.GPIO as GPIO
import time
import os

GPIO.setmode(GPIO.BCM)
GPIO.setup(22, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)  # Shutdown button
GPIO.setup(23, GPIO.OUT)
GPIO.output(23, True)  # Keep GoPiGo3 powered

print("GoPiGo3 Power Management started")
print("GPIO 23: HIGH (GoPiGo3 powered)")
print("GPIO 22: Monitoring shutdown button")

while True:
    if GPIO.input(22):
        print("Shutdown button pressed")
        os.system("shutdown now -h")
    time.sleep(0.1)
```

**파일 2: `/etc/systemd/system/gpg3_power.service`**
```ini
[Unit]
Description=GoPiGo3 Power Service
After=network.target

[Service]
Type=simple
User=root
ExecStart=/usr/bin/python3 /home/ubuntu/GoPiGo3/Software/gopigo3_power.py
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
```

**설치:**
```bash
sudo chmod +x /home/ubuntu/GoPiGo3/Software/gopigo3_power.py
sudo systemctl daemon-reload
sudo systemctl enable gpg3_power.service
sudo systemctl start gpg3_power.service
sudo systemctl status gpg3_power.service
```

### 우선순위 4: ROS2 드라이버에 GPIO 23 초기화 추가

**파일:** `~/nav2_ws/src/gopigo3_driver/gopigo3_driver/gopigo3_driver_node.py`

**수정 위치:** `__init__` 메소드 초반

```python
def __init__(self):
    super().__init__('gopigo3_driver')

    # 🔥 GPIO 23 초기화 추가 (GoPiGo3 power keep-alive)
    try:
        import RPi.GPIO as GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(23, GPIO.OUT)
        GPIO.output(23, True)
        self.get_logger().info('GPIO 23 set to HIGH (GoPiGo3 power keep-alive)')
    except Exception as e:
        self.get_logger().error(f'Failed to initialize GPIO 23: {e}')
        self.get_logger().error('Motors may not work! Consider installing gpg3_power.service')

    # 기존 GoPiGo3 초기화 코드...
    import sys
    sys.path.insert(0, '/home/ubuntu/GoPiGo3/Software/Python')
    import gopigo3
    self.gpg = gopigo3.GoPiGo3()

    # 나머지 코드...
```

### 우선순위 5: SPI 설정 확인

```bash
# SPI 활성화 확인
ls /dev/spi*  # /dev/spidev0.0, /dev/spidev0.1 있어야 함

# 없다면 부트 설정 확인
sudo nano /boot/firmware/config.txt  # Ubuntu는 /boot/firmware/!
# 다음 줄 추가/확인:
dtparam=spi=on

# 모듈 로드 확인
lsmod | grep spi
# spi_bcm2835가 보여야 함

# 없다면:
echo "spi-dev" | sudo tee -a /etc/modules

# 재부팅
sudo reboot
```

### 우선순위 6: 사용자 권한 확인

```bash
# 현재 사용자를 필요한 그룹에 추가
sudo usermod -a -G spi,gpio,i2c,dialout ubuntu

# 재로그인 필요
```

### 우선순위 7: 필수 Python 패키지 설치

```bash
# 시스템 패키지
sudo apt-get update
sudo apt-get install -y python3-rpi.gpio python3-spidev pigpio

# pip 패키지 (venv 외부에 설치)
sudo pip3 install RPi.GPIO spidev pigpio --break-system-packages
```

---

## 🧪 테스트 시퀀스

### 1단계: GPIO 23 테스트 (가장 먼저!)

```python
#!/usr/bin/env python3
import RPi.GPIO as GPIO
import sys
sys.path.insert(0, '/home/ubuntu/GoPiGo3/Software/Python')
import gopigo3
import time

print("=== GPIO 23 Power Keep-Alive Test ===")

# GPIO 23 활성화
GPIO.setmode(GPIO.BCM)
GPIO.setup(23, GPIO.OUT)
GPIO.output(23, True)
print("✅ GPIO 23 set to HIGH")

# GoPiGo3 초기화
GPG = gopigo3.GoPiGo3()
print(f"✅ GoPiGo3 connected")
print(f"   Battery: {GPG.get_voltage_battery()}V")
print(f"   Firmware: {GPG.get_version_firmware()}")

# Motor limits 설정
GPG.set_motor_limits(GPG.MOTOR_LEFT + GPG.MOTOR_RIGHT, 100, 1000)
print("✅ Motor limits set")

# 모터 테스트
print("\n=== Testing LEFT motor ===")
GPG.set_motor_dps(GPG.MOTOR_LEFT, 300)
time.sleep(2)
encoder = GPG.get_motor_encoder(GPG.MOTOR_LEFT)
GPG.set_motor_dps(GPG.MOTOR_LEFT, 0)

print(f"Encoder value: {encoder}")
if encoder != 0:
    print("🎉 SUCCESS! Motor is working!")
else:
    print("❌ FAILED! Motor still not working")

GPIO.cleanup()
```

### 2단계: Power Service 테스트

```bash
# 서비스 시작
sudo systemctl start gpg3_power.service

# 상태 확인
sudo systemctl status gpg3_power.service
# "GPIO 23: HIGH" 메시지가 보여야 함

# GPIO 상태 확인
sudo python3 -c "import RPi.GPIO as GPIO; GPIO.setmode(GPIO.BCM); print('GPIO 23:', GPIO.input(23))"
# 출력: GPIO 23: 1 (HIGH)
```

### 3단계: ROS2 드라이버 테스트

```bash
# 터미널 1: 드라이버 실행
cd ~/nav2_ws
source /opt/ros/humble/setup.zsh
source install/setup.zsh
ros2 launch gopigo3_driver gopigo3_bringup.launch.py

# 터미널 2: 텔레옵
source ~/nav2_ws/install/setup.zsh
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# i 키로 전진 테스트
```

---

## 📋 체크리스트

Ubuntu 22.04 설정 완료 확인:

- [ ] pigpiod 서비스 활성화 및 실행 중
- [ ] SPI 활성화 (`/dev/spidev0.1` 존재)
- [ ] GPIO 23 자동 초기화 (gpg3_power.service 또는 ROS2 드라이버)
- [ ] 사용자가 spi, gpio, i2c 그룹에 속함
- [ ] RPi.GPIO, spidev 패키지 설치됨
- [ ] GoPiGo3 Python 라이브러리 올바른 경로에 설치
- [ ] 테스트 스크립트로 모터 작동 확인
- [ ] ROS2 드라이버에서 모터 작동 확인

---

## 🔍 추가 디버깅

### GPIO 상태 확인

```bash
# GPIO 23 상태
sudo python3 -c "import RPi.GPIO as GPIO; GPIO.setmode(GPIO.BCM); GPIO.setup(23, GPIO.IN); print('GPIO 23:', 'HIGH' if GPIO.input(23) else 'LOW')"
```

### GoPiGo3 통신 확인

```python
import sys
sys.path.insert(0, '/home/ubuntu/GoPiGo3/Software/Python')
import gopigo3

GPG = gopigo3.GoPiGo3()
print("Battery:", GPG.get_voltage_battery(), "V")
print("5V:", GPG.get_voltage_5v(), "V")
print("Firmware:", GPG.get_version_firmware())
print("Hardware:", GPG.get_version_hardware())
print("Manufacturer:", GPG.get_manufacturer())
print("Board:", GPG.get_board())
```

### pigpiod 확인

```bash
# 프로세스 확인
ps aux | grep pigpiod

# 포트 확인
sudo netstat -tulpn | grep 8888
# pigpiod는 localhost:8888에서 실행됨
```

---

## 📝 주요 발견 요약

1. **GPIO 23 핀이 핵심**: HIGH 상태가 아니면 GoPiGo3가 모터에 전원을 공급하지 않음
2. **라이브러리 버전은 같음**: GoPiGo OS와 Ubuntu 모두 gopigo3.py v1.3.2 사용
3. **차이는 시스템 설정**: Power management 서비스, pigpiod, SPI 설정
4. **해결책**: GPIO 23 초기화 + Power management 서비스 설치

---

## 🎯 예상 결과

위의 작업을 완료하면:
- ✅ GPIO 23이 HIGH 상태로 유지됨
- ✅ GoPiGo3가 RPi 켜짐 상태로 인식
- ✅ 모터에 전원 공급
- ✅ 엔코더 값 변화 확인
- ✅ ROS2 `/cmd_vel` 명령으로 로봇 제어 가능

**핵심: GPIO 23 없이는 아무것도 작동하지 않습니다!**
