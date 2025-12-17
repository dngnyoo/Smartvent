#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32 # ROS 2 메시지 타입

# ================= [기존 코드] 라이브러리 =================
import pigpio
import time
import os
import sys

# ================= [기존 코드] 설정값 유지 =================
SDA_PIN = 17
SCL_PIN = 27
ADS_ADDR = 0x48
BAUD_RATE = 100000

REG_CONVERSION = 0x00
REG_CONFIG = 0x01
CONFIG_HI = 0xC2
CONFIG_LO = 0x83

GAS_THRESHOLD = 1.0  # volts (이 값을 넘으면 경고)

class GasPublisher(Node):
    def __init__(self):
        super().__init__('gas_publisher')
        
        # [ROS 2 추가] 퍼블리셔 생성 (토픽명: /gas_level)
        self.publisher_ = self.create_publisher(Float32, 'gas_level', 10)
        
        # [ROS 2 추가] 1초마다 센서를 읽도록 타이머 설정 (기존 while True 대체)
        self.timer = self.create_timer(1.0, self.timer_callback)

        # ================= [기존 코드] PIGPIO 초기화 =================
        self.pi = pigpio.pi()
        if not self.pi.connected:
            print("Failed to connect to pigpiod!")
            sys.exit(1)

        try:
            self.pi.bb_i2c_open(SDA_PIN, SCL_PIN, BAUD_RATE)
        except:
            self.pi.bb_i2c_close(SDA_PIN)
            self.pi.bb_i2c_open(SDA_PIN, SCL_PIN, BAUD_RATE)

        self.write_config()
        print("✅ Gas Sensor Node Started...")
        self.last_alert = 0
        self.ALERT_COOLDOWN = 5

    # ================= [기존 코드] 함수들 =================
    def speak_danger(self):
        os.system('espeak "Danger danger. Gas level is very high." --stdout | aplay >/dev/null 2>&1 &')

    def write_config(self):
        cmd = [4, ADS_ADDR, 2, 7, 3, REG_CONFIG, CONFIG_HI, CONFIG_LO, 3, 0]
        self.pi.bb_i2c_zip(SDA_PIN, cmd)

    def read_value(self):
        self.pi.bb_i2c_zip(SDA_PIN, [4, ADS_ADDR, 2, 7, 1, REG_CONVERSION, 3, 0])
        count, data = self.pi.bb_i2c_zip(SDA_PIN, [4, ADS_ADDR, 2, 6, 2, 3, 0])

        if count > 0 and len(data) >= 2:
            value = (data[0] << 8) | data[1]
            if value > 32767:
                value -= 65536
            return value
        return None

    # ================= [통합] 주기적으로 실행되는 함수 =================
    def timer_callback(self):
        raw_val = self.read_value()

        if raw_val is not None:
            voltage = raw_val * 0.000125
            
            # 1. [ROS 2] 화면 디스플레이로 값 보내기 (Publish)
            # 디스플레이는 50.0 이상일 때 경고를 띄웁니다.
            # 전압 1.0V를 기준으로 100을 곱해서 보냅니다. (1.0V -> 100.0)
            msg = Float32()
            msg.data = voltage * 100.0
            self.publisher_.publish(msg)
            
            print(f"Gas Value: {raw_val} | Voltage: {voltage:.2f}V -> ROS msg: {msg.data:.1f}")

            # 2. [기존 기능] 오디오 경고
            if voltage > GAS_THRESHOLD:
                print("🚨 DANGER: GAS LEVEL HIGH 🚨")
                if time.time() - self.last_alert > self.ALERT_COOLDOWN:
                    self.speak_danger()
                    self.last_alert = time.time()
        else:
            print("Error reading sensor")

    def cleanup(self):
        self.pi.bb_i2c_close(SDA_PIN)
        self.pi.stop()

def main(args=None):
    rclpy.init(args=args)
    node = GasPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()