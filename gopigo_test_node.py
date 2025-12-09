#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import RPi.GPIO as GPIO
import sys
import time

# GoPiGo3 라이브러리 경로 추가 (기존 유지)
sys.path.insert(0, '/home/ubuntu/GoPiGo3/Software/Python')
import gopigo3

class GoPiGoTestNode(Node):
    def __init__(self):
        super().__init__('gopigo_test_node')
        self.get_logger().info("=== GPIO 23 Power Keep-Alive Node Started ===")

        # 1. GPIO 23 활성화 (전원 유지)
        self.setup_gpio()

        # 2. GoPiGo3 초기화
        try:
            self.gpg = gopigo3.GoPiGo3()
            self.get_logger().info(f"✅ GoPiGo3 connected")
            self.get_logger().info(f"   Battery: {self.gpg.get_voltage_battery()}V")
            self.get_logger().info(f"   Firmware: {self.gpg.get_version_firmware()}")
        except Exception as e:
            self.get_logger().error(f"❌ GoPiGo3 Connection Failed: {e}")
            return

        # 3. Motor limits 설정
        try:
            self.gpg.set_motor_limits(self.gpg.MOTOR_LEFT + self.gpg.MOTOR_RIGHT, 100, 1000)
            self.get_logger().info("✅ Motor limits set")
        except IOError:
            self.get_logger().error("❌ Failed to set motor limits")

        # 4. 테스트 실행 (노드 시작 시 1회 실행)
        self.run_motor_test()

    def setup_gpio(self):
        try:
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(23, GPIO.OUT)
            GPIO.output(23, True)
            self.get_logger().info("✅ GPIO 23 set to HIGH")
        except Exception as e:
            self.get_logger().error(f"❌ GPIO Setup Failed: {e}")

    def run_motor_test(self):
        self.get_logger().info("\n=== Testing LEFT motor ===")
        
        # 모터 동작
        self.gpg.set_motor_dps(self.gpg.MOTOR_LEFT, 300)
        
        # ROS 2에서는 time.sleep 대신 루프를 막지 않는 방식을 선호하지만, 
        # 단순 테스트를 위해 여기서는 sleep을 사용합니다.
        time.sleep(2) 
        
        # 엔코더 값 읽기
        encoder = self.gpg.get_motor_encoder(self.gpg.MOTOR_LEFT)
        self.gpg.set_motor_dps(self.gpg.MOTOR_LEFT, 0) # 정지

        self.get_logger().info(f"Encoder value: {encoder}")
        
        if encoder != 0:
            self.get_logger().info("🎉 SUCCESS! Motor is working!")
        else:
            self.get_logger().warn("❌ FAILED! Motor still not working")

    def stop_robot(self):
        # 종료 시 안전하게 멈춤
        self.get_logger().info("Stopping GoPiGo3...")
        if hasattr(self, 'gpg'):
            self.gpg.reset_all()
        GPIO.cleanup()

def main(args=None):
    rclpy.init(args=args)
    
    node = GoPiGoTestNode()

    try:
        # 노드를 계속 실행 상태로 유지 (Ctrl+C를 누를 때까지)
        # 만약 테스트 후 바로 종료되길 원하면 이 줄을 지우셔도 됩니다.
        rclpy.spin(node) 
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()