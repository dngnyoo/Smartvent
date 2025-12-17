import rclpy
from rclpy.node import Node
from easygopigo3 import EasyGoPiGo3
import time

class UltrasonicTestNode(Node):
    def __init__(self):
        super().__init__('ultrasonic_test_node')
        
        # 1. GoPiGo3 객체 및 센서 초기화
        try:
            self.gpg = EasyGoPiGo3()
            self.dist_sensor = self.gpg.init_distance_sensor()
            self.get_logger().info("GoPiGo3 & Distance Sensor Connected!")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to GoPiGo3: {e}")
            return

        # 2. 0.5초마다 센서 값을 읽는 타이머 설정
        self.timer = self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        # 센서값 읽기 (단위: mm)
        try:
            distance_mm = self.dist_sensor.read_mm()
            
            # 터미널에 로그 출력
            self.get_logger().info(f'Distance: {distance_mm} mm')
            
        except Exception as e:
            self.get_logger().warn(f"Sensor read error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicTestNode()
    
    try:
        # 노드 실행 (Ctrl+C를 누를 때까지 계속 실행)
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Test Stopped by User')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()