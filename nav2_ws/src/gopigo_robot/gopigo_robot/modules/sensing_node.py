import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

# 사용자 모듈 가져오기 (경로에 맞게 수정 필요)
# from .modules.ultrasonic import Ultrasonic # 예시 클래스
from easygopigo3 import EasyGoPiGo3 # 센서 초기화용

class SensingNode(Node):
    def __init__(self):
        super().__init__('sensing_node')
        
        # 1. 하드웨어 연결 (센서용)
        try:
            self.gpg = EasyGoPiGo3()
            self.dist_sensor = self.gpg.init_distance_sensor()
            # self.mq2 = ... (가스 센서 등 추가 가능)
            self.get_logger().info("센서 노드 연결 성공")
        except Exception as e:
            self.get_logger().error(f"센서 연결 실패: {e}")
            return

        # 2. 토픽 발행 설정
        self.pub_dist = self.create_publisher(Float32, '/sensor/distance', 10)
        
        # 3. 주기적으로 센서 읽기 (0.1초마다)
        self.timer = self.create_timer(0.1, self.update_sensors)

    def update_sensors(self):
        try:
            # 거리 값 읽기
            mm = self.dist_sensor.read_mm()
            
            # ROS 메시지로 포장해서 발송
            msg = Float32()
            msg.data = float(mm)
            self.pub_dist.publish(msg)
            
        except Exception as e:
            self.get_logger().warn(f"센서 읽기 에러: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = SensingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()