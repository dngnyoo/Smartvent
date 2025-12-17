import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')
        
        # 1. [명령] 운전기사(driver)에게 보낼 토픽 (/cmd_vel)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # 2. [감지] 센서 노드에서 받을 토픽 (/sensor/distance)
        self.sub_dist = self.create_subscription(
            Float32, 
            '/sensor/distance', 
            self.sensor_callback, 
            10
        )
        
        # 상태 변수
        self.safe_distance = 300.0  # 30cm (단위: mm)
        self.current_distance = 9999.0 # 초기값

    def sensor_callback(self, msg):
        self.current_distance = msg.data
        self.decide_move()

    def decide_move(self):
        cmd = Twist()
        
        if self.current_distance < self.safe_distance:
            # [장애물 발견] 멈추고 회전
            self.get_logger().warn(f'장애물 감지! ({self.current_distance:.1f}mm) 회피 중...')
            cmd.linear.x = 0.0
            cmd.angular.z = 1.0  # 왼쪽으로 회전 (속도 1.0 rad/s)
        else:
            # [안전] 전진
            cmd.linear.x = 0.1   # 앞으로 전진 (속도 0.1 m/s)
            cmd.angular.z = 0.0
        
        # 명령 전송
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # 종료 시 정지 명령
        node.cmd_pub.publish(Twist())
    finally:
        node.destroy_node()
        rclpy.shutdown()