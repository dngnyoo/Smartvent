import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from easygopigo3 import EasyGoPiGo3
import time

class ActionNode(Node):
    def __init__(self):
        super().__init__('action_node')
        
        # 1. 하드웨어 연결
        try:
            self.gpg = EasyGoPiGo3()
            self.gpg.reset_all()
            self.get_logger().info("LED 액션 노드 시작 (유지시간: 1.5초)")
        except Exception as e:
            self.get_logger().error(f"하드웨어 없음: {e}")
            self.gpg = None

        # 2. 상태 변수
        self.last_turn_cmd = 0   # 1:왼쪽, -1:오른쪽, 0:없음
        self.last_cmd_time = 0.0 # 마지막으로 회전 명령 받은 시각
        
        # [설정] 이 시간을 조절해서 LED 켜지는 길이를 바꿉니다.
        self.keep_alive_time = 1.5  # 회전 명령 끊겨도 1.5초간 유지

        # 3. 토픽 구독
        self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)

        # 4. LED 관리 타이머 (0.1초마다 실행)
        self.create_timer(0.1, self.update_led)

    def cmd_callback(self, msg):
        # 왼쪽 회전 명령 감지
        if msg.angular.z > 0.1:
            self.last_turn_cmd = 1
            self.last_cmd_time = time.time() # 시간 갱신 (도장 쾅!)
            
        # 오른쪽 회전 명령 감지
        elif msg.angular.z < -0.1:
            self.last_turn_cmd = -1
            self.last_cmd_time = time.time() # 시간 갱신 (도장 쾅!)

        # 직진 명령이 오면 시간을 갱신하지 않음 -> 자연스럽게 타이머에 의해 꺼짐

    def update_led(self):
        if self.gpg is None:
            return

        now = time.time()
        time_diff = now - self.last_cmd_time

        # [핵심] 마지막 회전 명령 후 1.5초가 아직 안 지났다면?
        if time_diff < self.keep_alive_time:
            
            # 깜빡이 효과 (0.2초 간격으로 켜졌다 꺼졌다 함)
            # 현재 시간을 0.2로 나눈 몫이 짝수면 ON, 홀수면 OFF
            should_be_on = int(now / 0.2) % 2 == 0

            if should_be_on:
                if self.last_turn_cmd == 1:   # 왼쪽 켜기
                    self.gpg.led_on(1)
                    self.gpg.led_off(0)
                elif self.last_turn_cmd == -1: # 오른쪽 켜기
                    self.gpg.led_off(1)
                    self.gpg.led_on(0)
            else:
                # 잠깐 꺼짐 (깜빡이는 효과)
                self.gpg.led_off(1)
                self.gpg.led_off(0)
                
        else:
            # 1.5초 지남 -> 완전히 끄기
            self.gpg.led_off(1)
            self.gpg.led_off(0)
            self.last_turn_cmd = 0

def main(args=None):
    rclpy.init(args=args)
    node = ActionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node.gpg:
            node.gpg.led_off(0)
            node.gpg.led_off(1)
    finally:
        node.destroy_node()
        rclpy.shutdown()