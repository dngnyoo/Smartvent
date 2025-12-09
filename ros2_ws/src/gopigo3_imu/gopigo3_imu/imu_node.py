#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
# GoPiGo3 전용 라이브러리 불러오기
from di_sensors.inertial_measurement_unit import InertialMeasurementUnit

class GoPiGoImuNode(Node):
    def __init__(self):
        super().__init__('gopigo_imu_node')
        self.publisher_ = self.create_publisher(Imu, 'imu', 10)
        self.timer = self.create_timer(0.05, self.timer_callback) # 0.05초 = 20Hz

        # 사용자 코드에 있던 RPI_1SW 방식 그대로 사용
        self.get_logger().info('GoPiGo3 IMU 연결 시도 중...')
        try:
            self.imu = InertialMeasurementUnit(bus="RPI_1")
            self.get_logger().info('IMU 연결 성공!')
        except Exception as e:
            self.get_logger().error(f'IMU 연결 실패: {e}')
            exit(1)

    def timer_callback(self):
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "imu_link"

        try:
            # 1. 쿼터니언 (방향)
            # BNO055는 오일러 각(도)을 줌 -> 라디안 변환 -> 쿼터니언 변환
            euler = self.imu.read_euler()
            # 순서: Heading(Yaw), Roll, Pitch
            yaw = math.radians(euler[0])
            roll = math.radians(euler[1])
            pitch = math.radians(euler[2])
            
            # 오일러 -> 쿼터니언 공식
            cy = math.cos(yaw * 0.5)
            sy = math.sin(yaw * 0.5)
            cp = math.cos(pitch * 0.5)
            sp = math.sin(pitch * 0.5)
            cr = math.cos(roll * 0.5)
            sr = math.sin(roll * 0.5)

            msg.orientation.w = cr * cp * cy + sr * sp * sy
            msg.orientation.x = sr * cp * cy - cr * sp * sy
            msg.orientation.y = cr * sp * cy + sr * cp * sy
            msg.orientation.z = cr * cp * sy - sr * sp * cy

            # 2. 각속도 (Gyro)
            gyro = self.imu.read_gyroscope()
            msg.angular_velocity.x = math.radians(gyro[0])
            msg.angular_velocity.y = math.radians(gyro[1])
            msg.angular_velocity.z = math.radians(gyro[2])

            # 3. 선형 가속도 (Accel)
            accel = self.imu.read_accelerometer()
            msg.linear_acceleration.x = accel[0]
            msg.linear_acceleration.y = accel[1]
            msg.linear_acceleration.z = accel[2]

            self.publisher_.publish(msg)

        except Exception as e:
            self.get_logger().warn(f'센서 읽기 실패: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = GoPiGoImuNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()