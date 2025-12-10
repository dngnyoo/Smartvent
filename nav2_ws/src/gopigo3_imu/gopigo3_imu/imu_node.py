#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from di_sensors.inertial_measurement_unit import InertialMeasurementUnit

class GoPiGoImuNode(Node):
    def __init__(self):
        super().__init__('gopigo_imu_node')
        self.publisher_ = self.create_publisher(Imu, 'imu', 10)
        self.timer = self.create_timer(0.1, self.timer_callback) # 10Hz로 조금 천천히

        self.get_logger().info('IMU (하드웨어 I2C) 연결 시도 중...')
        try:
            # [중요] RPI_1 (하드웨어) 사용
            self.imu = InertialMeasurementUnit(bus="RPI_1SW")
            self.get_logger().info('IMU 연결 성공!')
        except Exception as e:
            self.get_logger().error(f'IMU 초기화 실패: {e}')
            # 초기화 실패해도 노드가 죽지 않게 넘김 (재시도 로직은 복잡하므로 생략)

    def timer_callback(self):
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "imu_link"

        try:
            # 1. 데이터 읽기 (읽기 실패 시 None 반환 가능성 있음)
            euler = self.imu.read_euler()
            gyro = self.imu.read_gyroscope()
            accel = self.imu.read_accelerometer()

            # [안전장치] 데이터가 None이거나 비어있으면 건너뛰기
            if not euler or not gyro or not accel:
                return 

            # 2. 데이터 변환
            # Heading(Yaw), Roll, Pitch
            yaw = math.radians(euler[0])
            roll = math.radians(euler[1])
            pitch = math.radians(euler[2])
            
            # Quaternion 변환
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

            msg.angular_velocity.x = math.radians(gyro[0])
            msg.angular_velocity.y = math.radians(gyro[1])
            msg.angular_velocity.z = math.radians(gyro[2])

            msg.linear_acceleration.x = accel[0]
            msg.linear_acceleration.y = accel[1]
            msg.linear_acceleration.z = accel[2]

            self.publisher_.publish(msg)

        except Exception as e:
            # 가끔 나는 에러는 무시하고 로그만 띄움 (도배 방지 위해 debug 레벨 권장하지만 일단 warn으로)
            # self.get_logger().warn(f'센서 읽기 실패: {e}')
            pass # 너무 시끄러우면 pass로 변경

def main(args=None):
    rclpy.init(args=args)
    node = GoPiGoImuNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()