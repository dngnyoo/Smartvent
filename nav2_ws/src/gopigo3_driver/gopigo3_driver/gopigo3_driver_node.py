#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
import math
import time
import sys

# Add GoPiGo3 Python library path
sys.path.insert(0, '/home/ubuntu/GoPiGo3/Software/Python')
import gopigo3


class GoPiGo3DriverNode(Node):
    def __init__(self):
        super().__init__('gopigo3_driver')

        # CRITICAL: Initialize GPIO 23 for GoPiGo3 power keep-alive
        # Without this, GoPiGo3 board thinks RPi is shutdown and won't power motors
        try:
            import RPi.GPIO as GPIO
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(23, GPIO.OUT)
            GPIO.output(23, True)
            self.get_logger().info('GPIO 23 set to HIGH (GoPiGo3 power keep-alive)')
        except Exception as e:
            self.get_logger().warn(f'Failed to initialize GPIO 23: {e}')
            self.get_logger().warn('Make sure gpg3_power.service is running for reliable operation')

        # Parameters
        self.declare_parameter('wheel_base', 0.117)  # meters (distance between wheels)
        self.declare_parameter('wheel_diameter', 0.066)  # meters
        self.declare_parameter('encoder_ticks_per_rotation', 360)  # GoPiGo3 uses 360 degrees
        self.declare_parameter('publish_rate', 20.0)  # Hz

        self.wheel_base = self.get_parameter('wheel_base').value
        self.wheel_diameter = self.get_parameter('wheel_diameter').value
        self.encoder_ticks_per_rotation = self.get_parameter('encoder_ticks_per_rotation').value
        self.publish_rate = self.get_parameter('publish_rate').value

        # Initialize GoPiGo3
        try:
            self.gpg = gopigo3.GoPiGo3()
            self.get_logger().info('GoPiGo3 initialized successfully')

            # Log hardware info
            try:
                manufacturer = self.gpg.get_manufacturer()
                board = self.gpg.get_board()
                voltage = self.gpg.get_voltage_battery()
                self.get_logger().info(f'Manufacturer: {manufacturer}')
                self.get_logger().info(f'Board: {board}')
                self.get_logger().info(f'Battery: {voltage:.2f}V')
            except Exception as e:
                self.get_logger().warn(f'Could not read hardware info: {e}')

            # CRITICAL: Set motor limits BEFORE using motors
            # Without this, motors will not respond to commands!
            try:
                # Set PWM limit to 100% and DPS limit to 1000
                self.gpg.set_motor_limits(self.gpg.MOTOR_LEFT + self.gpg.MOTOR_RIGHT, 100, 1000)
                self.get_logger().info('Motor limits set: PWM=100%, DPS=1000')
            except Exception as e:
                self.get_logger().error(f'Failed to set motor limits: {e}')
                raise

            # Reset encoders to zero
            try:
                left_enc = self.gpg.get_motor_encoder(self.gpg.MOTOR_LEFT)
                right_enc = self.gpg.get_motor_encoder(self.gpg.MOTOR_RIGHT)
                self.gpg.offset_motor_encoder(self.gpg.MOTOR_LEFT, left_enc)
                self.gpg.offset_motor_encoder(self.gpg.MOTOR_RIGHT, right_enc)
                self.get_logger().info('Motor encoders reset to zero')
            except Exception as e:
                self.get_logger().warn(f'Could not reset encoders: {e}')

        except Exception as e:
            self.get_logger().error(f'Failed to initialize GoPiGo3: {e}')
            raise

        # Robot state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.left_encoder_prev = 0
        self.right_encoder_prev = 0
        self.last_time = self.get_clock().now()

        # Initialize encoders
        try:
            self.left_encoder_prev = self.gpg.get_motor_encoder(self.gpg.MOTOR_LEFT)
            self.right_encoder_prev = self.gpg.get_motor_encoder(self.gpg.MOTOR_RIGHT)
            self.get_logger().info(f'Initial encoders - Left: {self.left_encoder_prev}, Right: {self.right_encoder_prev}')
        except Exception as e:
            self.get_logger().warn(f'Failed to read initial encoders: {e}')

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', 10)

        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # Timers
        self.timer = self.create_timer(1.0 / self.publish_rate, self.update)

        self.get_logger().info('GoPiGo3 driver node started')

    def cmd_vel_callback(self, msg):
        """Convert cmd_vel to motor speeds"""
        # Extract linear and angular velocities
        linear_vel = msg.linear.x  # m/s
        angular_vel = msg.angular.z  # rad/s

        # Calculate wheel velocities using differential drive kinematics
        # v_left = linear_vel - (angular_vel * wheel_base / 2)
        # v_right = linear_vel + (angular_vel * wheel_base / 2)
        v_left = linear_vel - (angular_vel * self.wheel_base / 2.0)
        v_right = linear_vel + (angular_vel * self.wheel_base / 2.0)

        # Convert m/s to degrees per second
        # wheel_circumference = pi * diameter
        # rotations_per_second = velocity / circumference
        # degrees_per_second = rotations_per_second * 360
        wheel_circumference = math.pi * self.wheel_diameter

        dps_left = (v_left / wheel_circumference) * 360.0
        dps_right = (v_right / wheel_circumference) * 360.0

        # Send to motors
        try:
            self.get_logger().info(f'cmd_vel: linear={linear_vel:.2f}, angular={angular_vel:.2f} -> dps_left={dps_left:.2f}, dps_right={dps_right:.2f}')
            self.gpg.set_motor_dps(self.gpg.MOTOR_LEFT, dps_left)
            self.gpg.set_motor_dps(self.gpg.MOTOR_RIGHT, dps_right)
        except Exception as e:
            self.get_logger().error(f'Failed to set motor speeds: {e}')

    def update(self):
        """Update odometry and publish"""
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        try:
            # Read encoders
            left_encoder = self.gpg.get_motor_encoder(self.gpg.MOTOR_LEFT)
            right_encoder = self.gpg.get_motor_encoder(self.gpg.MOTOR_RIGHT)

            # Calculate encoder deltas (in degrees)
            delta_left = left_encoder - self.left_encoder_prev
            delta_right = right_encoder - self.right_encoder_prev

            self.left_encoder_prev = left_encoder
            self.right_encoder_prev = right_encoder

            # Convert encoder ticks to distance
            wheel_circumference = math.pi * self.wheel_diameter
            distance_left = (delta_left / 360.0) * wheel_circumference
            distance_right = (delta_right / 360.0) * wheel_circumference

            # Calculate velocities
            v_left = distance_left / dt if dt > 0 else 0.0
            v_right = distance_right / dt if dt > 0 else 0.0

            # Calculate robot velocities
            v = (distance_left + distance_right) / 2.0
            omega = (distance_right - distance_left) / self.wheel_base

            # Update pose
            if abs(omega) < 1e-6:
                # Straight line motion
                self.x += v * math.cos(self.theta)
                self.y += v * math.sin(self.theta)
            else:
                # Arc motion
                radius = v / omega
                self.x += radius * (math.sin(self.theta + omega) - math.sin(self.theta))
                self.y += -radius * (math.cos(self.theta + omega) - math.cos(self.theta))
                self.theta += omega

            # Normalize theta
            self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

            # Publish odometry
            self.publish_odometry(current_time, v / dt if dt > 0 else 0.0, omega / dt if dt > 0 else 0.0)

            # Publish joint states
            self.publish_joint_states(current_time, left_encoder, right_encoder)

        except Exception as e:
            self.get_logger().error(f'Update error: {e}')

    def publish_odometry(self, current_time, linear_vel, angular_vel):
        """Publish odometry message"""
        # Create quaternion from theta
        qz = math.sin(self.theta / 2.0)
        qw = math.cos(self.theta / 2.0)

        # Publish TF
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(t)

        # Publish odometry message
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        odom.twist.twist.linear.x = linear_vel
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = angular_vel

        self.odom_pub.publish(odom)

    def publish_joint_states(self, current_time, left_encoder, right_encoder):
        """Publish joint states"""
        joint_state = JointState()
        joint_state.header.stamp = current_time.to_msg()
        joint_state.name = ['left_wheel_joint', 'right_wheel_joint']

        # Convert encoder degrees to radians
        left_pos = math.radians(left_encoder)
        right_pos = math.radians(right_encoder)

        joint_state.position = [left_pos, right_pos]
        joint_state.velocity = []
        joint_state.effort = []

        self.joint_state_pub.publish(joint_state)

    def destroy_node(self):
        """Cleanup"""
        self.get_logger().info('Stopping motors...')
        try:
            self.gpg.reset_all()
        except:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = GoPiGo3DriverNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
        import traceback
        traceback.print_exc()
    finally:
        if node is not None:
            try:
                node.destroy_node()
            except:
                pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
