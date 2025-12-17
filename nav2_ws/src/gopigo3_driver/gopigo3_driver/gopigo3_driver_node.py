#!/usr/bin/env python3
"""
GoPiGo3 ROS2 Driver Node for Ubuntu 22.04 + ROS2 Humble

This node provides:
- Motor control via /cmd_vel topic
- Wheel odometry via /odom topic
- Joint states via /joint_states topic
- TF transform from odom -> base_footprint

IMPORTANT: GPIO 23 must be HIGH for GoPiGo3 motors to work!
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32
from tf2_ros import TransformBroadcaster
import math
import time
import sys


class GoPiGo3DriverNode(Node):
    def __init__(self):
        super().__init__('gopigo3_driver')

        # ============================================
        # CRITICAL: GPIO 23 Power Keep-Alive Signal
        # Without this, GoPiGo3 motors will NOT work!
        # ============================================
        self._init_gpio23()

        # Parameters
        self.declare_parameter('wheel_base', 0.117)  # meters (117mm)
        self.declare_parameter('wheel_diameter', 0.0665)  # meters (66.5mm)
        self.declare_parameter('encoder_ticks_per_rotation', 6)  # magnetic positions
        self.declare_parameter('motor_gear_ratio', 120)  # gear ratio
        self.declare_parameter('publish_rate', 20.0)  # Hz
        self.declare_parameter('config_file_path', '/home/ubuntu/Dexter/gpg3_config.json')
        self.declare_parameter('publish_tf', True)  # Set to False when using EKF

        self.wheel_base = self.get_parameter('wheel_base').value
        self.wheel_diameter = self.get_parameter('wheel_diameter').value
        self.encoder_ticks_per_rotation = self.get_parameter('encoder_ticks_per_rotation').value
        self.motor_gear_ratio = self.get_parameter('motor_gear_ratio').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.config_file_path = self.get_parameter('config_file_path').value
        self.publish_tf_enabled = self.get_parameter('publish_tf').value

        # Derived parameters
        self.wheel_circumference = math.pi * self.wheel_diameter
        self.motor_ticks_per_degree = (self.motor_gear_ratio * self.encoder_ticks_per_rotation) / 360.0

        # Initialize GoPiGo3
        self._init_gopigo3()

        # Robot state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.left_encoder_prev = 0
        self.right_encoder_prev = 0
        self.last_time = self.get_clock().now()

        # Velocity state (for odometry)
        self.linear_vel = 0.0
        self.angular_vel = 0.0

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
        self.battery_pub = self.create_publisher(Float32, 'battery_voltage', 10)

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
        self.battery_timer = self.create_timer(5.0, self.publish_battery)  # Every 5 seconds

        # Motor limits for speed control
        try:
            self.gpg.set_motor_limits(
                self.gpg.MOTOR_LEFT + self.gpg.MOTOR_RIGHT,
                power=100,
                dps=1000
            )
        except Exception as e:
            self.get_logger().warn(f'Failed to set motor limits: {e}')

        self.get_logger().info('GoPiGo3 driver node started')
        self.get_logger().info(f'  Wheel base: {self.wheel_base} m')
        self.get_logger().info(f'  Wheel diameter: {self.wheel_diameter} m')
        self.get_logger().info(f'  Wheel circumference: {self.wheel_circumference:.4f} m')

    def _init_gpio23(self):
        """
        Initialize GPIO 23 to HIGH for GoPiGo3 power keep-alive.

        This is CRITICAL for motor operation. Without this, GoPiGo3 assumes
        the Raspberry Pi is shut down and disables motor power.
        """
        try:
            import RPi.GPIO as GPIO
            GPIO.setmode(GPIO.BCM)
            GPIO.setwarnings(False)
            GPIO.setup(23, GPIO.OUT)
            GPIO.output(23, True)
            self.get_logger().info('GPIO 23 set to HIGH (GoPiGo3 power keep-alive)')
            self.gpio_initialized = True
        except ImportError:
            self.get_logger().error('RPi.GPIO not available. Motors may not work!')
            self.get_logger().error('Install with: sudo apt install python3-rpi.gpio')
            self.gpio_initialized = False
        except Exception as e:
            self.get_logger().error(f'Failed to initialize GPIO 23: {e}')
            self.get_logger().error('Motors may not work without gpg3_power.service!')
            self.gpio_initialized = False

    def _init_gopigo3(self):
        """Initialize GoPiGo3 using the official library"""
        try:
            # Add GoPiGo3 library path
            sys.path.insert(0, '/home/ubuntu/GoPiGo3/Software/Python')
            import gopigo3

            # Initialize with config file (allows custom wheel parameters)
            self.gpg = gopigo3.GoPiGo3(config_file_path=self.config_file_path)

            # Log hardware info
            battery = self.gpg.get_voltage_battery()
            voltage_5v = self.gpg.get_voltage_5v()
            firmware = self.gpg.get_version_firmware()
            manufacturer = self.gpg.get_manufacturer()
            board = self.gpg.get_board()

            self.get_logger().info('GoPiGo3 initialized successfully')
            self.get_logger().info(f'  Manufacturer: {manufacturer}')
            self.get_logger().info(f'  Board: {board}')
            self.get_logger().info(f'  Firmware: {firmware}')
            self.get_logger().info(f'  Battery: {battery:.2f}V')
            self.get_logger().info(f'  5V Rail: {voltage_5v:.2f}V')

        except Exception as e:
            self.get_logger().error(f'Failed to initialize GoPiGo3: {e}')
            self.get_logger().error('Make sure:')
            self.get_logger().error('  1. SPI is enabled: ls /dev/spidev0.1')
            self.get_logger().error('  2. pigpiod is running: systemctl status pigpiod')
            self.get_logger().error('  3. GPIO 23 is HIGH (power keep-alive)')
            raise

    def cmd_vel_callback(self, msg):
        """Convert cmd_vel to motor speeds using differential drive kinematics"""
        linear_vel = msg.linear.x  # m/s
        angular_vel = msg.angular.z  # rad/s

        # Differential drive kinematics:
        # v_left = linear_vel - (angular_vel * wheel_base / 2)
        # v_right = linear_vel + (angular_vel * wheel_base / 2)
        v_left = linear_vel - (angular_vel * self.wheel_base / 2.0)
        v_right = linear_vel + (angular_vel * self.wheel_base / 2.0)

        # Convert m/s to degrees per second
        # dps = (velocity / wheel_circumference) * 360
        dps_left = (v_left / self.wheel_circumference) * 360.0
        dps_right = (v_right / self.wheel_circumference) * 360.0

        # Send to motors
        try:
            self.gpg.set_motor_dps(self.gpg.MOTOR_LEFT, int(dps_left))
            self.gpg.set_motor_dps(self.gpg.MOTOR_RIGHT, int(dps_right))

            # Debug logging (comment out for production)
            # self.get_logger().debug(
            #     f'cmd_vel: linear={linear_vel:.3f} angular={angular_vel:.3f} -> '
            #     f'dps_L={dps_left:.1f} dps_R={dps_right:.1f}'
            # )
        except Exception as e:
            self.get_logger().error(f'Failed to set motor speeds: {e}')

    def update(self):
        """Update odometry from wheel encoders and publish"""
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9

        if dt < 0.001:  # Skip if dt is too small
            return

        self.last_time = current_time

        try:
            # Read encoders (in degrees)
            left_encoder = self.gpg.get_motor_encoder(self.gpg.MOTOR_LEFT)
            right_encoder = self.gpg.get_motor_encoder(self.gpg.MOTOR_RIGHT)

            # Calculate encoder deltas (in degrees)
            delta_left = left_encoder - self.left_encoder_prev
            delta_right = right_encoder - self.right_encoder_prev

            # Handle encoder overflow (32-bit signed)
            if delta_left > 180000:
                delta_left -= 360000
            elif delta_left < -180000:
                delta_left += 360000
            if delta_right > 180000:
                delta_right -= 360000
            elif delta_right < -180000:
                delta_right += 360000

            self.left_encoder_prev = left_encoder
            self.right_encoder_prev = right_encoder

            # Convert encoder degrees to distance (meters)
            distance_left = (delta_left / 360.0) * self.wheel_circumference
            distance_right = (delta_right / 360.0) * self.wheel_circumference

            # Calculate robot movement
            distance = (distance_left + distance_right) / 2.0
            d_theta = (distance_right - distance_left) / self.wheel_base

            # Update pose using midpoint integration
            if abs(d_theta) < 1e-6:
                # Straight line motion
                self.x += distance * math.cos(self.theta)
                self.y += distance * math.sin(self.theta)
            else:
                # Arc motion
                radius = distance / d_theta
                self.x += radius * (math.sin(self.theta + d_theta) - math.sin(self.theta))
                self.y += radius * (-math.cos(self.theta + d_theta) + math.cos(self.theta))
                self.theta += d_theta

            # Normalize theta to [-pi, pi]
            self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

            # Calculate velocities
            self.linear_vel = distance / dt if dt > 0 else 0.0
            self.angular_vel = d_theta / dt if dt > 0 else 0.0

            # Publish odometry
            self.publish_odometry(current_time)

            # Publish joint states
            self.publish_joint_states(current_time, left_encoder, right_encoder)

        except Exception as e:
            self.get_logger().error(f'Update error: {e}')

    def publish_odometry(self, current_time):
        """Publish odometry message and TF"""
        # Create quaternion from theta (yaw)
        qz = math.sin(self.theta / 2.0)
        qw = math.cos(self.theta / 2.0)

        # Publish TF: odom -> base_footprint (only if not using EKF)
        if self.publish_tf_enabled:
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

        # Pose
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        # Pose covariance (diagonal: x, y, z, roll, pitch, yaw)
        # Higher values = less confidence
        odom.pose.covariance[0] = 0.01   # x
        odom.pose.covariance[7] = 0.01   # y
        odom.pose.covariance[14] = 1e6   # z (not measured)
        odom.pose.covariance[21] = 1e6   # roll (not measured)
        odom.pose.covariance[28] = 1e6   # pitch (not measured)
        odom.pose.covariance[35] = 0.03  # yaw

        # Twist (velocity)
        odom.twist.twist.linear.x = self.linear_vel
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = self.angular_vel

        # Twist covariance
        odom.twist.covariance[0] = 0.01   # linear.x
        odom.twist.covariance[7] = 1e6    # linear.y (not measured)
        odom.twist.covariance[14] = 1e6   # linear.z (not measured)
        odom.twist.covariance[21] = 1e6   # angular.x (not measured)
        odom.twist.covariance[28] = 1e6   # angular.y (not measured)
        odom.twist.covariance[35] = 0.03  # angular.z

        self.odom_pub.publish(odom)

    def publish_joint_states(self, current_time, left_encoder, right_encoder):
        """Publish joint states for robot_state_publisher"""
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

    def publish_battery(self):
        """Publish battery voltage"""
        try:
            voltage = self.gpg.get_voltage_battery()
            msg = Float32()
            msg.data = voltage
            self.battery_pub.publish(msg)

            # Warn if battery is low
            if voltage < 9.0:
                self.get_logger().warn(f'Low battery: {voltage:.2f}V')
        except Exception as e:
            self.get_logger().error(f'Failed to read battery voltage: {e}')

    def destroy_node(self):
        """Cleanup on shutdown"""
        self.get_logger().info('Shutting down GoPiGo3 driver...')
        try:
            # Stop motors
            self.gpg.set_motor_dps(self.gpg.MOTOR_LEFT, 0)
            self.gpg.set_motor_dps(self.gpg.MOTOR_RIGHT, 0)
            self.gpg.reset_all()
            self.get_logger().info('Motors stopped')
        except Exception as e:
            self.get_logger().error(f'Error stopping motors: {e}')

        # Note: We don't cleanup GPIO 23 here because gpg3_power.service should keep it HIGH
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

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
        if rclpy.ok():
            try:
                node.destroy_node()
            except:
                pass
            rclpy.shutdown()


if __name__ == '__main__':
    main()
