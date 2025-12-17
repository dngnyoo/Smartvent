#!/usr/bin/env python3
"""
Gas Simulator Node for Testing

This node simulates gas detection for testing the gas obstacle avoidance system
before the real MQ-2 sensor is connected.

Usage:
    1. Run this node
    2. Press 'g' in terminal to trigger a gas detection at current robot position
    3. Press 'c' to clear all gas obstacles
    4. Or use ROS2 service calls

Topics Published:
    /gas_detection (std_msgs/Float32): Simulated gas concentration

Services:
    /simulate_gas: Trigger gas detection
    /simulate_gas_at_pose: Trigger gas at specific location

Author: GoPiGo3 Team
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
from std_srvs.srv import Empty, SetBool
from geometry_msgs.msg import PoseStamped
import sys
import select
import termios
import tty
import threading


class GasSimulatorNode(Node):
    """
    Simulates gas sensor readings for testing.

    This allows testing the gas avoidance system without the physical MQ-2 sensor.
    """

    def __init__(self):
        super().__init__('gas_simulator_node')

        # Parameters
        self.declare_parameter('simulated_concentration', 500.0)  # ppm when triggered
        self.declare_parameter('publish_rate', 10.0)  # Hz

        self.simulated_concentration = self.get_parameter('simulated_concentration').value
        self.publish_rate = self.get_parameter('publish_rate').value

        # State
        self.gas_detected = False
        self.current_concentration = 0.0

        # Publishers
        self.gas_pub = self.create_publisher(Float32, '/gas_detection', 10)
        self.gas_bool_pub = self.create_publisher(Bool, '/gas_detected', 10)

        # Services
        self.trigger_srv = self.create_service(
            Empty,
            '/simulate_gas',
            self.trigger_gas_callback
        )

        self.toggle_srv = self.create_service(
            SetBool,
            '/toggle_gas',
            self.toggle_gas_callback
        )

        # Timer for continuous publishing
        self.timer = self.create_timer(
            1.0 / self.publish_rate,
            self.publish_gas_reading
        )

        # Keyboard input thread
        self.running = True
        self.keyboard_thread = threading.Thread(target=self.keyboard_listener)
        self.keyboard_thread.daemon = True
        self.keyboard_thread.start()

        self.get_logger().info('=== Gas Simulator Node Started ===')
        self.get_logger().info('Press "g" to trigger gas detection')
        self.get_logger().info('Press "t" to toggle continuous gas mode')
        self.get_logger().info('Press "c" to clear (call clear_gas_obstacles service)')
        self.get_logger().info('Press "q" to quit')
        self.get_logger().info('==================================')

    def publish_gas_reading(self):
        """Publish current gas concentration"""
        msg = Float32()
        msg.data = self.current_concentration
        self.gas_pub.publish(msg)

        bool_msg = Bool()
        bool_msg.data = self.gas_detected
        self.gas_bool_pub.publish(bool_msg)

    def trigger_gas_callback(self, request, response):
        """Service callback to trigger single gas detection"""
        self.trigger_gas_pulse()
        return response

    def toggle_gas_callback(self, request, response):
        """Service callback to toggle continuous gas detection"""
        self.gas_detected = request.data
        if self.gas_detected:
            self.current_concentration = self.simulated_concentration
            self.get_logger().warn(f'⚠️  Gas detection ENABLED - concentration: {self.current_concentration}')
        else:
            self.current_concentration = 0.0
            self.get_logger().info('Gas detection DISABLED')
        response.success = True
        response.message = f'Gas detection {"enabled" if self.gas_detected else "disabled"}'
        return response

    def trigger_gas_pulse(self):
        """Trigger a single gas detection pulse"""
        self.get_logger().warn(f'⚠️  TRIGGERING GAS DETECTION: {self.simulated_concentration} ppm')
        self.current_concentration = self.simulated_concentration
        self.gas_detected = True

        # Publish immediately
        msg = Float32()
        msg.data = self.current_concentration
        self.gas_pub.publish(msg)

        # Reset after a short delay
        self.create_timer(0.5, self.reset_gas_pulse, callback_group=None)

    def reset_gas_pulse(self):
        """Reset gas after pulse"""
        self.current_concentration = 0.0
        self.gas_detected = False
        self.get_logger().info('Gas pulse ended')

    def keyboard_listener(self):
        """Listen for keyboard input"""
        # Save terminal settings
        try:
            old_settings = termios.tcgetattr(sys.stdin)
        except:
            self.get_logger().warn('Cannot access terminal for keyboard input')
            return

        try:
            tty.setcbreak(sys.stdin.fileno())

            while self.running and rclpy.ok():
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    key = sys.stdin.read(1).lower()

                    if key == 'g':
                        # Trigger gas detection
                        self.trigger_gas_pulse()

                    elif key == 't':
                        # Toggle continuous gas
                        self.gas_detected = not self.gas_detected
                        if self.gas_detected:
                            self.current_concentration = self.simulated_concentration
                            self.get_logger().warn(f'⚠️  Continuous gas mode ON: {self.current_concentration} ppm')
                        else:
                            self.current_concentration = 0.0
                            self.get_logger().info('Continuous gas mode OFF')

                    elif key == 'c':
                        # Clear gas obstacles
                        self.get_logger().info('Requesting clear gas obstacles...')
                        self.call_clear_service()

                    elif key == 'q':
                        self.get_logger().info('Quitting...')
                        self.running = False
                        rclpy.shutdown()
                        break

        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

    def call_clear_service(self):
        """Call the clear gas obstacles service"""
        client = self.create_client(Empty, '/clear_gas_obstacles')
        if client.wait_for_service(timeout_sec=1.0):
            future = client.call_async(Empty.Request())
            self.get_logger().info('Clear service called')
        else:
            self.get_logger().warn('Clear service not available')

    def destroy_node(self):
        self.running = False
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GasSimulatorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
