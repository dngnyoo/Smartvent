#!/usr/bin/env python3
"""
Patrol and Evacuation Node for GoPiGo3

Normal Mode: Patrol between Start Point and Exit 1
Emergency Mode: When gas level exceeds threshold at Exit 1,
                navigate to nearest exit (Exit 2 or Exit 3)

Waypoints are calculated relative to the robot's starting position.

Topics:
  - Subscribes: /gas_level (std_msgs/Float32)
  - Uses: Nav2 NavigateToPose action

Author: Claude Code
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rcl_interfaces.srv import GetParameters

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32
from action_msgs.msg import GoalStatus

from tf2_ros import Buffer, TransformListener, TransformException

import math
from enum import Enum


class RobotState(Enum):
    IDLE = 0
    GOING_TO_EXIT1 = 1
    GOING_TO_START = 2
    EVACUATING = 3
    EVACUATION_COMPLETE = 4


class PatrolEvacuationNode(Node):
    # Relative offsets from start position (in meters)
    RELATIVE_WAYPOINTS = {
        'start': {'dx': 0.0, 'dy': 0.0},
        'exit1': {'dx': 4.11, 'dy': -11.28},
        'exit2': {'dx': 3.92, 'dy': -13.55},
        'exit3': {'dx': 5.09, 'dy': 5.48},
    }

    def __init__(self):
        super().__init__('patrol_evacuation_node')

        # Callback group for concurrent execution
        self.callback_group = ReentrantCallbackGroup()

        # ========== TF Buffer for getting robot position ==========
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ========== Waypoints will be calculated from current position ==========
        self.waypoints = {}
        self.waypoints_initialized = False

        # ========== Parameters ==========
        self.declare_parameter('patrol_enabled', True)
        self.patrol_enabled = self.get_parameter('patrol_enabled').value

        # ========== Get gas threshold from gas_publisher node ==========
        self.gas_threshold = self.fetch_gas_threshold_from_publisher()

        # ========== State ==========
        self.state = RobotState.IDLE
        self.current_gas_level = 0.0
        self.emergency_triggered = False
        self.navigation_in_progress = False

        # ========== Nav2 Action Client ==========
        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose',
            callback_group=self.callback_group
        )

        # ========== Subscribers ==========
        self.gas_sub = self.create_subscription(
            Float32,
            'gas_level',
            self.gas_callback,
            10,
            callback_group=self.callback_group
        )

        # ========== Timer for patrol logic ==========
        self.patrol_timer = self.create_timer(
            1.0,
            self.patrol_loop,
            callback_group=self.callback_group
        )

        self.get_logger().info('=' * 50)
        self.get_logger().info('Patrol Evacuation Node Started')
        self.get_logger().info(f'Gas threshold: {self.gas_threshold}')
        self.get_logger().info('Waypoints will be calculated from current position')
        self.get_logger().info('=' * 50)

        # Wait for Nav2 action server
        self.get_logger().info('Waiting for Nav2 action server...')
        self.nav_client.wait_for_server()
        self.get_logger().info('Nav2 action server connected!')

        # Initialize waypoints from current position
        self.initialize_waypoints_from_current_position()

        # Start patrol if enabled
        if self.patrol_enabled and self.waypoints_initialized:
            self.get_logger().info('Starting patrol mode: Start <-> Exit 1')
            self.state = RobotState.GOING_TO_EXIT1

    def get_current_robot_position(self):
        """Get current robot position from TF (map -> base_footprint)"""
        try:
            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_footprint',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=5.0)
            )
            x = transform.transform.translation.x
            y = transform.transform.translation.y

            # Extract yaw from quaternion
            qz = transform.transform.rotation.z
            qw = transform.transform.rotation.w
            yaw = 2.0 * math.atan2(qz, qw)

            return x, y, yaw
        except TransformException as ex:
            self.get_logger().error(f'Could not get robot position: {ex}')
            return None, None, None

    def initialize_waypoints_from_current_position(self):
        """Initialize waypoints based on current robot position"""
        self.get_logger().info('Initializing waypoints from current position...')

        # Wait a bit for TF to be available
        for attempt in range(10):
            x, y, yaw = self.get_current_robot_position()
            if x is not None:
                break
            self.get_logger().info(f'Waiting for TF... (attempt {attempt + 1}/10)')
            rclpy.spin_once(self, timeout_sec=1.0)

        if x is None:
            self.get_logger().error('Failed to get current position. Using default (0, 0)')
            x, y, yaw = 0.0, 0.0, 0.0

        self.get_logger().info(f'Current robot position: ({x:.2f}, {y:.2f}), yaw: {yaw:.2f} rad')

        # Calculate absolute waypoints from relative offsets
        for name, offset in self.RELATIVE_WAYPOINTS.items():
            self.waypoints[name] = {
                'x': x + offset['dx'],
                'y': y + offset['dy'],
                'yaw': 0.0  # All waypoints face forward
            }

        self.waypoints_initialized = True

        self.get_logger().info('=' * 50)
        self.get_logger().info('Waypoints initialized (map frame):')
        for name, wp in self.waypoints.items():
            self.get_logger().info(f'  {name}: ({wp["x"]:.2f}, {wp["y"]:.2f})')
        self.get_logger().info('=' * 50)

    def gas_callback(self, msg: Float32):
        """Handle gas sensor readings"""
        self.current_gas_level = msg.data

        # Check for emergency condition at Exit 1
        if (self.state == RobotState.GOING_TO_EXIT1 or
            self.state == RobotState.GOING_TO_START):

            if self.current_gas_level > self.gas_threshold and not self.emergency_triggered:
                self.get_logger().warn('=' * 50)
                self.get_logger().warn(f'🚨 GAS ALERT! Level: {self.current_gas_level:.1f} > {self.gas_threshold}')
                self.get_logger().warn('🚨 INITIATING EMERGENCY EVACUATION!')
                self.get_logger().warn('=' * 50)

                self.emergency_triggered = True
                self.cancel_current_navigation()
                self.state = RobotState.EVACUATING

    def patrol_loop(self):
        """Main patrol logic loop"""
        if self.navigation_in_progress:
            return

        if self.state == RobotState.GOING_TO_EXIT1:
            self.get_logger().info('📍 Navigating to Exit 1...')
            self.navigate_to('exit1')

        elif self.state == RobotState.GOING_TO_START:
            self.get_logger().info('📍 Navigating to Start Point...')
            self.navigate_to('start')

        elif self.state == RobotState.EVACUATING:
            # Find nearest exit (Exit 2 or Exit 3) from current position (Exit 1 area)
            nearest_exit = self.find_nearest_exit()
            self.get_logger().warn(f'🚨 Evacuating to nearest exit: {nearest_exit}')
            self.navigate_to(nearest_exit)

        elif self.state == RobotState.EVACUATION_COMPLETE:
            self.get_logger().info('✅ Evacuation complete. Robot stopped.')
            # Stay at evacuation point - do nothing

    def find_nearest_exit(self) -> str:
        """Find nearest exit from Exit 1 (Exit 2 or Exit 3)"""
        exit1 = self.waypoints['exit1']
        exit2 = self.waypoints['exit2']
        exit3 = self.waypoints['exit3']

        dist_to_exit2 = math.sqrt(
            (exit1['x'] - exit2['x'])**2 + (exit1['y'] - exit2['y'])**2
        )
        dist_to_exit3 = math.sqrt(
            (exit1['x'] - exit3['x'])**2 + (exit1['y'] - exit3['y'])**2
        )

        self.get_logger().info(f'Distance to Exit 2: {dist_to_exit2:.2f}m')
        self.get_logger().info(f'Distance to Exit 3: {dist_to_exit3:.2f}m')

        if dist_to_exit2 <= dist_to_exit3:
            return 'exit2'
        else:
            return 'exit3'

    def navigate_to(self, waypoint_name: str):
        """Send navigation goal to Nav2"""
        if waypoint_name not in self.waypoints:
            self.get_logger().error(f'Unknown waypoint: {waypoint_name}')
            return

        waypoint = self.waypoints[waypoint_name]

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = waypoint['x']
        goal_msg.pose.pose.position.y = waypoint['y']
        goal_msg.pose.pose.position.z = 0.0

        # Convert yaw to quaternion (simplified, only yaw rotation)
        yaw = waypoint['yaw']
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.navigation_in_progress = True

        self.get_logger().info(f'Sending goal: {waypoint_name} ({waypoint["x"]:.2f}, {waypoint["y"]:.2f})')

        send_goal_future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self.navigation_feedback_callback
        )
        send_goal_future.add_done_callback(
            lambda future: self.goal_response_callback(future, waypoint_name)
        )

    def goal_response_callback(self, future, waypoint_name: str):
        """Handle goal acceptance/rejection"""
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error(f'Goal to {waypoint_name} was rejected!')
            self.navigation_in_progress = False
            return

        self.get_logger().info(f'Goal to {waypoint_name} accepted')

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda future: self.navigation_result_callback(future, waypoint_name)
        )

    def navigation_feedback_callback(self, feedback_msg):
        """Handle navigation feedback"""
        # Optional: log progress
        pass

    def navigation_result_callback(self, future, waypoint_name: str):
        """Handle navigation completion"""
        result = future.result()
        status = result.status

        self.navigation_in_progress = False

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'✅ Reached {waypoint_name}!')
            self.handle_waypoint_reached(waypoint_name)
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn(f'Navigation to {waypoint_name} was canceled')
        else:
            self.get_logger().error(f'Navigation to {waypoint_name} failed with status: {status}')
            # Retry logic could be added here

    def handle_waypoint_reached(self, waypoint_name: str):
        """Handle logic when a waypoint is reached"""
        if self.state == RobotState.EVACUATING:
            # Evacuation complete
            self.state = RobotState.EVACUATION_COMPLETE
            self.get_logger().warn('=' * 50)
            self.get_logger().warn(f'🏁 EVACUATION COMPLETE at {waypoint_name}')
            self.get_logger().warn('Robot will remain at safe location')
            self.get_logger().warn('=' * 50)

        elif self.state == RobotState.GOING_TO_EXIT1:
            # Reached Exit 1, check gas and go back to start
            self.get_logger().info(f'At Exit 1. Gas level: {self.current_gas_level:.1f}')

            if self.current_gas_level > self.gas_threshold:
                # Trigger evacuation
                self.emergency_triggered = True
                self.state = RobotState.EVACUATING
            else:
                # Continue patrol
                self.state = RobotState.GOING_TO_START

        elif self.state == RobotState.GOING_TO_START:
            # Reached start, go to Exit 1
            self.get_logger().info('At Start Point. Continuing patrol...')
            self.state = RobotState.GOING_TO_EXIT1

    def cancel_current_navigation(self):
        """Cancel any ongoing navigation"""
        self.get_logger().info('Canceling current navigation...')
        # Nav2 will handle the cancellation when new goal is sent

    def fetch_gas_threshold_from_publisher(self) -> float:
        """
        Fetch gas_threshold_voltage from gas_publisher node and convert to ROS message scale.
        gas_publisher publishes: msg.data = voltage * 100.0
        So threshold in voltage * 100 = threshold for comparison
        """
        default_threshold = 100.0  # Default: 1.0V * 100 = 100.0

        # Create service client to get parameters from gas_publisher
        client = self.create_client(
            GetParameters,
            '/gas_publisher/get_parameters'
        )

        # Wait for service with timeout
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn('gas_publisher node not available, using default threshold')
            return default_threshold

        # Request the gas_threshold_voltage parameter
        request = GetParameters.Request()
        request.names = ['gas_threshold_voltage']

        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() is not None:
            result = future.result()
            if len(result.values) > 0 and result.values[0].type != 0:
                voltage_threshold = result.values[0].double_value
                # Convert voltage to ROS message scale (voltage * 100)
                threshold = voltage_threshold * 100.0
                self.get_logger().info(f'Fetched gas threshold from gas_publisher: {voltage_threshold}V -> {threshold}')
                return threshold

        self.get_logger().warn('Failed to get gas_threshold_voltage, using default')
        return default_threshold


def main(args=None):
    rclpy.init(args=args)

    node = PatrolEvacuationNode()

    # Use multi-threaded executor for concurrent callbacks
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down patrol node...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
