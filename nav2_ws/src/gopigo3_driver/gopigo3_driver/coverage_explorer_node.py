#!/usr/bin/env python3
"""
Coverage Explorer Node for GoPiGo3

This node performs autonomous exploration with coverage patterns:
1. First tries frontier-based exploration (if explore_lite finds frontiers)
2. When no frontiers found, returns to start position
3. Performs zigzag coverage pattern based on known map boundaries
4. Repeats coverage 2 times to ensure complete mapping

This approach ensures the robot covers areas that frontier-based
exploration might miss.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import Bool

import numpy as np
import math
import time
from enum import Enum


class ExplorationState(Enum):
    WAITING = 0
    FRONTIER_EXPLORATION = 1
    RETURNING_HOME = 2
    COVERAGE_PATTERN = 3
    COMPLETED = 4


class CoverageExplorerNode(Node):
    def __init__(self):
        super().__init__('coverage_explorer')

        # Parameters
        self.declare_parameter('coverage_spacing', 0.5)  # meters between zigzag lines
        self.declare_parameter('coverage_margin', 0.3)   # margin from walls
        self.declare_parameter('coverage_passes', 2)     # number of coverage passes
        self.declare_parameter('frontier_timeout', 30.0) # seconds to wait for frontier progress
        self.declare_parameter('goal_tolerance', 0.3)    # meters

        self.coverage_spacing = self.get_parameter('coverage_spacing').value
        self.coverage_margin = self.get_parameter('coverage_margin').value
        self.coverage_passes = self.get_parameter('coverage_passes').value
        self.frontier_timeout = self.get_parameter('frontier_timeout').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value

        # State
        self.state = ExplorationState.WAITING
        self.start_pose = None
        self.current_pose = None
        self.map_data = None
        self.map_info = None
        self.coverage_waypoints = []
        self.current_waypoint_idx = 0
        self.coverage_pass = 0
        self.last_progress_time = time.time()
        self.last_position = None
        self.frontier_active = False

        # Callback group for concurrent callbacks
        self.callback_group = ReentrantCallbackGroup()

        # Nav2 action client
        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose',
            callback_group=self.callback_group
        )

        # Subscribers
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10,
            callback_group=self.callback_group
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odometry/filtered',
            self.odom_callback,
            10,
            callback_group=self.callback_group
        )

        # Subscribe to explore_lite status (if available)
        self.explore_resume_sub = self.create_subscription(
            Bool,
            '/explore/resume',
            self.explore_resume_callback,
            10,
            callback_group=self.callback_group
        )

        # Publisher for cmd_vel (for simple movements)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Timer for main logic
        self.timer = self.create_timer(1.0, self.main_loop)

        # Wait for Nav2
        self.get_logger().info('Waiting for Nav2 action server...')
        self.nav_client.wait_for_server()
        self.get_logger().info('Coverage Explorer ready!')

        # Start in frontier exploration mode
        self.state = ExplorationState.FRONTIER_EXPLORATION
        self.last_progress_time = time.time()

    def map_callback(self, msg):
        """Store map data for coverage planning"""
        self.map_data = np.array(msg.data).reshape(msg.info.height, msg.info.width)
        self.map_info = msg.info

    def odom_callback(self, msg):
        """Track robot position"""
        self.current_pose = msg.pose.pose

        # Store start position
        if self.start_pose is None:
            self.start_pose = msg.pose.pose
            self.get_logger().info(f'Start position stored: ({self.start_pose.position.x:.2f}, {self.start_pose.position.y:.2f})')

        # Check for progress (movement)
        if self.last_position is not None:
            dx = msg.pose.pose.position.x - self.last_position[0]
            dy = msg.pose.pose.position.y - self.last_position[1]
            dist = math.sqrt(dx*dx + dy*dy)
            if dist > 0.1:  # Moved more than 10cm
                self.last_progress_time = time.time()

        self.last_position = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def explore_resume_callback(self, msg):
        """Track explore_lite activity"""
        self.frontier_active = msg.data

    def main_loop(self):
        """Main state machine loop"""
        if self.current_pose is None or self.map_data is None:
            return

        if self.state == ExplorationState.FRONTIER_EXPLORATION:
            self.handle_frontier_exploration()
        elif self.state == ExplorationState.RETURNING_HOME:
            self.handle_returning_home()
        elif self.state == ExplorationState.COVERAGE_PATTERN:
            self.handle_coverage_pattern()
        elif self.state == ExplorationState.COMPLETED:
            pass  # Done

    def handle_frontier_exploration(self):
        """Monitor frontier exploration and switch to coverage if stuck"""
        time_since_progress = time.time() - self.last_progress_time

        if time_since_progress > self.frontier_timeout:
            self.get_logger().info(f'No progress for {self.frontier_timeout}s, switching to coverage mode')
            self.state = ExplorationState.RETURNING_HOME
            self.navigate_to_start()

    def handle_returning_home(self):
        """Check if we've returned to start"""
        if self.current_pose is None or self.start_pose is None:
            return

        dx = self.current_pose.position.x - self.start_pose.position.x
        dy = self.current_pose.position.y - self.start_pose.position.y
        dist = math.sqrt(dx*dx + dy*dy)

        if dist < self.goal_tolerance:
            self.get_logger().info('Returned to start, generating coverage pattern')
            self.generate_coverage_waypoints()
            self.state = ExplorationState.COVERAGE_PATTERN
            self.current_waypoint_idx = 0
            self.send_next_waypoint()

    def handle_coverage_pattern(self):
        """Execute coverage pattern waypoints"""
        if not self.coverage_waypoints:
            return

        if self.current_waypoint_idx >= len(self.coverage_waypoints):
            self.coverage_pass += 1
            self.get_logger().info(f'Coverage pass {self.coverage_pass} completed')

            if self.coverage_pass >= self.coverage_passes:
                self.get_logger().info('All coverage passes completed!')
                self.state = ExplorationState.COMPLETED
            else:
                # Start next pass
                self.current_waypoint_idx = 0
                self.coverage_waypoints.reverse()  # Go in opposite direction
                self.send_next_waypoint()

    def navigate_to_start(self):
        """Send navigation goal to start position"""
        if self.start_pose is None:
            return

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose = self.start_pose

        self.get_logger().info('Navigating to start position...')
        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(self.goal_response_callback)

    def generate_coverage_waypoints(self):
        """Generate zigzag coverage waypoints based on map"""
        if self.map_data is None or self.map_info is None:
            self.get_logger().warn('No map data for coverage planning')
            return

        # Find map bounds (free space only)
        free_cells = np.where(self.map_data == 0)  # 0 = free space
        if len(free_cells[0]) == 0:
            self.get_logger().warn('No free space found in map')
            return

        # Convert to world coordinates
        resolution = self.map_info.resolution
        origin_x = self.map_info.origin.position.x
        origin_y = self.map_info.origin.position.y

        min_row, max_row = np.min(free_cells[0]), np.max(free_cells[0])
        min_col, max_col = np.min(free_cells[1]), np.max(free_cells[1])

        min_x = origin_x + min_col * resolution + self.coverage_margin
        max_x = origin_x + max_col * resolution - self.coverage_margin
        min_y = origin_y + min_row * resolution + self.coverage_margin
        max_y = origin_y + max_row * resolution - self.coverage_margin

        self.get_logger().info(f'Coverage area: x=[{min_x:.2f}, {max_x:.2f}], y=[{min_y:.2f}, {max_y:.2f}]')

        # Generate zigzag pattern
        self.coverage_waypoints = []
        y = min_y
        direction = 1  # 1 = left to right, -1 = right to left

        while y <= max_y:
            if direction == 1:
                self.coverage_waypoints.append((min_x, y))
                self.coverage_waypoints.append((max_x, y))
            else:
                self.coverage_waypoints.append((max_x, y))
                self.coverage_waypoints.append((min_x, y))

            y += self.coverage_spacing
            direction *= -1

        self.get_logger().info(f'Generated {len(self.coverage_waypoints)} coverage waypoints')

    def send_next_waypoint(self):
        """Send the next coverage waypoint"""
        if self.current_waypoint_idx >= len(self.coverage_waypoints):
            return

        x, y = self.coverage_waypoints[self.current_waypoint_idx]

        # Check if waypoint is in free space
        if not self.is_point_free(x, y):
            self.get_logger().info(f'Skipping blocked waypoint ({x:.2f}, {y:.2f})')
            self.current_waypoint_idx += 1
            self.send_next_waypoint()
            return

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.w = 1.0

        self.get_logger().info(f'Coverage waypoint {self.current_waypoint_idx + 1}/{len(self.coverage_waypoints)}: ({x:.2f}, {y:.2f})')

        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(self.coverage_goal_response_callback)

    def is_point_free(self, x, y):
        """Check if a point is in free space on the map"""
        if self.map_data is None or self.map_info is None:
            return True

        # Convert world coordinates to map cell
        col = int((x - self.map_info.origin.position.x) / self.map_info.resolution)
        row = int((y - self.map_info.origin.position.y) / self.map_info.resolution)

        # Check bounds
        if row < 0 or row >= self.map_info.height or col < 0 or col >= self.map_info.width:
            return False

        # Check if free (0 = free, 100 = occupied, -1 = unknown)
        return self.map_data[row, col] == 0

    def goal_response_callback(self, future):
        """Handle goal acceptance for return home"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Return home goal rejected')
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.return_home_result_callback)

    def return_home_result_callback(self, future):
        """Handle return home completion"""
        result = future.result()
        self.get_logger().info('Return home navigation completed')
        # State transition happens in handle_returning_home based on position

    def coverage_goal_response_callback(self, future):
        """Handle goal acceptance for coverage waypoints"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn(f'Coverage waypoint {self.current_waypoint_idx} rejected, skipping')
            self.current_waypoint_idx += 1
            self.send_next_waypoint()
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.coverage_result_callback)

    def coverage_result_callback(self, future):
        """Handle coverage waypoint completion"""
        self.current_waypoint_idx += 1

        if self.current_waypoint_idx < len(self.coverage_waypoints):
            self.send_next_waypoint()
        else:
            # Pass completed, handled in main_loop
            pass


def main(args=None):
    rclpy.init(args=args)
    node = CoverageExplorerNode()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
