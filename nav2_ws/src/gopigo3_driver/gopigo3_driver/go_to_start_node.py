#!/usr/bin/env python3
"""
Go To Start Node - 저장된 시작 위치로 자동 이동

이 노드는:
1. JSON 파일에서 맵핑 종료 위치를 로드하여 initial pose 설정 (로봇 현재 위치)
2. JSON 파일에서 맵핑 시작 위치 로드
3. Nav2 NavigateToPose 액션으로 시작 위치로 이동
4. 도착 후 자동 종료

사용법:
    ros2 run gopigo3_driver go_to_start --ros-args -p positions_file:=/home/ubuntu/maps/20231215_143022_positions.json

또는 맵 파일 경로로:
    ros2 run gopigo3_driver go_to_start --ros-args -p map_file:=/home/ubuntu/maps/20231215_143022
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from action_msgs.msg import GoalStatus

import json
import os
import math
import time


class GoToStartNode(Node):
    def __init__(self):
        super().__init__('go_to_start')

        # Parameters
        self.declare_parameter('positions_file', '')
        self.declare_parameter('map_file', '')

        positions_file = self.get_parameter('positions_file').value
        map_file = self.get_parameter('map_file').value

        # Determine positions file path
        if positions_file:
            self.positions_file = positions_file
        elif map_file:
            self.positions_file = f'{map_file}_positions.json'
        else:
            self.get_logger().error('Either positions_file or map_file parameter is required!')
            raise SystemExit(1)

        # Publishers
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            10
        )

        # Nav2 action client
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Position data
        self.start_position = None
        self.end_position = None

        self.get_logger().info('=' * 50)
        self.get_logger().info('Go To Start Node')
        self.get_logger().info(f'Positions file: {self.positions_file}')
        self.get_logger().info('=' * 50)

        # Load positions first
        if not self.load_positions():
            self.get_logger().error('Failed to load positions!')
            raise SystemExit(1)

        # Step 1: Set initial pose (where robot currently is = end position)
        self.create_timer(2.0, self.set_initial_pose_timer)

    def set_initial_pose_timer(self):
        """One-shot timer to set initial pose."""
        self.set_initial_pose()
        # After setting initial pose, wait then start navigation
        self.create_timer(3.0, self.start_navigation_timer)

    def start_navigation_timer(self):
        """One-shot timer to start navigation."""
        self.start_navigation()

    def set_initial_pose(self):
        """Set robot's initial pose to the end position (where it currently is)."""
        if self.end_position is None:
            self.get_logger().warn('No end position available, skipping initial pose')
            return

        x, y, yaw = self.end_position
        self.get_logger().info(f'Setting initial pose (end position): x={x:.2f}, y={y:.2f}, yaw={math.degrees(yaw):.1f}°')

        # Create initial pose message
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 0.0

        # Convert yaw to quaternion
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = math.sin(yaw / 2)
        msg.pose.pose.orientation.w = math.cos(yaw / 2)

        # Set covariance (small values = high confidence)
        # Covariance is 6x6 matrix flattened: [x, y, z, roll, pitch, yaw]
        msg.pose.covariance = [
            0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.07
        ]

        # Publish multiple times to ensure it's received
        for _ in range(3):
            self.initial_pose_pub.publish(msg)
            time.sleep(0.1)

        self.get_logger().info('Initial pose set!')

    def start_navigation(self):
        """Start navigation to start position."""
        if self.start_position is None:
            self.get_logger().error('No start position available!')
            raise SystemExit(1)

        x, y, yaw = self.start_position
        self.get_logger().info(f'Target start position: x={x:.2f}, y={y:.2f}, yaw={math.degrees(yaw):.1f}°')

        # Wait for Nav2 action server
        self.get_logger().info('Waiting for Nav2 action server...')
        if not self.nav_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Nav2 action server not available!')
            raise SystemExit(1)

        # Create goal
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.position.z = 0.0

        # Convert yaw to quaternion
        goal.pose.pose.orientation.x = 0.0
        goal.pose.pose.orientation.y = 0.0
        goal.pose.pose.orientation.z = math.sin(yaw / 2)
        goal.pose.pose.orientation.w = math.cos(yaw / 2)

        # Send goal
        self.get_logger().info('Navigating to start position...')
        send_goal_future = self.nav_client.send_goal_async(
            goal,
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)

    def load_positions(self):
        """Load start and end positions from JSON file."""
        if not os.path.exists(self.positions_file):
            self.get_logger().error(f'Positions file not found: {self.positions_file}')
            return False

        try:
            with open(self.positions_file, 'r') as f:
                data = json.load(f)

            # Try new format first (from map_saver_node)
            start_map = data.get('start_position', {}).get('map_frame', {})
            if start_map:
                x = start_map.get('x')
                y = start_map.get('y')
                yaw = start_map.get('yaw', 0.0) or 0.0
                if x is not None and y is not None:
                    self.start_position = (x, y, yaw)
                    self.get_logger().info(f'Loaded start position: ({x:.2f}, {y:.2f})')

            # Try old format (from wall_following_explorer)
            if self.start_position is None:
                start_map = data.get('start_position_map', {})
                x = start_map.get('x')
                y = start_map.get('y')
                if x is not None and y is not None:
                    self.start_position = (x, y, 0.0)
                    self.get_logger().info(f'Loaded start position (old format): ({x:.2f}, {y:.2f})')

            # Load end position (where robot currently is)
            # Try new format first
            end_map = data.get('end_position', {}).get('map_frame', {})
            if end_map:
                x = end_map.get('x')
                y = end_map.get('y')
                yaw = end_map.get('yaw', 0.0) or 0.0
                if x is not None and y is not None:
                    self.end_position = (x, y, yaw)
                    self.get_logger().info(f'Loaded end position: ({x:.2f}, {y:.2f})')

            # Try old format
            if self.end_position is None:
                end_map = data.get('end_position_map', {})
                x = end_map.get('x')
                y = end_map.get('y')
                if x is not None and y is not None:
                    self.end_position = (x, y, 0.0)
                    self.get_logger().info(f'Loaded end position (old format): ({x:.2f}, {y:.2f})')

            if self.start_position is None:
                self.get_logger().error('Could not find start position in JSON')
                return False

            return True

        except json.JSONDecodeError as e:
            self.get_logger().error(f'Invalid JSON: {e}')
            return False
        except Exception as e:
            self.get_logger().error(f'Error loading positions: {e}')
            return False

    def feedback_callback(self, feedback_msg):
        """Navigation feedback."""
        feedback = feedback_msg.feedback
        remaining = feedback.distance_remaining
        self.get_logger().info(f'Distance remaining: {remaining:.2f}m')

    def goal_response_callback(self, future):
        """Handle goal acceptance/rejection."""
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Navigation goal rejected!')
            raise SystemExit(1)

        self.get_logger().info('Navigation goal accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        """Handle navigation result."""
        result = future.result()
        status = result.status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('=' * 50)
            self.get_logger().info('ARRIVED AT START POSITION!')
            self.get_logger().info('=' * 50)
            self.get_logger().info('Robot is now at the mapping start location.')
            self.get_logger().info('')
            self.get_logger().info('You can now send custom goals:')
            self.get_logger().info('  ros2 run gopigo3_driver goal_sender')
            self.get_logger().info('=' * 50)
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn('Navigation was canceled')
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().error('Navigation aborted - could not reach goal')
        else:
            self.get_logger().warn(f'Navigation finished with status: {status}')

        # Shutdown node
        raise SystemExit(0)


def main(args=None):
    rclpy.init(args=args)
    node = GoToStartNode()

    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    except KeyboardInterrupt:
        node.get_logger().info('Navigation canceled by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
