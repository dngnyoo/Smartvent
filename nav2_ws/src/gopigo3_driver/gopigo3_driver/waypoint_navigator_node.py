#!/usr/bin/env python3
"""
Waypoint Navigator Node - Collect waypoints via clicks and navigate in patterns.

Features:
1. COLLECT mode: Click on map to save waypoints (start point is always 0,0)
2. NAVIGATE mode: Automatically navigate through waypoints in zigzag pattern
3. Repeats the cycle a configurable number of times

Usage:
    ros2 run gopigo3_driver waypoint_navigator

Services:
    /waypoint_navigator/start_collecting - Start collecting waypoints from clicks
    /waypoint_navigator/stop_collecting  - Stop collecting and show saved waypoints
    /waypoint_navigator/start_navigation - Start navigating through waypoints
    /waypoint_navigator/stop_navigation  - Stop navigation
    /waypoint_navigator/clear_waypoints  - Clear all saved waypoints
    /waypoint_navigator/list_waypoints   - List all saved waypoints

Parameters:
    ~/repeat_cycles: Number of times to repeat the full pattern (default: 1, 0=infinite)
    ~/return_to_start: Return to start point after completing waypoints (default: true)
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger, Empty
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
import json
import os


class WaypointNavigatorNode(Node):
    def __init__(self):
        super().__init__('waypoint_navigator')

        # Callback group for concurrent callbacks
        self.callback_group = ReentrantCallbackGroup()

        # Parameters
        self.declare_parameter('repeat_cycles', 1)  # 0 = infinite
        self.declare_parameter('return_to_start', True)
        self.declare_parameter('waypoints_file', '/home/ubuntu/waypoints.json')

        self.repeat_cycles = self.get_parameter('repeat_cycles').value
        self.return_to_start = self.get_parameter('return_to_start').value
        self.waypoints_file = self.get_parameter('waypoints_file').value

        # State
        self.collecting = False
        self.navigating = False
        self.waypoints = []  # List of (x, y, yaw) tuples
        self.current_waypoint_index = 0
        self.current_cycle = 0

        # Start point is always origin (where robot started mapping)
        self.start_point = (0.0, 0.0, 0.0)  # x, y, yaw

        # Nav2 action client
        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose',
            callback_group=self.callback_group
        )

        # Subscribe to goal clicks (from Foxglove via goal_relay)
        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10,
            callback_group=self.callback_group
        )

        # Services
        self.create_service(Trigger, '~/start_collecting', self.start_collecting_cb)
        self.create_service(Trigger, '~/stop_collecting', self.stop_collecting_cb)
        self.create_service(Trigger, '~/start_navigation', self.start_navigation_cb)
        self.create_service(Trigger, '~/stop_navigation', self.stop_navigation_cb)
        self.create_service(Trigger, '~/clear_waypoints', self.clear_waypoints_cb)
        self.create_service(Trigger, '~/list_waypoints', self.list_waypoints_cb)
        self.create_service(Trigger, '~/save_waypoints', self.save_waypoints_cb)
        self.create_service(Trigger, '~/load_waypoints', self.load_waypoints_cb)

        # Load existing waypoints if available
        self.load_waypoints_from_file()

        self.get_logger().info('=' * 50)
        self.get_logger().info('Waypoint Navigator Node Started')
        self.get_logger().info('=' * 50)
        self.get_logger().info('Services:')
        self.get_logger().info('  ros2 service call /waypoint_navigator/start_collecting std_srvs/srv/Trigger')
        self.get_logger().info('  ros2 service call /waypoint_navigator/stop_collecting std_srvs/srv/Trigger')
        self.get_logger().info('  ros2 service call /waypoint_navigator/start_navigation std_srvs/srv/Trigger')
        self.get_logger().info('  ros2 service call /waypoint_navigator/stop_navigation std_srvs/srv/Trigger')
        self.get_logger().info('  ros2 service call /waypoint_navigator/list_waypoints std_srvs/srv/Trigger')
        self.get_logger().info('  ros2 service call /waypoint_navigator/clear_waypoints std_srvs/srv/Trigger')
        self.get_logger().info('  ros2 service call /waypoint_navigator/save_waypoints std_srvs/srv/Trigger')
        self.get_logger().info('  ros2 service call /waypoint_navigator/load_waypoints std_srvs/srv/Trigger')
        self.get_logger().info('=' * 50)
        if self.waypoints:
            self.get_logger().info(f'Loaded {len(self.waypoints)} waypoints from file')

    def goal_callback(self, msg: PoseStamped):
        """Handle incoming goal clicks."""
        if not self.collecting:
            return

        # Extract position and orientation
        x = msg.pose.position.x
        y = msg.pose.position.y

        # Convert quaternion to yaw (simplified)
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w
        yaw = 2.0 * (qw * qz)  # Simplified for 2D

        # Add waypoint
        self.waypoints.append((x, y, qz, qw))

        self.get_logger().info(f'Waypoint {len(self.waypoints)} saved: x={x:.2f}, y={y:.2f}')
        self.get_logger().info(f'  Total waypoints: {len(self.waypoints)}')

    def start_collecting_cb(self, request, response):
        """Start collecting waypoints from map clicks."""
        self.collecting = True
        self.get_logger().info('=' * 40)
        self.get_logger().info('COLLECTING MODE STARTED')
        self.get_logger().info('Click on the map in Foxglove to add waypoints')
        self.get_logger().info('Call stop_collecting when done')
        self.get_logger().info('=' * 40)
        response.success = True
        response.message = f'Collecting waypoints. Current count: {len(self.waypoints)}'
        return response

    def stop_collecting_cb(self, request, response):
        """Stop collecting waypoints."""
        self.collecting = False
        self.get_logger().info('=' * 40)
        self.get_logger().info('COLLECTING MODE STOPPED')
        self.get_logger().info(f'Total waypoints collected: {len(self.waypoints)}')
        for i, wp in enumerate(self.waypoints):
            self.get_logger().info(f'  [{i+1}] x={wp[0]:.2f}, y={wp[1]:.2f}')
        self.get_logger().info('=' * 40)
        response.success = True
        response.message = f'Stopped collecting. Total: {len(self.waypoints)} waypoints'
        return response

    def start_navigation_cb(self, request, response):
        """Start navigating through waypoints."""
        if not self.waypoints:
            response.success = False
            response.message = 'No waypoints! Use start_collecting first.'
            return response

        if self.navigating:
            response.success = False
            response.message = 'Already navigating!'
            return response

        # Wait for Nav2 action server
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            response.success = False
            response.message = 'Nav2 action server not available!'
            return response

        self.navigating = True
        self.current_waypoint_index = 0
        self.current_cycle = 0

        self.get_logger().info('=' * 40)
        self.get_logger().info('NAVIGATION STARTED')
        self.get_logger().info(f'Waypoints: {len(self.waypoints)}')
        self.get_logger().info(f'Cycles: {self.repeat_cycles} (0=infinite)')
        self.get_logger().info(f'Return to start: {self.return_to_start}')
        self.get_logger().info('=' * 40)

        # Start navigation
        self.navigate_to_next_waypoint()

        response.success = True
        response.message = f'Navigation started with {len(self.waypoints)} waypoints'
        return response

    def stop_navigation_cb(self, request, response):
        """Stop navigation."""
        self.navigating = False
        self.get_logger().info('Navigation stopped by user')
        response.success = True
        response.message = 'Navigation stopped'
        return response

    def clear_waypoints_cb(self, request, response):
        """Clear all waypoints."""
        self.waypoints = []
        self.get_logger().info('All waypoints cleared')
        response.success = True
        response.message = 'Waypoints cleared'
        return response

    def list_waypoints_cb(self, request, response):
        """List all waypoints."""
        self.get_logger().info('=' * 40)
        self.get_logger().info(f'WAYPOINTS ({len(self.waypoints)} total):')
        self.get_logger().info(f'  [Start] x=0.00, y=0.00')
        for i, wp in enumerate(self.waypoints):
            self.get_logger().info(f'  [{i+1}] x={wp[0]:.2f}, y={wp[1]:.2f}')
        self.get_logger().info('=' * 40)
        response.success = True
        response.message = f'{len(self.waypoints)} waypoints'
        return response

    def save_waypoints_cb(self, request, response):
        """Save waypoints to file."""
        try:
            data = {
                'waypoints': self.waypoints,
                'repeat_cycles': self.repeat_cycles,
                'return_to_start': self.return_to_start
            }
            with open(self.waypoints_file, 'w') as f:
                json.dump(data, f, indent=2)
            self.get_logger().info(f'Waypoints saved to {self.waypoints_file}')
            response.success = True
            response.message = f'Saved {len(self.waypoints)} waypoints to {self.waypoints_file}'
        except Exception as e:
            self.get_logger().error(f'Failed to save waypoints: {e}')
            response.success = False
            response.message = str(e)
        return response

    def load_waypoints_cb(self, request, response):
        """Load waypoints from file."""
        if self.load_waypoints_from_file():
            response.success = True
            response.message = f'Loaded {len(self.waypoints)} waypoints'
        else:
            response.success = False
            response.message = 'Failed to load waypoints'
        return response

    def load_waypoints_from_file(self):
        """Load waypoints from JSON file."""
        try:
            if os.path.exists(self.waypoints_file):
                with open(self.waypoints_file, 'r') as f:
                    data = json.load(f)
                self.waypoints = [tuple(wp) for wp in data.get('waypoints', [])]
                self.get_logger().info(f'Loaded {len(self.waypoints)} waypoints from {self.waypoints_file}')
                return True
        except Exception as e:
            self.get_logger().warn(f'Could not load waypoints: {e}')
        return False

    def navigate_to_next_waypoint(self):
        """Navigate to the next waypoint in sequence."""
        if not self.navigating:
            return

        # Determine target
        if self.current_waypoint_index < len(self.waypoints):
            # Navigate to next waypoint
            wp = self.waypoints[self.current_waypoint_index]
            target_x, target_y = wp[0], wp[1]
            target_qz, target_qw = wp[2], wp[3]
            target_name = f'Waypoint {self.current_waypoint_index + 1}'
        elif self.return_to_start:
            # Return to start
            target_x, target_y = self.start_point[0], self.start_point[1]
            target_qz, target_qw = 0.0, 1.0
            target_name = 'Start Point'
        else:
            # Cycle complete, check if more cycles needed
            self.current_cycle += 1
            if self.repeat_cycles == 0 or self.current_cycle < self.repeat_cycles:
                self.get_logger().info(f'Cycle {self.current_cycle} complete. Starting next cycle...')
                self.current_waypoint_index = 0
                self.navigate_to_next_waypoint()
            else:
                self.get_logger().info('=' * 40)
                self.get_logger().info('ALL CYCLES COMPLETE!')
                self.get_logger().info('=' * 40)
                self.navigating = False
            return

        # Create goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = target_x
        goal_msg.pose.pose.position.y = target_y
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.z = target_qz
        goal_msg.pose.pose.orientation.w = target_qw

        self.get_logger().info(f'Navigating to {target_name}: x={target_x:.2f}, y={target_y:.2f}')

        # Send goal
        send_goal_future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Handle goal acceptance/rejection."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected!')
            return

        self.get_logger().info('Goal accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)

    def feedback_callback(self, feedback_msg):
        """Handle navigation feedback."""
        # Could add progress logging here if needed
        pass

    def goal_result_callback(self, future):
        """Handle navigation result."""
        result = future.result()
        status = result.status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Goal reached!')

            # Move to next waypoint
            if self.current_waypoint_index < len(self.waypoints):
                self.current_waypoint_index += 1
            elif self.return_to_start:
                # Just returned to start, cycle complete
                self.current_cycle += 1
                if self.repeat_cycles == 0 or self.current_cycle < self.repeat_cycles:
                    self.get_logger().info(f'Cycle {self.current_cycle} complete. Starting next cycle...')
                    self.current_waypoint_index = 0
                else:
                    self.get_logger().info('=' * 40)
                    self.get_logger().info('ALL CYCLES COMPLETE!')
                    self.get_logger().info('=' * 40)
                    self.navigating = False
                    return

            # Continue to next waypoint
            if self.navigating:
                self.navigate_to_next_waypoint()
        else:
            self.get_logger().warn(f'Navigation failed with status: {status}')
            self.get_logger().warn('Stopping navigation')
            self.navigating = False


def main(args=None):
    rclpy.init(args=args)
    node = WaypointNavigatorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
