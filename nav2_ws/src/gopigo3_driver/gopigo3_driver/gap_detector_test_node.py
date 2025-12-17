#!/usr/bin/env python3
"""
Gap Detector Test Node (v3)

Logic:
1. If no obstacle within detection_distance (60cm) in front → move forward
2. If obstacle detected within detection_distance → measure gap width between obstacles
3. If gap >= min_passable_gap (27cm) → move forward 5cm and stop
4. If gap < min_passable_gap → rotate right 90 degrees and stop

Usage:
    ros2 run gopigo3_driver gap_detector_test
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
import math
from enum import Enum


class RobotState(Enum):
    IDLE = 0
    MOVING_FORWARD = 1
    ANALYZING_GAP = 2
    PASSING_GAP = 3
    APPROACHING_WALL = 4  # Moving to rotation clearance distance
    ROTATING_RIGHT = 5
    STOPPED = 6


class GapDetectorTestNode(Node):
    def __init__(self):
        super().__init__('gap_detector_test')

        # Parameters
        self.declare_parameter('robot_width', 0.17)       # 17cm
        self.declare_parameter('safety_margin', 0.05)     # 5cm each side
        self.declare_parameter('forward_speed', 0.10)     # m/s
        self.declare_parameter('rotation_speed', 0.5)     # rad/s
        self.declare_parameter('detection_distance', 0.3) # Obstacle detection at 30cm
        self.declare_parameter('scan_angle_range', 30.0)  # Check ±30 degrees in front
        self.declare_parameter('forward_distance', 0.05)  # Move 5cm when gap is passable
        self.declare_parameter('rotation_clearance', 0.25) # 25cm clearance for rotation

        self.robot_width = self.get_parameter('robot_width').value
        self.safety_margin = self.get_parameter('safety_margin').value
        self.forward_speed = self.get_parameter('forward_speed').value
        self.rotation_speed = self.get_parameter('rotation_speed').value
        self.detection_distance = self.get_parameter('detection_distance').value
        self.scan_angle_range = self.get_parameter('scan_angle_range').value
        self.forward_distance = self.get_parameter('forward_distance').value
        self.rotation_clearance = self.get_parameter('rotation_clearance').value

        # Minimum passable gap = robot width + margins on both sides
        self.min_passable_gap = self.robot_width + (self.safety_margin * 2)

        # State
        self.state = RobotState.IDLE
        self.scan_data = None
        self.current_position = None
        self.start_position = None

        # Rotation tracking
        self.accumulated_rotation = 0.0
        self.target_rotation = math.pi / 2  # 90 degrees
        self.last_time = None

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)

        # Timer for control loop
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info('=' * 60)
        self.get_logger().info('Gap Detector Test Node (v3)')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'Robot width: {self.robot_width * 100:.1f} cm')
        self.get_logger().info(f'Safety margin: {self.safety_margin * 100:.1f} cm (each side)')
        self.get_logger().info(f'Min passable gap: {self.min_passable_gap * 100:.1f} cm')
        self.get_logger().info(f'Detection distance: {self.detection_distance * 100:.0f} cm')
        self.get_logger().info(f'Scan angle range: ±{self.scan_angle_range:.0f} degrees')
        self.get_logger().info(f'Forward distance (after gap check): {self.forward_distance * 100:.0f} cm')
        self.get_logger().info(f'Rotation clearance: {self.rotation_clearance * 100:.0f} cm')
        self.get_logger().info('=' * 60)
        self.get_logger().info('Logic:')
        self.get_logger().info('  1. No obstacle within 30cm → Move forward')
        self.get_logger().info('  2. Obstacle detected → Measure gap')
        self.get_logger().info('  3. Gap >= 27cm → Continue forward')
        self.get_logger().info('  4. Gap < 27cm → Move to 25cm from wall, rotate right 90°, stop')
        self.get_logger().info('=' * 60)
        self.get_logger().info('Waiting for LiDAR data...')

    def scan_callback(self, msg: LaserScan):
        self.scan_data = msg

    def odom_callback(self, msg: Odometry):
        self.current_position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        )

    def get_front_data(self):
        """Get LiDAR data for front area only."""
        if self.scan_data is None:
            return None, None

        scan = self.scan_data
        ranges = np.array(scan.ranges)
        num_ranges = len(ranges)

        angles = np.linspace(scan.angle_min, scan.angle_max, num_ranges)
        ranges = np.where(np.isfinite(ranges), ranges, scan.range_max)

        # Front area (±scan_angle_range degrees)
        front_angle_rad = math.radians(self.scan_angle_range)
        front_mask = np.abs(angles) <= front_angle_rad

        return ranges[front_mask], angles[front_mask]

    def check_obstacle_in_front(self):
        """
        Check if there's any obstacle within detection_distance in front.
        Returns (has_obstacle, min_distance)
        """
        front_ranges, front_angles = self.get_front_data()
        if front_ranges is None:
            return False, float('inf')

        min_dist = np.min(front_ranges)
        has_obstacle = min_dist <= self.detection_distance

        return has_obstacle, min_dist

    def measure_gap_width(self):
        """
        Measure the gap width when obstacles are detected.
        Finds obstacles on left and right, calculates distance between them.
        Returns gap_width in meters.
        """
        front_ranges, front_angles = self.get_front_data()
        if front_ranges is None:
            return float('inf')

        # Find all points that are obstacles (within detection distance)
        obstacle_mask = front_ranges <= self.detection_distance

        if not np.any(obstacle_mask):
            # No obstacles, infinite gap
            return float('inf')

        obstacle_ranges = front_ranges[obstacle_mask]
        obstacle_angles = front_angles[obstacle_mask]

        self.get_logger().info('=' * 60)
        self.get_logger().info('GAP MEASUREMENT')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'Obstacles detected: {len(obstacle_ranges)} points')

        # Find leftmost and rightmost obstacles
        left_idx = np.argmin(obstacle_angles)  # Most negative angle (left)
        right_idx = np.argmax(obstacle_angles)  # Most positive angle (right)

        left_dist = obstacle_ranges[left_idx]
        left_angle = obstacle_angles[left_idx]
        right_dist = obstacle_ranges[right_idx]
        right_angle = obstacle_angles[right_idx]

        # Convert to Cartesian coordinates
        left_x = left_dist * math.cos(left_angle)
        left_y = left_dist * math.sin(left_angle)
        right_x = right_dist * math.cos(right_angle)
        right_y = right_dist * math.sin(right_angle)

        # Gap width is the Y-distance between left and right obstacles
        # (perpendicular to robot's forward direction)
        gap_width = abs(right_y - left_y)

        self.get_logger().info(f'Left obstacle:  dist={left_dist*100:.1f}cm, angle={math.degrees(left_angle):.1f}°')
        self.get_logger().info(f'Right obstacle: dist={right_dist*100:.1f}cm, angle={math.degrees(right_angle):.1f}°')
        self.get_logger().info(f'Left point (x,y):  ({left_x*100:.1f}, {left_y*100:.1f}) cm')
        self.get_logger().info(f'Right point (x,y): ({right_x*100:.1f}, {right_y*100:.1f}) cm')
        self.get_logger().info(f'Gap width (Y-distance): {gap_width*100:.1f} cm')
        self.get_logger().info('=' * 60)

        return gap_width

    def control_loop(self):
        """Main control loop."""
        if self.scan_data is None:
            return

        twist = Twist()

        if self.state == RobotState.IDLE:
            self.state = RobotState.MOVING_FORWARD
            self.get_logger().info('Starting... Moving forward until obstacle detected.')

        elif self.state == RobotState.MOVING_FORWARD:
            has_obstacle, min_dist = self.check_obstacle_in_front()

            if has_obstacle:
                # Obstacle detected! Stop and analyze gap
                twist.linear.x = 0.0
                self.cmd_vel_pub.publish(twist)

                self.get_logger().info(f'OBSTACLE DETECTED at {min_dist*100:.1f} cm!')
                self.state = RobotState.ANALYZING_GAP
            else:
                # No obstacle, keep moving
                twist.linear.x = self.forward_speed
                self.get_logger().info(f'Moving forward... min_dist={min_dist*100:.1f}cm',
                                       throttle_duration_sec=1.0)

        elif self.state == RobotState.ANALYZING_GAP:
            gap_width = self.measure_gap_width()

            if gap_width >= self.min_passable_gap:
                self.get_logger().info('=' * 40)
                self.get_logger().info(f'GAP IS PASSABLE!')
                self.get_logger().info(f'  Gap width: {gap_width*100:.1f} cm >= {self.min_passable_gap*100:.1f} cm')
                self.get_logger().info('Continuing forward...')
                self.get_logger().info('=' * 40)

                # Continue moving forward (go back to MOVING_FORWARD state)
                self.state = RobotState.MOVING_FORWARD
            else:
                self.get_logger().warn('=' * 40)
                self.get_logger().warn(f'GAP TOO NARROW!')
                self.get_logger().warn(f'  Gap width: {gap_width*100:.1f} cm < {self.min_passable_gap*100:.1f} cm')
                self.get_logger().warn(f'Approaching wall to {self.rotation_clearance*100:.0f}cm before rotating...')
                self.get_logger().warn('=' * 40)

                self.state = RobotState.APPROACHING_WALL

        elif self.state == RobotState.PASSING_GAP:
            if self.current_position is not None and self.start_position is not None:
                dx = self.current_position[0] - self.start_position[0]
                dy = self.current_position[1] - self.start_position[1]
                distance_traveled = math.sqrt(dx*dx + dy*dy)

                if distance_traveled >= self.forward_distance:
                    self.get_logger().info('Passed through gap! Stopping.')
                    self.state = RobotState.STOPPED
                    twist.linear.x = 0.0
                else:
                    twist.linear.x = self.forward_speed
                    self.get_logger().info(f'Passing gap... {distance_traveled*100:.1f}/{self.forward_distance*100:.0f} cm')
            else:
                twist.linear.x = self.forward_speed

        elif self.state == RobotState.APPROACHING_WALL:
            # Move forward until we reach rotation_clearance distance from wall
            has_obstacle, min_dist = self.check_obstacle_in_front()

            if min_dist <= self.rotation_clearance:
                # Close enough to wall, start rotating
                twist.linear.x = 0.0
                self.cmd_vel_pub.publish(twist)

                self.get_logger().info(f'Reached {min_dist*100:.1f}cm from wall. Rotating right 90°...')
                self.state = RobotState.ROTATING_RIGHT
                self.accumulated_rotation = 0.0
                self.last_time = self.get_clock().now()
            else:
                # Keep approaching
                twist.linear.x = self.forward_speed
                self.get_logger().info(f'Approaching wall... {min_dist*100:.1f}cm (target: {self.rotation_clearance*100:.0f}cm)',
                                       throttle_duration_sec=0.5)

        elif self.state == RobotState.ROTATING_RIGHT:
            twist.angular.z = -self.rotation_speed

            current_time = self.get_clock().now()
            if self.last_time is not None:
                dt = (current_time - self.last_time).nanoseconds / 1e9
                self.accumulated_rotation += self.rotation_speed * dt
            self.last_time = current_time

            self.get_logger().info(f'Rotating... {math.degrees(self.accumulated_rotation):.1f}/90.0°')

            if self.accumulated_rotation >= self.target_rotation:
                self.get_logger().info('Rotation complete! Stopping.')
                self.state = RobotState.STOPPED
                twist.angular.z = 0.0

        elif self.state == RobotState.STOPPED:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.get_logger().info('Test complete. Robot stopped.', throttle_duration_sec=5.0)

        self.cmd_vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = GapDetectorTestNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        twist = Twist()
        node.cmd_vel_pub.publish(twist)
        node.get_logger().info('Stopping robot...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
