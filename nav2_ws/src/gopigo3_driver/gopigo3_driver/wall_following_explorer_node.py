#!/usr/bin/env python3
"""
Wall Following Explorer Node for GoPiGo3

자율 SLAM 맵핑을 위한 벽 따라가기 탐색 노드

동작 순서:
1. 오른쪽 벽 따라가기 (Right-hand rule)
2. 장애물 감지 시 갭 분석 → 통과 가능하면 오른쪽 우선 통과
3. 양쪽 막히면 왼쪽 90° 회전, 그래도 막히면 180° 후진
4. 원위치 도달 OR 최대 시간 경과 시 → 지그재그 커버리지
5. 커버리지 완료 또는 95% 도달 시 → 시작점 복귀

무한 루프 방지:
- 최대 탐색 시간 제한
- 동일 위치 반복 감지
- 방문 기록 관리

Usage:
    ros2 run gopigo3_driver wall_following_explorer
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
from nav2_msgs.action import NavigateToPose
from tf2_ros import Buffer, TransformListener, TransformException

import numpy as np
import math
import time
import os
import subprocess
from datetime import datetime
from enum import Enum
from collections import deque


class ExplorationState(Enum):
    IDLE = 0
    WALL_FOLLOWING = 1
    ANALYZING_GAP = 2
    PASSING_GAP = 3
    ROTATING_LEFT = 4
    ROTATING_RIGHT = 5
    BACKING_UP = 6
    ZIGZAG_COVERAGE = 7
    RETURNING_HOME = 8
    COMPLETED = 9
    ESCAPE_FORWARD = 10  # 제자리 회전 탈출용
    SAVING_MAP = 11      # 맵 저장 중


class WallFollowingExplorerNode(Node):
    def __init__(self):
        super().__init__('wall_following_explorer')

        # ========================
        # Parameters
        # ========================
        # Robot dimensions
        self.declare_parameter('robot_width', 0.17)       # 17cm
        self.declare_parameter('safety_margin', 0.05)     # 5cm each side

        # Movement speeds
        self.declare_parameter('forward_speed', 0.12)     # m/s
        self.declare_parameter('rotation_speed', 0.5)     # rad/s

        # Wall following (30cm 거리 유지)
        self.declare_parameter('wall_follow_distance', 0.30)  # 30cm from wall
        self.declare_parameter('wall_detect_range', 0.50)     # 50cm wall detection range
        self.declare_parameter('front_obstacle_distance', 0.35)  # 35cm front detection

        # Gap detection (from gap_detector_test_node)
        self.declare_parameter('scan_angle_range', 30.0)  # ±30 degrees
        self.declare_parameter('rotation_clearance', 0.30)  # 30cm for rotation

        # Loop prevention
        self.declare_parameter('max_exploration_time', 1200.0)  # 20 minutes max
        self.declare_parameter('position_memory_size', 100)    # Remember 100 positions
        self.declare_parameter('loop_detect_radius', 0.3)      # 30cm radius for loop detection
        self.declare_parameter('loop_detect_count', 100)        # 100 visits = loop

        # Coverage
        self.declare_parameter('coverage_spacing', 0.4)   # 40cm between zigzag lines
        self.declare_parameter('coverage_passes', 3)      # 3 passes (user requested 5, reduced for efficiency)
        self.declare_parameter('home_radius', 0.5)        # 50cm radius for "home reached"

        # Map stability detection (맵 변화 감지)
        self.declare_parameter('map_stable_time', 30.0)   # 30초간 변화 없으면 완료
        self.declare_parameter('map_change_threshold', 0.01)  # 1% 미만 변화 = 안정
        self.declare_parameter('maps_directory', '/home/ubuntu/maps')  # 맵 저장 디렉토리

        # Get parameters
        self.robot_width = self.get_parameter('robot_width').value
        self.safety_margin = self.get_parameter('safety_margin').value
        self.forward_speed = self.get_parameter('forward_speed').value
        self.rotation_speed = self.get_parameter('rotation_speed').value
        self.wall_follow_distance = self.get_parameter('wall_follow_distance').value
        self.wall_detect_range = self.get_parameter('wall_detect_range').value
        self.front_obstacle_distance = self.get_parameter('front_obstacle_distance').value
        self.scan_angle_range = self.get_parameter('scan_angle_range').value
        self.rotation_clearance = self.get_parameter('rotation_clearance').value
        self.max_exploration_time = self.get_parameter('max_exploration_time').value
        self.position_memory_size = self.get_parameter('position_memory_size').value
        self.loop_detect_radius = self.get_parameter('loop_detect_radius').value
        self.loop_detect_count = self.get_parameter('loop_detect_count').value
        self.coverage_spacing = self.get_parameter('coverage_spacing').value
        self.coverage_passes = self.get_parameter('coverage_passes').value
        self.home_radius = self.get_parameter('home_radius').value
        self.map_stable_time = self.get_parameter('map_stable_time').value
        self.map_change_threshold = self.get_parameter('map_change_threshold').value
        self.maps_directory = self.get_parameter('maps_directory').value

        # Calculated values
        self.min_passable_gap = self.robot_width + (self.safety_margin * 2)  # 27cm

        # ========================
        # State variables
        # ========================
        self.state = ExplorationState.IDLE
        self.scan_data = None
        self.current_position = None
        self.current_yaw = 0.0
        self.start_position = None
        self.start_time = None

        # Rotation tracking
        self.accumulated_rotation = 0.0
        self.target_rotation = 0.0
        self.last_rotation_time = None

        # Position history for loop detection
        self.position_history = deque(maxlen=self.position_memory_size)
        self.last_recorded_position = None

        # Coverage state
        self.coverage_waypoints = []
        self.current_waypoint_idx = 0
        self.coverage_pass = 0
        self.map_data = None
        self.map_info = None

        # Map stability detection (맵 변화 감지)
        self.previous_map_data = None
        self.map_stable_start_time = None
        self.last_map_change_time = None
        self.end_position = None
        self.end_position_map = None
        self.saved_map_path = None

        # Wall following state
        self.wall_lost_count = 0

        # Spin detection (제자리 회전 감지)
        self.total_rotation_count = 0.0  # 총 회전량 (라디안)
        self.spin_check_position = None  # 회전 체크 시작 위치
        self.escape_target_angle = 0.0   # 탈출 방향 각도
        self.escape_start_time = None    # 탈출 전진 시작 시간
        self.is_escape_rotation = False  # 탈출을 위한 회전인지 플래그

        # ========================
        # Callback group
        # ========================
        self.callback_group = ReentrantCallbackGroup()

        # ========================
        # TF2 Buffer for map frame lookup
        # ========================
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.start_position_map = None  # map 프레임 기준 시작 위치

        # ========================
        # Publishers
        # ========================
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # ========================
        # Subscribers
        # ========================
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10,
            callback_group=self.callback_group)

        # Use /odom directly (EKF disabled by default)
        # If EKF is enabled, /odometry/filtered will be available instead
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10,
            callback_group=self.callback_group)

        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10,
            callback_group=self.callback_group)

        # ========================
        # Nav2 Action Client
        # ========================
        self.nav_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose',
            callback_group=self.callback_group)

        # ========================
        # Timer
        # ========================
        self.timer = self.create_timer(0.1, self.control_loop)

        # ========================
        # Startup info
        # ========================
        self.get_logger().info('=' * 60)
        self.get_logger().info('Wall Following Explorer Node')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'Robot width: {self.robot_width * 100:.1f} cm')
        self.get_logger().info(f'Min passable gap: {self.min_passable_gap * 100:.1f} cm')
        self.get_logger().info(f'Wall follow distance: {self.wall_follow_distance * 100:.0f} cm')
        self.get_logger().info(f'Max exploration time: {self.max_exploration_time:.0f} sec')
        self.get_logger().info('=' * 60)
        self.get_logger().info('Strategy:')
        self.get_logger().info('  1. Right-hand wall following')
        self.get_logger().info('  2. Gap detection & passage (right priority)')
        self.get_logger().info('  3. Left turn if blocked, 180° if both blocked')
        self.get_logger().info('  4. Zigzag coverage when wall following complete')
        self.get_logger().info('  5. Return to start (0,0)')
        self.get_logger().info('=' * 60)
        self.get_logger().info('Waiting for sensor data...')

    # ========================
    # Callbacks
    # ========================
    def scan_callback(self, msg: LaserScan):
        self.scan_data = msg

    def get_robot_position_in_map(self):
        """Get robot position in map frame using TF2."""
        try:
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time())
            return (
                transform.transform.translation.x,
                transform.transform.translation.y
            )
        except TransformException as e:
            self.get_logger().debug(f'TF lookup failed: {e}')
            return None

    def odom_callback(self, msg: Odometry):
        self.current_position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        )

        # Extract yaw from quaternion
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

        # Store start position (odom frame)
        if self.start_position is None:
            self.start_position = self.current_position
            self.get_logger().info(f'Start position (odom): ({self.start_position[0]:.2f}, {self.start_position[1]:.2f})')

        # Store start position in MAP frame (더 정확한 복귀를 위해)
        if self.start_position_map is None:
            map_pos = self.get_robot_position_in_map()
            if map_pos is not None:
                self.start_position_map = map_pos
                self.get_logger().info(f'Start position (map): ({self.start_position_map[0]:.2f}, {self.start_position_map[1]:.2f})')

        # Record position for loop detection (every 20cm)
        if self.last_recorded_position is None:
            self.last_recorded_position = self.current_position
            self.position_history.append(self.current_position)
        else:
            dx = self.current_position[0] - self.last_recorded_position[0]
            dy = self.current_position[1] - self.last_recorded_position[1]
            if math.sqrt(dx*dx + dy*dy) > 0.2:
                self.position_history.append(self.current_position)
                self.last_recorded_position = self.current_position

    def map_callback(self, msg: OccupancyGrid):
        new_map_data = np.array(msg.data).reshape(msg.info.height, msg.info.width)

        # 맵 변화 감지 (탐색 중일 때만)
        if self.state == ExplorationState.WALL_FOLLOWING and self.previous_map_data is not None:
            # 맵 크기가 같을 때만 비교
            if self.previous_map_data.shape == new_map_data.shape:
                # 변화된 셀 수 계산 (known 영역만)
                known_mask = (self.previous_map_data != -1) | (new_map_data != -1)
                if np.any(known_mask):
                    changed_cells = np.sum((self.previous_map_data != new_map_data) & known_mask)
                    total_known_cells = np.sum(known_mask)
                    change_ratio = changed_cells / total_known_cells if total_known_cells > 0 else 0

                    if change_ratio < self.map_change_threshold:
                        # 변화 적음 - 안정 시간 추적
                        if self.map_stable_start_time is None:
                            self.map_stable_start_time = time.time()
                            self.get_logger().info(f'Map becoming stable (change: {change_ratio*100:.2f}%)')
                        else:
                            stable_duration = time.time() - self.map_stable_start_time
                            if stable_duration >= self.map_stable_time:
                                # 맵이 충분히 안정됨 - 맵핑 완료!
                                self.get_logger().info('=' * 60)
                                self.get_logger().info(f'MAP STABLE for {stable_duration:.0f} seconds!')
                                self.get_logger().info('Transitioning to map save...')
                                self.get_logger().info('=' * 60)
                                self.state = ExplorationState.SAVING_MAP
                    else:
                        # 변화 있음 - 안정 타이머 리셋
                        if self.map_stable_start_time is not None:
                            self.get_logger().info(f'Map changed ({change_ratio*100:.2f}%), resetting stability timer')
                        self.map_stable_start_time = None
                        self.last_map_change_time = time.time()

        self.previous_map_data = new_map_data.copy()
        self.map_data = new_map_data
        self.map_info = msg.info

    # ========================
    # LiDAR Analysis Functions
    # ========================
    def get_range_at_angle(self, angle_deg):
        """Get LiDAR range at specific angle (degrees). 0 = front, 90 = left, -90 = right."""
        if self.scan_data is None:
            return float('inf')

        scan = self.scan_data
        angle_rad = math.radians(angle_deg)

        # Find index for this angle
        if angle_rad < scan.angle_min or angle_rad > scan.angle_max:
            return float('inf')

        index = int((angle_rad - scan.angle_min) / scan.angle_increment)
        if 0 <= index < len(scan.ranges):
            r = scan.ranges[index]
            if math.isfinite(r) and scan.range_min < r < scan.range_max:
                return r
        return float('inf')

    def get_min_range_in_sector(self, start_angle_deg, end_angle_deg):
        """Get minimum range in an angular sector."""
        if self.scan_data is None:
            return float('inf')

        scan = self.scan_data
        ranges = np.array(scan.ranges)
        num_ranges = len(ranges)
        angles = np.linspace(scan.angle_min, scan.angle_max, num_ranges)

        start_rad = math.radians(start_angle_deg)
        end_rad = math.radians(end_angle_deg)

        mask = (angles >= start_rad) & (angles <= end_rad)
        sector_ranges = ranges[mask]

        # Filter valid ranges
        valid = sector_ranges[np.isfinite(sector_ranges) &
                             (sector_ranges > scan.range_min) &
                             (sector_ranges < scan.range_max)]

        return np.min(valid) if len(valid) > 0 else float('inf')

    def check_front_obstacle(self):
        """Check for obstacle in front (±30 degrees)."""
        return self.get_min_range_in_sector(-self.scan_angle_range, self.scan_angle_range)

    def check_right_wall(self):
        """Check distance to right wall (-60 to -120 degrees)."""
        return self.get_min_range_in_sector(-120, -60)

    def check_left_wall(self):
        """Check distance to left wall (60 to 120 degrees)."""
        return self.get_min_range_in_sector(60, 120)

    def detect_right_gap(self):
        """Detect explorable gap on right side while wall following.

        벽을 따라가다가 오른쪽에 탐색 가능한 공간이 있는지 감지.
        갭이 있으면 (angle, depth) 반환, 없으면 None 반환.
        """
        if self.scan_data is None:
            return None

        # 오른쪽 -45도에서 -90도 사이에서 갑자기 거리가 멀어지는 곳 찾기
        right_45 = self.get_range_at_angle(-45)
        right_60 = self.get_range_at_angle(-60)
        right_75 = self.get_range_at_angle(-75)
        right_90 = self.get_range_at_angle(-90)

        # 현재 벽 거리 (오른쪽 벽)
        right_wall = self.check_right_wall()

        # 갭 감지: 특정 방향의 거리가 벽 거리보다 훨씬 멀면 (2배 이상) 갭 존재
        gap_threshold = right_wall * 2.0 if right_wall < float('inf') else 1.0

        # 각 방향에서 갭 체크
        for angle, dist in [(-45, right_45), (-60, right_60), (-75, right_75)]:
            if dist > gap_threshold and dist > 0.5:  # 최소 50cm 이상 깊이
                # 갭 입구 폭 체크
                entrance_clear = self.get_min_range_in_sector(angle - 15, angle + 15)
                if entrance_clear > self.min_passable_gap:
                    self.get_logger().info(f'Right gap detected at {angle}°: depth={dist*100:.0f}cm, entrance={entrance_clear*100:.0f}cm')
                    return (angle, dist)

        return None

    def find_nearest_wall(self):
        """Find the direction and distance to the nearest wall/obstacle.

        가장 가까운 벽/장애물의 방향과 거리를 반환.
        열린 공간에서 벽을 찾아갈 때 사용.
        """
        if self.scan_data is None:
            return 0.0, float('inf')

        scan = self.scan_data
        ranges = np.array(scan.ranges)
        num_ranges = len(ranges)
        angles = np.linspace(scan.angle_min, scan.angle_max, num_ranges)

        # 유효한 범위만 필터링
        valid_mask = np.isfinite(ranges) & (ranges > scan.range_min) & (ranges < scan.range_max)

        if not np.any(valid_mask):
            return 0.0, float('inf')

        valid_ranges = ranges[valid_mask]
        valid_angles = angles[valid_mask]

        # 가장 가까운 점 찾기
        min_idx = np.argmin(valid_ranges)
        min_range = valid_ranges[min_idx]
        min_angle_rad = valid_angles[min_idx]
        min_angle_deg = math.degrees(min_angle_rad)

        return min_angle_deg, min_range

    def find_farthest_direction(self, prefer_right=True):
        """Find the best escape direction, preferring right side (right-hand rule).

        오른쪽 방향을 우선으로 탈출 방향을 찾음 (Right-hand rule 유지).
        같은 거리면 오른쪽(음수 각도)을 선택.
        """
        if self.scan_data is None:
            return 0.0, 0.0  # angle_deg, distance

        # 각 방향별로 통과 가능 여부 체크 (±15도 범위의 최소값)
        # 오른쪽 우선 순서: -90, -60, -30, -120, 0, 30, 60, 90, 120, -150, 150
        if prefer_right:
            check_angles = [-90, -60, -30, -120, 0, 30, 60, 90, 120, -150, 150]
        else:
            check_angles = [90, 60, 30, 120, 0, -30, -60, -90, -120, 150, -150]

        # 먼저 통과 가능한 방향 중 가장 먼 곳 찾기
        best_angle = 0.0
        best_clear_distance = 0.0

        for check_angle in check_angles:
            # 해당 방향 ±15도 범위의 최소 거리 (실제 통과 가능 거리)
            clear_dist = self.get_min_range_in_sector(check_angle - 15, check_angle + 15)

            # 로봇이 통과 가능한 거리(27cm 이상)
            if clear_dist > self.min_passable_gap:
                # 첫 번째 통과 가능한 방향 선택 (오른쪽 우선 순서이므로)
                # 단, 거리가 50cm 이상이면 바로 선택
                if clear_dist > 0.5:
                    self.get_logger().info(f'Good escape direction: {check_angle}° with {clear_dist*100:.0f}cm clearance')
                    return check_angle, clear_dist

                # 그렇지 않으면 가장 먼 곳 기록
                if clear_dist > best_clear_distance:
                    best_clear_distance = clear_dist
                    best_angle = check_angle

        if best_clear_distance > self.min_passable_gap:
            self.get_logger().info(f'Best escape direction: {best_angle}° with {best_clear_distance*100:.0f}cm clearance')
            return best_angle, best_clear_distance

        # 어떤 방향도 통과 불가능하면 후진 필요
        self.get_logger().warn('No clear direction found! Need to back up.')
        return 180.0, 0.0  # 180도 = 후진 신호

    def measure_gap_width(self):
        """Measure gap width when obstacle detected in front."""
        if self.scan_data is None:
            return float('inf')

        scan = self.scan_data
        ranges = np.array(scan.ranges)
        num_ranges = len(ranges)
        angles = np.linspace(scan.angle_min, scan.angle_max, num_ranges)

        # Front sector
        front_angle_rad = math.radians(self.scan_angle_range)
        front_mask = np.abs(angles) <= front_angle_rad
        front_ranges = ranges[front_mask]
        front_angles = angles[front_mask]

        # Replace invalid with max range
        front_ranges = np.where(np.isfinite(front_ranges), front_ranges, scan.range_max)

        # Find obstacles within detection distance
        obstacle_mask = front_ranges <= self.front_obstacle_distance
        if not np.any(obstacle_mask):
            return float('inf')

        obstacle_ranges = front_ranges[obstacle_mask]
        obstacle_angles = front_angles[obstacle_mask]

        # Find leftmost and rightmost obstacles
        left_idx = np.argmax(obstacle_angles)
        right_idx = np.argmin(obstacle_angles)

        left_dist = obstacle_ranges[left_idx]
        left_angle = obstacle_angles[left_idx]
        right_dist = obstacle_ranges[right_idx]
        right_angle = obstacle_angles[right_idx]

        # Convert to Cartesian and calculate Y-distance
        left_y = left_dist * math.sin(left_angle)
        right_y = right_dist * math.sin(right_angle)

        gap_width = abs(left_y - right_y)

        self.get_logger().info(f'Gap analysis: left={left_dist*100:.0f}cm@{math.degrees(left_angle):.0f}°, '
                               f'right={right_dist*100:.0f}cm@{math.degrees(right_angle):.0f}°, '
                               f'gap={gap_width*100:.0f}cm')

        return gap_width

    def check_gap_passable(self, direction='right'):
        """Check if gap on specified side is passable.

        갭 통과 가능 여부 판단:
        - 갭 입구 방향의 공간이 로봇 폭보다 넓은지 확인
        - 갭 뒤에 벽이 있어도 입구가 충분히 넓으면 통과 가능
        """
        if direction == 'right':
            # 오른쪽 갭 입구 체크 (-15 to -45 degrees) - 더 좁은 범위로 입구만 체크
            entrance_range = self.get_min_range_in_sector(-45, -15)
            # 갭 깊이 체크 (얼마나 들어갈 수 있는지)
            depth_range = self.get_range_at_angle(-30)
        else:
            # 왼쪽 갭 입구 체크 (15 to 45 degrees)
            entrance_range = self.get_min_range_in_sector(15, 45)
            depth_range = self.get_range_at_angle(30)

        # 입구가 로봇 폭보다 넓고, 최소 30cm 이상 들어갈 수 있으면 통과 가능
        is_passable = entrance_range > self.min_passable_gap and depth_range > 0.30

        self.get_logger().debug(f'Gap {direction}: entrance={entrance_range*100:.0f}cm, depth={depth_range*100:.0f}cm, passable={is_passable}')

        return is_passable

    # ========================
    # Loop Detection
    # ========================
    def detect_loop(self):
        """Detect if robot is stuck in a loop."""
        if self.current_position is None or len(self.position_history) < 10:
            return False

        # Count how many times we've been near current position
        visit_count = 0
        for pos in self.position_history:
            dx = self.current_position[0] - pos[0]
            dy = self.current_position[1] - pos[1]
            if math.sqrt(dx*dx + dy*dy) < self.loop_detect_radius:
                visit_count += 1

        if visit_count >= self.loop_detect_count:
            self.get_logger().warn(f'Loop detected! Visited this area {visit_count} times')
            return True

        return False

    def get_recent_visit_count(self):
        """Count recent visits to current area (for avoiding repetition)."""
        if self.current_position is None or len(self.position_history) < 5:
            return 0

        # 최근 50개 위치 중 현재 위치 근처에 몇 번 왔는지
        recent_positions = list(self.position_history)[-50:]
        visit_count = 0
        for pos in recent_positions:
            dx = self.current_position[0] - pos[0]
            dy = self.current_position[1] - pos[1]
            if math.sqrt(dx*dx + dy*dy) < 0.5:  # 50cm 반경
                visit_count += 1

        return visit_count

    def is_back_at_start(self):
        """Check if robot returned to start position."""
        if self.current_position is None or self.start_position is None:
            return False

        dx = self.current_position[0] - self.start_position[0]
        dy = self.current_position[1] - self.start_position[1]
        dist = math.sqrt(dx*dx + dy*dy)

        return dist < self.home_radius

    def check_time_limit(self):
        """Check if max exploration time exceeded."""
        if self.start_time is None:
            return False

        elapsed = time.time() - self.start_time
        return elapsed > self.max_exploration_time

    # ========================
    # Map Saving Functions
    # ========================
    def save_map(self):
        """Save map using slam_toolbox serialize_map service."""
        # 타임스탬프 생성 (YYYYMMDD_HHMMSS)
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        map_filename = f'{timestamp}'
        map_full_path = os.path.join(self.maps_directory, map_filename)

        # 디렉토리 확인/생성
        try:
            os.makedirs(self.maps_directory, exist_ok=True)
        except Exception as e:
            self.get_logger().error(f'Failed to create maps directory: {e}')
            return None

        self.get_logger().info(f'Saving map to: {map_full_path}')

        # slam_toolbox의 serialize_map 서비스 호출
        try:
            result = subprocess.run(
                ['ros2', 'service', 'call', '/slam_toolbox/serialize_map',
                 'slam_toolbox/srv/SerializePoseGraph',
                 f'{{filename: "{map_full_path}"}}'],
                capture_output=True,
                text=True,
                timeout=30
            )

            if result.returncode == 0:
                self.get_logger().info(f'Map saved successfully: {map_full_path}')
                self.saved_map_path = map_full_path
                return map_full_path
            else:
                self.get_logger().error(f'Failed to save map: {result.stderr}')
                return None

        except subprocess.TimeoutExpired:
            self.get_logger().error('Map save timeout')
            return None
        except Exception as e:
            self.get_logger().error(f'Map save error: {e}')
            return None

    def save_positions_to_file(self):
        """Save start and end positions to a JSON file."""
        if self.saved_map_path is None:
            return

        import json

        positions_file = f'{self.saved_map_path}_positions.json'

        positions_data = {
            'start_position_odom': {
                'x': self.start_position[0] if self.start_position else None,
                'y': self.start_position[1] if self.start_position else None
            },
            'start_position_map': {
                'x': self.start_position_map[0] if self.start_position_map else None,
                'y': self.start_position_map[1] if self.start_position_map else None
            },
            'end_position_odom': {
                'x': self.end_position[0] if self.end_position else None,
                'y': self.end_position[1] if self.end_position else None
            },
            'end_position_map': {
                'x': self.end_position_map[0] if self.end_position_map else None,
                'y': self.end_position_map[1] if self.end_position_map else None
            },
            'exploration_time_seconds': time.time() - self.start_time if self.start_time else 0,
            'timestamp': datetime.now().isoformat()
        }

        try:
            with open(positions_file, 'w') as f:
                json.dump(positions_data, f, indent=2)
            self.get_logger().info(f'Positions saved to: {positions_file}')
        except Exception as e:
            self.get_logger().error(f'Failed to save positions: {e}')

    # ========================
    # Movement Control
    # ========================
    def stop_robot(self):
        """Stop all robot motion."""
        twist = Twist()
        self.cmd_vel_pub.publish(twist)

    def start_rotation(self, angle_deg):
        """Start rotation by specified degrees (positive = left, negative = right)."""
        self.target_rotation = abs(math.radians(angle_deg))
        self.accumulated_rotation = 0.0
        self.last_rotation_time = self.get_clock().now()

        if angle_deg > 0:
            self.state = ExplorationState.ROTATING_LEFT
            self.get_logger().info(f'Starting left rotation: {angle_deg}°')
        else:
            self.state = ExplorationState.ROTATING_RIGHT
            self.get_logger().info(f'Starting right rotation: {abs(angle_deg)}°')

    def get_angle_to_target(self, target_x, target_y):
        """Calculate angle to target from current position."""
        if self.current_position is None:
            return 0.0

        dx = target_x - self.current_position[0]
        dy = target_y - self.current_position[1]
        target_angle = math.atan2(dy, dx)

        # 현재 yaw와의 차이 계산
        angle_diff = target_angle - self.current_yaw

        # -π ~ π 범위로 정규화
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi

        return math.degrees(angle_diff)

    def get_distance_to_target(self, target_x, target_y):
        """Calculate distance to target from current position."""
        if self.current_position is None:
            return float('inf')

        dx = target_x - self.current_position[0]
        dy = target_y - self.current_position[1]
        return math.sqrt(dx*dx + dy*dy)

    # ========================
    # Coverage Pattern
    # ========================
    def generate_coverage_waypoints(self):
        """Generate zigzag coverage waypoints based on map."""
        if self.map_data is None or self.map_info is None:
            self.get_logger().warn('No map data for coverage')
            return

        # Find free space bounds
        free_cells = np.where(self.map_data == 0)
        if len(free_cells[0]) == 0:
            return

        resolution = self.map_info.resolution
        origin_x = self.map_info.origin.position.x
        origin_y = self.map_info.origin.position.y

        min_row, max_row = np.min(free_cells[0]), np.max(free_cells[0])
        min_col, max_col = np.min(free_cells[1]), np.max(free_cells[1])

        margin = 0.3  # 30cm margin from walls
        min_x = origin_x + min_col * resolution + margin
        max_x = origin_x + max_col * resolution - margin
        min_y = origin_y + min_row * resolution + margin
        max_y = origin_y + max_row * resolution - margin

        self.get_logger().info(f'Coverage area: x=[{min_x:.2f}, {max_x:.2f}], y=[{min_y:.2f}, {max_y:.2f}]')

        # Generate zigzag
        self.coverage_waypoints = []
        y = min_y
        direction = 1

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

    def navigate_to_point(self, x, y):
        """Send navigation goal using Nav2."""
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.w = 1.0

        self.get_logger().info(f'Navigating to ({x:.2f}, {y:.2f})')
        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(self.nav_goal_response_callback)

    def nav_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Navigation goal rejected')
            self.current_waypoint_idx += 1
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.nav_result_callback)

    def nav_result_callback(self, future):
        """Handle navigation completion."""
        if self.state == ExplorationState.ZIGZAG_COVERAGE:
            self.current_waypoint_idx += 1
            if self.current_waypoint_idx >= len(self.coverage_waypoints):
                self.coverage_pass += 1
                self.get_logger().info(f'Coverage pass {self.coverage_pass}/{self.coverage_passes} complete')

                if self.coverage_pass >= self.coverage_passes:
                    self.state = ExplorationState.RETURNING_HOME
                    self.navigate_to_point(self.start_position[0], self.start_position[1])
                else:
                    self.current_waypoint_idx = 0
                    self.coverage_waypoints.reverse()
                    x, y = self.coverage_waypoints[0]
                    self.navigate_to_point(x, y)
            else:
                x, y = self.coverage_waypoints[self.current_waypoint_idx]
                self.navigate_to_point(x, y)

        elif self.state == ExplorationState.RETURNING_HOME:
            self.get_logger().info('=' * 60)
            self.get_logger().info('EXPLORATION COMPLETE!')
            self.get_logger().info('=' * 60)
            self.state = ExplorationState.COMPLETED
            self.stop_robot()

    # ========================
    # Main Control Loop
    # ========================
    def control_loop(self):
        """Main state machine control loop."""
        if self.scan_data is None:
            return

        twist = Twist()

        # ========================
        # IDLE State
        # ========================
        if self.state == ExplorationState.IDLE:
            if self.current_position is not None:
                self.start_time = time.time()
                self.state = ExplorationState.WALL_FOLLOWING
                self.get_logger().info('Starting wall following exploration...')

        # ========================
        # WALL_FOLLOWING State
        # ========================
        elif self.state == ExplorationState.WALL_FOLLOWING:
            # Check termination conditions
            if self.check_time_limit():
                # 시간 초과 - 맵 저장 후 종료
                self.get_logger().info(f'Time limit (20 min) reached, saving map...')
                self.state = ExplorationState.SAVING_MAP
                return

            # Check if back at start (after some exploration)
            # 시작점 복귀 시 맵 저장 후 완료 처리
            if len(self.position_history) > 50 and self.is_back_at_start():
                self.get_logger().info('=' * 60)
                self.get_logger().info('Returned to start! Saving map...')
                self.get_logger().info('=' * 60)
                self.state = ExplorationState.SAVING_MAP
                return

            # Check for loop
            if self.detect_loop():
                # 루프 감지 - 맵 저장 후 종료
                self.get_logger().info(f'Loop detected (100 visits), saving map...')
                self.state = ExplorationState.SAVING_MAP
                return

            # ========================================
            # 단순화된 Right-Hand Rule 벽 따라가기
            # ========================================
            # 원칙 1: 오른쪽에 벽/장애물을 두고 전진
            # 원칙 2: 전방 막히면 왼쪽으로 회전
            # 원칙 3: 왼쪽도 막히면 후진 후 왼쪽 회전
            # ========================================

            front_dist = self.check_front_obstacle()
            right_dist = self.check_right_wall()
            left_dist = self.check_left_wall()

            # 디버그 로그 (매 1초마다)
            if not hasattr(self, '_last_debug_time') or time.time() - self._last_debug_time > 1.0:
                self._last_debug_time = time.time()
                self.get_logger().info(f'[WALL_FOLLOW] front={front_dist*100:.0f}cm, right={right_dist*100:.0f}cm, left={left_dist*100:.0f}cm')

            # ---- CASE 1: 전방에 장애물 ----
            if front_dist < self.front_obstacle_distance:
                self.wall_lost_count = 0

                # 왼쪽이 열려있으면 왼쪽으로 회전
                if left_dist > self.min_passable_gap + 0.1:  # 37cm 이상
                    self.get_logger().info(f'Front blocked ({front_dist*100:.0f}cm), turning LEFT')
                    self.start_rotation(90)  # 왼쪽 90도
                else:
                    # 왼쪽도 막혀있으면 후진 필요
                    self.get_logger().info(f'Front AND left blocked, backing up')
                    self.state = ExplorationState.BACKING_UP
                return

            # ---- CASE 2: 오른쪽에 벽이 있음 (정상 벽 따라가기) ----
            if right_dist < self.wall_detect_range:
                self.wall_lost_count = 0

                # 벽과의 거리에 따라 조향
                if right_dist < self.wall_follow_distance * 0.6:
                    # 너무 가까움 - 왼쪽으로 살짝 틀면서 전진
                    twist.linear.x = self.forward_speed * 0.7
                    twist.angular.z = 0.4
                    self.get_logger().debug(f'Too close to wall ({right_dist*100:.0f}cm), turning left slightly')

                elif right_dist > self.wall_follow_distance * 1.4:
                    # 너무 멂 - 오른쪽으로 살짝 틀면서 전진
                    twist.linear.x = self.forward_speed
                    twist.angular.z = -0.25
                    self.get_logger().debug(f'Too far from wall ({right_dist*100:.0f}cm), turning right slightly')

                else:
                    # 적정 거리 - 직진
                    twist.linear.x = self.forward_speed
                    twist.angular.z = 0.0

            # ---- CASE 3: 오른쪽에 벽이 없음 (벽 모서리 or 열린 공간) ----
            # Right-hand rule: 벽 모서리를 돌아서 벽을 계속 오른쪽에 유지
            else:
                self.wall_lost_count += 1

                # 처음 벽이 없어진 순간
                if self.wall_lost_count == 1:
                    self.get_logger().info('Right wall lost! Starting corner maneuver...')

                # Phase 1: 처음 0.5초(5회)는 직진 - 모서리를 완전히 지나감
                if self.wall_lost_count <= 5:
                    twist.linear.x = self.forward_speed
                    twist.angular.z = 0.0
                    self.get_logger().info(f'Passing corner, moving forward ({self.wall_lost_count}/5)')

                # Phase 2: 그 다음 1.5초(15회)는 오른쪽으로 틀면서 전진 - 새 벽 찾기
                elif self.wall_lost_count <= 20:
                    twist.linear.x = self.forward_speed
                    twist.angular.z = -0.35  # 오른쪽으로 틀면서 전진
                    self.get_logger().info(f'Curving right to find wall ({self.wall_lost_count}/20)')

                # Phase 3: 2초 이상 벽을 못 찾으면 - 열린 공간
                # 가장 가까운 벽/장애물 방향으로 직진
                else:
                    # 가장 가까운 벽 방향 찾기
                    nearest_angle, nearest_dist = self.find_nearest_wall()

                    if nearest_dist < 2.0:  # 2m 이내에 벽이 있으면
                        # 그 방향으로 회전 후 직진
                        if abs(nearest_angle) > 15:
                            self.get_logger().info(f'Open space: turning to nearest wall at {nearest_angle:.0f}° ({nearest_dist*100:.0f}cm)')
                            self.wall_lost_count = 0
                            self.start_rotation(nearest_angle)
                            return
                        else:
                            # 이미 벽 방향을 향하고 있으면 직진
                            twist.linear.x = self.forward_speed
                            twist.angular.z = 0.0
                            self.get_logger().info(f'Moving to nearest wall ({nearest_dist*100:.0f}cm)')
                    else:
                        # 2m 이내에 벽이 없으면 그냥 직진
                        twist.linear.x = self.forward_speed
                        twist.angular.z = 0.0
                        self.get_logger().info('No wall nearby, moving forward')

                    # 벽을 찾으면 카운트 리셋
                    if self.wall_lost_count > 50:
                        self.wall_lost_count = 0

        # ========================
        # ANALYZING_GAP State (더 이상 사용하지 않음 - WALL_FOLLOWING에서 직접 처리)
        # ========================
        elif self.state == ExplorationState.ANALYZING_GAP:
            # 이 상태는 더 이상 사용하지 않음
            # WALL_FOLLOWING에서 직접 처리하도록 변경
            self.state = ExplorationState.WALL_FOLLOWING

        # ========================
        # ROTATING States
        # ========================
        elif self.state in [ExplorationState.ROTATING_LEFT, ExplorationState.ROTATING_RIGHT]:
            current_time = self.get_clock().now()
            if self.last_rotation_time is not None:
                dt = (current_time - self.last_rotation_time).nanoseconds / 1e9
                rotation_amount = self.rotation_speed * dt
                self.accumulated_rotation += rotation_amount
                self.total_rotation_count += rotation_amount  # 총 회전량 누적

            self.last_rotation_time = current_time

            # 제자리 회전 감지: 위치가 거의 변하지 않으면서 10바퀴(20π 라디안) 이상 회전
            if self.current_position is not None:
                if self.spin_check_position is None:
                    self.spin_check_position = self.current_position
                    self.total_rotation_count = 0.0
                else:
                    dx = self.current_position[0] - self.spin_check_position[0]
                    dy = self.current_position[1] - self.spin_check_position[1]
                    distance_moved = math.sqrt(dx*dx + dy*dy)

                    # 30cm 이상 이동하면 회전 카운트 리셋
                    if distance_moved > 0.3:
                        self.spin_check_position = self.current_position
                        self.total_rotation_count = 0.0

                    # 10바퀴 = 20π 라디안 ≈ 62.83 라디안
                    if self.total_rotation_count >= 20 * math.pi:
                        self.get_logger().warn(f'Spinning detected! ({self.total_rotation_count/(2*math.pi):.1f} rotations)')

                        # 가장 먼 방향 찾기
                        angle_deg, distance = self.find_farthest_direction()
                        self.get_logger().info(f'Escaping to farthest direction: {angle_deg:.0f}° at {distance*100:.0f}cm')

                        # 해당 방향으로 회전 후 전진
                        self.escape_target_angle = angle_deg
                        self.total_rotation_count = 0.0
                        self.spin_check_position = None

                        if abs(angle_deg) > 10:  # 10도 이상 회전 필요하면
                            self.is_escape_rotation = True  # 탈출 회전 플래그 설정
                            self.start_rotation(angle_deg)
                        else:
                            # 이미 해당 방향을 향하고 있으면 바로 전진
                            self.state = ExplorationState.ESCAPE_FORWARD
                            self.escape_start_time = time.time()
                        return

            if self.accumulated_rotation >= self.target_rotation:
                self.get_logger().info('Rotation complete')
                if self.is_escape_rotation:
                    # 탈출 회전 완료 -> 20cm 전진
                    self.is_escape_rotation = False
                    self.state = ExplorationState.ESCAPE_FORWARD
                    self.escape_start_time = time.time()
                    self.get_logger().info('Starting escape forward (20cm)')
                else:
                    self.state = ExplorationState.WALL_FOLLOWING
                twist.angular.z = 0.0
            else:
                if self.state == ExplorationState.ROTATING_LEFT:
                    twist.angular.z = self.rotation_speed
                else:
                    twist.angular.z = -self.rotation_speed

        # ========================
        # ESCAPE_FORWARD State (제자리 회전 탈출)
        # ========================
        elif self.state == ExplorationState.ESCAPE_FORWARD:
            # 20cm 전진 (약 1.7초 at 0.12m/s)
            if self.escape_start_time is None:
                self.escape_start_time = time.time()

            elapsed = time.time() - self.escape_start_time
            escape_duration = 0.20 / self.forward_speed  # 20cm / speed

            if elapsed < escape_duration:
                # 전방 장애물 체크
                front_dist = self.check_front_obstacle()
                if front_dist > 0.25:  # 25cm 이상 여유 있으면 전진
                    twist.linear.x = self.forward_speed
                    self.get_logger().info(f'Escaping forward... {elapsed:.1f}s / {escape_duration:.1f}s')
                else:
                    # 장애물 있으면 탈출 종료
                    self.get_logger().warn('Obstacle during escape, stopping')
                    self.escape_start_time = None
                    self.state = ExplorationState.WALL_FOLLOWING
            else:
                self.get_logger().info('Escape complete (20cm forward)')
                self.escape_start_time = None
                self.spin_check_position = self.current_position
                self.total_rotation_count = 0.0
                self.state = ExplorationState.WALL_FOLLOWING

        # ========================
        # BACKING_UP State
        # ========================
        elif self.state == ExplorationState.BACKING_UP:
            # 후진 후 왼쪽 90도 회전 (장애물을 오른쪽에 두기 위해)
            if not hasattr(self, 'backup_start_time'):
                self.backup_start_time = time.time()
                self.get_logger().info('Backing up to escape...')

            elapsed = time.time() - self.backup_start_time

            # 2초 동안 후진 (약 24cm)
            if elapsed < 2.0:
                twist.linear.x = -self.forward_speed
                twist.angular.z = 0.0
                self.get_logger().info(f'Backing up... {elapsed:.1f}s / 2.0s')
            else:
                # 후진 완료, 왼쪽 90도 회전 (장애물을 오른쪽에 두기 위해)
                self.get_logger().info('Backup complete, turning LEFT 90° (right-hand rule)')
                del self.backup_start_time
                self.start_rotation(90)  # 왼쪽 90도

        # ========================
        # ZIGZAG_COVERAGE State (handled by Nav2 callbacks)
        # ========================
        elif self.state == ExplorationState.ZIGZAG_COVERAGE:
            pass  # Navigation handled asynchronously

        # ========================
        # SAVING_MAP State - 맵 저장 및 완료 처리
        # ========================
        elif self.state == ExplorationState.SAVING_MAP:
            # 로봇 정지
            self.stop_robot()

            # 종료 위치 저장
            self.end_position = self.current_position
            self.end_position_map = self.get_robot_position_in_map()

            # 맵 저장
            self.get_logger().info('Saving map...')
            map_path = self.save_map()

            if map_path:
                # 위치 정보 저장
                self.save_positions_to_file()

                # 탐색 시간 계산
                exploration_time = time.time() - self.start_time if self.start_time else 0
                minutes = int(exploration_time // 60)
                seconds = int(exploration_time % 60)

                # 완료 메시지 출력
                self.get_logger().info('=' * 60)
                self.get_logger().info('MAPPING COMPLETE!')
                self.get_logger().info('=' * 60)
                self.get_logger().info(f'Exploration time: {minutes}m {seconds}s')
                self.get_logger().info(f'Map saved to: {map_path}')
                self.get_logger().info(f'  - {map_path}.data')
                self.get_logger().info(f'  - {map_path}.posegraph')
                self.get_logger().info(f'  - {map_path}_positions.json')
                self.get_logger().info('')
                self.get_logger().info('To navigate with this map:')
                self.get_logger().info('  1. Stop current SLAM: Ctrl+C')
                self.get_logger().info(f'  2. Launch navigation:')
                self.get_logger().info(f'     ros2 launch gopigo3_driver navigation.launch.py map:={map_path}')
                self.get_logger().info('')
                self.get_logger().info('Start position (map frame):')
                if self.start_position_map:
                    self.get_logger().info(f'  x={self.start_position_map[0]:.2f}, y={self.start_position_map[1]:.2f}')
                self.get_logger().info('End position (map frame):')
                if self.end_position_map:
                    self.get_logger().info(f'  x={self.end_position_map[0]:.2f}, y={self.end_position_map[1]:.2f}')
                self.get_logger().info('=' * 60)
            else:
                self.get_logger().error('Failed to save map!')

            self.state = ExplorationState.COMPLETED
            return

        # ========================
        # RETURNING_HOME State - 직접 시작점으로 이동 (Nav2 없이)
        # ========================
        elif self.state == ExplorationState.RETURNING_HOME:
            # map 프레임에서 현재 위치 가져오기
            current_map_pos = self.get_robot_position_in_map()

            # 목표 위치 설정 (map 프레임 사용 시 map 좌표, 아니면 odom 좌표)
            if self.start_position_map is not None and current_map_pos is not None:
                target_x, target_y = self.start_position_map
                current_x, current_y = current_map_pos
            else:
                target_x, target_y = self.start_position
                current_x, current_y = self.current_position

            # 목표까지 거리 계산
            dx = target_x - current_x
            dy = target_y - current_y
            dist_to_home = math.sqrt(dx*dx + dy*dy)

            # 도착 확인 (50cm 이내)
            if dist_to_home < self.home_radius:
                self.get_logger().info('=' * 60)
                self.get_logger().info('ARRIVED AT START POSITION!')
                self.get_logger().info('=' * 60)
                self.state = ExplorationState.COMPLETED
                self.stop_robot()
                return

            # 목표 방향 계산 (map 프레임 기준)
            target_angle = math.atan2(dy, dx)
            angle_diff = target_angle - self.current_yaw

            # -π ~ π 범위로 정규화
            while angle_diff > math.pi:
                angle_diff -= 2 * math.pi
            while angle_diff < -math.pi:
                angle_diff += 2 * math.pi

            angle_to_home = math.degrees(angle_diff)

            # 전방 장애물 체크
            front_dist = self.check_front_obstacle()

            if front_dist < self.front_obstacle_distance:
                # 장애물 있으면 피해가기 (왼쪽으로 돌아서)
                self.get_logger().info(f'Obstacle on way home, turning left')
                twist.linear.x = 0.0
                twist.angular.z = 0.3  # 왼쪽으로 회전
            elif abs(angle_to_home) > 15:
                # 방향이 15도 이상 틀리면 회전
                twist.linear.x = 0.0
                if angle_to_home > 0:
                    twist.angular.z = 0.4  # 왼쪽으로
                else:
                    twist.angular.z = -0.4  # 오른쪽으로
                self.get_logger().info(f'Turning to home: {angle_to_home:.0f}° (dist: {dist_to_home*100:.0f}cm)')
            else:
                # 방향 맞으면 전진
                twist.linear.x = self.forward_speed
                twist.angular.z = angle_to_home * 0.02  # 약간의 방향 보정
                self.get_logger().info(f'Moving to home: {dist_to_home*100:.0f}cm remaining')

        # ========================
        # COMPLETED State
        # ========================
        elif self.state == ExplorationState.COMPLETED:
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        # Publish command
        self.cmd_vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = WallFollowingExplorerNode()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.stop_robot()
        node.get_logger().info('Stopping robot...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
