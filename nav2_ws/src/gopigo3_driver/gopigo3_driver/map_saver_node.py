#!/usr/bin/env python3
"""
Map Saver Node - SLAM 맵 저장 및 위치 정보 기록

이 노드는:
1. 노드 시작 시 시작 위치 기록
2. 맵 저장 시 종료 위치 기록
3. slam_toolbox의 serialize_map 서비스를 호출하여 맵 저장
4. 타임스탬프 기반 파일명 자동 생성 (YYYYMMDD_HHMMSS)

사용법 (수동 맵핑 시작 시 함께 실행):
    ros2 run gopigo3_driver map_saver --ros-args -p auto_save:=false

맵핑 완료 후 저장:
    ros2 service call /save_map std_srvs/srv/Trigger

또는 자동 저장 모드 (바로 저장하고 종료):
    ros2 run gopigo3_driver map_saver
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformListener, TransformException

import os
import json
import subprocess
import math
from datetime import datetime


class MapSaverNode(Node):
    def __init__(self):
        super().__init__('map_saver')

        # Parameters
        self.declare_parameter('maps_directory', '/home/ubuntu/maps')
        self.declare_parameter('auto_save', True)  # True면 시작 시 바로 저장

        self.maps_directory = self.get_parameter('maps_directory').value
        self.auto_save = self.get_parameter('auto_save').value

        # TF2 for map frame position
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Current position tracking
        self.current_position_odom = None
        self.current_yaw_odom = None

        # Start position (recorded when node starts)
        self.start_position_odom = None
        self.start_position_map = None
        self.start_yaw_odom = None
        self.start_yaw_map = None
        self.start_time = None

        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # Service for manual save
        self.save_service = self.create_service(
            Trigger,
            '/save_map',
            self.save_map_callback
        )

        self.get_logger().info('=' * 50)
        self.get_logger().info('Map Saver Node Started')
        self.get_logger().info(f'Maps directory: {self.maps_directory}')
        self.get_logger().info('=' * 50)

        # Record start position after brief delay (wait for TF)
        self.create_timer(2.0, self.record_start_position_timer)

        # Auto save if enabled
        if self.auto_save:
            # Wait a moment for TF and odom to be ready
            self.create_timer(3.0, self.auto_save_timer)
        else:
            self.get_logger().info('')
            self.get_logger().info('Manual save mode enabled.')
            self.get_logger().info('To save map, run:')
            self.get_logger().info('  ros2 service call /save_map std_srvs/srv/Trigger')
            self.get_logger().info('')

    def record_start_position_timer(self):
        """One-shot timer to record start position."""
        self.record_start_position()

    def auto_save_timer(self):
        """One-shot timer for auto save."""
        self.get_logger().info('Auto-saving map...')
        success, message, path = self.save_map()

        if success:
            self.get_logger().info(f'Map saved successfully!')
            self.get_logger().info(f'Path: {path}')
        else:
            self.get_logger().error(f'Map save failed: {message}')

        # Shutdown after save
        self.get_logger().info('Map saver shutting down...')
        raise SystemExit(0)

    def odom_callback(self, msg: Odometry):
        """Track current position from odometry."""
        self.current_position_odom = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        )

        # Extract yaw from quaternion
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_yaw_odom = math.atan2(siny_cosp, cosy_cosp)

    def record_start_position(self):
        """Record the start position when mapping begins."""
        import time
        self.start_time = time.time()

        # Record odom position
        if self.current_position_odom:
            self.start_position_odom = self.current_position_odom
            self.start_yaw_odom = self.current_yaw_odom
            self.get_logger().info(f'Start position (odom): ({self.start_position_odom[0]:.2f}, {self.start_position_odom[1]:.2f})')

        # Record map frame position
        map_pos = self.get_position_in_map_frame()
        if map_pos:
            self.start_position_map = (map_pos[0], map_pos[1])
            self.start_yaw_map = map_pos[2]
            self.get_logger().info(f'Start position (map): ({self.start_position_map[0]:.2f}, {self.start_position_map[1]:.2f})')

    def get_position_in_map_frame(self):
        """Get current robot position in map frame using TF2."""
        try:
            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_footprint',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )

            x = transform.transform.translation.x
            y = transform.transform.translation.y

            # Extract yaw
            q = transform.transform.rotation
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            import math
            yaw = math.atan2(siny_cosp, cosy_cosp)

            return (x, y, yaw)

        except TransformException as e:
            self.get_logger().warn(f'Could not get map frame position: {e}')
            return None

    def save_map(self):
        """
        Save map using slam_toolbox serialize_map service.

        Returns:
            (success: bool, message: str, path: str or None)
        """
        # Generate timestamp filename
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        map_filename = timestamp
        map_full_path = os.path.join(self.maps_directory, map_filename)

        # Create directory if needed
        try:
            os.makedirs(self.maps_directory, exist_ok=True)
        except Exception as e:
            return False, f'Failed to create maps directory: {e}', None

        self.get_logger().info(f'Saving map to: {map_full_path}')

        # Call slam_toolbox serialize_map service
        try:
            result = subprocess.run(
                ['ros2', 'service', 'call', '/slam_toolbox/serialize_map',
                 'slam_toolbox/srv/SerializePoseGraph',
                 f'{{filename: "{map_full_path}"}}'],
                capture_output=True,
                text=True,
                timeout=30
            )

            if result.returncode != 0:
                return False, f'Service call failed: {result.stderr}', None

        except subprocess.TimeoutExpired:
            return False, 'Map save timeout', None
        except Exception as e:
            return False, f'Error: {e}', None

        # Save position information
        self.save_position_info(map_full_path)

        # Print summary
        self.get_logger().info('=' * 50)
        self.get_logger().info('MAP SAVED SUCCESSFULLY!')
        self.get_logger().info('=' * 50)
        self.get_logger().info(f'Files created:')
        self.get_logger().info(f'  - {map_full_path}.data')
        self.get_logger().info(f'  - {map_full_path}.posegraph')
        self.get_logger().info(f'  - {map_full_path}_positions.json')
        self.get_logger().info('')
        self.get_logger().info('To navigate with this map:')
        self.get_logger().info(f'  ros2 launch gopigo3_driver navigation.launch.py map:={map_full_path}')
        self.get_logger().info('=' * 50)

        return True, 'Map saved successfully', map_full_path

    def save_position_info(self, map_path):
        """Save start and end positions to JSON file."""
        import time
        positions_file = f'{map_path}_positions.json'

        # Get current map frame position (end position)
        end_map_pos = self.get_position_in_map_frame()

        # Calculate exploration time
        exploration_time = time.time() - self.start_time if self.start_time else 0

        positions_data = {
            'map_file': map_path,
            'timestamp': datetime.now().isoformat(),
            'exploration_time_seconds': exploration_time,
            'start_position': {
                'odom_frame': {
                    'x': self.start_position_odom[0] if self.start_position_odom else None,
                    'y': self.start_position_odom[1] if self.start_position_odom else None,
                    'yaw': self.start_yaw_odom
                },
                'map_frame': {
                    'x': self.start_position_map[0] if self.start_position_map else None,
                    'y': self.start_position_map[1] if self.start_position_map else None,
                    'yaw': self.start_yaw_map
                }
            },
            'end_position': {
                'odom_frame': {
                    'x': self.current_position_odom[0] if self.current_position_odom else None,
                    'y': self.current_position_odom[1] if self.current_position_odom else None,
                    'yaw': self.current_yaw_odom
                },
                'map_frame': {
                    'x': end_map_pos[0] if end_map_pos else None,
                    'y': end_map_pos[1] if end_map_pos else None,
                    'yaw': end_map_pos[2] if end_map_pos else None
                }
            },
            'description': 'start_position: where mapping started. end_position: where map was saved. Use end_position for Nav2 initial pose.'
        }

        try:
            with open(positions_file, 'w') as f:
                json.dump(positions_data, f, indent=2)
            self.get_logger().info(f'Position info saved to: {positions_file}')

            # Print position summary
            self.get_logger().info('')
            self.get_logger().info('Position Summary:')
            if self.start_position_map:
                self.get_logger().info(f'  Start (map): ({self.start_position_map[0]:.2f}, {self.start_position_map[1]:.2f})')
            if end_map_pos:
                self.get_logger().info(f'  End (map):   ({end_map_pos[0]:.2f}, {end_map_pos[1]:.2f})')

        except Exception as e:
            self.get_logger().error(f'Failed to save positions: {e}')

    def save_map_callback(self, request, response):
        """Service callback for manual map save."""
        success, message, path = self.save_map()
        response.success = success
        response.message = message if not success else f'Map saved to {path}'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = MapSaverNode()

    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
