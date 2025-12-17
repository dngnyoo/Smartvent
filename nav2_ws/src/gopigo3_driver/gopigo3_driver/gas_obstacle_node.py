#!/usr/bin/env python3
"""
Gas Obstacle Node for Nav2

This node listens to gas detection data and publishes virtual obstacles
to the Nav2 costmap when hazardous gas is detected.

When gas concentration exceeds the threshold, the robot's current position
is marked as a hazardous zone and Nav2 will automatically replan the path
to avoid this area.

Topics:
    Subscribed:
        /gas_detection (std_msgs/Float32): Gas concentration level
        /gas_detected (std_msgs/Bool): Simple gas detection flag (alternative)

    Published:
        /gas_obstacles (nav_msgs/OccupancyGrid): Virtual obstacles for costmap
        /gas_zones (visualization_msgs/MarkerArray): Visualization markers

Services:
    /clear_gas_obstacles: Clear all gas obstacles
    /add_gas_obstacle: Manually add a gas obstacle at specific location

Author: GoPiGo3 Team
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import Float32, Bool
from std_srvs.srv import Empty
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import Buffer, TransformListener, TransformException
import numpy as np
import math
from dataclasses import dataclass
from typing import List, Tuple
import time


@dataclass
class GasZone:
    """Represents a detected gas hazard zone"""
    x: float
    y: float
    radius: float
    concentration: float
    timestamp: float
    decay_rate: float = 0.0  # For future: gas dissipation modeling


class GasObstacleNode(Node):
    """
    Node that converts gas detection into virtual obstacles for Nav2.

    When hazardous gas is detected, this node:
    1. Records the robot's current position as a gas zone
    2. Publishes this zone as an obstacle to the costmap
    3. Nav2 automatically replans to avoid the hazardous area
    """

    def __init__(self):
        super().__init__('gas_obstacle_node')

        # Parameters
        self.declare_parameter('gas_threshold', 300.0)  # ppm threshold for danger
        self.declare_parameter('obstacle_radius', 0.5)  # meters - radius of virtual obstacle
        self.declare_parameter('inflation_radius', 0.3)  # additional safety margin
        self.declare_parameter('map_resolution', 0.05)  # meters per cell
        self.declare_parameter('map_size', 10.0)  # meters - size of local obstacle map
        self.declare_parameter('update_rate', 2.0)  # Hz
        self.declare_parameter('zone_timeout', 300.0)  # seconds - how long zones persist
        self.declare_parameter('use_bool_topic', False)  # Use Bool instead of Float32

        # Get parameters
        self.gas_threshold = self.get_parameter('gas_threshold').value
        self.obstacle_radius = self.get_parameter('obstacle_radius').value
        self.inflation_radius = self.get_parameter('inflation_radius').value
        self.map_resolution = self.get_parameter('map_resolution').value
        self.map_size = self.get_parameter('map_size').value
        self.update_rate = self.get_parameter('update_rate').value
        self.zone_timeout = self.get_parameter('zone_timeout').value
        self.use_bool_topic = self.get_parameter('use_bool_topic').value

        # Storage for gas zones
        self.gas_zones: List[GasZone] = []
        self.current_concentration = 0.0

        # TF for getting robot position
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # QoS for costmap compatibility
        costmap_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )

        # Publishers
        self.obstacle_pub = self.create_publisher(
            OccupancyGrid,
            '/gas_obstacles',
            costmap_qos
        )

        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/gas_zones',
            10
        )

        self.alert_pub = self.create_publisher(
            Bool,
            '/gas_alert',
            10
        )

        # Subscribers
        if self.use_bool_topic:
            self.gas_sub = self.create_subscription(
                Bool,
                '/gas_detected',
                self.gas_bool_callback,
                10
            )
            self.get_logger().info('Subscribing to /gas_detected (Bool)')
        else:
            self.gas_sub = self.create_subscription(
                Float32,
                '/gas_detection',
                self.gas_float_callback,
                10
            )
            self.get_logger().info(f'Subscribing to /gas_detection (Float32), threshold: {self.gas_threshold}')

        # Services
        self.clear_srv = self.create_service(
            Empty,
            '/clear_gas_obstacles',
            self.clear_obstacles_callback
        )

        # Timer for publishing obstacles
        self.timer = self.create_timer(
            1.0 / self.update_rate,
            self.publish_obstacles
        )

        self.get_logger().info('Gas Obstacle Node started')
        self.get_logger().info(f'  Obstacle radius: {self.obstacle_radius}m')
        self.get_logger().info(f'  Zone timeout: {self.zone_timeout}s')

    def get_robot_position(self) -> Tuple[float, float] | None:
        """Get current robot position in map frame"""
        try:
            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_footprint',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5)
            )
            return (
                transform.transform.translation.x,
                transform.transform.translation.y
            )
        except TransformException as e:
            self.get_logger().warn(f'Could not get robot position: {e}')
            return None

    def gas_float_callback(self, msg: Float32):
        """Handle gas concentration reading"""
        self.current_concentration = msg.data

        if msg.data >= self.gas_threshold:
            self.handle_gas_detection(msg.data)

    def gas_bool_callback(self, msg: Bool):
        """Handle simple gas detection flag"""
        if msg.data:
            # Use threshold value as placeholder concentration
            self.current_concentration = self.gas_threshold
            self.handle_gas_detection(self.gas_threshold)
        else:
            self.current_concentration = 0.0

    def handle_gas_detection(self, concentration: float):
        """Process gas detection and create obstacle zone"""
        pos = self.get_robot_position()
        if pos is None:
            self.get_logger().warn('Gas detected but cannot get robot position!')
            return

        x, y = pos

        # Check if this position already has a zone nearby
        for zone in self.gas_zones:
            dist = math.sqrt((zone.x - x)**2 + (zone.y - y)**2)
            if dist < self.obstacle_radius:
                # Update existing zone
                zone.concentration = max(zone.concentration, concentration)
                zone.timestamp = time.time()
                self.get_logger().info(f'Updated gas zone at ({x:.2f}, {y:.2f})')
                return

        # Create new gas zone
        new_zone = GasZone(
            x=x,
            y=y,
            radius=self.obstacle_radius,
            concentration=concentration,
            timestamp=time.time()
        )
        self.gas_zones.append(new_zone)

        self.get_logger().warn(
            f'⚠️  GAS DETECTED! Concentration: {concentration:.1f}, '
            f'Creating obstacle zone at ({x:.2f}, {y:.2f})'
        )

        # Publish alert
        alert_msg = Bool()
        alert_msg.data = True
        self.alert_pub.publish(alert_msg)

    def publish_obstacles(self):
        """Publish gas zones as occupancy grid and markers"""
        current_time = time.time()

        # Remove expired zones
        self.gas_zones = [
            zone for zone in self.gas_zones
            if current_time - zone.timestamp < self.zone_timeout
        ]

        # Publish occupancy grid if we have zones
        if self.gas_zones:
            self.publish_occupancy_grid()

        # Always publish markers (empty if no zones)
        self.publish_markers()

    def publish_occupancy_grid(self):
        """Create and publish occupancy grid with gas obstacles"""
        # Get robot position for map centering
        pos = self.get_robot_position()
        if pos is None:
            return

        robot_x, robot_y = pos

        # Calculate map bounds to cover all gas zones
        all_x = [zone.x for zone in self.gas_zones] + [robot_x]
        all_y = [zone.y for zone in self.gas_zones] + [robot_y]

        min_x = min(all_x) - self.obstacle_radius - 1.0
        max_x = max(all_x) + self.obstacle_radius + 1.0
        min_y = min(all_y) - self.obstacle_radius - 1.0
        max_y = max(all_y) + self.obstacle_radius + 1.0

        # Ensure minimum map size
        width = max(max_x - min_x, self.map_size)
        height = max(max_y - min_y, self.map_size)

        # Create occupancy grid
        grid = OccupancyGrid()
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.header.frame_id = 'map'

        grid.info.resolution = self.map_resolution
        grid.info.width = int(width / self.map_resolution)
        grid.info.height = int(height / self.map_resolution)
        grid.info.origin.position.x = min_x
        grid.info.origin.position.y = min_y
        grid.info.origin.position.z = 0.0
        grid.info.origin.orientation.w = 1.0

        # Initialize with unknown (-1) - won't affect costmap
        # We only mark obstacle cells as 100
        data = np.full((grid.info.height, grid.info.width), -1, dtype=np.int8)

        # Mark gas zones as obstacles
        total_radius = self.obstacle_radius + self.inflation_radius

        for zone in self.gas_zones:
            # Convert zone position to grid coordinates
            zone_grid_x = int((zone.x - min_x) / self.map_resolution)
            zone_grid_y = int((zone.y - min_y) / self.map_resolution)
            radius_cells = int(total_radius / self.map_resolution)

            # Fill circular obstacle
            for dy in range(-radius_cells, radius_cells + 1):
                for dx in range(-radius_cells, radius_cells + 1):
                    gx = zone_grid_x + dx
                    gy = zone_grid_y + dy

                    if 0 <= gx < grid.info.width and 0 <= gy < grid.info.height:
                        dist = math.sqrt(dx**2 + dy**2) * self.map_resolution

                        if dist <= self.obstacle_radius:
                            # Lethal obstacle (100)
                            data[gy, gx] = 100
                        elif dist <= total_radius:
                            # Inflated zone (50-99 based on distance)
                            inflation_cost = int(100 - (dist - self.obstacle_radius) / self.inflation_radius * 50)
                            data[gy, gx] = max(data[gy, gx], inflation_cost)

        grid.data = data.flatten().tolist()
        self.obstacle_pub.publish(grid)

    def publish_markers(self):
        """Publish visualization markers for gas zones"""
        marker_array = MarkerArray()

        # Delete old markers first
        delete_marker = Marker()
        delete_marker.header.frame_id = 'map'
        delete_marker.header.stamp = self.get_clock().now().to_msg()
        delete_marker.ns = 'gas_zones'
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)

        # Create new markers for each zone
        for i, zone in enumerate(self.gas_zones):
            # Cylinder marker for the zone
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'gas_zones'
            marker.id = i
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD

            marker.pose.position.x = zone.x
            marker.pose.position.y = zone.y
            marker.pose.position.z = 0.25  # Half height
            marker.pose.orientation.w = 1.0

            marker.scale.x = zone.radius * 2
            marker.scale.y = zone.radius * 2
            marker.scale.z = 0.5  # Height

            # Color: yellow to red based on concentration
            marker.color.r = 1.0
            marker.color.g = max(0.0, 1.0 - (zone.concentration / (self.gas_threshold * 2)))
            marker.color.b = 0.0
            marker.color.a = 0.6

            marker.lifetime.sec = 1  # Refresh every second

            marker_array.markers.append(marker)

            # Text marker showing concentration
            text_marker = Marker()
            text_marker.header.frame_id = 'map'
            text_marker.header.stamp = self.get_clock().now().to_msg()
            text_marker.ns = 'gas_labels'
            text_marker.id = i
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD

            text_marker.pose.position.x = zone.x
            text_marker.pose.position.y = zone.y
            text_marker.pose.position.z = 0.7

            text_marker.scale.z = 0.2  # Text height

            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0

            text_marker.text = f'GAS\n{zone.concentration:.0f}ppm'
            text_marker.lifetime.sec = 1

            marker_array.markers.append(text_marker)

        self.marker_pub.publish(marker_array)

    def clear_obstacles_callback(self, request, response):
        """Service callback to clear all gas obstacles"""
        self.gas_zones.clear()
        self.get_logger().info('All gas obstacles cleared')
        return response


def main(args=None):
    rclpy.init(args=args)
    node = GasObstacleNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
