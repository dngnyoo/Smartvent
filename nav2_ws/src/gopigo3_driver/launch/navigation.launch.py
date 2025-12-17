#!/usr/bin/env python3
"""
GoPiGo3 Navigation Launch File

This launches everything needed for autonomous navigation:
- gopigo3_bringup (robot drivers, sensors, EKF)
- slam_toolbox in localization mode (load saved map)
- Nav2 (navigation stack)

Usage:
    ros2 launch gopigo3_driver navigation.launch.py map:=/home/ubuntu/maps/my_room

    The map parameter should be the path WITHOUT extension.
    slam_toolbox will load .data and .posegraph files.

Prerequisites:
    - A map created using slam.launch.py and saved with serialize_map service
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Package directories
    pkg_share = FindPackageShare('gopigo3_driver').find('gopigo3_driver')

    # Config files
    nav2_config = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    slam_localization_config = os.path.join(pkg_share, 'config', 'slam_toolbox_localization.yaml')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    map_file = LaunchConfiguration('map')
    autostart = LaunchConfiguration('autostart', default='true')

    # ========================
    # Include gopigo3_bringup
    # ========================
    # This starts: robot_state_publisher, gopigo3_driver, bno055_imu,
    #              ekf_node, rplidar, foxglove_bridge
    gopigo3_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gopigo3_driver'),
                'launch',
                'gopigo3_bringup.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'lidar': 'true',
            'foxglove': 'true',
            'ekf': 'true'
        }.items()
    )

    # ========================
    # SLAM Toolbox (Localization Mode)
    # ========================
    # Load saved map and localize robot
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='localization_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_localization_config,
            {
                'use_sim_time': use_sim_time,
                'map_file_name': map_file,
                'map_start_at_dock': True  # Start at the position where map was saved
            }
        ]
    )

    # ========================
    # Nav2 Controller Server
    # ========================
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_config, {'use_sim_time': use_sim_time}],
        remappings=[('cmd_vel', 'cmd_vel_nav')]
    )

    # ========================
    # Nav2 Planner Server
    # ========================
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_config, {'use_sim_time': use_sim_time}]
    )

    # ========================
    # Nav2 Smoother Server
    # ========================
    smoother_server = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        parameters=[nav2_config, {'use_sim_time': use_sim_time}]
    )

    # ========================
    # Nav2 Behavior Server
    # ========================
    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[nav2_config, {'use_sim_time': use_sim_time}]
    )

    # ========================
    # Nav2 BT Navigator
    # ========================
    # Remap goal_pose to goal_pose_internal so goals go through goal_relay first
    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_config, {'use_sim_time': use_sim_time}],
        remappings=[('goal_pose', 'goal_pose_internal')]
    )

    # ========================
    # Nav2 Waypoint Follower
    # ========================
    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[nav2_config, {'use_sim_time': use_sim_time}]
    )

    # ========================
    # Nav2 Velocity Smoother
    # ========================
    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[nav2_config, {'use_sim_time': use_sim_time}],
        remappings=[
            ('cmd_vel', 'cmd_vel_nav'),
            ('cmd_vel_smoothed', 'cmd_vel')
        ]
    )

    # ========================
    # Nav2 Lifecycle Manager
    # ========================
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'node_names': [
                'controller_server',
                'planner_server',
                'smoother_server',
                'behavior_server',
                'bt_navigator',
                'waypoint_follower',
                'velocity_smoother'
            ]
        }]
    )

    # ========================
    # Goal Relay Node
    # ========================
    # Transforms goals from base_link to map frame for Foxglove compatibility
    goal_relay = Node(
        package='gopigo3_driver',
        executable='goal_relay',
        name='goal_relay',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # ========================
    # Gas Obstacle Node
    # ========================
    # Converts gas detection to virtual obstacles in the costmap
    gas_obstacle = Node(
        package='gopigo3_driver',
        executable='gas_obstacle',
        name='gas_obstacle',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'gas_threshold': 300.0,      # ppm - adjust based on MQ-2 sensor calibration
            'obstacle_radius': 0.5,       # meters - size of virtual obstacle
            'inflation_radius': 0.3,      # meters - additional safety margin
            'zone_timeout': 300.0         # seconds - how long zones persist
        }]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time if true'
        ),
        DeclareLaunchArgument(
            'map',
            default_value='/home/ubuntu/maps/my_room',
            description='Full path to map file (without extension)'
        ),
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Automatically start lifecycle nodes'
        ),
        gopigo3_bringup,
        slam_toolbox_node,
        controller_server,
        planner_server,
        smoother_server,
        behavior_server,
        bt_navigator,
        waypoint_follower,
        velocity_smoother,
        lifecycle_manager,
        goal_relay,
        gas_obstacle
    ])
