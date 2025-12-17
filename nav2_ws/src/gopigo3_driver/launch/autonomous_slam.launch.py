#!/usr/bin/env python3
"""
GoPiGo3 Autonomous SLAM Launch File

This launches everything needed for autonomous exploration and mapping:
- gopigo3_bringup (robot drivers, sensors, EKF)
- slam_toolbox (SLAM mapping)
- Nav2 (navigation stack for path planning and obstacle avoidance)
- explore_lite (autonomous frontier exploration)

The robot will automatically explore the environment, avoiding obstacles
and creating a map without manual teleop control.

Usage:
    ros2 launch gopigo3_driver autonomous_slam.launch.py

To save the map when finished:
    # Using slam_toolbox serialization (recommended):
    ros2 service call /slam_toolbox/serialize_pose_graph slam_toolbox/srv/SerializePoseGraph "{filename: '/home/ubuntu/maps/my_home'}"

    # Or using map_saver:
    ros2 run nav2_map_server map_saver_cli -f ~/maps/my_home

Prerequisites:
    - m-explore-ros2 package installed:
      cd ~/nav2_ws/src
      git clone -b humble https://github.com/robo-friends/m-explore-ros2.git
      cd ~/nav2_ws && colcon build --packages-select explore_lite
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Package directories
    pkg_share = FindPackageShare('gopigo3_driver').find('gopigo3_driver')

    # Config files
    nav2_config = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    slam_config = os.path.join(pkg_share, 'config', 'slam_toolbox.yaml')
    explore_config = os.path.join(pkg_share, 'config', 'explore.yaml')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    autostart = LaunchConfiguration('autostart', default='true')

    # ========================
    # Include gopigo3_bringup
    # ========================
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
    # SLAM Toolbox (Mapping Mode)
    # ========================
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_config,
            {'use_sim_time': use_sim_time}
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
    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
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
    # NOTE: explore_lite uses /global_costmap/costmap from planner_server
    # which is managed by this lifecycle manager
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
                'velocity_smoother'
            ]
        }]
    )

    # ========================
    # Explore Lite (Frontier Exploration)
    # ========================
    # Delayed start to allow SLAM and Nav2 to initialize
    explore_node = TimerAction(
        period=10.0,  # Wait 10 seconds for SLAM and Nav2 to start
        actions=[
            Node(
                package='explore_lite',
                executable='explore',
                name='explore_node',
                output='screen',
                parameters=[explore_config, {'use_sim_time': use_sim_time}]
            )
        ]
    )

    # ========================
    # Coverage Explorer (Backup exploration)
    # ========================
    # Monitors progress and performs zigzag coverage if frontier exploration stalls
    coverage_explorer_node = TimerAction(
        period=15.0,  # Wait 15 seconds for explore_lite to start first
        actions=[
            Node(
                package='gopigo3_driver',
                executable='coverage_explorer',
                name='coverage_explorer',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'coverage_spacing': 0.5,    # 50cm between zigzag lines
                    'coverage_margin': 0.3,     # 30cm from walls
                    'coverage_passes': 2,       # 2 coverage passes
                    'frontier_timeout': 30.0,   # Switch to coverage after 30s no progress
                    'goal_tolerance': 0.3       # 30cm goal tolerance
                }]
            )
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time if true'
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
        velocity_smoother,
        lifecycle_manager,
        explore_node,
        coverage_explorer_node
    ])
