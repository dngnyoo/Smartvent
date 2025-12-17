#!/usr/bin/env python3
"""
GoPiGo3 SLAM Launch File

This launches everything needed for SLAM mapping:
- gopigo3_bringup (robot drivers, rplidar, ekf, foxglove)
- slam_toolbox (SLAM)

Usage:
    ros2 launch gopigo3_driver slam.launch.py

To save the map:
    ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map
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
    slam_config = os.path.join(pkg_share, 'config', 'slam_toolbox.yaml')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    enable_ekf = LaunchConfiguration('ekf', default='false')  # EKF disabled by default for stability

    # ========================
    # Include gopigo3_bringup
    # ========================
    # This includes: robot_state_publisher, gopigo3_driver, bno055_imu,
    #                rplidar, ekf, foxglove_bridge
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
            'ekf': enable_ekf
        }.items()
    )

    # ========================
    # SLAM Toolbox
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

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time if true'
        ),
        DeclareLaunchArgument(
            'ekf',
            default_value='false',
            description='Enable EKF sensor fusion (disabled by default due to IMU instability)'
        ),
        gopigo3_bringup,
        slam_toolbox_node
    ])
