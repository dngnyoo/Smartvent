#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Package directories
    pkg_share = FindPackageShare('gopigo3_driver').find('gopigo3_driver')

    # Paths
    urdf_file = os.path.join(pkg_share, 'urdf', 'gopigo3.urdf.xacro')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # Process the URDF file
    robot_description = Command(['xacro ', urdf_file])

    # Nodes
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description
        }]
    )

    gopigo3_driver_node = Node(
        package='gopigo3_driver',
        executable='gopigo3_driver',
        name='gopigo3_driver',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'wheel_base': 0.117,
            'wheel_diameter': 0.066,
            'encoder_ticks_per_rotation': 360,
            'publish_rate': 20.0
        }]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time if true'
        ),
        robot_state_publisher_node,
        gopigo3_driver_node
    ])
