#!/usr/bin/env python3
"""
GoPiGo3 Robot Bringup Launch File

This launches all robot drivers and sensors:
- robot_state_publisher (URDF/TF)
- gopigo3_driver (motor control, odometry)
- chevron_cmd_vel_node (GUI display)
- bno055_imu (IMU sensor)
- gas_publisher (gas sensor)
- robot_localization EKF (sensor fusion)
- rplidar (LiDAR sensor)
- foxglove_bridge (visualization)
- slam_toolbox (optional, for SLAM mapping)
- patrol_evacuation (optional, for autonomous patrol)
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Package directories
    pkg_share = FindPackageShare('gopigo3_driver').find('gopigo3_driver')

    # Paths
    urdf_file = os.path.join(pkg_share, 'urdf', 'gopigo3.urdf.xacro')
    ekf_config_file = os.path.join(pkg_share, 'config', 'ekf.yaml')
    slam_config_file = os.path.join(pkg_share, 'config', 'slam_toolbox.yaml')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    enable_lidar = LaunchConfiguration('lidar', default='true')
    enable_foxglove = LaunchConfiguration('foxglove', default='true')
    enable_ekf = LaunchConfiguration('ekf', default='true')
    enable_slam = LaunchConfiguration('slam', default='false')
    enable_gas_sensor = LaunchConfiguration('gas_sensor', default='true')
    enable_patrol = LaunchConfiguration('patrol', default='false')
    lidar_serial_port = LaunchConfiguration('lidar_port', default='/dev/rplidar')

    # Process the URDF file - wrap in ParameterValue to avoid YAML parsing issues
    robot_description = ParameterValue(Command(['xacro ', urdf_file]), value_type=str)

    # ========================
    # Robot State Publisher
    # ========================
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

    # ========================
    # GoPiGo3 Driver
    # ========================
    gopigo3_driver_node = TimerAction(
        period=2.0,
        actions=[Node(
            package='gopigo3_driver',
            executable='gopigo3_driver',
            name='gopigo3_driver',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'wheel_base': 0.117,
                'wheel_diameter': 0.0665,
                'encoder_ticks_per_rotation': 6,
                'motor_gear_ratio': 120,
                'publish_rate': 20.0,
                'config_file_path': '/home/ubuntu/Dexter/gpg3_config.json',
                'publish_tf': True
            }]
        )]
    )

    # ========================
    # [추가됨] Chevron Display
    # ========================
    chevron_cmd_vel_node = Node(
        package='gopigo3_driver',
        executable='chevron_cmd_vel_node', # setup.py의 entry_points 이름과 일치해야 함
        name='chevron_display_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
        }]
    )

    # ========================
    # BNO055 IMU Driver
    # ========================
    bno055_imu_node = Node(
        package='gopigo3_driver',
        executable='bno055_imu',
        name='bno055_imu',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'frame_id': 'imu_link',
            'publish_rate': 50.0,
            'i2c_bus': 'RPI_1SW'
        }]
    )

    # ========================
    # RPLidar A1
    # ========================
    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_node',
        output='screen',
        condition=IfCondition(enable_lidar),
        parameters=[{
            'serial_port': lidar_serial_port,
            'serial_baudrate': 115200,
            'frame_id': 'laser_frame',
            'angle_compensate': True,
            'scan_mode': 'Sensitivity'
        }]
    )

    # ========================
    # Robot Localization EKF
    # ========================
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        condition=IfCondition(enable_ekf),
        parameters=[ekf_config_file]
    )

    # ========================
    # Foxglove Bridge
    # ========================
    foxglove_bridge_node = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen',
        condition=IfCondition(enable_foxglove),
        parameters=[{
            'port': 8765,
            'address': '0.0.0.0',
            'tls': False,
            'topic_whitelist': ['.*'],
            'send_buffer_limit': 10000000,
            'use_sim_time': use_sim_time
        }]
    )

    # ========================
    # SLAM Toolbox (optional)
    # ========================
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        condition=IfCondition(enable_slam),
        parameters=[
            slam_config_file,
            {'use_sim_time': use_sim_time}
        ]
    )

    # ========================
    # Gas Sensor Publisher
    # ========================
    gas_publisher_node = Node(
        package='gopigo3_driver',
        executable='gas_publisher',
        name='gas_publisher',
        output='screen',
        condition=IfCondition(enable_gas_sensor),
        parameters=[{
            'publish_rate': 1.0,
            'gas_threshold_voltage': 1.0,
            'enable_audio_alert': True,
            'alert_cooldown': 5.0
        }]
    )

    # ========================
    # Patrol Evacuation (optional)
    # Fetches gas_threshold from gas_publisher node automatically
    # ========================
    patrol_evacuation_node = Node(
        package='gopigo3_driver',
        executable='patrol_evacuation',
        name='patrol_evacuation',
        output='screen',
        condition=IfCondition(enable_patrol),
        parameters=[{
            'patrol_enabled': True
        }]
    )

    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time if true'
        ),
        DeclareLaunchArgument(
            'lidar',
            default_value='true',
            description='Enable LiDAR sensor'
        ),
        DeclareLaunchArgument(
            'foxglove',
            default_value='true',
            description='Enable Foxglove Bridge for visualization'
        ),
        DeclareLaunchArgument(
            'lidar_port',
            default_value='/dev/rplidar',
            description='LiDAR serial port (use /dev/rplidar symlink)'
        ),
        DeclareLaunchArgument(
            'ekf',
            default_value='true',
            description='Enable EKF sensor fusion (wheel odom + IMU)'
        ),
        DeclareLaunchArgument(
            'slam',
            default_value='false',
            description='Enable SLAM Toolbox for mapping'
        ),
        DeclareLaunchArgument(
            'gas_sensor',
            default_value='true',
            description='Enable gas sensor publisher'
        ),
        DeclareLaunchArgument(
            'patrol',
            default_value='false',
            description='Enable patrol and evacuation mode'
        ),
        # Launch nodes
        robot_state_publisher_node,
        gopigo3_driver_node,
        chevron_cmd_vel_node,
        bno055_imu_node,
        gas_publisher_node,
        ekf_node,
        rplidar_node,
        foxglove_bridge_node,
        slam_toolbox_node,
        patrol_evacuation_node
    ])