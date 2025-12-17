from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'gopigo3_driver'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'docs'), glob('docs/*.md')),
    ],
    install_requires=['setuptools', 'spidev'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@gopigo3.local',
    description='GoPiGo3 ROS2 driver with IMU support for SLAM and Nav2',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'gopigo3_driver = gopigo3_driver.gopigo3_driver_node:main',
            

            'chevron_cmd_vel_node = gopigo3_driver.chevron_cmd_vel_node:main',
            
            'bno055_imu = gopigo3_driver.bno055_imu_node:main',
            'bno055_imu_raw = gopigo3_driver.bno055_imu_node:main_raw',
            'coverage_explorer = gopigo3_driver.coverage_explorer_node:main',
            'goal_relay = gopigo3_driver.goal_relay_node:main',
            'waypoint_navigator = gopigo3_driver.waypoint_navigator_node:main',
            'gap_detector_test = gopigo3_driver.gap_detector_test_node:main',
            'wall_following_explorer = gopigo3_driver.wall_following_explorer_node:main',
            'map_saver = gopigo3_driver.map_saver_node:main',
            'go_to_start = gopigo3_driver.go_to_start_node:main',
            'gas_obstacle = gopigo3_driver.gas_obstacle_node:main',
            'gas_simulator = gopigo3_driver.gas_simulator_node:main',
            'patrol_evacuation = gopigo3_driver.patrol_evacuation_node:main',
            'gas_publisher = gopigo3_driver.gas_publisher_node:main',
        ],
    },
)