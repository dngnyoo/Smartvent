from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'gopigo_robot'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='User',
    maintainer_email='user@todo.todo',
    description='GoPiGo3 ROS2 Custom Package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sensors = gopigo_robot.modules.sensing_node:main',
            'actions = gopigo_robot.modules.action_node:main',
            'controller = gopigo_robot.modules.control_node:main',
        ],
    },
)