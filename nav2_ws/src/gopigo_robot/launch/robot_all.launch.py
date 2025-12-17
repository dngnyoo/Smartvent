import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    # 다른 런치 파일을 불러오는 부분(IncludeLaunchDescription)은 
    # 참조할 필요가 없으므로 모두 삭제했습니다.

    return LaunchDescription([
        # 1. 감각 노드 (Sensors)
        Node(
            package='gopigo_robot',
            executable='sensors',        # setup.py의 entry_points 이름
            name='sensing_node',
            output='screen'
        ),

        # 2. 표현 노드 (Actions)
        Node(
            package='gopigo_robot',
            executable='actions',        # setup.py의 entry_points 이름
            name='action_node',
            output='screen'
        ),

        # 3. 두뇌 노드 (Controller)
        Node(
            package='gopigo_robot',
            executable='controller',     # setup.py의 entry_points 이름
            name='control_node',
            output='screen'
        ),
    ])