#!/usr/bin/env python3
"""
Step 5: Foxglove Bridge 설정

이 스크립트는 맥북에서 Foxglove Studio로 로봇 데이터를 시각화할 수 있도록
라즈베리파이에 Foxglove Bridge를 설정합니다.

Foxglove Bridge는 WebSocket을 통해 ROS2 데이터를 Foxglove Studio로 전송합니다.

사용법:
    python3 step5_setup_foxglove.py
"""

import sys
import os
import subprocess

def print_header(title):
    print("\n" + "=" * 60)
    print(f"  {title}")
    print("=" * 60)

def print_result(test_name, success, message=""):
    status = "✓ PASS" if success else "✗ FAIL"
    color = "\033[92m" if success else "\033[91m"
    reset = "\033[0m"
    print(f"{color}{status}{reset} - {test_name}")
    if message:
        print(f"       {message}")

def wait_for_enter(prompt="계속하려면 Enter를 누르세요..."):
    input(f"\n>>> {prompt}")

def run_command(cmd, timeout=60):
    """명령어 실행"""
    try:
        result = subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=timeout)
        return result.returncode == 0, result.stdout.strip(), result.stderr.strip()
    except subprocess.TimeoutExpired:
        return False, "", "타임아웃"
    except Exception as e:
        return False, "", str(e)

def get_ip_address():
    """현재 IP 주소 가져오기"""
    try:
        import socket
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except:
        return "알 수 없음"

def main():
    print_header("Step 5: Foxglove Bridge 설정")

    print("""
    Foxglove Bridge는 ROS2 데이터를 WebSocket으로 스트리밍하여
    맥북의 Foxglove Studio에서 시각화할 수 있게 합니다.

    장점:
    • 라즈베리파이에서 RViz2를 실행하지 않아도 됨
    • 네트워크를 통해 원격으로 시각화
    • 가벼운 WebSocket 프로토콜 사용
    • 크로스 플랫폼 (Mac, Windows, Linux, 웹브라우저)
    """)

    all_passed = True

    # ========================
    # Step 1: Foxglove Bridge 설치 확인
    # ========================
    print("\n[단계 1/4] Foxglove Bridge 패키지 확인...")

    success, stdout, _ = run_command("ros2 pkg list | grep foxglove_bridge")

    if success and 'foxglove_bridge' in stdout:
        print_result("foxglove_bridge", True, "이미 설치됨")
    else:
        print("    foxglove_bridge 패키지가 설치되어 있지 않습니다.")
        response = input("    지금 설치하시겠습니까? (y/n): ").lower()

        if response == 'y':
            print("\n    foxglove_bridge 설치 중...")
            print("    (인터넷 연결이 필요하며, 1-2분 소요됩니다)")

            success, _, stderr = run_command(
                "sudo apt update && sudo apt install -y ros-humble-foxglove-bridge",
                timeout=300
            )

            if success:
                print_result("foxglove_bridge 설치", True)
            else:
                print_result("foxglove_bridge 설치", False, stderr[:100])
                all_passed = False
        else:
            print("    설치를 건너뜁니다.")
            all_passed = False

    # ========================
    # Step 2: Foxglove Bridge 테스트
    # ========================
    print("\n[단계 2/4] Foxglove Bridge 연결 테스트...")

    ip_address = get_ip_address()
    print(f"\n    라즈베리파이 IP 주소: {ip_address}")
    print("    Foxglove Bridge 기본 포트: 8765")
    print(f"\n    맥북에서 연결할 URL: ws://{ip_address}:8765")

    print("""
    테스트를 위해 Foxglove Bridge를 잠시 실행합니다.
    맥북에서 Foxglove Studio를 열고 연결해보세요.

    맥북에서:
    1. https://foxglove.dev/download 에서 Foxglove Studio 다운로드
    2. 또는 웹 브라우저에서 https://studio.foxglove.dev 접속
    3. "Open connection" → "Foxglove WebSocket" 선택
    4. URL 입력: ws://{ip_address}:8765
    """.format(ip_address=ip_address))

    response = input("Foxglove Bridge 테스트를 진행하시겠습니까? (y/n): ").lower()

    if response == 'y':
        print("\n    Foxglove Bridge 시작 중...")
        print("    (맥북에서 Foxglove Studio로 연결하세요)")
        print("    (Ctrl+C로 종료)\n")

        try:
            subprocess.run(
                "ros2 launch foxglove_bridge foxglove_bridge_launch.xml",
                shell=True,
                timeout=60
            )
        except subprocess.TimeoutExpired:
            print("\n    60초 타임아웃")
        except KeyboardInterrupt:
            print("\n    Bridge 종료됨")

        response = input("\n    맥북에서 연결이 성공했습니까? (y/n): ").lower()
        print_result("Foxglove Bridge 연결", response == 'y')
        if response != 'y':
            all_passed = False

    # ========================
    # Step 3: Launch 파일에 Foxglove 추가
    # ========================
    print("\n[단계 3/4] Foxglove Bridge를 Launch 파일에 통합...")

    foxglove_launch_content = '''#!/usr/bin/env python3
"""
GoPiGo3 + Foxglove Bridge Launch File

이 파일은 로봇 드라이버와 함께 Foxglove Bridge를 실행합니다.
맥북의 Foxglove Studio에서 로봇 데이터를 시각화할 수 있습니다.

사용법:
    ros2 launch gopigo3_driver gopigo3_foxglove.launch.py
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
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
        ])
    )

    # ========================
    # Foxglove Bridge
    # ========================
    foxglove_bridge = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen',
        parameters=[{
            'port': 8765,
            'address': '0.0.0.0',
            'tls': False,
            'topic_whitelist': ['.*'],  # 모든 토픽 허용
            'send_buffer_limit': 10000000,
            'use_sim_time': False,
        }]
    )

    return LaunchDescription([
        gopigo3_bringup,
        foxglove_bridge
    ])
'''

    launch_file = os.path.expanduser(
        "~/nav2_ws/src/gopigo3_driver/launch/gopigo3_foxglove.launch.py"
    )

    try:
        with open(launch_file, 'w') as f:
            f.write(foxglove_launch_content)
        print_result("Foxglove Launch 파일 생성", True, launch_file)
    except Exception as e:
        print_result("Foxglove Launch 파일 생성", False, str(e))
        all_passed = False

    # ========================
    # Step 4: SLAM + Foxglove Launch 파일
    # ========================
    slam_foxglove_content = '''#!/usr/bin/env python3
"""
GoPiGo3 SLAM + Foxglove Bridge Launch File

SLAM 매핑과 Foxglove Bridge를 함께 실행합니다.
맥북에서 실시간으로 맵 생성 과정을 볼 수 있습니다.

사용법:
    ros2 launch gopigo3_driver slam_foxglove.launch.py
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ========================
    # Include SLAM launch
    # ========================
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gopigo3_driver'),
                'launch',
                'slam.launch.py'
            ])
        ])
    )

    # ========================
    # Foxglove Bridge
    # ========================
    foxglove_bridge = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen',
        parameters=[{
            'port': 8765,
            'address': '0.0.0.0',
            'tls': False,
            'topic_whitelist': ['.*'],
            'send_buffer_limit': 10000000,
            'use_sim_time': False,
        }]
    )

    return LaunchDescription([
        slam_launch,
        foxglove_bridge
    ])
'''

    slam_launch_file = os.path.expanduser(
        "~/nav2_ws/src/gopigo3_driver/launch/slam_foxglove.launch.py"
    )

    try:
        with open(slam_launch_file, 'w') as f:
            f.write(slam_foxglove_content)
        print_result("SLAM+Foxglove Launch 파일 생성", True, slam_launch_file)
    except Exception as e:
        print_result("SLAM+Foxglove Launch 파일 생성", False, str(e))
        all_passed = False

    # ========================
    # 결과 및 사용법 안내
    # ========================
    print_header("설정 완료!")

    if all_passed:
        print(f"""
\033[92m모든 설정이 완료되었습니다!\033[0m

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

📡 라즈베리파이 IP 주소: {ip_address}
🔗 Foxglove 연결 URL: ws://{ip_address}:8765

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

▶ 사용 방법:

1. 워크스페이스 다시 빌드:
   cd ~/nav2_ws
   colcon build --symlink-install
   source install/setup.bash

2. 로봇 + Foxglove 시작:
   ros2 launch gopigo3_driver gopigo3_foxglove.launch.py

3. SLAM + Foxglove 시작:
   ros2 launch gopigo3_driver slam_foxglove.launch.py

4. 맥북에서 Foxglove Studio 연결:
   - 앱: https://foxglove.dev/download
   - 웹: https://studio.foxglove.dev
   - "Open connection" → "Foxglove WebSocket"
   - URL: ws://{ip_address}:8765

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

📊 Foxglove Studio에서 추천 패널:

• 3D: 로봇 모델, TF, 라이다 스캔, 맵 시각화
• Raw Messages: 토픽 데이터 확인
• Plot: IMU, 오도메트리 그래프
• Image: 카메라 영상 (사용 시)
• Map: 2D 맵 뷰
• Teleop: 조이스틱으로 로봇 제어

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
""".format(ip_address=ip_address))
    else:
        print("\033[91m일부 설정이 실패했습니다. 위의 오류를 확인하세요.\033[0m")

    return all_passed


if __name__ == '__main__':
    try:
        success = main()
        sys.exit(0 if success else 1)
    except KeyboardInterrupt:
        print("\n설정 중단됨")
        sys.exit(1)
