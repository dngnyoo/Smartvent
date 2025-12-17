#!/usr/bin/env python3
"""
Step 4: ROS2 통합 테스트

이 스크립트는 모든 ROS2 노드가 올바르게 작동하는지 확인합니다:
- gopigo3_driver (모터 + 오도메트리)
- bno055_imu (IMU)
- 토픽 발행 확인
- TF 변환 확인

사용법:
    python3 step4_test_ros2.py
"""

import sys
import os
import time
import subprocess
import signal

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

def run_command(cmd, timeout=10):
    """명령어 실행 후 결과 반환"""
    try:
        result = subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=timeout)
        return result.returncode == 0, result.stdout.strip(), result.stderr.strip()
    except subprocess.TimeoutExpired:
        return False, "", "명령어 타임아웃"
    except Exception as e:
        return False, "", str(e)

def check_topic_exists(topic_name, timeout=5):
    """토픽이 존재하고 데이터가 발행되는지 확인"""
    cmd = f"timeout {timeout} ros2 topic echo {topic_name} --once"
    success, stdout, stderr = run_command(cmd, timeout + 2)
    return success and stdout, stdout

def main():
    print_header("Step 4: ROS2 통합 테스트")

    # ROS2 환경 확인
    if 'ROS_DISTRO' not in os.environ:
        print("\033[91m오류: ROS2 환경이 설정되지 않았습니다.\033[0m")
        print("다음 명령어를 실행하세요:")
        print("  source /opt/ros/humble/setup.bash")
        print("  source ~/nav2_ws/install/setup.bash")
        return False

    print(f"ROS2 버전: {os.environ.get('ROS_DISTRO', 'unknown')}")

    all_passed = True
    nodes_running = False
    launch_process = None

    # ========================
    # Test 1: 패키지 빌드 확인
    # ========================
    print("\n[테스트 1/5] gopigo3_driver 패키지 확인...")

    success, stdout, _ = run_command("ros2 pkg list | grep gopigo3_driver")

    if success and 'gopigo3_driver' in stdout:
        print_result("gopigo3_driver 패키지", True, "설치됨")
    else:
        print_result("gopigo3_driver 패키지", False, "패키지를 찾을 수 없음")
        print("       해결: cd ~/nav2_ws && colcon build && source install/setup.bash")
        all_passed = False
        return all_passed

    # ========================
    # Test 2: 필요한 ROS2 패키지 확인
    # ========================
    print("\n[테스트 2/5] 필요한 ROS2 패키지 확인...")

    required_packages = [
        ('robot_state_publisher', 'ros-humble-robot-state-publisher'),
        ('robot_localization', 'ros-humble-robot-localization'),
        ('slam_toolbox', 'ros-humble-slam-toolbox'),
        ('nav2_bringup', 'ros-humble-navigation2'),
        ('rplidar_ros', 'ros-humble-rplidar-ros'),
    ]

    missing_packages = []
    for pkg, apt_name in required_packages:
        success, stdout, _ = run_command(f"ros2 pkg list | grep {pkg}")
        if success and pkg in stdout:
            print(f"    ✓ {pkg}")
        else:
            print(f"    ✗ {pkg} - sudo apt install {apt_name}")
            missing_packages.append(apt_name)

    if missing_packages:
        print_result("필요한 패키지", False, f"{len(missing_packages)}개 패키지 누락")
        print(f"       설치: sudo apt install {' '.join(missing_packages)}")
        all_passed = False
    else:
        print_result("필요한 패키지", True, "모두 설치됨")

    # ========================
    # Test 3: 노드 실행 테스트
    # ========================
    print("\n[테스트 3/5] gopigo3_bringup 실행 테스트...")
    print("""
    GoPiGo3 드라이버 노드를 실행하여 토픽이 발행되는지 확인합니다.
    이 테스트는 약 15초 소요됩니다.

    ⚠️  주의: 이 테스트 동안 모터 명령을 보내지 마세요.
    """)

    response = input("테스트를 진행하시겠습니까? (y/n): ").lower()

    if response != 'y':
        print("    테스트 건너뜀")
    else:
        print("\n    gopigo3_bringup 시작 중...")

        # Launch 파일 실행
        launch_cmd = "ros2 launch gopigo3_driver gopigo3_bringup.launch.py"
        launch_process = subprocess.Popen(
            launch_cmd,
            shell=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            preexec_fn=os.setsid
        )
        nodes_running = True

        # 노드가 시작될 때까지 대기
        print("    노드 시작 대기 중 (10초)...")
        time.sleep(10)

        # ========================
        # Test 4: 토픽 확인
        # ========================
        print("\n[테스트 4/5] 토픽 발행 확인...")

        topics_to_check = [
            ('/odom', 'nav_msgs/msg/Odometry', '휠 오도메트리'),
            ('/joint_states', 'sensor_msgs/msg/JointState', '조인트 상태'),
            ('/imu/data', 'sensor_msgs/msg/Imu', 'IMU 데이터'),
            ('/tf', 'tf2_msgs/msg/TFMessage', 'TF 변환'),
        ]

        for topic, msg_type, desc in topics_to_check:
            success, data = check_topic_exists(topic, timeout=5)
            if success:
                print_result(f"{desc} ({topic})", True, "데이터 수신됨")
            else:
                print_result(f"{desc} ({topic})", False, "데이터 없음")
                all_passed = False

        # ========================
        # Test 5: TF 트리 확인
        # ========================
        print("\n[테스트 5/5] TF 트리 확인...")

        success, stdout, _ = run_command("ros2 run tf2_tools view_frames --ros-args -p output_file:=/tmp/frames", timeout=8)

        # tf2_ros로 프레임 확인
        success, stdout, _ = run_command("ros2 run tf2_ros tf2_echo odom base_footprint", timeout=5)

        if "Transform" in stdout or "Translation" in stdout:
            print_result("TF: odom -> base_footprint", True)
        else:
            print_result("TF: odom -> base_footprint", False, "변환 없음")
            all_passed = False

        success, stdout, _ = run_command("ros2 run tf2_ros tf2_echo base_link imu_link", timeout=5)
        if "Transform" in stdout or "Translation" in stdout:
            print_result("TF: base_link -> imu_link", True)
        else:
            print_result("TF: base_link -> imu_link", False, "변환 없음")
            all_passed = False

    # ========================
    # 정리
    # ========================
    if nodes_running and launch_process:
        print("\n    노드 종료 중...")
        try:
            os.killpg(os.getpgid(launch_process.pid), signal.SIGTERM)
            launch_process.wait(timeout=5)
        except:
            try:
                os.killpg(os.getpgid(launch_process.pid), signal.SIGKILL)
            except:
                pass
        print("    ✓ 노드 종료됨")

    # ========================
    # cmd_vel 테스트 (선택)
    # ========================
    if all_passed:
        print("\n" + "─" * 60)
        print("  🎮 모터 제어 테스트 (선택)")
        print("─" * 60)
        print("""
    cmd_vel 토픽으로 모터를 제어할 수 있는지 테스트합니다.

    ⚠️  경고: 로봇이 움직입니다! 바닥에 놓거나 들어 올리세요.
        """)

        response = input("모터 제어 테스트를 진행하시겠습니까? (y/n): ").lower()

        if response == 'y':
            wait_for_enter("로봇을 준비하고 Enter를 누르세요...")

            # 노드 다시 시작
            print("\n    노드 시작 중...")
            launch_process = subprocess.Popen(
                "ros2 launch gopigo3_driver gopigo3_bringup.launch.py",
                shell=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                preexec_fn=os.setsid
            )
            time.sleep(8)

            try:
                # 전진
                print("\n    테스트 1: 전진 (2초)...")
                subprocess.run(
                    'ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}, angular: {z: 0.0}}"',
                    shell=True, timeout=5
                )
                time.sleep(2)

                # 정지
                subprocess.run(
                    'ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}"',
                    shell=True, timeout=5
                )
                time.sleep(1)

                # 회전
                print("    테스트 2: 제자리 회전 (2초)...")
                subprocess.run(
                    'ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.5}}"',
                    shell=True, timeout=5
                )
                time.sleep(2)

                # 정지
                subprocess.run(
                    'ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}"',
                    shell=True, timeout=5
                )

                response = input("\n    로봇이 올바르게 움직였습니까? (y/n): ").lower()
                print_result("모터 제어", response == 'y')

            except Exception as e:
                print(f"    오류: {e}")
            finally:
                # 정지 명령
                subprocess.run(
                    'ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}"',
                    shell=True, timeout=5
                )
                # 노드 종료
                try:
                    os.killpg(os.getpgid(launch_process.pid), signal.SIGTERM)
                    launch_process.wait(timeout=5)
                except:
                    pass

    # ========================
    # 결과 요약
    # ========================
    print_header("테스트 결과 요약")
    if all_passed:
        print("\033[92m모든 테스트 통과! Step 5 (Foxglove 설정)로 진행하세요.\033[0m")
        print("\n다음 단계: python3 step5_setup_foxglove.py")
    else:
        print("\033[91m일부 테스트 실패. 위의 오류를 해결하세요.\033[0m")

    return all_passed


if __name__ == '__main__':
    try:
        success = main()
        sys.exit(0 if success else 1)
    except KeyboardInterrupt:
        print("\n테스트 중단됨")
        sys.exit(1)
