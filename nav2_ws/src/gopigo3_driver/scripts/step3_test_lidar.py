#!/usr/bin/env python3
"""
Step 3: RPLidar A1 Test

이 스크립트는 RPLidar A1M8 센서가 올바르게 연결되어 있고
데이터를 읽을 수 있는지 확인합니다.

Usage:
    python3 step3_test_lidar.py
"""

import sys
import os
import time
import subprocess

def print_header(title):
    print("\n" + "=" * 60)
    print(f"  {title}")
    print("=" * 60)

def print_result(test_name, success, message=""):
    status = "PASS" if success else "FAIL"
    color = "\033[92m" if success else "\033[91m"
    reset = "\033[0m"
    symbol = "[OK]" if success else "[X]"
    print(f"{color}{symbol} {status}{reset} - {test_name}")
    if message:
        print(f"       {message}")

def wait_for_enter(prompt="Press Enter to continue..."):
    input(f"\n>>> {prompt}")

def run_command(cmd):
    """명령어 실행 후 결과 반환"""
    try:
        result = subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=10)
        return result.returncode == 0, result.stdout.strip()
    except subprocess.TimeoutExpired:
        return False, "Command timeout"
    except Exception as e:
        return False, str(e)

def main():
    print_header("Step 3: RPLidar A1 Test")

    all_passed = True

    # ========================
    # Test 1: USB 장치 확인
    # ========================
    print("\n[Test 1/4] Checking USB Device...")

    # ttyUSB 장치 찾기
    usb_devices = []
    for dev in os.listdir('/dev'):
        if dev.startswith('ttyUSB'):
            usb_devices.append(f"/dev/{dev}")

    if usb_devices:
        print_result("USB Device", True, f"Found: {', '.join(usb_devices)}")
    else:
        print_result("USB Device", False, "No ttyUSB device found")
        print("       Check:")
        print("       1. RPLidar USB cable is connected")
        print("       2. Try a different USB port")
        print("       3. Run: dmesg | tail to see USB connection log")
        all_passed = False

    # ========================
    # Test 2: USB 권한 확인
    # ========================
    print("\n[Test 2/4] Checking USB Permissions...")

    if usb_devices:
        device = usb_devices[0]  # 첫 번째 장치 사용
        readable = os.access(device, os.R_OK)
        writable = os.access(device, os.W_OK)

        if readable and writable:
            print_result("USB Permissions", True, f"{device} read/write OK")
        else:
            print_result("USB Permissions", False, f"{device} permission denied")
            print("       Fix:")
            print("       sudo usermod -aG dialout $USER")
            print("       Then logout/login or reboot")
            all_passed = False
    else:
        print_result("USB Permissions", False, "No USB device to test")
        all_passed = False

    # ========================
    # Test 3: ROS2 rplidar_ros 패키지 확인
    # ========================
    print("\n[Test 3/4] Checking ROS2 rplidar_ros Package...")

    success, output = run_command("ros2 pkg list | grep rplidar")

    if success and 'rplidar' in output:
        print_result("rplidar_ros Package", True, "Installed")
    else:
        print_result("rplidar_ros Package", False, "Not installed")
        print("       Install: sudo apt install ros-humble-rplidar-ros")
        all_passed = False

    # ========================
    # Test 4: RPLidar 실제 테스트
    # ========================
    print("\n[Test 4/4] RPLidar Connection Test...")

    if not usb_devices:
        print_result("RPLidar Connection", False, "No USB device")
        all_passed = False
    else:
        print("""
    This test will start the RPLidar and check if it receives scan data.
    The LiDAR motor will spin during this test.

    [!] WARNING: LiDAR motor will spin!
        """)

        response = input("Proceed with test? (y/n): ").lower()

        if response == 'y':
            device = usb_devices[0]
            print(f"\n    Testing RPLidar on {device}...")
            print("    (10 second test, will auto-stop)")
            print("    (Ctrl+C to cancel early)\n")

            # rplidar_ros 노드 실행
            cmd = f"""
            timeout 10 ros2 run rplidar_ros rplidar_node --ros-args \
                -p serial_port:={device} \
                -p serial_baudrate:=115200 \
                -p frame_id:=laser_frame \
                2>&1 | head -20
            """

            try:
                process = subprocess.Popen(
                    cmd, shell=True,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                    text=True
                )

                # 출력 읽기
                output_lines = []
                start_time = time.time()

                while time.time() - start_time < 12:
                    line = process.stdout.readline()
                    if not line and process.poll() is not None:
                        break
                    if line:
                        output_lines.append(line.strip())
                        print(f"    {line.strip()}")

                process.terminate()

                # 결과 분석
                output_text = "\n".join(output_lines)
                if "RPLIDAR S/N" in output_text or "health status" in output_text:
                    print_result("RPLidar Connection", True, "LiDAR working correctly")
                elif "cannot" in output_text.lower() or "error" in output_text.lower():
                    print_result("RPLidar Connection", False, "Connection error")
                    all_passed = False
                else:
                    print_result("RPLidar Connection", True, "Node started (verify manually)")

            except KeyboardInterrupt:
                print("\n    Test cancelled")
            except Exception as e:
                print_result("RPLidar Connection", False, str(e))
                all_passed = False
        else:
            print("    Test skipped")

    # ========================
    # udev 규칙 설정 안내
    # ========================
    print("\n" + "-" * 60)
    print("  Recommended: Set up udev rules")
    print("-" * 60)
    print("""
    You can set up a udev rule so RPLidar is always at /dev/rplidar:

    sudo bash -c 'cat > /etc/udev/rules.d/99-rplidar.rules << EOF
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", \\
    MODE:="0666", GROUP:="dialout", SYMLINK+="rplidar"
EOF'

    sudo udevadm control --reload-rules
    sudo udevadm trigger
    """)

    response = input("Set up udev rule now? (y/n): ").lower()

    if response == 'y':
        rule = 'SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0666", GROUP:="dialout", SYMLINK+="rplidar"'
        cmd = f"echo '{rule}' | sudo tee /etc/udev/rules.d/99-rplidar.rules"

        success, _ = run_command(cmd)
        if success:
            run_command("sudo udevadm control --reload-rules")
            run_command("sudo udevadm trigger")
            print("    [OK] udev rule configured.")
            print("    Reconnect USB to access via /dev/rplidar")
        else:
            print("    [X] Failed to configure udev rule")

    # ========================
    # 결과 요약
    # ========================
    print_header("Test Results Summary")
    if all_passed:
        print("\033[92mAll tests passed! Proceed to Step 4.\033[0m")
        print("\nNext step: python3 step4_test_ros2.py")
    else:
        print("\033[91mSome tests failed. Please fix the errors above.\033[0m")

    return all_passed


if __name__ == '__main__':
    try:
        # ROS2 환경 확인
        if 'ROS_DISTRO' not in os.environ:
            print("[!] Warning: ROS2 environment not set.")
            print("   Run: source /opt/ros/humble/setup.bash")

        success = main()
        sys.exit(0 if success else 1)
    except KeyboardInterrupt:
        print("\nTest interrupted")
        sys.exit(1)
