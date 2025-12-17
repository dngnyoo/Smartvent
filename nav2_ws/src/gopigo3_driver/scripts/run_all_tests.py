#!/usr/bin/env python3
"""
GoPiGo3 ROS2 전체 테스트 및 설정 가이드

이 스크립트는 GoPiGo3 + ROS2 + SLAM + Nav2 설정을 위한
모든 테스트를 순서대로 진행합니다.

사용법:
    python3 run_all_tests.py
"""

import sys
import os
import subprocess

def print_header(title):
    print("\n" + "=" * 70)
    print(f"  {title}")
    print("=" * 70)

def print_step(step_num, title, description):
    print(f"""
┌────────────────────────────────────────────────────────────────────┐
│  Step {step_num}: {title:<55}│
├────────────────────────────────────────────────────────────────────┤
│  {description:<64}│
└────────────────────────────────────────────────────────────────────┘
""")

def run_step(script_name):
    """테스트 스크립트 실행"""
    script_path = os.path.join(os.path.dirname(__file__), script_name)
    try:
        result = subprocess.run([sys.executable, script_path])
        return result.returncode == 0
    except Exception as e:
        print(f"오류: {e}")
        return False

def main():
    print_header("GoPiGo3 + ROS2 Humble + SLAM + Nav2 설정 가이드")

    print("""
    이 가이드는 GoPiGo3 로봇을 ROS2 Humble에서 SLAM과 Nav2로
    자율 주행하도록 설정하는 전체 과정을 안내합니다.

    ┌─────────────────────────────────────────────────────────────┐
    │  구성 요소:                                                  │
    │  • GoPiGo3 보드 + Raspberry Pi 4                            │
    │  • BNO055 IMU 센서                                          │
    │  • RPLidar A1M8                                              │
    │  • Ubuntu 22.04 + ROS2 Humble                                │
    └─────────────────────────────────────────────────────────────┘

    각 단계는 독립적으로 실행할 수도 있습니다:
    • python3 step1_test_gopigo3.py  - GoPiGo3 하드웨어 테스트
    • python3 step2_test_imu.py      - IMU 테스트 및 캘리브레이션
    • python3 step3_test_lidar.py    - LiDAR 테스트
    • python3 step4_test_ros2.py     - ROS2 통합 테스트
    • python3 step5_setup_foxglove.py - Foxglove 설정
    """)

    steps = [
        ("step1_test_gopigo3.py", "GoPiGo3 하드웨어", "모터, 엔코더, SPI 통신 테스트"),
        ("step2_test_imu.py", "BNO055 IMU", "센서 연결, 캘리브레이션 가이드"),
        ("step3_test_lidar.py", "RPLidar A1", "USB 연결, 스캔 데이터 테스트"),
        ("step4_test_ros2.py", "ROS2 통합", "노드, 토픽, TF 테스트"),
        ("step5_setup_foxglove.py", "Foxglove", "원격 시각화 설정"),
    ]

    results = []

    for i, (script, title, desc) in enumerate(steps, 1):
        print_step(i, title, desc)

        response = input(f"Step {i}을 실행하시겠습니까? (y/n/q=종료): ").lower()

        if response == 'q':
            print("\n가이드를 종료합니다.")
            break
        elif response == 'y':
            success = run_step(script)
            results.append((f"Step {i}: {title}", success))

            if not success:
                print(f"\n⚠️  Step {i}에서 문제가 발생했습니다.")
                response = input("계속 진행하시겠습니까? (y/n): ").lower()
                if response != 'y':
                    break
        else:
            results.append((f"Step {i}: {title}", None))  # 건너뜀
            print(f"Step {i} 건너뜀")

    # ========================
    # 최종 결과 요약
    # ========================
    print_header("최종 결과 요약")

    passed = 0
    failed = 0
    skipped = 0

    for name, result in results:
        if result is True:
            print(f"  \033[92m✓ PASS\033[0m  {name}")
            passed += 1
        elif result is False:
            print(f"  \033[91m✗ FAIL\033[0m  {name}")
            failed += 1
        else:
            print(f"  \033[93m- SKIP\033[0m  {name}")
            skipped += 1

    print(f"\n  통과: {passed}  실패: {failed}  건너뜀: {skipped}")

    if failed == 0 and passed > 0:
        print("""
\033[92m
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  🎉 축하합니다! 모든 설정이 완료되었습니다!
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
\033[0m
다음 단계:

1. 워크스페이스 빌드 (변경사항이 있는 경우):
   cd ~/nav2_ws && colcon build --symlink-install && source install/setup.bash

2. SLAM 매핑 시작:
   ros2 launch gopigo3_driver slam_foxglove.launch.py

3. 맥북에서 Foxglove Studio로 시각화:
   - https://studio.foxglove.dev 접속
   - ws://[라즈베리파이IP]:8765 로 연결

4. 텔레오퍼레이션으로 맵 생성:
   ros2 run teleop_twist_keyboard teleop_twist_keyboard

5. 맵 저장:
   ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map

6. 자율 주행 시작:
   ros2 launch gopigo3_driver navigation.launch.py map:=~/maps/my_map.yaml
""")
    elif failed > 0:
        print("""
\033[91m
일부 테스트가 실패했습니다. 각 단계의 오류 메시지를 확인하고 해결하세요.
필요한 경우 개별 테스트 스크립트를 다시 실행할 수 있습니다.
\033[0m
""")


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("\n\n가이드가 중단되었습니다.")
        sys.exit(1)
