#!/usr/bin/env python3
"""
Step 2: BNO055 IMU Sensor Test and Calibration

이 스크립트는 BNO055 IMU 센서 연결을 확인하고,
사용자가 단계별로 캘리브레이션을 수행할 수 있도록 안내합니다.

Usage:
    python3 step2_test_imu.py
"""

import sys
import time

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

def print_calibration_status(sys_cal, gyro_cal, accel_cal, mag_cal):
    """캘리브레이션 상태를 시각적으로 표시"""
    def bar(level):
        filled = "=" * level
        empty = "-" * (3 - level)
        return f"[{filled}{empty}]"

    print(f"\r  System: {bar(sys_cal)} {sys_cal}/3  |  "
          f"Gyro: {bar(gyro_cal)} {gyro_cal}/3  |  "
          f"Accel: {bar(accel_cal)} {accel_cal}/3  |  "
          f"Mag: {bar(mag_cal)} {mag_cal}/3", end="", flush=True)

def wait_for_enter(prompt="Press Enter to continue..."):
    input(f"\n>>> {prompt}")

def main():
    print_header("Step 2: BNO055 IMU Sensor Test and Calibration")

    all_passed = True

    # ========================
    # Test 1: I2C 장치 확인
    # ========================
    print("\n[Test 1/4] Checking I2C Device...")
    import os
    i2c_exists = os.path.exists('/dev/i2c-1')
    print_result("I2C Device", i2c_exists,
                 "/dev/i2c-1 found" if i2c_exists else "/dev/i2c-1 not found - I2C not enabled")
    if not i2c_exists:
        print("       Fix: sudo raspi-config -> Interface Options -> I2C -> Enable")
        all_passed = False

    # ========================
    # Test 2: DI_Sensors 라이브러리
    # ========================
    print("\n[Test 2/4] Checking DI_Sensors Library...")
    try:
        sys.path.insert(0, '/home/ubuntu/DI_Sensors/Python')
        from di_sensors.inertial_measurement_unit import InertialMeasurementUnit
        print_result("DI_Sensors Library", True, "InertialMeasurementUnit imported successfully")
    except ImportError as e:
        print_result("DI_Sensors Library", False, str(e))
        print("       Fix: cd ~/DI_Sensors/Python && sudo python3 setup.py install")
        all_passed = False
        return all_passed

    # ========================
    # Test 3: IMU 연결
    # ========================
    print("\n[Test 3/4] Checking BNO055 IMU Connection...")
    print("       Initializing IMU... (takes about 2 seconds)")
    try:
        imu = InertialMeasurementUnit(bus="RPI_1SW")
        revision = imu.BNO055.get_revision()
        print_result("BNO055 IMU Connection", True)
        print(f"       SW Rev: {revision[0]}, BL Rev: {revision[1]}")
        print(f"       Accel ID: {revision[2]}, Mag ID: {revision[3]}, Gyro ID: {revision[4]}")
    except Exception as e:
        print_result("BNO055 IMU Connection", False, str(e))
        print("       Check:")
        print("       1. IMU connected to GoPiGo3 I2C port")
        print("       2. I2C address: sudo i2cdetect -y 1 (look for 0x28 or 0x29)")
        all_passed = False
        return all_passed

    # ========================
    # Test 4: 센서 데이터 읽기
    # ========================
    print("\n[Test 4/4] Reading Sensor Data...")
    try:
        # 쿼터니언
        qx, qy, qz, qw = imu.read_quaternion()
        print(f"       Quaternion: x={qx:.3f}, y={qy:.3f}, z={qz:.3f}, w={qw:.3f}")

        # 오일러 각
        euler = imu.read_euler()
        print(f"       Euler: heading={euler[0]:.1f} deg, roll={euler[1]:.1f} deg, pitch={euler[2]:.1f} deg")

        # 자이로스코프
        gyro = imu.read_gyroscope()
        print(f"       Gyroscope: x={gyro[0]:.2f}, y={gyro[1]:.2f}, z={gyro[2]:.2f} deg/s")

        # 가속도계
        accel = imu.read_accelerometer()
        print(f"       Accelerometer: x={accel[0]:.2f}, y={accel[1]:.2f}, z={accel[2]:.2f} m/s^2")

        # 지자기 센서
        mag = imu.read_magnetometer()
        print(f"       Magnetometer: x={mag[0]:.2f}, y={mag[1]:.2f}, z={mag[2]:.2f} uT")

        print_result("Sensor Data Reading", True)
    except Exception as e:
        print_result("Sensor Data Reading", False, str(e))
        all_passed = False

    # ========================
    # 캘리브레이션 섹션
    # ========================
    print_header("BNO055 Calibration Guide")

    print("""
The BNO055 requires calibration of 4 sensors:

1. Gyroscope - Fastest to calibrate
   -> Keep the robot stationary on a flat surface

2. Accelerometer
   -> Place robot in 6 different positions (like faces of a cube)

3. Magnetometer - Takes longest
   -> Move robot slowly in a figure-8 pattern

4. System
   -> Automatically calibrated when above 3 are done
""")

    response = input("Do you want to start calibration? (y/n): ").lower()

    if response != 'y':
        print("\nSkipping calibration.")
        print("Note: The sensor will still work but with reduced accuracy.")
    else:
        calibrate_imu(imu)

    # ========================
    # 실시간 데이터 확인 (선택)
    # ========================
    print("\n" + "-" * 60)
    response = input("Do you want to view real-time sensor data? (y/n): ").lower()

    if response == 'y':
        print("\nReal-time data (Ctrl+C to stop):\n")
        try:
            while True:
                euler = imu.read_euler()
                gyro = imu.read_gyroscope()
                sys_cal, gyro_cal, accel_cal, mag_cal = imu.BNO055.get_calibration_status()

                print(f"\rHeading: {euler[0]:7.2f} deg  Roll: {euler[1]:7.2f} deg  Pitch: {euler[2]:7.2f} deg  "
                      f"GyroZ: {gyro[2]:7.2f} deg/s  "
                      f"Cal[S:{sys_cal} G:{gyro_cal} A:{accel_cal} M:{mag_cal}]", end="")
                time.sleep(0.1)
        except KeyboardInterrupt:
            print("\n\nReal-time data stopped")

    # ========================
    # 결과 요약
    # ========================
    print_header("Test Results Summary")
    if all_passed:
        print("\033[92mAll tests passed! Proceed to Step 3.\033[0m")
        print("\nNext step: python3 step3_test_lidar.py")
    else:
        print("\033[91mSome tests failed. Please fix the errors above.\033[0m")

    return all_passed


def calibrate_imu(imu):
    """단계별 IMU 캘리브레이션 가이드"""

    print_header("Starting Calibration")

    # ========================
    # Step A: 자이로스코프 캘리브레이션
    # ========================
    print("\n" + "-" * 60)
    print("  Step A: Gyroscope Calibration")
    print("-" * 60)
    print("""
    Action: Place the robot on a flat surface and keep it completely still.

    +-----------------------------+
    |         [Robot]             |
    |    ================         |
    |    Flat Surface             |
    +-----------------------------+

    Keep still for 3-5 seconds until gyroscope is calibrated.
    """)

    wait_for_enter("Place robot on flat surface, then press Enter...")

    print("\n    Calibrating gyroscope...")
    start_time = time.time()
    while time.time() - start_time < 10:
        sys_cal, gyro_cal, accel_cal, mag_cal = imu.BNO055.get_calibration_status()
        print_calibration_status(sys_cal, gyro_cal, accel_cal, mag_cal)

        if gyro_cal == 3:
            print("\n    [OK] Gyroscope calibration complete!")
            break
        time.sleep(0.3)
    else:
        print("\n    [!] Timeout. Continuing...")

    # ========================
    # Step B: 가속도계 캘리브레이션
    # ========================
    print("\n" + "-" * 60)
    print("  Step B: Accelerometer Calibration")
    print("-" * 60)
    print("""
    Action: Place robot in 6 different positions, holding each for 2-3 seconds.

    1) Normal position (wheels down)
    2) Upside down (wheels up)
    3) Tilted left
    4) Tilted right
    5) Nose up
    6) Nose down

    Hold each position briefly to let accelerometer learn gravity direction.
    """)

    positions = [
        ("Normal position (wheels down)", "[Robot]==== Floor"),
        ("Upside down (wheels up)", "Floor ====[Robot]"),
        ("Tilted left", "[Robot] |  Floor"),
        ("Tilted right", "Floor  | [Robot]"),
        ("Nose up", " [Robot]\n |\nFloor"),
        ("Nose down", "Floor\n |\n [Robot]"),
    ]

    for i, (desc, diagram) in enumerate(positions, 1):
        print(f"\n    Position {i}/6: {desc}")
        print(f"    {diagram}")
        wait_for_enter(f"Place robot in this position, then press Enter...")

        print("    Measuring...")
        for _ in range(20):
            sys_cal, gyro_cal, accel_cal, mag_cal = imu.BNO055.get_calibration_status()
            print_calibration_status(sys_cal, gyro_cal, accel_cal, mag_cal)

            if accel_cal == 3:
                print("\n    [OK] Accelerometer calibration complete!")
                break
            time.sleep(0.2)
        else:
            continue
        break

    # ========================
    # Step C: 지자기 센서 캘리브레이션
    # ========================
    print("\n" + "-" * 60)
    print("  Step C: Magnetometer Calibration")
    print("-" * 60)
    print("""
    Action: Hold robot and move it slowly in a figure-8 pattern.

         8 - Move in figure-8 pattern slowly

    Tips:
    - Rotate in all directions (X, Y, Z axes) for faster calibration
    - Stay away from magnets and large metal objects
    - This may take 30 seconds to 1 minute
    """)

    wait_for_enter("Hold the robot and press Enter when ready...")

    print("\n    Calibrating magnetometer... (Move in figure-8 pattern)")
    print("    (Press Ctrl+C to skip)")

    try:
        start_time = time.time()
        while time.time() - start_time < 120:  # 최대 2분
            sys_cal, gyro_cal, accel_cal, mag_cal = imu.BNO055.get_calibration_status()
            print_calibration_status(sys_cal, gyro_cal, accel_cal, mag_cal)

            if mag_cal == 3:
                print("\n    [OK] Magnetometer calibration complete!")
                break
            time.sleep(0.3)
        else:
            print("\n    [!] Timeout. Continuing...")
    except KeyboardInterrupt:
        print("\n    Skipping...")

    # ========================
    # 최종 상태 확인
    # ========================
    print("\n" + "-" * 60)
    print("  Final Calibration Status")
    print("-" * 60)

    sys_cal, gyro_cal, accel_cal, mag_cal = imu.BNO055.get_calibration_status()
    print()
    print_calibration_status(sys_cal, gyro_cal, accel_cal, mag_cal)
    print("\n")

    if sys_cal == 3:
        print("    [OK] Full calibration complete! System is fully calibrated.")
    else:
        print("    [!] Some sensors not fully calibrated.")
        print("    The sensor will still work, but try again later for better accuracy.")

    # ========================
    # 캘리브레이션 데이터 저장 (선택)
    # ========================
    print("\n" + "-" * 60)
    response = input("Save calibration data for future use? (y/n): ").lower()

    if response == 'y':
        try:
            cal_data = imu.BNO055.get_calibration()
            import json
            import os

            cal_file = os.path.expanduser("~/Dexter/bno055_calibration.json")
            os.makedirs(os.path.dirname(cal_file), exist_ok=True)

            with open(cal_file, 'w') as f:
                json.dump(list(cal_data), f)

            print(f"    Calibration data saved to: {cal_file}")
            print("    Load this data on next boot to skip calibration.")
        except Exception as e:
            print(f"    Save failed: {e}")


if __name__ == '__main__':
    try:
        success = main()
        sys.exit(0 if success else 1)
    except KeyboardInterrupt:
        print("\nTest interrupted")
        sys.exit(1)
