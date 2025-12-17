#!/usr/bin/env python3
"""
Step 1: GoPiGo3 Hardware Test

이 스크립트는 GoPiGo3 보드와의 통신, 모터, 엔코더가 정상적으로
작동하는지 확인합니다.

Usage:
    python3 step1_test_gopigo3.py
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

def wait_for_enter(prompt="Press Enter to continue..."):
    input(f"\n>>> {prompt}")

def main():
    print_header("Step 1: GoPiGo3 Hardware Test")

    all_passed = True

    # ========================
    # Test 1: GPIO 23 확인
    # ========================
    print("\n[Test 1/6] Checking GPIO 23 (Power Keep-Alive Signal)...")
    try:
        import RPi.GPIO as GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(23, GPIO.OUT)
        GPIO.output(23, True)
        print_result("GPIO 23 Setup", True, "GPIO 23 set to HIGH")
    except ImportError:
        print_result("GPIO 23 Setup", False, "RPi.GPIO library not found")
        print("       Install: sudo apt install python3-rpi.gpio")
        all_passed = False
    except Exception as e:
        print_result("GPIO 23 Setup", False, str(e))
        all_passed = False

    # ========================
    # Test 2: SPI 장치 확인
    # ========================
    print("\n[Test 2/6] Checking SPI Device...")
    import os
    spi_exists = os.path.exists('/dev/spidev0.1')
    print_result("SPI Device", spi_exists,
                 "/dev/spidev0.1 found" if spi_exists else "/dev/spidev0.1 not found - SPI not enabled")
    if not spi_exists:
        print("       Fix: sudo raspi-config -> Interface Options -> SPI -> Enable")
        all_passed = False

    # ========================
    # Test 3: GoPiGo3 라이브러리 import
    # ========================
    print("\n[Test 3/6] Checking GoPiGo3 Library...")
    try:
        sys.path.insert(0, '/home/ubuntu/GoPiGo3/Software/Python')
        import gopigo3
        print_result("GoPiGo3 Library", True, "gopigo3 module imported successfully")
    except ImportError as e:
        print_result("GoPiGo3 Library", False, str(e))
        print("       Fix: cd ~/GoPiGo3/Software/Python && sudo python3 setup.py install")
        all_passed = False
        return all_passed

    # ========================
    # Test 4: GoPiGo3 보드 연결
    # ========================
    print("\n[Test 4/6] Checking GoPiGo3 Board Connection...")
    try:
        gpg = gopigo3.GoPiGo3()
        manufacturer = gpg.get_manufacturer()
        board = gpg.get_board()
        firmware = gpg.get_version_firmware()
        hardware = gpg.get_version_hardware()

        print_result("GoPiGo3 Board Connection", True)
        print(f"       Manufacturer: {manufacturer}")
        print(f"       Board: {board}")
        print(f"       Firmware: {firmware}")
        print(f"       Hardware: {hardware}")
    except Exception as e:
        print_result("GoPiGo3 Board Connection", False, str(e))
        all_passed = False
        return all_passed

    # ========================
    # Test 5: 배터리 전압
    # ========================
    print("\n[Test 5/6] Checking Battery Voltage...")
    try:
        battery = gpg.get_voltage_battery()
        voltage_5v = gpg.get_voltage_5v()

        battery_ok = battery > 9.0
        print_result("Battery Voltage", battery_ok, f"Battery: {battery:.2f}V, 5V Rail: {voltage_5v:.2f}V")
        if not battery_ok:
            print("       Warning: Battery voltage is low (recommended > 9V)")
            all_passed = False
    except Exception as e:
        print_result("Battery Voltage", False, str(e))
        all_passed = False

    # ========================
    # Test 6: 엔코더 읽기
    # ========================
    print("\n[Test 6/6] Reading Encoders...")
    try:
        left_enc = gpg.get_motor_encoder(gpg.MOTOR_LEFT)
        right_enc = gpg.get_motor_encoder(gpg.MOTOR_RIGHT)
        print_result("Encoder Reading", True, f"Left: {left_enc} deg, Right: {right_enc} deg")
    except Exception as e:
        print_result("Encoder Reading", False, str(e))
        all_passed = False

    # ========================
    # 모터 테스트 (선택)
    # ========================
    print("\n" + "-" * 60)
    response = input("Do you want to test the motors? (y/n): ").lower()

    if response == 'y':
        print("\n[!] WARNING: The robot will move! Place it on the floor or hold it up.")
        wait_for_enter("Press Enter when ready...")

        try:
            print("Testing LEFT wheel forward (2 seconds)...")
            gpg.set_motor_dps(gpg.MOTOR_LEFT, 100)
            time.sleep(2)
            gpg.set_motor_dps(gpg.MOTOR_LEFT, 0)

            print("Testing RIGHT wheel forward (2 seconds)...")
            gpg.set_motor_dps(gpg.MOTOR_RIGHT, 100)
            time.sleep(2)
            gpg.set_motor_dps(gpg.MOTOR_RIGHT, 0)

            print("Testing BOTH wheels forward (2 seconds)...")
            gpg.set_motor_dps(gpg.MOTOR_LEFT + gpg.MOTOR_RIGHT, 100)
            time.sleep(2)
            gpg.set_motor_dps(gpg.MOTOR_LEFT + gpg.MOTOR_RIGHT, 0)

            response = input("Did all wheels rotate correctly? (y/n): ").lower()
            print_result("Motor Test", response == 'y')
            if response != 'y':
                all_passed = False

        except Exception as e:
            print_result("Motor Test", False, str(e))
            all_passed = False
        finally:
            gpg.reset_all()

    # ========================
    # 결과 요약
    # ========================
    print_header("Test Results Summary")
    if all_passed:
        print("\033[92mAll tests passed! Proceed to Step 2.\033[0m")
        print("\nNext step: python3 step2_test_imu.py")
    else:
        print("\033[91mSome tests failed. Please fix the errors above.\033[0m")

    return all_passed

if __name__ == '__main__':
    try:
        success = main()
        sys.exit(0 if success else 1)
    except KeyboardInterrupt:
        print("\nTest interrupted")
        sys.exit(1)
