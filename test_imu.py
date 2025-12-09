#!/usr/bin/env python3
"""
IMU Sensor Test Script for DEXTER IMU (BNO055)
Tests: Connection, Calibration Status, and Data Reading
"""

import sys
import time

# Add DI_Sensors library path
sys.path.insert(0, '/home/ubuntu/DI_Sensors/Python')
from di_sensors.inertial_measurement_unit import InertialMeasurementUnit

def test_imu():
    print("=" * 60)
    print("DEXTER IMU Sensor Test")
    print("=" * 60)

    try:
        # Initialize IMU
        print("\n[1/5] Initializing IMU...")
        imu = InertialMeasurementUnit(bus="RPI_1SW")
        print("✅ IMU initialized successfully")

        # Read calibration status
        print("\n[2/5] Checking calibration status...")
        cal_status = imu.BNO055.get_calibration_status()
        print(f"   System: {cal_status[0]}/3")
        print(f"   Gyro:   {cal_status[1]}/3")
        print(f"   Accel:  {cal_status[2]}/3")
        print(f"   Mag:    {cal_status[3]}/3")
        if all(x >= 2 for x in cal_status):
            print("✅ IMU is well calibrated")
        else:
            print("⚠️  IMU needs calibration (move robot in figure-8 pattern)")

        # Read temperature
        print("\n[3/5] Reading temperature...")
        temp = imu.read_temperature()
        print(f"   Temperature: {temp}°C")

        # Read orientation (Euler angles)
        print("\n[4/5] Reading orientation (Euler angles)...")
        for i in range(5):
            euler = imu.read_euler()
            print(f"   Reading {i+1}: Heading={euler[0]:6.1f}° Roll={euler[1]:6.1f}° Pitch={euler[2]:6.1f}°")
            time.sleep(0.2)

        # Read all sensor data
        print("\n[5/5] Reading all sensor data...")
        print("   Magnetometer (μT):", imu.read_magnetometer())
        print("   Gyroscope (°/s):", imu.read_gyroscope())
        print("   Accelerometer (m/s²):", imu.read_accelerometer())
        print("   Linear Acceleration (m/s²):", imu.read_linear_acceleration())

        print("\n" + "=" * 60)
        print("🎉 IMU Test Complete - All sensors working!")
        print("=" * 60)

        return True

    except Exception as e:
        print(f"\n❌ Error: {e}")
        print("\nTroubleshooting:")
        print("1. Check I2C connection: i2cdetect -y 1")
        print("2. Verify IMU address (should be 0x28 or 0x29)")
        print("3. Install DI_Sensors: cd ~/DI_Sensors/Python && sudo python3 setup.py install")
        return False

if __name__ == "__main__":
    test_imu()
