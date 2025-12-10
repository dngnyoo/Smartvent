#!/usr/bin/env python3
"""
Auto-advancing IMU Calibration Script for GoPiGo3 (BNO055)
"""

import sys
import time

# Add DI_Sensors library path
sys.path.insert(0, '/home/ubuntu/DI_Sensors/Python')
from di_sensors.inertial_measurement_unit import InertialMeasurementUnit

def print_header(text):
    print("\n" + "=" * 60)
    print(text)
    print("=" * 60)

def print_step(step, total, text):
    print(f"\n[Step {step}/{total}] {text}")

def print_calibration_status(cal_status):
    """Print color-coded calibration status"""
    system, gyro, accel, mag = cal_status
    
    def status_icon(level):
        if level >= 3: return "✅"
        elif level >= 2: return "⚠️ "
        elif level >= 1: return "🔶"
        else: return "❌"
    
    print(f"   System: {status_icon(system)} {system}/3")
    print(f"   Gyro:   {status_icon(gyro)} {gyro}/3")
    print(f"   Accel:  {status_icon(accel)} {accel}/3")
    print(f"   Mag:    {status_icon(mag)} {mag}/3")
    
    return system, gyro, accel, mag

# [Modified] Auto-countdown function instead of manual input
def auto_wait(seconds=3, message="Proceeding to next step"):
    print(f"\n⏳ {message} (Waiting {seconds}s)...", end="", flush=True)
    for i in range(seconds, 0, -1):
        time.sleep(1)
        print(f" {i}..", end="", flush=True)
    print(" START! 🚀")

def calibrate_gyroscope(imu):
    print_step(1, 3, "Gyroscope Calibration")
    print("   ⚙️  Place robot on a FLAT, STABLE surface.")
    print("   ⚙️  Do NOT move the robot.")
    auto_wait(3, "Starting measurement")
    
    print("   🔄 Calibrating (5 seconds)...")
    for i in range(5):
        cal = imu.BNO055.get_calibration_status()
        print(f"      Gyro: {cal[1]}/3", end="\r")
        time.sleep(1)
    
    cal = imu.BNO055.get_calibration_status()
    if cal[1] >= 3:
        print("\n   ✅ Gyroscope calibration complete!")
    else:
        print(f"\n   ⚠️  Gyro status: {cal[1]}/3 (May need more time)")

def calibrate_accelerometer(imu):
    print_step(2, 3, "Accelerometer Calibration")
    print("   📦 You need to place the robot in 6 different orientations.")
    
    positions = [
        ("1/6: Flat on bottom (Normal)", "Wheels down"),
        ("2/6: Upside down", "Wheels up"),
        ("3/6: Standing on front", "Front down"),
        ("4/6: Standing on back", "Back down"),
        ("5/6: Standing on left side", "Left side down"),
        ("6/6: Standing on right side", "Right side down"),
    ]
    
    for pos_num, (desc, detail) in enumerate(positions, 1):
        print(f"\n   Position {desc}")
        # [Modified] Give 5 seconds to change position
        auto_wait(5, "Get ready / Hold position")
        
        print("   📸 Capturing data...", end="")
        for i in range(3):
            time.sleep(1)
            print(".", end="", flush=True)
        print(" Done!")
        
        cal = imu.BNO055.get_calibration_status()
        print(f"   Current Accel: {cal[2]}/3")
        
        if cal[2] >= 3:
            print("   ✅ Accelerometer calibration complete!")
            break

def calibrate_magnetometer(imu):
    print_step(3, 3, "Magnetometer Calibration")
    print("   🧲 Move robot in a FIGURE-8 pattern in the air.")
    auto_wait(3, "Get ready")
    
    print("\n   🔄 Start moving! (Keep moving for 20 seconds)...")
    start_time = time.time()
    last_mag = 0
    
    while time.time() - start_time < 20:
        cal = imu.BNO055.get_calibration_status()
        mag = cal[3]
        
        if mag != last_mag:
            print(f"   Mag: {mag}/3 {'✅' if mag >= 3 else '🔶'}")
            last_mag = mag
        
        if mag >= 3:
            print("   ✅ Magnetometer calibration complete!")
            break
        
        time.sleep(0.5)

def main():
    print_header("🎯 Auto-Advancing IMU Calibration Wizard")
    
    try:
        # [Modified] Use Hardware I2C (RPI_1) for stability
        print("\n🔌 Connecting to sensor...")
        imu = InertialMeasurementUnit(bus="RPI_1SW")
        print("✅ IMU connected successfully")
        
        # Check initial status
        print("\n📊 Initial Status:")
        cal = imu.BNO055.get_calibration_status()
        system, gyro, accel, mag = print_calibration_status(cal)
        
        if system == 3 and gyro == 3 and accel == 3 and mag == 3:
            print("\n✅ All sensors are already calibrated!")
            return
        
        # Start Calibration
        if gyro < 3: calibrate_gyroscope(imu)
        else: print("\n✅ Gyroscope: Already calibrated (Skipping)")
        
        if accel < 3: calibrate_accelerometer(imu)
        else: print("\n✅ Accelerometer: Already calibrated (Skipping)")
        
        if mag < 3: calibrate_magnetometer(imu)
        else: print("\n✅ Magnetometer: Already calibrated (Skipping)")
        
        # Final Result
        print_header("📊 Final Calibration Status")
        cal = imu.BNO055.get_calibration_status()
        print_calibration_status(cal)
        
        if all(x >= 2 for x in cal):
            print("\n🎉 SUCCESS! IMU is calibrated and ready for SLAM.")
        else:
            print("\n⚠️  Calibration incomplete. You may need to run this again.")
        
    except Exception as e:
        print(f"\n❌ Error: {e}")
        return False

if __name__ == "__main__":
    main()