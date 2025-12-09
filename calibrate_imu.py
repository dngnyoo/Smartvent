#!/usr/bin/env python3
"""
Interactive IMU Calibration Script for DEXTER IMU (BNO055)
Guides user through step-by-step calibration process
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

def wait_for_user(prompt="Press ENTER to continue..."):
    input(f"\n{prompt}")

def calibrate_gyroscope(imu):
    print_step(1, 3, "Gyroscope Calibration")
    print("   ⚙️  Place robot on a FLAT, STABLE surface")
    print("   ⚙️  Do NOT move the robot")
    wait_for_user()
    
    print("   🔄 Calibrating gyroscope (5 seconds)...")
    for i in range(5):
        cal = imu.BNO055.get_calibration_status()
        print(f"      Gyro: {cal[1]}/3", end="\r")
        time.sleep(1)
    
    cal = imu.BNO055.get_calibration_status()
    if cal[1] >= 3:
        print("\n   ✅ Gyroscope calibration complete!")
    else:
        print(f"\n   ⚠️  Gyro calibration: {cal[1]}/3 (may need more time)")

def calibrate_accelerometer(imu):
    print_step(2, 3, "Accelerometer Calibration")
    print("   📦 You need to place the robot in 6 different orientations")
    print("   📦 Hold each position steady for 3 seconds")
    
    positions = [
        ("1/6: Flat on bottom (normal position)", "wheels down"),
        ("2/6: Upside down", "wheels up"),
        ("3/6: Standing on front", "front down"),
        ("4/6: Standing on back", "back down"),
        ("5/6: Standing on left side", "left side down"),
        ("6/6: Standing on right side", "right side down"),
    ]
    
    for pos_num, (desc, detail) in enumerate(positions, 1):
        print(f"\n   Position {desc}")
        print(f"   ({detail})")
        wait_for_user()
        
        print("   🔄 Hold steady...", end="")
        for i in range(3):
            time.sleep(1)
            print(".", end="", flush=True)
        print(" Done!")
        
        cal = imu.BNO055.get_calibration_status()
        print(f"   Current Accel: {cal[2]}/3")
        
        if cal[2] >= 3:
            print("   ✅ Accelerometer fully calibrated!")
            break
    
    cal = imu.BNO055.get_calibration_status()
    if cal[2] < 3:
        print(f"\n   ⚠️  Accel calibration: {cal[2]}/3 (try more positions if needed)")

def calibrate_magnetometer(imu):
    print_step(3, 3, "Magnetometer Calibration")
    print("   🧲 Move robot in a FIGURE-8 pattern in the air")
    print("   🧲 Rotate slowly in all directions")
    print("   🧲 Keep moving for 15-20 seconds")
    wait_for_user()
    
    print("\n   🔄 Calibrating magnetometer (move the robot now!)...")
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
    
    cal = imu.BNO055.get_calibration_status()
    if cal[3] < 3:
        print(f"\n   ⚠️  Mag calibration: {cal[3]}/3 (continue moving in figure-8)")

def main():
    print_header("🎯 Interactive IMU Calibration Wizard")
    print("\nThis wizard will guide you through calibrating your IMU sensor.")
    print("Calibration improves accuracy for SLAM and navigation.")
    print("\n⏱️  Estimated time: 2-3 minutes")
    
    wait_for_user("Press ENTER to begin...")
    
    try:
        # Initialize IMU
        print("\n🔌 Initializing IMU sensor...")
        imu = InertialMeasurementUnit(bus="RPI_1SW")
        print("✅ IMU connected successfully")
        
        # Check initial calibration
        print("\n📊 Initial Calibration Status:")
        cal = imu.BNO055.get_calibration_status()
        system, gyro, accel, mag = print_calibration_status(cal)
        
        # Determine what needs calibration
        needs_cal = []
        if gyro < 3: needs_cal.append("Gyroscope")
        if accel < 3: needs_cal.append("Accelerometer")
        if mag < 3: needs_cal.append("Magnetometer")
        
        if not needs_cal:
            print("\n✅ All sensors already calibrated!")
            print("   No calibration needed.")
            return
        
        print(f"\n📋 Sensors needing calibration: {', '.join(needs_cal)}")
        
        # Start calibration process
        print_header("Starting Calibration Process")
        
        if gyro < 3:
            calibrate_gyroscope(imu)
        else:
            print("\n✅ Gyroscope already calibrated, skipping...")
        
        if accel < 3:
            calibrate_accelerometer(imu)
        else:
            print("\n✅ Accelerometer already calibrated, skipping...")
        
        if mag < 3:
            calibrate_magnetometer(imu)
        else:
            print("\n✅ Magnetometer already calibrated, skipping...")
        
        # Final status
        print_header("📊 Final Calibration Status")
        cal = imu.BNO055.get_calibration_status()
        system, gyro, accel, mag = print_calibration_status(cal)
        
        # Overall result
        print("\n" + "=" * 60)
        if all(x >= 2 for x in cal):
            print("🎉 SUCCESS! IMU is well calibrated and ready for SLAM!")
            print("\n💡 Tips for maintaining calibration:")
            print("   - Calibration persists while powered on")
            print("   - Will need recalibration after power cycle")
            print("   - For best results, calibrate before each mission")
        else:
            print("⚠️  Calibration incomplete. Recommendations:")
            if gyro < 2:
                print("   - Gyro: Keep robot completely still on flat surface")
            if accel < 2:
                print("   - Accel: Try all 6 orientations, hold steady")
            if mag < 2:
                print("   - Mag: Move in figure-8, away from metal objects")
            print("\n   You can run this script again to continue calibration.")
        
        print("=" * 60)
        
    except Exception as e:
        print(f"\n❌ Error: {e}")
        print("\nTroubleshooting:")
        print("1. Check I2C connection: i2cdetect -y 1")
        print("2. Verify IMU at address 0x28")
        print("3. Ensure DI_Sensors is properly installed")
        return False

if __name__ == "__main__":
    main()
