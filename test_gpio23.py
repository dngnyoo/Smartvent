#!/usr/bin/env python3
import RPi.GPIO as GPIO
import sys
sys.path.insert(0, '/home/ubuntu/GoPiGo3/Software/Python')
import gopigo3
import time

print("=== GPIO 23 Power Keep-Alive Test ===")

# GPIO 23 활성화
GPIO.setmode(GPIO.BCM)
GPIO.setup(23, GPIO.OUT)
GPIO.output(23, True)
print("✅ GPIO 23 set to HIGH")

# GoPiGo3 초기화
GPG = gopigo3.GoPiGo3()
print(f"✅ GoPiGo3 connected")
print(f"   Battery: {GPG.get_voltage_battery()}V")
print(f"   Firmware: {GPG.get_version_firmware()}")

# Motor limits 설정
GPG.set_motor_limits(GPG.MOTOR_LEFT + GPG.MOTOR_RIGHT, 100, 1000)
print("✅ Motor limits set")

# 모터 테스트
print("\n=== Testing LEFT motor ===")
GPG.set_motor_dps(GPG.MOTOR_LEFT, 300)
time.sleep(2)
encoder = GPG.get_motor_encoder(GPG.MOTOR_LEFT)
GPG.set_motor_dps(GPG.MOTOR_LEFT, 0)

print(f"Encoder value: {encoder}")
if encoder != 0:
    print("🎉 SUCCESS! Motor is working!")
else:
    print("❌ FAILED! Motor still not working")

GPIO.cleanup()
