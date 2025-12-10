import pigpio
import time
import struct

# --- Configuration ---
SDA_PIN = 17  # Reassigned SDA Pin (GPIO 17)
SCL_PIN = 27  # Reassigned SCL Pin (GPIO 27)
ADS_ADDR = 0x48
BAUD_RATE = 100000  # 100kbps

# ADS1115 Registers and Configuration
REG_CONVERSION = 0x00
REG_CONFIG = 0x01

# Config High Byte: OS(1) + Input A0(100) + 4.096V(001) + Continuous Mode(0) = 0xC2
CONFIG_HI = 0xC2 
# Config Low Byte: 128SPS(100) + Defaults = 0x83
CONFIG_LO = 0x83

pi = pigpio.pi()

if not pi.connected:
    print("Failed to connect to pigpiod! (Please check 'sudo systemctl start pigpiod')")
    exit()

# 1. Open Software I2C Bus (Close and reopen if already open)
try:
    pi.bb_i2c_open(SDA_PIN, SCL_PIN, BAUD_RATE)
except:
    pi.bb_i2c_close(SDA_PIN)
    pi.bb_i2c_open(SDA_PIN, SCL_PIN, BAUD_RATE)

print(f"Starting MQ-2 Sensor (Using GPIO {SDA_PIN}, {SCL_PIN})...")

def write_config():
    # Construct I2C Command: [Start, Addr(W), Reg(Config), High Byte, Low Byte, Stop]
    # 2=Start, 7=Write, 3=Stop
    cmd = [4, ADS_ADDR, 2, 7, 3, REG_CONFIG, CONFIG_HI, CONFIG_LO, 3, 0]
    pi.bb_i2c_zip(SDA_PIN, cmd)

def read_value():
    # Step 1: Set pointer to Conversion Register
    # [Start, Addr(W), Reg(Conversion), Stop]
    pi.bb_i2c_zip(SDA_PIN, [4, ADS_ADDR, 2, 7, 1, REG_CONVERSION, 3, 0])
    
    # Step 2: Read 2 bytes of data
    # [Start, Addr(R), Read 2 bytes, Stop]
    # 6=Read
    count, data = pi.bb_i2c_zip(SDA_PIN, [4, ADS_ADDR, 2, 6, 2, 3, 0])
    
    if count > 0:
        # Combine bytes (Big Endian)
        val = (data[0] << 8) | data[1]
        
        # Handle negative values (16-bit signed integer)
        if val > 32767: 
            val -= 65536 
        return val
    return None

try:
    write_config()
    time.sleep(0.1) # Wait for config to apply

    while True:
        raw_val = read_value()
        
        if raw_val is not None:
            # Calculate voltage based on Gain 1 (Range: +/- 4.096V)
            voltage = raw_val * 0.000125
            
            # Print formatted output
            print(f"Gas Sensor Value: {raw_val} | Voltage: {voltage:.2f}V")
            
            # Threshold check (Adjust 2.5V as needed)
            if voltage > 2.5:
                print("🚨 WARNING: Gas Detected!")
        else:
            print("Error reading data")
            
        time.sleep(1)

except KeyboardInterrupt:
    print("\nExiting...")
    pi.bb_i2c_close(SDA_PIN)
    pi.stop()