import pigpio
import time
import os

# --- Audio Alert Functions ---
ALARM_FILE = "alarm.mp3"

def play_beep_alarm():
    os.system(f"mpg123 -q {ALARM_FILE} &")

def speak_alert(message):
    os.system(f'espeak "{message}" --stdout | aplay &')

# --- Configuration ---
SDA_PIN = 17
SCL_PIN = 27
ADS_ADDR = 0x48
BAUD_RATE = 100000

REG_CONVERSION = 0x00
REG_CONFIG = 0x01

CONFIG_HI = 0xC2
CONFIG_LO = 0x83

# >>> NEW THRESHOLD <<<
GAS_THRESHOLD = 1.0   # Trigger alert above 1.0V

pi = pigpio.pi()

if not pi.connected:
    print("Failed to connect to pigpiod!")
    exit()

try:
    pi.bb_i2c_open(SDA_PIN, SCL_PIN, BAUD_RATE)
except:
    pi.bb_i2c_close(SDA_PIN)
    pi.bb_i2c_open(SDA_PIN, SCL_PIN, BAUD_RATE)

print(f"Starting MQ-2 Sensor...")

def write_config():
    cmd = [4, ADS_ADDR, 2, 7, 3, REG_CONFIG, CONFIG_HI, CONFIG_LO, 3, 0]
    pi.bb_i2c_zip(SDA_PIN, cmd)

def read_value():
    pi.bb_i2c_zip(SDA_PIN, [4, ADS_ADDR, 2, 7, 1, REG_CONVERSION, 3, 0])
    count, data = pi.bb_i2c_zip(SDA_PIN, [4, ADS_ADDR, 2, 6, 2, 3, 0])
    
    if count > 0:
        val = (data[0] << 8) | data[1]
        if val > 32767:
            val -= 65536
        return val
    return None

try:
    write_config()
    time.sleep(0.1)

    last_voice_alert = 0

    while True:
        raw_val = read_value()
        
        if raw_val is not None:
            voltage = raw_val * 0.000125
            print(f"Gas Sensor Value: {raw_val} | Voltage: {voltage:.2f}V")

            # >>> NEW ALERT CONDITION <<<
            if voltage > GAS_THRESHOLD:
                print("🚨 WARNING: Gas Voltage Above 1.0V!")

                play_beep_alarm()

                if time.time() - last_voice_alert > 5:
                    speak_alert("Warning. Gas level is too high. Please check the environment.")
                    last_voice_alert = time.time()

        else:
            print("Error reading data")

        time.sleep(1)

except KeyboardInterrupt:
    print("\nExiting...")
    pi.bb_i2c_close(SDA_PIN)
    pi.stop()
