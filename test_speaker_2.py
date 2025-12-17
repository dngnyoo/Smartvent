import pigpio
import time
import os

# ================= AUDIO ALERT =================
def speak_danger():
    os.system('espeak "Danger danger. Gas level is very high." --stdout | aplay >/dev/null 2>&1 &')

# ================= CONFIGURATION =================
SDA_PIN = 17
SCL_PIN = 27
ADS_ADDR = 0x48
BAUD_RATE = 100000

REG_CONVERSION = 0x00
REG_CONFIG = 0x01

CONFIG_HI = 0xC2
CONFIG_LO = 0x83

GAS_THRESHOLD = 1.0  # volts

# ================= INIT PIGPIO =================
pi = pigpio.pi()

if not pi.connected:
    print("Failed to connect to pigpiod!")
    exit(1)

try:
    pi.bb_i2c_open(SDA_PIN, SCL_PIN, BAUD_RATE)
except:
    pi.bb_i2c_close(SDA_PIN)
    pi.bb_i2c_open(SDA_PIN, SCL_PIN, BAUD_RATE)

print("Starting MQ-2 Gas Monitoring...")

# ================= ADS1115 FUNCTIONS =================
def write_config():
    cmd = [4, ADS_ADDR, 2, 7, 3, REG_CONFIG, CONFIG_HI, CONFIG_LO, 3, 0]
    pi.bb_i2c_zip(SDA_PIN, cmd)

def read_value():
    pi.bb_i2c_zip(SDA_PIN, [4, ADS_ADDR, 2, 7, 1, REG_CONVERSION, 3, 0])
    count, data = pi.bb_i2c_zip(SDA_PIN, [4, ADS_ADDR, 2, 6, 2, 3, 0])

    if count > 0 and len(data) >= 2:
        value = (data[0] << 8) | data[1]
        if value > 32767:
            value -= 65536
        return value
    return None

# ================= MAIN LOOP =================
try:
    write_config()
    time.sleep(0.1)

    last_alert = 0
    ALERT_COOLDOWN = 5  # seconds

    while True:
        raw_val = read_value()

        if raw_val is not None:
            voltage = raw_val * 0.000125
            print(f"Gas Value: {raw_val} | Voltage: {voltage:.2f}V")

            if voltage > GAS_THRESHOLD:
                print("🚨 DANGER: GAS LEVEL HIGH 🚨")

                if time.time() - last_alert > ALERT_COOLDOWN:
                    speak_danger()
                    last_alert = time.time()

        else:
            print("Error reading sensor")

        time.sleep(1)

except KeyboardInterrupt:
    print("\nExiting...")

finally:
    pi.bb_i2c_close(SDA_PIN)
    pi.stop()
