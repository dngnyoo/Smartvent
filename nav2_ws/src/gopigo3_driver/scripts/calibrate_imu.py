#!/usr/bin/env python3
"""
BNO055 IMU Calibration Guide Script

This interactive script guides the user through the IMU calibration process
before starting SLAM or navigation.

Usage:
    python3 calibrate_imu.py

The script will:
1. Connect to the BNO055 IMU
2. Guide user through calibration steps
3. Wait until calibration reaches acceptable levels
4. Optionally save calibration data for future use
"""

import sys
import time
import signal

# Add DI_Sensors library path
sys.path.insert(0, '/home/ubuntu/DI_Sensors/Python')

try:
    from di_sensors.inertial_measurement_unit import InertialMeasurementUnit
except ImportError:
    print("Error: DI_Sensors library not found")
    print("Install: cd ~/DI_Sensors/Python && sudo python3 setup.py install")
    sys.exit(1)


class Colors:
    """ANSI color codes for terminal output"""
    RED = '\033[91m'
    GREEN = '\033[92m'
    YELLOW = '\033[93m'
    BLUE = '\033[94m'
    MAGENTA = '\033[95m'
    CYAN = '\033[96m'
    WHITE = '\033[97m'
    BOLD = '\033[1m'
    RESET = '\033[0m'


def print_header(text):
    print(f"\n{Colors.CYAN}{Colors.BOLD}{'='*60}{Colors.RESET}")
    print(f"{Colors.CYAN}{Colors.BOLD}  {text}{Colors.RESET}")
    print(f"{Colors.CYAN}{Colors.BOLD}{'='*60}{Colors.RESET}\n")


def print_step(step_num, text):
    print(f"\n{Colors.YELLOW}{Colors.BOLD}[Step {step_num}]{Colors.RESET} {text}")


def print_success(text):
    print(f"{Colors.GREEN}[OK]{Colors.RESET} {text}")


def print_warning(text):
    print(f"{Colors.YELLOW}[!]{Colors.RESET} {text}")


def print_error(text):
    print(f"{Colors.RED}[ERROR]{Colors.RESET} {text}")


def print_info(text):
    print(f"{Colors.BLUE}[INFO]{Colors.RESET} {text}")


def calibration_bar(value, max_value=3):
    """Create a visual progress bar for calibration status"""
    filled = '█' * value
    empty = '░' * (max_value - value)

    if value == max_value:
        color = Colors.GREEN
    elif value >= 2:
        color = Colors.YELLOW
    else:
        color = Colors.RED

    return f"{color}{filled}{empty}{Colors.RESET} {value}/{max_value}"


def safe_read_calibration(imu, retries=3):
    """Safely read calibration status with retries"""
    for attempt in range(retries):
        try:
            time.sleep(0.05)  # Small delay before read
            status = imu.BNO055.get_calibration_status()
            return status
        except Exception as e:
            if attempt < retries - 1:
                time.sleep(0.1)
            else:
                return None
    return None


def safe_read_euler(imu, retries=3):
    """Safely read Euler angles with retries"""
    for attempt in range(retries):
        try:
            time.sleep(0.05)
            euler = imu.read_euler()
            return euler
        except Exception as e:
            if attempt < retries - 1:
                time.sleep(0.1)
            else:
                return None
    return None


class IMUCalibrator:
    def __init__(self):
        self.imu = None
        self.running = True

        # Set up signal handler for clean exit
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)

    def _signal_handler(self, signum, frame):
        print("\n\nCalibration interrupted by user.")
        self.running = False
        sys.exit(0)

    def connect(self):
        """Connect to BNO055 IMU"""
        print_info("Connecting to BNO055 IMU...")

        try:
            self.imu = InertialMeasurementUnit(bus='RPI_1SW')
            time.sleep(0.5)  # Allow sensor to stabilize

            # Get revision info
            try:
                revision = self.imu.BNO055.get_revision()
                print_success(f"Connected to BNO055 (SW Rev: {revision[0]}, BL Rev: {revision[1]})")
            except:
                print_success("Connected to BNO055")

            return True

        except Exception as e:
            print_error(f"Failed to connect to BNO055: {e}")
            print_info("Check:")
            print("  1. I2C is enabled: sudo raspi-config")
            print("  2. BNO055 is at 0x28 or 0x29: sudo i2cdetect -y 1")
            return False

    def get_calibration_status(self):
        """Get current calibration status"""
        status = safe_read_calibration(self.imu)
        if status is None:
            return None
        return {
            'sys': status[0],
            'gyro': status[1],
            'accel': status[2],
            'mag': status[3]
        }

    def display_status(self, status):
        """Display calibration status with visual bars"""
        if status is None:
            print_warning("Could not read calibration status (I2C error)")
            return

        print(f"\r  Gyro:  {calibration_bar(status['gyro'])}  ", end='')
        print(f"Accel: {calibration_bar(status['accel'])}  ", end='')
        print(f"Mag:   {calibration_bar(status['mag'])}  ", end='')
        print(f"Sys:   {calibration_bar(status['sys'])}  ", end='')
        sys.stdout.flush()

    def wait_for_calibration(self, sensor_name, target_value=3, timeout=60):
        """Wait for a specific sensor to reach calibration target"""
        start_time = time.time()
        last_status = None

        while self.running and (time.time() - start_time) < timeout:
            status = self.get_calibration_status()

            if status is not None:
                self.display_status(status)

                if status[sensor_name] >= target_value:
                    print()  # New line after progress
                    return True

                last_status = status

            time.sleep(0.5)

        print()
        return False

    def calibrate_gyroscope(self):
        """Guide user through gyroscope calibration"""
        while True:
            print_step(1, "Gyroscope Calibration")
            print(f"""
    {Colors.WHITE}The gyroscope calibrates automatically when the robot is stationary.{Colors.RESET}

    Instructions:
    - Place the robot on a flat, stable surface
    - Do NOT move or touch the robot
    - Wait for Gyro to reach 3/3

    {Colors.YELLOW}Press Enter when ready...{Colors.RESET}""")

            input()

            print_info("Calibrating gyroscope... Keep the robot still!")

            if self.wait_for_calibration('gyro', target_value=3, timeout=30):
                print_success("Gyroscope calibrated!")
                return True
            else:
                status = self.get_calibration_status()
                if status and status['gyro'] >= 2:
                    print_warning(f"Gyroscope partially calibrated ({status['gyro']}/3). This is acceptable.")
                    response = input(f"{Colors.YELLOW}Continue anyway? (y) or Retry? (n): {Colors.RESET}").lower()
                    if response != 'n':
                        return True
                else:
                    print_warning("Gyroscope calibration timeout.")
                    response = input(f"{Colors.YELLOW}Retry calibration? (y/n): {Colors.RESET}").lower()
                    if response != 'y':
                        return False

    def calibrate_accelerometer(self):
        """Guide user through accelerometer calibration"""
        while True:
            print_step(2, "Accelerometer Calibration")
            print(f"""
    {Colors.WHITE}The accelerometer calibrates by placing the robot in different orientations.{Colors.RESET}

    Instructions:
    - Place robot flat (normal position) for 3 seconds
    - Tilt robot left ~45° and hold for 3 seconds
    - Tilt robot right ~45° and hold for 3 seconds
    - Tilt robot forward ~45° and hold for 3 seconds
    - Tilt robot backward ~45° and hold for 3 seconds
    - Return to flat position

    {Colors.YELLOW}Press Enter when ready...{Colors.RESET}""")

            input()

            print_info("Calibrating accelerometer... Follow the tilt instructions!")

            if self.wait_for_calibration('accel', target_value=3, timeout=60):
                print_success("Accelerometer calibrated!")
                return True
            else:
                status = self.get_calibration_status()
                if status and status['accel'] >= 2:
                    print_warning(f"Accelerometer partially calibrated ({status['accel']}/3). This is acceptable.")
                    response = input(f"{Colors.YELLOW}Continue anyway? (y) or Retry? (n): {Colors.RESET}").lower()
                    if response != 'n':
                        return True
                else:
                    print_warning("Accelerometer calibration incomplete.")
                    response = input(f"{Colors.YELLOW}Retry calibration? (y/n): {Colors.RESET}").lower()
                    if response != 'y':
                        return False

    def calibrate_magnetometer(self):
        """Guide user through magnetometer calibration"""
        while True:
            print_step(3, "Magnetometer Calibration")
            print(f"""
    {Colors.WHITE}The magnetometer calibrates by rotating the robot in a figure-8 pattern.{Colors.RESET}

    Instructions:
    - Pick up the robot gently
    - Move it slowly in a figure-8 (∞) pattern in the air
    - Rotate it in all directions while doing this
    - Continue for 30-60 seconds

    {Colors.MAGENTA}Note: Stay away from metal objects and electronics!{Colors.RESET}

    {Colors.YELLOW}Press Enter when ready...{Colors.RESET}""")

            input()

            print_info("Calibrating magnetometer... Move in figure-8 pattern!")

            if self.wait_for_calibration('mag', target_value=3, timeout=90):
                print_success("Magnetometer calibrated!")
                return True
            else:
                status = self.get_calibration_status()
                if status and status['mag'] >= 2:
                    print_warning(f"Magnetometer partially calibrated ({status['mag']}/3). This may affect heading accuracy.")
                    response = input(f"{Colors.YELLOW}Continue anyway? (y) or Retry? (n): {Colors.RESET}").lower()
                    if response != 'n':
                        return True
                else:
                    print_warning("Magnetometer calibration incomplete.")
                    response = input(f"{Colors.YELLOW}Retry calibration? (y/n): {Colors.RESET}").lower()
                    if response != 'y':
                        return False

    def verify_calibration(self):
        """Verify overall calibration and show final status"""
        while True:
            print_step(4, "Verification")
            print_info("Verifying calibration status...")

            time.sleep(1)
            status = self.get_calibration_status()

            if status is None:
                print_error("Could not read calibration status")
                return False

            print(f"""
    {Colors.BOLD}Final Calibration Status:{Colors.RESET}

    Gyroscope:     {calibration_bar(status['gyro'])}
    Accelerometer: {calibration_bar(status['accel'])}
    Magnetometer:  {calibration_bar(status['mag'])}
    System:        {calibration_bar(status['sys'])}
            """)

            # Check which sensors need recalibration
            needs_recalib = []
            if status['gyro'] < 2:
                needs_recalib.append('gyro')
                print_warning("Gyroscope calibration too low!")
            if status['accel'] < 1:
                needs_recalib.append('accel')
                print_warning("Accelerometer calibration too low!")
            if status['mag'] < 1:
                needs_recalib.append('mag')
                print_warning("Magnetometer calibration too low!")

            if status['sys'] >= 2:
                print_success("System calibration is GOOD. Ready for SLAM!")
            elif status['sys'] >= 1:
                print_warning("System calibration is ACCEPTABLE. SLAM may have some drift.")
            else:
                print_error("System calibration is LOW.")

            # If calibration is not acceptable, offer to recalibrate specific sensors
            if needs_recalib:
                print(f"\n{Colors.YELLOW}The following sensors need recalibration: {', '.join(needs_recalib)}{Colors.RESET}")
                response = input(f"{Colors.YELLOW}Recalibrate these sensors? (y/n): {Colors.RESET}").lower()
                if response == 'y':
                    if 'gyro' in needs_recalib:
                        self.calibrate_gyroscope()
                    if 'accel' in needs_recalib:
                        self.calibrate_accelerometer()
                    if 'mag' in needs_recalib:
                        self.calibrate_magnetometer()
                    continue  # Loop back to verify again
                else:
                    return False
            else:
                return True

    def test_readings(self):
        """Show live IMU readings to verify sensor is working"""
        print_step(5, "Live Sensor Test")
        print_info("Showing live readings for 10 seconds...")
        print_info("Tilt or rotate the robot to verify sensor response.\n")

        start_time = time.time()
        while self.running and (time.time() - start_time) < 10:
            euler = safe_read_euler(self.imu)

            if euler:
                heading, roll, pitch = euler
                print(f"\r  Heading: {heading:6.1f}°  Roll: {roll:6.1f}°  Pitch: {pitch:6.1f}°  ", end='')
            else:
                print(f"\r  (Reading error - retrying...)                              ", end='')

            sys.stdout.flush()
            time.sleep(0.2)

        print("\n")
        print_success("Sensor test complete!")

    def run(self):
        """Run the full calibration process"""
        print_header("BNO055 IMU Calibration Guide")

        print(f"""
    This script will guide you through calibrating the BNO055 IMU.
    Proper calibration is essential for accurate SLAM and navigation.

    The process takes about 2-3 minutes.
        """)

        input(f"{Colors.YELLOW}Press Enter to begin...{Colors.RESET}")

        # Connect to IMU
        if not self.connect():
            return False

        # Show initial status
        print_info("\nInitial calibration status:")
        status = self.get_calibration_status()
        if status:
            print(f"  Gyro: {status['gyro']}/3  Accel: {status['accel']}/3  Mag: {status['mag']}/3  Sys: {status['sys']}/3\n")

        # Run calibration steps
        self.calibrate_gyroscope()
        self.calibrate_accelerometer()
        self.calibrate_magnetometer()

        # Verify and test
        self.verify_calibration()

        response = input(f"\n{Colors.YELLOW}Run live sensor test? (y/n): {Colors.RESET}").lower()
        if response == 'y':
            self.test_readings()

        print_header("Calibration Complete")
        print(f"""
    You can now start the IMU node and SLAM:

    {Colors.CYAN}# Terminal 1: Start IMU node{Colors.RESET}
    ros2 run gopigo3_driver bno055_imu

    {Colors.CYAN}# Terminal 2: Start SLAM (after LiDAR and driver){Colors.RESET}
    ros2 launch slam_toolbox online_async_launch.py
        """)

        return True


def main():
    calibrator = IMUCalibrator()
    success = calibrator.run()
    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
