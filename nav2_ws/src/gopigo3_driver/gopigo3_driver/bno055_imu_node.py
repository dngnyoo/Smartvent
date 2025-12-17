#!/usr/bin/env python3
"""
BNO055 IMU ROS2 Node for GoPiGo3 + Ubuntu 22.04 + ROS2 Humble

This node reads data from the BNO055 9-DOF IMU sensor and publishes:
- /imu/data: sensor_msgs/Imu (orientation, angular velocity, linear acceleration)
- /imu/mag: sensor_msgs/MagneticField (magnetometer data)
- /imu/temp: sensor_msgs/Temperature (temperature)

The BNO055 uses the DI_Sensors library with Software I2C on the GoPiGo3.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField, Temperature
from std_msgs.msg import Header
import math
import sys
import time
import threading
import json
import os


class BNO055IMUNode(Node):
    # Delay between I2C reads to prevent bus overload (software I2C is slow)
    I2C_READ_DELAY = 0.01  # 10ms delay between consecutive I2C reads
    MAX_RETRIES = 3  # Number of retries on I2C error
    CALIBRATION_FILE = '/home/ubuntu/.bno055_calibration.json'

    def __init__(self):
        super().__init__('bno055_imu')
        
        # [ADDED] Declare a parameter to control calibration requirement (Default: False)
        # If set to True, the node will publish data even if the sensor is not fully calibrated.
        self.declare_parameter('ignore_calibration', False) 

        # ... (Rest of existing initialization code) ...
        # Parameters
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('publish_rate', 10.0)  # Hz (reduced for software I2C stability)
        self.declare_parameter('i2c_bus', 'RPI_1SW')  # Software I2C
        self.declare_parameter('publish_tf', False)
        self.declare_parameter('load_calibration', True)  # Load saved calibration on startup
        self.declare_parameter('save_calibration', True)  # Save calibration when fully calibrated

        self.frame_id = self.get_parameter('frame_id').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.i2c_bus = self.get_parameter('i2c_bus').value
        self.publish_tf = self.get_parameter('publish_tf').value
        self.load_calibration = self.get_parameter('load_calibration').value
        self.save_calibration = self.get_parameter('save_calibration').value
        self.calibration_saved = False  # Track if we've saved calibration this session

        # Error tracking
        self.consecutive_errors = 0
        self.max_consecutive_errors = 10

        # Calibration status tracking (must be before _init_imu)
        self.last_calib_status = None

        # I2C access lock to prevent concurrent access from multiple timers
        self.i2c_lock = threading.Lock()

        # Initialize IMU sensor
        self._init_imu()

        # Publishers
        self.imu_pub = self.create_publisher(Imu, 'imu/data', 10)
        self.mag_pub = self.create_publisher(MagneticField, 'imu/mag', 10)
        self.temp_pub = self.create_publisher(Temperature, 'imu/temp', 10)

        # Timer for publishing
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_imu_data)

        # Calibration check timer
        self.calib_timer = self.create_timer(5.0, self.check_calibration)

        self.get_logger().info('BNO055 IMU node started')
        self.get_logger().info(f'  Frame ID: {self.frame_id}')
        self.get_logger().info(f'  Publish rate: {self.publish_rate} Hz')
        self.get_logger().info(f'  I2C bus: {self.i2c_bus}')

    def _init_imu(self):
        """Initialize BNO055 IMU using DI_Sensors library"""
        try:
            # Add DI_Sensors library path
            sys.path.insert(0, '/home/ubuntu/DI_Sensors/Python')
            from di_sensors.inertial_measurement_unit import InertialMeasurementUnit
            from di_sensors import BNO055

            # Initialize IMU with Software I2C
            self.imu = InertialMeasurementUnit(bus=self.i2c_bus)

            # Get and log revision info
            try:
                revision = self.imu.BNO055.get_revision()
                self.get_logger().info(f'BNO055 initialized successfully')
                self.get_logger().info(f'  SW Rev: {revision[0]}, BL Rev: {revision[1]}')
                self.get_logger().info(f'  Accel ID: {revision[2]}, Mag ID: {revision[3]}, Gyro ID: {revision[4]}')
            except Exception as e:
                self.get_logger().warn(f'Could not get BNO055 revision: {e}')

            # Try to load saved calibration data
            if self.load_calibration:
                self._load_calibration()

            # Check initial calibration
            self.check_calibration()

        except ImportError as e:
            self.get_logger().error(f'Failed to import DI_Sensors: {e}')
            self.get_logger().error('Install DI_Sensors: cd ~/DI_Sensors/Python && sudo python3 setup.py install')
            raise
        except Exception as e:
            self.get_logger().error(f'Failed to initialize BNO055: {e}')
            self.get_logger().error('Check I2C connection and make sure BNO055 is at address 0x28 or 0x29')
            raise

    def _load_calibration(self):
        """Load calibration data from file and apply to BNO055"""
        if not os.path.exists(self.CALIBRATION_FILE):
            self.get_logger().info('No saved calibration file found')
            return False

        try:
            with open(self.CALIBRATION_FILE, 'r') as f:
                calib_data = json.load(f)

            offsets = calib_data.get('offsets', [])
            if len(offsets) != 22:
                self.get_logger().warn('Invalid calibration data format')
                return False

            # Get BNO055 I2C address (default 0x28)
            bno055_addr = 0x28

            # Set calibration offsets
            # BNO055 needs to be in CONFIG mode to write calibration
            self.imu.BNO055.set_mode(0x00)  # CONFIG_MODE
            time.sleep(0.05)

            # Write calibration data to registers (0x55-0x6A)
            for i, value in enumerate(offsets):
                self.imu.BNO055.i2c_bus.write_reg_8(
                    bno055_addr,
                    0x55 + i,
                    value & 0xFF
                )
                time.sleep(0.01)

            # Return to NDOF mode
            time.sleep(0.05)
            self.imu.BNO055.set_mode(0x0C)  # NDOF_MODE
            time.sleep(0.05)

            self.get_logger().info(f'Loaded calibration from {self.CALIBRATION_FILE}')
            return True

        except Exception as e:
            self.get_logger().warn(f'Failed to load calibration: {e}')
            # Try to return to NDOF mode even on error
            try:
                self.imu.BNO055.set_mode(0x0C)
                time.sleep(0.05)
            except:
                pass
            return False

    def _save_calibration(self):
        """Save current calibration data to file"""
        try:
            # Get BNO055 I2C address (default 0x28)
            bno055_addr = 0x28

            # BNO055 needs to be in CONFIG mode to read calibration
            self.imu.BNO055.set_mode(0x00)  # CONFIG_MODE
            time.sleep(0.05)

            # Read calibration data from registers (0x55-0x6A, 22 bytes)
            offsets = []
            for i in range(22):
                value = self.imu.BNO055.i2c_bus.read_8(
                    bno055_addr,
                    0x55 + i
                )
                offsets.append(value)
                time.sleep(0.01)

            # Return to NDOF mode
            time.sleep(0.05)
            self.imu.BNO055.set_mode(0x0C)  # NDOF_MODE
            time.sleep(0.05)

            # Save to file
            calib_data = {
                'offsets': offsets,
                'timestamp': time.strftime('%Y-%m-%d %H:%M:%S')
            }

            with open(self.CALIBRATION_FILE, 'w') as f:
                json.dump(calib_data, f, indent=2)

            self.get_logger().info(f'Saved calibration to {self.CALIBRATION_FILE}')
            self.calibration_saved = True
            return True

        except Exception as e:
            self.get_logger().warn(f'Failed to save calibration: {e}')
            # Try to return to NDOF mode even on error
            try:
                self.imu.BNO055.set_mode(0x0C)
                time.sleep(0.05)
            except:
                pass
            return False

    def check_calibration(self):
        """Check and log IMU calibration status"""
        # Try to acquire lock, but don't block if busy
        if not self.i2c_lock.acquire(blocking=False):
            return  # Skip this calibration check if I2C is busy

        try:
            time.sleep(0.02)  # Small delay before I2C access
            # Get calibration status: (sys, gyro, accel, mag) - each 0-3
            sys_cal, gyro_cal, accel_cal, mag_cal = self.imu.BNO055.get_calibration_status()

            status = (sys_cal, gyro_cal, accel_cal, mag_cal)
            if status != self.last_calib_status:
                self.last_calib_status = status
                self.get_logger().info(
                    f'IMU Calibration - Sys: {sys_cal}/3, Gyro: {gyro_cal}/3, '
                    f'Accel: {accel_cal}/3, Mag: {mag_cal}/3'
                )

                if sys_cal < 2:
                    self.get_logger().warn(
                        'IMU not fully calibrated! Rotate the robot slowly in all directions.'
                    )

                # Save calibration when fully calibrated (all sensors at 3/3)
                if (self.save_calibration and not self.calibration_saved and
                    sys_cal >= 3 and gyro_cal >= 3 and accel_cal >= 3 and mag_cal >= 3):
                    self.get_logger().info('Full calibration achieved! Saving calibration data...')
                    self._save_calibration()

        except Exception as e:
            self.get_logger().warn(f'Failed to get calibration status: {e}')
        finally:
            self.i2c_lock.release()

    def _safe_i2c_read(self, read_func, name, default=None):
        """
        Safely read from I2C with retry logic.
        Software I2C can be flaky, so we retry on errors.
        """
        for attempt in range(self.MAX_RETRIES):
            try:
                result = read_func()
                return result
            except Exception as e:
                if attempt < self.MAX_RETRIES - 1:
                    time.sleep(self.I2C_READ_DELAY * 2)  # Longer delay before retry
                else:
                    raise e
        return default

    def publish_imu_data(self):
        """Read IMU data and publish to ROS2 topics"""
        # Acquire I2C lock to prevent concurrent access
        if not self.i2c_lock.acquire(blocking=False):
            return  # Skip this cycle if lock is held

        current_time = self.get_clock().now().to_msg()

        try:
            # Read quaternion orientation (BNO055 provides fused orientation)
            # Add delays between I2C reads for software I2C stability
            qx, qy, qz, qw = self._safe_i2c_read(self.imu.read_quaternion, 'quaternion')
            time.sleep(self.I2C_READ_DELAY)

            # Read gyroscope (angular velocity in degrees/sec)
            gx, gy, gz = self._safe_i2c_read(self.imu.read_gyroscope, 'gyroscope')
            time.sleep(self.I2C_READ_DELAY)

            # Read linear acceleration (m/s^2, gravity-compensated)
            ax, ay, az = self._safe_i2c_read(self.imu.read_linear_acceleration, 'acceleration')
            time.sleep(self.I2C_READ_DELAY)

            # Read magnetometer (micro-Teslas)
            mx, my, mz = self._safe_i2c_read(self.imu.read_magnetometer, 'magnetometer')

            # Reset error counter on success
            self.consecutive_errors = 0

            # Publish IMU message
            imu_msg = Imu()
            imu_msg.header.stamp = current_time
            imu_msg.header.frame_id = self.frame_id

            # Orientation (quaternion from BNO055)
            imu_msg.orientation.x = qx
            imu_msg.orientation.y = qy
            imu_msg.orientation.z = qz
            imu_msg.orientation.w = qw

            # Orientation covariance (row-major, 3x3)
            # BNO055 fusion provides good orientation, low covariance
            imu_msg.orientation_covariance[0] = 0.0025  # ~2.8 deg
            imu_msg.orientation_covariance[4] = 0.0025
            imu_msg.orientation_covariance[8] = 0.0025

            # Angular velocity (convert deg/s to rad/s)
            imu_msg.angular_velocity.x = math.radians(gx)
            imu_msg.angular_velocity.y = math.radians(gy)
            imu_msg.angular_velocity.z = math.radians(gz)

            # Angular velocity covariance
            imu_msg.angular_velocity_covariance[0] = 0.02
            imu_msg.angular_velocity_covariance[4] = 0.02
            imu_msg.angular_velocity_covariance[8] = 0.02

            # Linear acceleration (already in m/s^2)
            imu_msg.linear_acceleration.x = ax
            imu_msg.linear_acceleration.y = ay
            imu_msg.linear_acceleration.z = az

            # Linear acceleration covariance
            imu_msg.linear_acceleration_covariance[0] = 0.04
            imu_msg.linear_acceleration_covariance[4] = 0.04
            imu_msg.linear_acceleration_covariance[8] = 0.04

            self.imu_pub.publish(imu_msg)

            # Publish magnetometer message
            mag_msg = MagneticField()
            mag_msg.header.stamp = current_time
            mag_msg.header.frame_id = self.frame_id

            # Convert micro-Teslas to Teslas
            mag_msg.magnetic_field.x = mx * 1e-6
            mag_msg.magnetic_field.y = my * 1e-6
            mag_msg.magnetic_field.z = mz * 1e-6

            # Magnetometer covariance
            mag_msg.magnetic_field_covariance[0] = 0.0
            mag_msg.magnetic_field_covariance[4] = 0.0
            mag_msg.magnetic_field_covariance[8] = 0.0

            self.mag_pub.publish(mag_msg)

        except Exception as e:
            self.consecutive_errors += 1
            if self.consecutive_errors <= 3:
                # Only log first few errors to avoid flooding
                self.get_logger().warn(f'I2C read error (attempt {self.consecutive_errors}): {e}')
            elif self.consecutive_errors == self.max_consecutive_errors:
                self.get_logger().error(
                    f'Too many consecutive I2C errors ({self.consecutive_errors}). '
                    'Check I2C connection and BNO055 sensor.'
                )
            # Add recovery delay after error
            time.sleep(self.I2C_READ_DELAY * 5)
        finally:
            self.i2c_lock.release()

    def destroy_node(self):
        """Cleanup on shutdown"""
        self.get_logger().info('Shutting down BNO055 IMU node')
        super().destroy_node()


class BNO055IMURawNode(Node):
    """
    Alternative node that publishes raw IMU data without BNO055's fusion.
    Useful if you want to use robot_localization's EKF for sensor fusion instead.
    """
    # Delay between I2C reads to prevent bus overload (software I2C is slow)
    I2C_READ_DELAY = 0.01  # 10ms delay between consecutive I2C reads
    MAX_RETRIES = 3  # Number of retries on I2C error

    def __init__(self):
        super().__init__('bno055_imu_raw')

        # Parameters
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('publish_rate', 20.0)  # Reduced for software I2C stability
        self.declare_parameter('i2c_bus', 'RPI_1SW')

        # Error tracking
        self.consecutive_errors = 0
        self.max_consecutive_errors = 10

        self.frame_id = self.get_parameter('frame_id').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.i2c_bus = self.get_parameter('i2c_bus').value

        # Initialize IMU
        self._init_imu()

        # Publishers
        self.imu_pub = self.create_publisher(Imu, 'imu/data_raw', 10)
        self.mag_pub = self.create_publisher(MagneticField, 'imu/mag', 10)

        # Timer
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_imu_data)

        self.get_logger().info('BNO055 IMU Raw node started')

    def _init_imu(self):
        """Initialize BNO055 IMU in IMU mode (gyro + accel fusion, no magnetometer)"""
        try:
            sys.path.insert(0, '/home/ubuntu/DI_Sensors/Python')
            from di_sensors import BNO055
            from di_sensors.inertial_measurement_unit import InertialMeasurementUnit

            # Initialize IMU
            self.imu = InertialMeasurementUnit(bus=self.i2c_bus)

            # Note: For raw data, we could set OPERATION_MODE_AMG (accelerometer + magnetometer + gyroscope)
            # but NDOF mode (default) still provides raw sensor data

            self.get_logger().info('BNO055 initialized in raw mode')

        except Exception as e:
            self.get_logger().error(f'Failed to initialize BNO055: {e}')
            raise

    def _safe_i2c_read(self, read_func, name, default=None):
        """
        Safely read from I2C with retry logic.
        Software I2C can be flaky, so we retry on errors.
        """
        for attempt in range(self.MAX_RETRIES):
            try:
                result = read_func()
                return result
            except Exception as e:
                if attempt < self.MAX_RETRIES - 1:
                    time.sleep(self.I2C_READ_DELAY * 2)  # Longer delay before retry
                else:
                    raise e
        return default

    def publish_imu_data(self):
        """Publish raw IMU data (no orientation)"""
        current_time = self.get_clock().now().to_msg()

        try:
            # Read raw sensor data with delays between reads
            gx, gy, gz = self._safe_i2c_read(self.imu.read_gyroscope, 'gyroscope')
            time.sleep(self.I2C_READ_DELAY)

            ax, ay, az = self._safe_i2c_read(self.imu.read_accelerometer, 'accelerometer')
            time.sleep(self.I2C_READ_DELAY)

            mx, my, mz = self._safe_i2c_read(self.imu.read_magnetometer, 'magnetometer')

            # Reset error counter on success
            self.consecutive_errors = 0

            # IMU message (no orientation)
            imu_msg = Imu()
            imu_msg.header.stamp = current_time
            imu_msg.header.frame_id = self.frame_id

            # No orientation - set covariance to -1 to indicate not provided
            imu_msg.orientation_covariance[0] = -1.0

            # Angular velocity
            imu_msg.angular_velocity.x = math.radians(gx)
            imu_msg.angular_velocity.y = math.radians(gy)
            imu_msg.angular_velocity.z = math.radians(gz)
            imu_msg.angular_velocity_covariance[0] = 0.02
            imu_msg.angular_velocity_covariance[4] = 0.02
            imu_msg.angular_velocity_covariance[8] = 0.02

            # Linear acceleration (with gravity)
            imu_msg.linear_acceleration.x = ax
            imu_msg.linear_acceleration.y = ay
            imu_msg.linear_acceleration.z = az
            imu_msg.linear_acceleration_covariance[0] = 0.04
            imu_msg.linear_acceleration_covariance[4] = 0.04
            imu_msg.linear_acceleration_covariance[8] = 0.04

            self.imu_pub.publish(imu_msg)

            # Magnetometer message
            mag_msg = MagneticField()
            mag_msg.header.stamp = current_time
            mag_msg.header.frame_id = self.frame_id
            mag_msg.magnetic_field.x = mx * 1e-6
            mag_msg.magnetic_field.y = my * 1e-6
            mag_msg.magnetic_field.z = mz * 1e-6

            self.mag_pub.publish(mag_msg)

        except Exception as e:
            self.consecutive_errors += 1
            if self.consecutive_errors <= 3:
                self.get_logger().warn(f'I2C read error (attempt {self.consecutive_errors}): {e}')
            elif self.consecutive_errors == self.max_consecutive_errors:
                self.get_logger().error(
                    f'Too many consecutive I2C errors ({self.consecutive_errors}). '
                    'Check I2C connection and BNO055 sensor.'
                )
            time.sleep(self.I2C_READ_DELAY * 5)


def main(args=None):
    rclpy.init(args=args)

    try:
        node = BNO055IMUNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
        import traceback
        traceback.print_exc()
    finally:
        if rclpy.ok():
            try:
                node.destroy_node()
            except:
                pass
            rclpy.shutdown()


def main_raw(args=None):
    """Entry point for raw IMU node"""
    rclpy.init(args=args)

    try:
        node = BNO055IMURawNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
        import traceback
        traceback.print_exc()
    finally:
        if rclpy.ok():
            try:
                node.destroy_node()
            except:
                pass
            rclpy.shutdown()


if __name__ == '__main__':
    main()
