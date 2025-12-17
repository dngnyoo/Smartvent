#!/usr/bin/env python3
"""
Gas Sensor Publisher Node for GoPiGo3

Reads MQ-2 gas sensor via ADS1115 ADC and publishes gas level.

Topics:
  - Publishes: /gas_level (std_msgs/Float32)

Hardware:
  - ADS1115 ADC on Software I2C (GPIO 17 SDA, GPIO 27 SCL)
  - MQ-2 Gas Sensor connected to ADS1115 channel 0
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

import pigpio
import time
import os
import sys


class GasPublisherNode(Node):
    # Hardware configuration
    SDA_PIN = 17
    SCL_PIN = 27
    ADS_ADDR = 0x48
    BAUD_RATE = 100000

    REG_CONVERSION = 0x00
    REG_CONFIG = 0x01
    CONFIG_HI = 0xC2
    CONFIG_LO = 0x83

    def __init__(self):
        super().__init__('gas_publisher')

        # Parameters
        self.declare_parameter('publish_rate', 1.0)  # Hz
        self.declare_parameter('gas_threshold_voltage', 1.0)  # Volts for audio alert
        self.declare_parameter('enable_audio_alert', True)
        self.declare_parameter('alert_cooldown', 5.0)  # seconds

        publish_rate = self.get_parameter('publish_rate').value
        self.gas_threshold = self.get_parameter('gas_threshold_voltage').value
        self.enable_audio = self.get_parameter('enable_audio_alert').value
        self.alert_cooldown = self.get_parameter('alert_cooldown').value

        # Publisher
        self.publisher_ = self.create_publisher(Float32, 'gas_level', 10)

        # Timer
        self.timer = self.create_timer(1.0 / publish_rate, self.timer_callback)

        # Initialize pigpio
        self.pi = pigpio.pi()
        if not self.pi.connected:
            self.get_logger().error('Failed to connect to pigpiod!')
            sys.exit(1)

        try:
            self.pi.bb_i2c_open(self.SDA_PIN, self.SCL_PIN, self.BAUD_RATE)
        except:
            self.pi.bb_i2c_close(self.SDA_PIN)
            self.pi.bb_i2c_open(self.SDA_PIN, self.SCL_PIN, self.BAUD_RATE)

        self.write_config()

        self.last_alert = 0
        self.get_logger().info('Gas Sensor Node Started')
        self.get_logger().info(f'  Threshold: {self.gas_threshold}V')
        self.get_logger().info(f'  Audio alerts: {self.enable_audio}')

    def speak_danger(self):
        """Play audio warning"""
        os.system('espeak "Danger danger. Gas level is very high." --stdout | aplay >/dev/null 2>&1 &')

    def write_config(self):
        """Configure ADS1115"""
        cmd = [4, self.ADS_ADDR, 2, 7, 3, self.REG_CONFIG, self.CONFIG_HI, self.CONFIG_LO, 3, 0]
        self.pi.bb_i2c_zip(self.SDA_PIN, cmd)

    def read_value(self):
        """Read raw ADC value"""
        self.pi.bb_i2c_zip(self.SDA_PIN, [4, self.ADS_ADDR, 2, 7, 1, self.REG_CONVERSION, 3, 0])
        count, data = self.pi.bb_i2c_zip(self.SDA_PIN, [4, self.ADS_ADDR, 2, 6, 2, 3, 0])

        if count > 0 and len(data) >= 2:
            value = (data[0] << 8) | data[1]
            if value > 32767:
                value -= 65536
            return value
        return None

    def timer_callback(self):
        """Periodic sensor reading and publishing"""
        raw_val = self.read_value()

        if raw_val is not None:
            voltage = raw_val * 0.000125

            # Publish gas level (voltage * 100)
            # Display shows warning when > 50.0 (i.e., voltage > 0.5V)
            msg = Float32()
            msg.data = voltage * 100.0
            self.publisher_.publish(msg)

            self.get_logger().debug(f'Gas: {raw_val} | {voltage:.2f}V | ROS: {msg.data:.1f}')

            # Audio alert
            if self.enable_audio and voltage > self.gas_threshold:
                self.get_logger().warn(f'GAS LEVEL HIGH: {voltage:.2f}V')
                if time.time() - self.last_alert > self.alert_cooldown:
                    self.speak_danger()
                    self.last_alert = time.time()
        else:
            self.get_logger().warn('Error reading gas sensor')

    def cleanup(self):
        """Clean up pigpio resources"""
        try:
            self.pi.bb_i2c_close(self.SDA_PIN)
            self.pi.stop()
        except:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = GasPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
