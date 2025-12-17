#!/usr/bin/env python3
"""
GoPiGo3 Power Keep-Alive Script

This script sets GPIO 23 HIGH to signal to the GoPiGo3 board that
the Raspberry Pi is running. Without this, the GoPiGo3 board will
assume the Pi is shut down and disable motor power.

This should run as a systemd service at boot time.

Usage:
    sudo python3 gpg3_power.py

Install as service:
    sudo cp gpg3_power.service /etc/systemd/system/
    sudo systemctl daemon-reload
    sudo systemctl enable gpg3_power
    sudo systemctl start gpg3_power
"""

import signal
import sys
import time

try:
    import RPi.GPIO as GPIO
except ImportError:
    print("Error: RPi.GPIO not available")
    print("Install with: sudo apt install python3-rpi.gpio")
    sys.exit(1)


def setup_gpio():
    """Set up GPIO 23 as output and set HIGH"""
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    GPIO.setup(23, GPIO.OUT)
    GPIO.output(23, GPIO.HIGH)
    print("GPIO 23 set to HIGH - GoPiGo3 motor power enabled")


def cleanup(signum=None, frame=None):
    """Clean up GPIO on exit"""
    print("Cleaning up GPIO...")
    GPIO.cleanup()
    sys.exit(0)


def main():
    # Set up signal handlers
    signal.signal(signal.SIGTERM, cleanup)
    signal.signal(signal.SIGINT, cleanup)

    # Set up GPIO
    setup_gpio()

    # Keep running
    print("GoPiGo3 power service running. Press Ctrl+C to stop.")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        cleanup()


if __name__ == '__main__':
    main()
