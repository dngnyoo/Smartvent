# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

This is a GoPiGo3 robotics project running ROS 2 Humble on a Raspberry Pi 4 with Ubuntu 22.04. The project implements autonomous navigation with SLAM, Nav2, and sensor integration (IMU, LiDAR, ultrasonic). The robot platform is a Dexter Industries GoPiGo3 with custom ROS 2 drivers.

## Project Structure

- **nav2_ws/**: Main ROS 2 workspace for navigation
  - `src/gopigo3_driver/`: GoPiGo3 hardware driver (motor control, odometry, TF)
  - `src/gopigo_robot/`: High-level robot behaviors
  - `src/gopigo3_imu/`: IMU sensor integration
  - `src/rplidar_ros/`: RPLidar A1M8 driver
  - `src/slam_gmapping/`, `src/sllidar_ros2/`: SLAM implementations
- **ros2_ws/**: Secondary ROS 2 workspace
- **GoPiGo3/**: Dexter Industries GoPiGo3 Python library
- **DI_Sensors/**: Dexter Industries sensor library (BNO055 IMU, VL53L0X distance sensor)

## Build Commands

```bash
# Build nav2 workspace
cd ~/nav2_ws
source /opt/ros/humble/setup.zsh
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.zsh

# Install dependencies
rosdep install --from-paths src -y --ignore-src --rosdistro humble
```

## Running the Robot

### Prerequisites (first boot)

```bash
# Verify power management service
sudo systemctl status gpg3_power.service

# IMU calibration (required after each boot, takes 2-3 min)
sudo python3 ~/calibrate_imu.py
```

### Robot Bringup

```bash
cd ~/nav2_ws
source /opt/ros/humble/setup.zsh
source install/setup.zsh
ros2 launch gopigo3_driver gopigo3_bringup.launch.py
```

### Teleop Control

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
# i=forward, k=stop, ,=backward, j=left, l=right
```

### SLAM Mapping

```bash
# Run SLAM (integrated in gopigo3_bringup)
ros2 launch gopigo3_driver gopigo3_bringup.launch.py slam:=true

# Save map
ros2 run nav2_map_server map_saver_cli -f ~/maps/<map_name>
```

### Launch Arguments for gopigo3_bringup.launch.py

| Argument | Default | Description |
|----------|---------|-------------|
| `slam` | false | Enable SLAM Toolbox for mapping |
| `lidar` | true | Enable RPLidar |
| `ekf` | true | Enable EKF sensor fusion (wheel odom + IMU) |
| `foxglove` | true | Enable Foxglove Bridge visualization |
| `lidar_port` | /dev/rplidar | LiDAR serial port |
| `gas_sensor` | true | Enable gas sensor publisher |
| `patrol` | false | Enable patrol and evacuation mode |

## Robot Specifications

- **Wheel base**: 0.117m
- **Wheel diameter**: 0.066m
- **Encoder resolution**: 360 deg/rotation
- **Motor gear ratio**: 120

## Hardware Interfaces

| Interface | Device | Address/Pin | Purpose |
|-----------|--------|-------------|---------|
| SPI | /dev/spidev0.1 | CS1, 500kHz | GoPiGo3 board communication |
| Software I2C | GPIO 2,3 | - | Sensor communication |
| I2C 0x08 | - | GoPiGo3 board |
| I2C 0x28 | - | BNO055 IMU |
| I2C 0x29 | - | VL53L0X distance sensor |

## Critical: GPIO 23 Power Keep-Alive

GoPiGo3 requires GPIO 23 to be HIGH for motor power. Without this, SPI commands work but motors don't spin.

The `gpg3_power.service` handles this automatically. If motors don't work:
1. Check service status: `sudo systemctl status gpg3_power.service`
2. Manual test:
```python
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(23, GPIO.OUT)
GPIO.output(23, True)
```

## Sensor Notes

### BNO055 IMU
- Calibration resets on power cycle (Gyro 3/3, Accel 3/3, Mag 3/3 required)
- Uses Software I2C via DI_Sensors library
- Test: `~/test_imu.py`

### Camera Limitation
CSI cameras (Pi Camera V2) don't work on Ubuntu 22.04 due to missing Raspberry Pi GPU firmware integration. Use USB webcam or Intel RealSense D435i instead.

## Patrol and Evacuation Mode

Autonomous patrol between waypoints with gas-triggered emergency evacuation.

### Waypoints (map frame)
| Location | X | Y |
|----------|---|---|
| Start | 0.0 | 0.0 |
| Exit 1 | 4.11 | -11.28 |
| Exit 2 | 3.92 | -13.55 |
| Exit 3 | 5.09 | 5.48 |

### Usage
```bash
# Start with patrol mode via launch file
ros2 launch gopigo3_driver gopigo3_bringup.launch.py patrol:=true

# Or run patrol node separately (requires Nav2 running)
ros2 run gopigo3_driver patrol_evacuation

# With custom gas threshold
ros2 run gopigo3_driver patrol_evacuation --ros-args -p gas_threshold:=60.0
```

### Behavior
- **Normal**: Patrols Start → Exit 1 → Start → Exit 1 (repeat)
- **Emergency**: If gas_level exceeds threshold, evacuates to nearest exit (Exit 2)
- **Threshold**: Automatically fetched from `gas_publisher` node (`gas_threshold_voltage * 100`)
- **Topics**: Subscribes to `/gas_level` (std_msgs/Float32)

### Gas Sensor
Gas sensor is automatically started with `gopigo3_bringup.launch.py` (enabled by default).
```bash
# Disable gas sensor if not needed
ros2 launch gopigo3_driver gopigo3_bringup.launch.py gas_sensor:=false
```

## Shell Aliases (from .zshrc)

```bash
cw      # cd ~/ros2_ws
cs      # cd ~/ros2_ws/src
cb      # colcon build --symlink-install
cbp     # colcon build --symlink-install --packages-select <pkg>
tl      # ros2 topic list
te      # ros2 topic echo
nl      # ros2 node list
di      # rosdep install from src
```

## Reference Files

- IMU calibration: `~/calibrate_imu.py`
- IMU test: `~/test_imu.py`
- Power service: `~/gpg3_power.service`
- Camera test: `~/test_camera.py`
- Ultrasonic test: `~/test_ultrasonic.py`
