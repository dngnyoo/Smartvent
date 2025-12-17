# GoPiGo3 Nav2 Navigation Package Installation Guide

This guide explains how to install and use the manual SLAM mapping and Nav2 autonomous navigation features on the GoPiGo3 robot.

---

## Table of Contents
1. [Prerequisites](#1-prerequisites)
2. [Package Download](#2-package-download)
3. [Dependency Installation](#3-dependency-installation)
4. [Build and Configuration](#4-build-and-configuration)
5. [Installation Testing](#5-installation-testing)
6. [Integration with Existing Features](#6-integration-with-existing-features)
7. [Usage](#7-usage)
8. [Topics and TF Interface](#8-topics-and-tf-interface)
9. [Troubleshooting](#9-troubleshooting)
10. [Foxglove Visualization (Windows/Mac/Linux)](#10-foxglove-visualization-windowsmaclinux)
11. [Installation Checklist](#11-installation-checklist)

---

## 1. Prerequisites

### Hardware
- GoPiGo3 robot
- Raspberry Pi 4 (4GB or more recommended)
- RPLidar A1M8 (or compatible LiDAR)
- BNO055 IMU (optional)

### Software
- Ubuntu 22.04 (for Raspberry Pi)
- ROS2 Humble
- Python 3.10+

---

## 2. Package Download

### Method A: Git Clone (Recommended)
```bash
cd ~
git clone https://github.com/WonbumSohn/Robotics_SmartVent.git
```

### Method B: Copy Specific Folders Only
Required folders:
- `nav2_ws/src/gopigo3_driver/` - ROS2 package
- `maps/` - Sample maps (optional)

---

## 3. Dependency Installation

### ROS2 Package Installation
```bash
sudo apt update
sudo apt install -y \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-slam-toolbox \
  ros-humble-robot-localization \
  ros-humble-rplidar-ros \
  ros-humble-foxglove-bridge \
  ros-humble-tf2-ros \
  ros-humble-tf2-tools \
  ros-humble-tf2-geometry-msgs \
  ros-humble-xacro \
  ros-humble-joint-state-publisher \
  ros-humble-robot-state-publisher \
  ros-humble-teleop-twist-keyboard
```

### Python Dependencies
```bash
pip3 install spidev
```

### GoPiGo3 Library (if not installed)
```bash
cd ~
git clone https://github.com/DexterInd/GoPiGo3.git
cd GoPiGo3/Software/Python
sudo python3 setup.py install
```

### pigpio Daemon Setup
```bash
# Install pigpio if not present
sudo apt install -y pigpio

# Enable auto-start on boot
sudo systemctl enable pigpiod
sudo systemctl start pigpiod
```

### m-explore-ros2 Installation (for autonomous exploration, optional)
```bash
cd ~/nav2_ws/src
git clone -b humble https://github.com/robo-friends/m-explore-ros2.git
```

### udev Rules Setup (LiDAR device)
```bash
# Create RPLidar udev rule
echo 'KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0666", GROUP:="dialout", SYMLINK+="rplidar"' | sudo tee /etc/udev/rules.d/99-rplidar.rules

# Reload udev rules
sudo udevadm control --reload-rules
sudo udevadm trigger

# Add user to required groups
sudo usermod -a -G dialout $USER
sudo usermod -a -G spi $USER
sudo usermod -a -G gpio $USER
```

> **Note:** Re-login or reboot is required after group changes.

---

## 4. Build and Configuration

### 4.1 Integrating with Existing Workspace

If you already have `~/nav2_ws`:
```bash
# Copy only the gopigo3_driver package
cp -r ~/Robotics_SmartVent/nav2_ws/src/gopigo3_driver ~/nav2_ws/src/

# Build
cd ~/nav2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select gopigo3_driver --symlink-install
source install/setup.bash
```

### 4.2 Fresh Installation

```bash
# Create workspace
mkdir -p ~/nav2_ws/src
cd ~/nav2_ws/src

# Copy package
cp -r ~/Robotics_SmartVent/nav2_ws/src/gopigo3_driver .

# Install dependencies
cd ~/nav2_ws
rosdep install --from-paths src --ignore-src -r -y

# Build
source /opt/ros/humble/setup.bash
colcon build --packages-select gopigo3_driver --symlink-install
```

### 4.3 Environment Setup

Set up based on your shell:

```bash
# Check current shell
echo $SHELL
# /bin/zsh or /usr/bin/zsh → using zsh
# /bin/bash or /usr/bin/bash → using bash
```

**For zsh:**
```bash
echo "source /opt/ros/humble/setup.zsh" >> ~/.zshrc
echo "source ~/nav2_ws/install/setup.zsh" >> ~/.zshrc
source ~/.zshrc
```

**For bash:**
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source ~/nav2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 4.4 Map Folder Setup

```bash
# Create map storage folder
mkdir -p ~/maps

# Copy sample maps (optional)
cp ~/Robotics_SmartVent/maps/* ~/maps/
```

### 4.5 GoPiGo3 Calibration File Check

```bash
# Check if calibration file exists
ls -la /home/ubuntu/Dexter/gpg3_config.json

# If not present, create with default settings
mkdir -p /home/ubuntu/Dexter
cat > /home/ubuntu/Dexter/gpg3_config.json << 'EOF'
{
  "wheel-diameter": 66.5,
  "wheel-base-width": 117,
  "ticks": 6,
  "motor_gear_ratio": 120
}
EOF
```

---

## 5. Installation Testing

After building, run the following tests in order to verify the installation.

### 5.1 System Requirements Check

```bash
# Check Ubuntu version (22.04 required)
lsb_release -a

# Check ROS2 Humble installation
ros2 --version

# Check Python3
python3 --version
```

### 5.2 GoPiGo3 Hardware Communication Check

```bash
# Check SPI devices
ls /dev/spidev*
# Expected output: /dev/spidev0.0  /dev/spidev0.1

# GoPiGo3 Python test
python3 -c "import gopigo3; gpg = gopigo3.GoPiGo3(); print('Manufacturer:', gpg.get_manufacturer()); print('Board:', gpg.get_board()); print('Voltage:', gpg.get_voltage_battery())"
# Expected output: Manufacturer: Dexter Industries, Board: GoPiGo3, etc.
```

### 5.3 LiDAR Connection Check

```bash
# Check LiDAR device
ls -la /dev/rplidar
# or
ls -la /dev/ttyUSB*
```

### 5.4 ROS2 Package Check

```bash
# Check gopigo3_driver in package list
ros2 pkg list | grep gopigo3
# Expected output: gopigo3_driver

# Check available executables
ros2 pkg executables gopigo3_driver
```

### 5.5 Basic Bringup Test

```bash
# Terminal 1: Run robot driver
ros2 launch gopigo3_driver gopigo3_bringup.launch.py

# Terminal 2: Check topics
ros2 topic list
# Topics to verify:
# /odom, /odometry/filtered, /imu/data, /scan, /tf, /tf_static

# Check odometry data
ros2 topic echo /odometry/filtered --once
```

### 5.6 Motor Operation Test

```bash
# With bringup running, in Terminal 2:
# Move forward command
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}, angular: {z: 0.0}}" --once

# Robot should move slightly forward
# Stop command
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}" --once
```

### 5.7 TF Tree Check

```bash
# With bringup running
ros2 run tf2_tools view_frames

# Or real-time check
ros2 run tf2_ros tf2_echo odom base_footprint
```

---

## 6. Integration with Existing Features

### 6.1 TF Frame Structure

This package uses the following TF tree:
```
map
 └── odom (published by EKF or slam_toolbox)
      └── base_footprint
           └── base_link
                ├── left_wheel
                ├── right_wheel
                ├── laser_frame
                └── imu_link
```

### 6.2 Conflict Prevention Checklist

Check when using with other nodes:

| Item | This Package | Note |
|------|--------------|------|
| `odom → base_footprint` TF | Published by EKF | Do not publish from other nodes |
| `/cmd_vel` topic | Published by Nav2 | Caution when using with teleop |
| `/scan` topic | Requires LiDAR driver | Run rplidar_ros separately |
| `/imu/data` topic | Requires BNO055 node | Modify EKF config if no IMU |

### 6.3 Running with Other Nodes

**Example: Running with LiDAR**
```bash
# Terminal 1: LiDAR
ros2 launch rplidar_ros rplidar_a1_launch.py

# Terminal 2: Navigation
ros2 launch gopigo3_driver navigation.launch.py map:=/home/ubuntu/maps/20251212_172520
```

### 6.4 Launch File Integration

To integrate with existing launch files:
```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    gopigo3_driver_dir = get_package_share_directory('gopigo3_driver')

    # Include Nav2 Navigation
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gopigo3_driver_dir, 'launch', 'navigation.launch.py')
        ),
        launch_arguments={'map': '/home/ubuntu/maps/your_map'}.items()
    )

    return LaunchDescription([
        # Your existing nodes...
        navigation_launch,
    ])
```

---

## 7. Usage

### 7.1 Manual Mapping (Creating New Map)

```bash
# Terminal 1: Start SLAM
ros2 launch gopigo3_driver slam.launch.py

# Terminal 2: Map saver (prepare for saving)
ros2 run gopigo3_driver map_saver --ros-args -p auto_save:=false

# Terminal 3: Control robot with teleop
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**Keyboard Controls:**
- `i`: Forward
- `k`: Stop
- `,`: Backward
- `j`: Turn left
- `l`: Turn right

**Save map after mapping:**
```bash
ros2 service call /save_map std_srvs/srv/Trigger
```

Map files are saved to: `~/maps/YYYYMMDD_HHMMSS.data`, `.posegraph`, `_positions.json`

### 7.2 Autonomous Navigation with Saved Map

```bash
# Start Nav2 + Localization
ros2 launch gopigo3_driver navigation.launch.py \
  map:=/home/ubuntu/maps/20251212_172520
```

### 7.3 Setting Goal Points

**Method 1: Click in Foxglove**
1. Launch Foxglove Studio
2. Click desired position on map panel
3. `/move_base_simple/goal` → `goal_relay` → `/goal_pose` → Nav2

**Method 2: Command Line**
```bash
ros2 topic pub /goal_pose geometry_msgs/PoseStamped "{
  header: {frame_id: 'map'},
  pose: {
    position: {x: 1.0, y: 0.5, z: 0.0},
    orientation: {w: 1.0}
  }
}"
```

**Method 3: Auto Return to Start Position**
```bash
ros2 run gopigo3_driver go_to_start \
  --ros-args -p map_file:=/home/ubuntu/maps/20251212_172520
```

---

## 8. Topics and TF Interface

### Main Topics

| Topic | Type | Direction | Description |
|-------|------|-----------|-------------|
| `/cmd_vel` | `Twist` | Input | Robot velocity command |
| `/odom` | `Odometry` | Output | Wheel odometry |
| `/odometry/filtered` | `Odometry` | Output | EKF fused odometry |
| `/scan` | `LaserScan` | Input | LiDAR data |
| `/imu/data` | `Imu` | Input | IMU data |
| `/goal_pose` | `PoseStamped` | Input | Nav2 goal position |
| `/move_base_simple/goal` | `PoseStamped` | Input | Foxglove goal (relayed) |
| `/map` | `OccupancyGrid` | Output | Map data |

### Available Nodes

| Node | Run Command | Description |
|------|-------------|-------------|
| `gopigo3_driver` | `ros2 run gopigo3_driver gopigo3_driver` | Motor/odometry driver |
| `bno055_imu` | `ros2 run gopigo3_driver bno055_imu` | IMU driver |
| `map_saver` | `ros2 run gopigo3_driver map_saver` | Map saving node |
| `go_to_start` | `ros2 run gopigo3_driver go_to_start` | Return to start node |
| `goal_relay` | `ros2 run gopigo3_driver goal_relay` | Foxglove goal relay |

---

## 9. Troubleshooting

### 9.1 "pigpiod not running" Error
```bash
sudo pigpiod
# or
sudo systemctl start pigpiod
```

### 9.2 SPI Communication Error

```
[ERROR] No SPI response
```

**Solution:**
```bash
# Check SPI is enabled
sudo raspi-config
# Interface Options > SPI > Enable

# Start pigpiod
sudo pigpiod

# Reboot
sudo reboot
```

### 9.3 GoPiGo3 Board Communication Failure
```bash
# Check I2C (should respond at 0x08)
sudo i2cdetect -y 1

# Test in Python
sudo python3
>>> import sys
>>> sys.path.insert(0, '/home/ubuntu/GoPiGo3/Software/Python')
>>> import gopigo3
>>> GPG = gopigo3.GoPiGo3()
>>> print(GPG.get_manufacturer())
```

### 9.4 LiDAR Permission Error

```
[ERROR] Cannot open serial port
```

**Solution:**
```bash
sudo chmod 666 /dev/ttyUSB0
# or reapply udev rules
sudo udevadm control --reload-rules
sudo udevadm trigger
```

### 9.5 TF Timestamp Error
```
Extrapolation Error: Requested time X but earliest data is at time Y
```
**Solution:** Already set to `transform_tolerance: 2.0` in config files.
When using Foxglove, the `goal_relay` node automatically fixes timestamps.

### 9.6 Nav2 Server Timeout
```
Timed out while waiting for action server
```
**Solution:** Frequencies are already optimized for Raspberry Pi 4 performance.
If issues persist:
```yaml
# nav2_params.yaml
bt_navigator:
  default_server_timeout: 180  # 120 → 180
  wait_for_service_timeout: 15000  # 10000 → 15000
```

### 9.7 EKF Update Failure
```
Failed to meet update rate!
```
**Solution:** Lower frequency in `config/ekf.yaml`:
```yaml
frequency: 8.0  # 10.0 → 8.0
```

### 9.8 Package Not Found

```
Package 'gopigo3_driver' not found
```

**Solution:**
```bash
# For zsh
source ~/nav2_ws/install/setup.zsh

# For bash
source ~/nav2_ws/install/setup.bash

# Or rebuild
cd ~/nav2_ws && colcon build
```

### 9.9 Topic/TF Verification Commands
```bash
# List topics
ros2 topic list

# Check odometry
ros2 topic echo /odom --once

# Check TF tree
ros2 run tf2_ros tf2_echo map base_footprint

# Visualize TF tree
ros2 run tf2_tools view_frames
```

---

## 10. Foxglove Visualization (Windows/Mac/Linux)

Foxglove allows you to view the robot's map, position, LiDAR scan, etc. in real-time from a separate PC.

### 10.1 Foxglove Installation

**Method 1: Web Browser (No Installation Required)**
- Visit https://app.foxglove.dev
- Supports Chrome, Edge, Safari, etc.

**Method 2: Desktop App**
- https://foxglove.dev/download
- Download installer for Windows, Mac, or Linux

### 10.2 Connecting to Robot

1. Run bringup on the robot (Raspberry Pi):
   ```bash
   ros2 launch gopigo3_driver gopigo3_bringup.launch.py
   ```
   - `foxglove_bridge` automatically starts on port 8765

2. Connect from Foxglove:
   - Click **Open connection**
   - Select **Foxglove WebSocket**
   - Enter address: `ws://[Raspberry_Pi_IP]:8765`
   - Example: `ws://192.168.1.100:8765`

### 10.3 Finding Raspberry Pi IP

On the Raspberry Pi:
```bash
hostname -I
```

### 10.4 Network Requirements

| Device | Requirement |
|--------|-------------|
| Raspberry Pi (Robot) | WiFi connected |
| PC (Foxglove) | Same WiFi network |

### 10.5 Adding Foxglove Panels

After connecting, add the following panels for visualization:

| Panel | Topic | Description |
|-------|-------|-------------|
| Map | `/map` | 2D map |
| 3D | `/scan`, TF | LiDAR scan + robot position |
| Image | `/camera/image_raw` | Camera (if available) |
| Raw Messages | Any topic | View topic data |

### 10.6 Setting Goals in Foxglove

1. Select **Publish** tool in 3D or Map panel
2. Topic: `/move_base_simple/goal` or `/goal_pose`
3. Click desired position on map
4. `goal_relay` node forwards to Nav2

---

## 11. Installation Checklist

Verify all items after installation:

- [ ] Ubuntu 22.04 and ROS2 Humble installed
- [ ] GoPiGo3 Python test passed (`python3 -c "import gopigo3; ..."`)
- [ ] LiDAR device recognized (`/dev/rplidar` or `/dev/ttyUSB*`)
- [ ] `ros2 pkg list | grep gopigo3` shows output
- [ ] `ros2 launch gopigo3_driver gopigo3_bringup.launch.py` runs without errors
- [ ] `/odom`, `/odometry/filtered`, `/imu/data`, `/scan` topics publishing
- [ ] Robot movement tested with `cmd_vel`
- [ ] TF tree verified (`ros2 run tf2_tools view_frames`)
- [ ] (Optional) Foxglove connection tested

---

## License

MIT License

## Contact

GitHub Issues: https://github.com/WonbumSohn/Robotics_SmartVent/issues
